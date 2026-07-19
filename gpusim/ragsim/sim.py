"""ベルト式直動射出 + 自由飛行の CUDA バッチシミュレーション。

viewer/src/sim/rapier-rag.ts の simulateLinear + flightLoop の移植。
Rapier(陰的ばね, dt=1/240) と違い陽的(半陰的オイラー)なので、
1/240 のフレームを SUB サブステップに割って安定化する。
ばね・空力・ベルト送り・接触・静定判定まで全て [B, 442, 3] テンソルで
B 本の投擲を同時に計算する(GPU 並列化の本体)。
"""
from __future__ import annotations

import numpy as np
import torch

from . import mesh
from .targets import contact_ground

DT = 1 / 240
SUB = 64  # サブステップ(陽的ばねの安定条件 ω·dt<0.3, 減衰 c/m·dt<0.5 を満たす)
DTH = DT / SUB
G = 9.81

RHO = 1.225
CD_N = 1.5
CD_T = 0.02
MAGNUS_C = 0.6

REST_V = 0.28  # 静定判定: 全ノード速度がこれ未満
REST_STEPS = 16  # が連続この outer step 続いたら安静

# ベルト送り(simulateLinear 移植)
GAP_HALF = mesh.THICK * 0.75
GRIP_TOL = mesh.THICK * 5
TAU_FEED = 0.008
K_SQ = 1200.0
C_SQ = 4.0
FEED_MAX_T = 1.2


class _EagerFallback(Exception):
    """compile 無効時に eager 経路へ落とすための内部例外"""


class BeltSim:
    """対向ベルト射出 → 自由飛行 → 標的+床で静定 → 判定 をバッチで回す。

    speed/elev_deg: [B] 各投のベルト面速度・仰角。seeds: [B] 初期形状の乱数。
    record=True で record_stride おきに全ノード座標を CPU へ収録(動画用)。
    """

    def __init__(
        self,
        target,
        speed: np.ndarray,
        elev_deg: np.ndarray,
        seeds: np.ndarray,
        rag_m: float = 0.048,
        nip_len: float = 0.35,
        spin_frac: float = 0.0,
        pivot: tuple[float, float, float] = (0.0, 0.9, 0.0),
        max_time: float = 4.0,
        device: str = "cuda",
        record: bool = False,
        record_stride: int = 3,
    ) -> None:
        self.target = target
        self.device = torch.device(device)
        self.B = len(speed)
        self.max_time = max_time
        self.nip_len = nip_len
        self.spin_frac = spin_frac
        self.record = record
        self.record_stride = record_stride

        B, N = self.B, mesh.N_NODES
        dev = self.device
        f32 = torch.float32

        self.node_m = float(mesh.node_mass(rag_m))
        self.speed = torch.as_tensor(speed, dtype=f32, device=dev)
        elev = torch.as_tensor(np.asarray(elev_deg, dtype=np.float64) * np.pi / 180, dtype=f32, device=dev)
        self.pivot = torch.tensor(pivot, dtype=f32, device=dev)

        # 射出基底: e1=+z, lateral=e1×up=(-1,0,0), trackDir/squeeze は仰角依存 [B,3]
        z = torch.zeros(B, dtype=f32, device=dev)
        self.track = torch.stack([z, torch.sin(elev), torch.cos(elev)], dim=-1)
        self.squeeze = torch.stack([z, torch.cos(elev), -torch.sin(elev)], dim=-1)

        # 初期形状(ニップ後方へ寝かせて装填, numpy per-seed)
        lateral = np.array([-1.0, 0.0, 0.0])
        pos0 = np.zeros((B, N, 3), dtype=np.float32)
        tr = self.track.cpu().numpy().astype(np.float64)
        sq = self.squeeze.cpu().numpy().astype(np.float64)
        pv = np.asarray(pivot, dtype=np.float64)
        for b in range(B):
            rng = np.random.default_rng(int(seeds[b]))
            pos0[b] = mesh.place_hanging(pv, -tr[b], lateral, sq[b], rng)
        self.pos = torch.as_tensor(pos0, device=dev)
        self.vel = torch.zeros_like(self.pos)

        # ばね(静止長は初期距離 = TS版と同じ)
        ii, jj, ks, kd = mesh.build_springs()
        self.si = torch.as_tensor(ii, device=dev)
        self.sj = torch.as_tensor(jj, device=dev)
        self.ks = torch.as_tensor(ks, device=dev)
        self.kd = torch.as_tensor(kd, device=dev)
        d0 = self.pos[:, self.sj] - self.pos[:, self.si]
        self.rest = d0.norm(dim=-1)  # [B,E]

        # 空力: セル4隅(上層/下層)と 8ノード分配用 flat index
        qt = mesh.surface_quads(0)
        qb = mesh.surface_quads(1)
        self.ct = torch.as_tensor(qt, device=dev)  # [C,4]
        self.cb = torch.as_tensor(qb, device=dev)
        self.aero_scatter = torch.cat([self.ct, self.cb], dim=1).reshape(-1)  # [C*8]
        self.cell_a = (mesh.RAG_L / mesh.NY) * (mesh.RAG_W / mesh.NX)
        self.vol_per_node = mesh.RAG_W * mesh.RAG_L * mesh.THICK / mesh.N_NODES

        self.gvec = torch.tensor([0.0, -G, 0.0], dtype=f32, device=dev)
        if hasattr(target, "warmup"):
            target.warmup(dev)  # 定数テンソルをグラフ記録前に生成

    # ---------------- 力 ----------------
    def _spring_force(self, F: torch.Tensor, pos: torch.Tensor, vel: torch.Tensor) -> None:
        d = pos[:, self.sj] - pos[:, self.si]
        L = d.norm(dim=-1) + 1e-9
        u = d / L.unsqueeze(-1)
        rv = ((vel[:, self.sj] - vel[:, self.si]) * u).sum(-1)
        s = self.ks * (L - self.rest) + self.kd * rv
        fp = u * s.unsqueeze(-1)
        F.index_add_(1, self.si, fp)
        F.index_add_(1, self.sj, -fp)

    def _aero_force(self) -> torch.Tensor:
        """セル中面の圧力抗力+表面摩擦、全体ωのマグヌス(applyAero 移植)。[B,N,3]"""
        pos, vel = self.pos, self.vel  # 空力は outer step 頭の状態で計算し保持
        mids = (pos[:, self.ct] + pos[:, self.cb]) * 0.5  # [B,C,4,3]
        n = torch.cross(mids[:, :, 2] - mids[:, :, 0], mids[:, :, 3] - mids[:, :, 1], dim=-1)
        n = n / (n.norm(dim=-1, keepdim=True) + 1e-9)
        v = (vel[:, self.ct].sum(2) + vel[:, self.cb].sum(2)) / 8  # [B,C,3]
        vn = (v * n).sum(-1, keepdim=True)
        fN = n * vn * vn.abs() * (-0.5 * RHO * CD_N * self.cell_a)
        vt = v - n * vn
        fT = vt * vt.norm(dim=-1, keepdim=True) * (-0.5 * RHO * CD_T * self.cell_a)
        fq = (fN + fT) / 8  # 8ノードへ分配
        F = torch.zeros_like(pos)
        C = fq.shape[1]
        F.index_add_(1, self.aero_scatter, fq.unsqueeze(2).expand(-1, C, 8, 3).reshape(self.B, C * 8, 3))
        # マグヌス: 布全体の剛体近似 ω から F = C·ρ·体積·(ω×v)
        c = pos.mean(dim=1, keepdim=True)
        V = vel.mean(dim=1, keepdim=True)
        r = pos - c
        dv = vel - V
        Lm = torch.cross(r, dv, dim=-1).sum(dim=1)  # [B,3]
        I = (r * r).sum(dim=(1, 2)) + 1e-9
        omega = Lm / I.unsqueeze(-1)
        F += MAGNUS_C * RHO * self.vol_per_node * torch.cross(omega.unsqueeze(1).expand_as(vel), vel, dim=-1)
        return F

    def _belt_force(self, F: torch.Tensor, pos: torch.Tensor, vel: torch.Tensor, feeding: torch.Tensor) -> None:
        """ニップ内ノードを摩擦サーボでベルト面速度へ引き込み+法線締め込み"""
        rel = pos - self.pivot
        s_along = (rel * self.track.unsqueeze(1)).sum(-1)  # [B,N]
        off = (rel * self.squeeze.unsqueeze(1)).sum(-1)
        in_nip = (s_along >= -0.01) & (s_along <= self.nip_len) & (off.abs() <= GRIP_TOL) & feeding.unsqueeze(-1)
        v_top = self.speed * (1 - self.spin_frac / 2)
        v_bot = self.speed * (1 + self.spin_frac / 2)
        belt_v = torch.where(off >= 0, v_top.unsqueeze(-1), v_bot.unsqueeze(-1))
        v_track = (vel * self.track.unsqueeze(1)).sum(-1)
        v_sq = (vel * self.squeeze.unsqueeze(1)).sum(-1)
        target_off = torch.where(off >= 0, GAP_HALF, -GAP_HALF)
        fT = self.node_m * (belt_v - v_track) / TAU_FEED
        fN = self.node_m * (-K_SQ * (off - target_off) - C_SQ * v_sq)
        m = in_nip.to(F.dtype).unsqueeze(-1)
        F += (self.track.unsqueeze(1) * fT.unsqueeze(-1) + self.squeeze.unsqueeze(1) * fN.unsqueeze(-1)) * m

    def _substep_block(self, pos: torch.Tensor, vel: torch.Tensor, aeroF: torch.Tensor, feeding: torch.Tensor, active_f: torch.Tensor, use_belt: bool) -> tuple[torch.Tensor, torch.Tensor]:
        """1 outer step (=1/240s) 分の SUB サブステップ。torch.compile(CUDA Graphs) 対象。
        pos/vel はコピーを受け取り更新して返す(接触は in-place)。"""
        for _ in range(SUB):
            F = aeroF.clone()
            self._spring_force(F, pos, vel)
            if use_belt:
                self._belt_force(F, pos, vel, feeding)
            vel = vel + DTH * (F / self.node_m + self.gvec)
            vel = vel * active_f
            pos = pos + DTH * vel
            contact_ground(pos, vel)
            self.target.contact(pos, vel)
        return pos, vel

    def _advance_eager(self, aeroF, feeding, active_f, all_fed):
        """eager 経路の 1 outer step。エンジン差し替え用フック(Newton版・機構版がオーバーライド)。"""
        return self._substep_block(self.pos, self.vel, aeroF, feeding, active_f, not all_fed)

    def _release_mask(self, fed: torch.Tensor, step: int) -> torch.Tensor:
        """「離脱した」判定 [B]。既定=ベルト(布全体がニップ通過)。機構はオーバーライド。"""
        rel = self.pos - self.pivot
        s_along = (rel * self.track.unsqueeze(1)).sum(-1)
        min_s = s_along.min(dim=-1).values
        return (~fed) & (min_s > self.nip_len)

    def _extra_result(self) -> dict:
        """run() の返り値へ機構固有の追加情報(腕の軌跡など)を足すフック"""
        return {}

    # ---------------- メインループ ----------------
    @torch.no_grad()
    def run(self) -> dict:
        B, dev = self.B, self.device
        n_outer = int(round(self.max_time / DT))
        feed_max_steps = int(getattr(self, "feed_max_t", FEED_MAX_T) / DT)

        fed = torch.zeros(B, dtype=torch.bool, device=dev)
        active = torch.ones(B, dtype=torch.bool, device=dev)
        settled = torch.zeros(B, dtype=torch.bool, device=dev)
        settle_cnt = torch.zeros(B, dtype=torch.int32, device=dev)
        settle_step = torch.full((B,), n_outer - 1, dtype=torch.int32, device=dev)
        release_step = torch.full((B,), 0, dtype=torch.int32, device=dev)
        broke = torch.zeros(B, dtype=torch.bool, device=dev)

        rel_speed = torch.zeros(B, device=dev)
        rel_ang = torch.zeros(B, device=dev)
        rel_spin = torch.zeros(B, device=dev)

        mode, plane_v = self.target.arrival_plane()
        arrival = torch.full((B, 3), float("nan"), device=dev)
        arrival_vel = torch.full((B, 3), float("nan"), device=dev)  # 判定面通過時の重心速度(下降当ての診断用)
        has_arr = torch.zeros(B, dtype=torch.bool, device=dev)

        frames: list[np.ndarray] = []
        frame_steps: list[int] = []

        prev_c = self.pos.mean(dim=1)
        all_fed = False

        # torch.compile(CUDA Graphs) でサブステップの起動オーバーヘッドを削減。失敗時は eager。
        import os
        block = self._substep_block
        if os.environ.get("RAGSIM_COMPILE", "1") == "1" and getattr(self, "engine", "torch") == "torch":
            if not hasattr(BeltSim, "_compiled_block"):
                BeltSim._compiled_block = torch.compile(BeltSim._substep_block, mode="reduce-overhead", dynamic=False)

            def block(*a):
                # CUDA Graph の出力バッファを次イテレーションの入力に回すための区切り宣言
                torch.compiler.cudagraph_mark_step_begin()
                return BeltSim._compiled_block(self, *a)
        compile_ok = getattr(self, "engine", "torch") == "torch"

        SYNC_EVERY = 32  # GPU→CPU同期(発散チェック・全静定の早期打切り)の間引き

        for step in range(n_outer):
            aeroF = self._aero_force()
            feeding = ~fed
            active_f = active.view(B, 1, 1).to(self.pos.dtype)
            # 出力はグラフ再利用バッファなので、必ず永続バッファ self.pos/vel へ copy_ で戻す
            try:
                if compile_ok:
                    pos_out, vel_out = block(self.pos, self.vel, aeroF, feeding, active_f, not all_fed)
                else:
                    raise _EagerFallback
            except _EagerFallback:
                pos_out, vel_out = self._advance_eager(aeroF, feeding, active_f, all_fed)
            except Exception as e:  # compile 失敗 → 以降 eager
                compile_ok = False
                print(f"  (torch.compile 失敗 → eager 継続: {type(e).__name__})")
                pos_out, vel_out = self._advance_eager(aeroF, feeding, active_f, all_fed)
            if pos_out is not None:
                self.pos.copy_(pos_out)
                self.vel.copy_(vel_out)

            # --- outer step ごとの bookkeeping(同期なしのマスク更新のみ) ---
            c = self.pos.mean(dim=1)

            # 送り出し完了判定 + 離脱条件の計測(機構はサブクラスが _release_mask をオーバーライド)
            if not all_fed:
                newly = self._release_mask(fed, step)
                if step >= feed_max_steps:
                    newly = newly | (~fed)
                V = self.vel.mean(dim=1)
                sp = V.norm(dim=-1)
                ang = torch.atan2(V[:, 1], torch.hypot(V[:, 0], V[:, 2]))
                r = self.pos - c.unsqueeze(1)
                dv = self.vel - V.unsqueeze(1)
                Lm = torch.cross(r, dv, dim=-1).sum(dim=1)
                I = (r * r).sum(dim=(1, 2)) + 1e-9
                spn = Lm.norm(dim=-1) / I
                rel_speed = torch.where(newly, sp, rel_speed)
                rel_ang = torch.where(newly, ang, rel_ang)
                rel_spin = torch.where(newly, spn, rel_spin)
                release_step = torch.where(newly, torch.full_like(release_step, step), release_step)
                fed |= newly
                if step % 8 == 7 or step >= feed_max_steps:
                    all_fed = bool(fed.all())  # (同期は8stepに1回)

            # 散布ドット用の幾何到達(重心が判定面を通過した点)
            if mode == "y_down":
                crossed = (~has_arr) & fed & (c[:, 1] <= plane_v) & (prev_c[:, 1] > plane_v) & (c[:, 1] < prev_c[:, 1])
                f = ((prev_c[:, 1] - plane_v) / (prev_c[:, 1] - c[:, 1] + 1e-9)).clamp(0, 1)
            else:  # z_fwd
                crossed = (~has_arr) & fed & (c[:, 2] >= plane_v) & (prev_c[:, 2] < plane_v)
                f = ((plane_v - prev_c[:, 2]) / (c[:, 2] - prev_c[:, 2] + 1e-9)).clamp(0, 1)
            interp = prev_c + (c - prev_c) * f.unsqueeze(-1)
            arrival = torch.where(crossed.unsqueeze(-1), interp, arrival)
            arrival_vel = torch.where(crossed.unsqueeze(-1), (c - prev_c) / DT, arrival_vel)
            has_arr |= crossed
            prev_c = c

            # 静定検出(離脱後): 全ノード速度 < REST_V が REST_STEPS 連続
            max_v = self.vel.norm(dim=-1).max(dim=-1).values
            calm = fed & (max_v < REST_V)
            settle_cnt = torch.where(calm, settle_cnt + 1, torch.zeros_like(settle_cnt))
            newly_settled = (~settled) & (settle_cnt > REST_STEPS)
            settle_step = torch.where(newly_settled, torch.full_like(settle_step, step), settle_step)
            settled |= newly_settled
            active &= ~settled

            if self.record and step % self.record_stride == 0:
                frames.append(self.pos.detach().cpu().numpy().copy())
                frame_steps.append(step)

            # 同期を伴うチェックは間引いて実行
            if step % SYNC_EVERY == SYNC_EVERY - 1:
                bad = ~torch.isfinite(c).all(dim=-1)
                if bool(bad.any()):
                    broke |= bad
                    active &= ~bad
                    torch.nan_to_num_(self.pos)
                    torch.nan_to_num_(self.vel)
                if all_fed and bool(settled.all()):
                    break

        hit = self.target.judge(self.pos) & ~broke
        # 到達点が未確定(面を通過しなかった)ものは最終重心
        final_c = self.pos.mean(dim=1)
        arrival = torch.where(has_arr.unsqueeze(-1), arrival, final_c)
        radial = self.target.radial_of(arrival)

        return {
            "hit": hit.cpu().numpy(),
            "arrival_vel": arrival_vel.cpu().numpy(),
            "arrival": arrival.cpu().numpy(),
            "radial": radial.cpu().numpy(),
            "release_speed": rel_speed.cpu().numpy(),
            "release_ang_deg": (rel_ang * 180 / np.pi).cpu().numpy(),
            "release_spin": rel_spin.cpu().numpy(),
            "settle_step": settle_step.cpu().numpy(),
            "release_step": release_step.cpu().numpy(),
            "broke": broke.cpu().numpy(),
            "frames": np.stack(frames) if frames else None,  # [T,B,442,3]
            "frame_steps": np.asarray(frame_steps, dtype=np.int64) if frames else None,
            **self._extra_result(),
        }
