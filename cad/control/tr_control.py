"""TR 制御ロジックのリファレンス実装.

STM32 へそのまま移植できる粒度で書いてある（浮動小数・依存なし）。
`sim/tr_sim.py` がこのモジュールを使うので、**シミュレーションが仕様の検証になる**。

含まれるもの
  MecanumKinematics : 車体速度 → 4輪の角速度（逆運動学）と、その逆
  TrapezoidProfile  : 目標位置 → 速度指令（加減速制限つき・残距離から減速を先読み）
  CurrentBudget     : 規定 3.2.5（合計30A）を守る優先度つき電流配分器
  ShotSolver        : 目標座標 → (ヨー, 仰角, 初速) の弾道解
  MissionFSM        : 装填〜射出の状態機械。**インターロックがここに集約されている**

設計方針
  * 戦術判断（どこを撃つか・いつ戻るか）は人間が持つ。ここにあるのは
    「命中率に効く所だけを自動化する」ための下位ロジック（戦略書 §4.5）。
  * インターロックは「機構が壊れる／規定違反になる」組合せを機械的に禁止する。
    人間の操作ミスでも反則にならないことを、ソフト側で担保する。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# --- 機体諸元（tr_params と一致させること） -------------------------------
WHEEL_R = 0.050          # m  メカナム φ100
LX = 0.300               # m  ホイールベース/2
LY = 0.300               # m  トレッド/2
MASS = 34.82             # kg
ACCEL_MAX = 1.5          # m/s²
CRUISE = 0.80            # m/s
CREEP = 0.10             # m/s 机への接近・フォーク挿入時
YAW_LIMIT = math.radians(30.0)
PITCH_MIN, PITCH_MAX = math.radians(20.0), math.radians(70.0)
MUZZLE_MIN, MUZZLE_MAX = 2.5, 15.0   # 下限を 7→2.5 に拡張。固定バケツ①(1.2m)は
                                     # 2.7m/s が要る。7m/s では最短射程 3.2m で全滅
                                     # （scripts/target_ev.py）。低速射出の成立は要実測
# 射出点（車体座標）。⚠ **CAD の tr_params.NIP_Z が唯一の情報源**。
#   900 は「ローラー中心オフセット 45.8 + モーターオフセット 85 + モーター半径 21
#   = 151.8mm を台座上面 842 に足すと 994」で、94mm 足りずに仰角ユニット側板が
#   台座横梁・台座プレート・旋回アームを貫く高さ。CAD 側は 1000 に直してあるのに
#   ここだけ 900 のまま残っていた＝**実機は 100mm 低い所から撃つ前提で
#   弾道を解いていた**（近距離ほど山なりに外す）。
NIP = (0.235, 0.0, 1.000)
G = 9.81

# --- 電流諸元 --------------------------------------------------------------
LIMIT_A = 30.0
I_QUIESCENT = 1.2
KT_M3508 = 0.30
KT_M3508_RAW = KT_M3508 / (3591 / 187)
KT_M2006 = 1.0 / 3.0


# ===========================================================================
class MecanumKinematics:
    """メカナム4輪の逆/順運動学。

    車輪配置は正方（LX = LY = 0.30）。ローラーは 45°、左前と右後が同じ向き。
    ω_i = (vx ∓ vy ± (LX+LY)·ωz) / r
    """

    def __init__(self, r: float = WHEEL_R, lx: float = LX, ly: float = LY):
        self.r, self.k = r, lx + ly

    def inverse(self, vx: float, vy: float, wz: float) -> tuple[float, float, float, float]:
        """車体速度[m/s, rad/s] → (左前, 右前, 左後, 右後) の角速度[rad/s]。"""
        k = self.k
        return (
            (vx - vy - k * wz) / self.r,   # fl
            (vx + vy + k * wz) / self.r,   # fr
            (vx + vy - k * wz) / self.r,   # rl
            (vx - vy + k * wz) / self.r,   # rr
        )

    def forward(self, w_fl: float, w_fr: float, w_rl: float, w_rr: float):
        """4輪角速度 → 車体速度（オドメトリ。LiDARで補正する前提）。"""
        r = self.r / 4.0
        vx = r * (w_fl + w_fr + w_rl + w_rr)
        vy = r * (-w_fl + w_fr + w_rl - w_rr)
        wz = r * (-w_fl + w_fr - w_rl + w_rr) / self.k
        return vx, vy, wz


# ===========================================================================
# 計測輪（従動のオドメトリ輪）3 輪
# ===========================================================================
# 配置は cad/src/tr_assembly.py `_sensors()` の呼び出しと同じにすること。
#   (x[mm], y[mm], 転がり方向[deg])  角度は +X を 0 とする車体座標
ODO_WHEELS = ((210.0, 0.0, 90.0),        # ← この 1 輪だけ 90° 傾けてある
              (-180.0, 345.0, 0.0),
              (-180.0, -345.0, 0.0))
# ⚠ **転がり方向 0° の 2 輪は、x が式に現れない。**
#   v_i = vx − ω·y_i なので、X をどこへ動かしても解は変わらない
#   （2026-08-07 に −350 → −180 へ移したのは、オムニホイールの幅が増えて
#     エンコーダがメカナムに当たったため。制御側は Y だけ追随すればよい）。
#   効くのは 2 輪の Y 間隔（690mm）と、90° 輪の x（210mm）。


def _det3(m) -> float:
    (a, b, c), (d, e, f), (g, h, i) = m
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def _solve3(m, rhs):
    """3 元 1 次連立をクラメルで解く（マイコンに載せるので numpy は使わない）。"""
    det = _det3(m)
    out = []
    for col in range(3):
        mm = [list(row) for row in m]
        for r in range(3):
            mm[r][col] = rhs[r]
        out.append(_det3(mm) / det)
    return tuple(out)


class OdometryWheels:
    """従動の計測輪 3 輪から車体速度 (vx, vy, ωz) を解く。

    1 輪が測れるのは「その輪の**転がり方向**の床速度」ただ 1 つ:

        v_i = u_i · ( v + ω × r_i )
            = ux_i·vx + uy_i·vy + ( −ux_i·y_i + uy_i·x_i )·ω

    未知数は 3 つなので 3 輪あれば解ける — **係数行列の階数が 3 になる
    向きに置いたときだけ**。

    ⚠ **3 輪を同じ向きに置くと横速度は原理的に出ない。**（実際そうなっていた）
      u_i が全部 (1,0) なら 2 列目が全部 0 で、階数は 2 にしかならない。
      vy は「精度が悪い」のではなく、**どんなに測っても式に現れない**。
      メカナムは横行が主役なので、落ちるのはいちばん欲しい成分になる。
      → 1 輪を 90° 傾けて 2 列目を立てる。ここで階数を実際に確かめるので、
        配置を変えて向きが揃ったら、走らせる前にこの例外で気づける。
    """

    def __init__(self, wheels=ODO_WHEELS):
        self.a = []
        for x, y, deg in wheels:
            ux, uy = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            # ω の係数だけ mm → m に直す（v は m/s、ω は rad/s）
            self.a.append((ux, uy, (-ux * y + uy * x) / 1000.0))
        self.det = _det3(self.a)
        if abs(self.det) < 1e-9:
            raise ValueError("計測輪の向きが揃っている: "
                             "この配置では車体速度が解けない（階数 2）")

    def forward(self, v0: float, v1: float, v2: float):
        """3 輪の周速度 [m/s] → (vx[m/s], vy[m/s], ωz[rad/s])。"""
        return _solve3(self.a, (v0, v1, v2))


# ===========================================================================
class TrapezoidProfile:
    """残距離から減速を先読みする台形速度プロファイル。

    35kg の機体に目標位置をステップで与えると止まれず、机に突っ込んで反則になる
    （実際にシミュレーションで 80mm 突っ込んだ）。指令は必ずここを通す。
    """

    def __init__(self, cruise: float = CRUISE, accel: float = ACCEL_MAX,
                 deadband: float = 0.002):
        self.cruise, self.accel = cruise, accel
        # 残距離が deadband を切ったら目標へスナップする。
        # v = sqrt(2·a·d) は d→0 で v→0 なので、入れないと最後の数mmが永遠に詰まらず、
        # 「まだ動いている」判定が残ってインターロックが誤作動する。
        self.deadband = deadband
        self.cmd = 0.0

    def reset(self, pos: float) -> None:
        self.cmd = pos

    def step(self, target: float, dt: float, cruise: float | None = None) -> float:
        v_max = self.cruise if cruise is None else cruise
        d = target - self.cmd
        if abs(d) <= self.deadband:
            self.cmd = target
            return self.cmd
        v_lim = min(v_max, math.sqrt(2.0 * self.accel * abs(d)))
        step = max(-v_lim * dt, min(v_lim * dt, d))
        self.cmd += step
        return self.cmd


# ===========================================================================
@dataclass
class CurrentBudget:
    """規定 3.2.5（駆動系の遮断素子合計 30A）を守る配分器。

    優先度: 射出ローラー定速維持 > 砲塔 > 駆動 > 装填。
    「射出の再現性」が命中率に直結するので、走行を犠牲にしても射出を守る。
    """

    limit: float = LIMIT_A
    quiescent: float = I_QUIESCENT
    order: tuple[str, ...] = ("shooter", "turret", "drive", "loader")

    def allocate(self, demand: dict[str, float]) -> dict[str, float]:
        remain = self.limit - self.quiescent
        out: dict[str, float] = {}
        for key in self.order:
            want = max(0.0, demand.get(key, 0.0))
            give = min(want, max(remain, 0.0))
            out[key] = give
            remain -= give
        return out

    @staticmethod
    def drive_demand(accel: float, mu_roll: float = 0.03) -> float:
        f = MASS * abs(accel) + mu_roll * MASS * G
        return 4.0 * (f * WHEEL_R / 4.0) / KT_M3508

    @staticmethod
    def spinup_demand(cap_per_axis: float = 10.0) -> float:
        return 2.0 * cap_per_axis


# ===========================================================================
class ShotSolver:
    """目標座標 → (ヨー, 仰角, 初速)。

    真空弾道で初期解を出し、実測の弾道テーブル（`gpusim` で較正）で補正する
    前提。テーブルが無い段階でも「届くかどうか」の判定はここでできる。
    """

    # --- gpusim（布の空力を入れたGPUバッチシム）による較正 ------------------
    # ⚠ 座標系の食い違いを解消した結果。gpusim は
    #    「射出点が原点（高さ0）、横棒は高さ y=3.0m・距離 z=D」という Y-up 系で、
    #    上昇量は 3.0m（2.1m ではない）。この読み違いで以前は較正が破綻していた。
    #
    #   D=3.92m / 上昇 3.00m のとき
    #     仰角48° → 真空解 11.15 m/s に対し gpusim 8.00 → **0.717**
    #     仰角67° → 真空解  8.90 m/s に対し gpusim 8.25 → **0.927**
    #
    # 布は平板なので**揚力で真空弾道より飛ぶ**。低仰角ほど滞空中の水平速度が高く
    # 揚力が効くので補正が強い。仰角の一次関数として内挿する（2点しかないので暫定）。
    CALIB_PITCH = {48.0: 0.717, 67.0: 0.927}

    CALIB_MIN_DIST = 2.5      # 較正が効く最短距離。これより近い射は滞空が短く揚力が乗らない

    def _pitch_calib(self, pitch: float, dist: float = 99.0) -> float:
        """仰角に対する揚力補正（真空解に掛ける）。

        gpusim の較正は D=3.92m の高い弧に対するもの。近距離・低伸の射では
        滞空時間が短く揚力が乗らないので、**補正を掛けない**（安全側）。
        近距離の実測は 7月の試射で別途取ること。
        """
        if dist < self.CALIB_MIN_DIST:
            return 1.0
        d = math.degrees(pitch)
        ks = sorted(self.CALIB_PITCH)
        if d <= ks[0]:
            return self.CALIB_PITCH[ks[0]]
        if d >= ks[-1]:
            return self.CALIB_PITCH[ks[-1]]
        (a, b) = ks[0], ks[1]
        t = (d - a) / (b - a)
        return self.CALIB_PITCH[a] * (1 - t) + self.CALIB_PITCH[b] * t

    def __init__(self, nip=NIP):
        self.nip = nip
        self.table: dict[tuple[int, int, int], float] = {}   # 実測テーブル（8月に埋める）

    def solve(self, target_xyz, prefer_pitch: float = math.radians(52.0)):
        tx, ty, tz = target_xyz
        dx, dy = tx - self.nip[0], ty - self.nip[1]
        dist = math.hypot(dx, dy)
        rise = tz - self.nip[2]
        yaw = math.atan2(dy, dx)
        if abs(yaw) > YAW_LIMIT:
            return None, None, None, "ヨー範囲外（車体を振ること）"
        for pitch in (prefer_pitch, *[math.radians(a) for a in range(60, 19, -2)]):
            c, t = math.cos(pitch), math.tan(pitch)
            denom = 2.0 * c * c * (dist * t - rise)
            if denom <= 0:
                continue
            v2 = G * dist * dist / denom
            v = math.sqrt(v2)
            v *= self._pitch_calib(pitch, dist)    # 布の揚力ぶんの補正（近距離は掛けない）
            v *= self._calib(dist, rise, pitch)    # 実測テーブルがあれば更に補正
            if MUZZLE_MIN <= v <= MUZZLE_MAX and PITCH_MIN <= pitch <= PITCH_MAX:
                return yaw, pitch, v, ""
        return None, None, None, "射程外（速度域 7〜15 m/s に解なし）"

    def _calib(self, dist: float, rise: float, pitch: float) -> float:
        """最も近い較正点の補正係数を返す（無ければ 1.0 = 真空解のまま）。"""
        if not self.table:
            return 1.0
        key = min(self.table,
                  key=lambda k: (abs(k[0] - dist * 100) + abs(k[1] - rise * 100)
                                 + abs(k[2] - math.degrees(pitch)) * 2.0))
        d = abs(key[0] - dist * 100) + abs(key[1] - rise * 100)
        return self.table[key] if d < 120 else 1.0


# ===========================================================================
class MuzzleFeedForward:
    """厚み計測から初速のずれを予測し、ローラー回転数を先回りで補正する。

    誤差バジェット（scripts/accuracy_budget.py）によると、命中率は初速の再現性で
    ほぼ決まる。雑巾の個体差（1σ 1.5%）は選別できないが、**厚みを測れば予測できる**。
    シンギュレータからニップまで 0.33 秒あるので、その間に補正が入る。

    係数は 7月の試射で実測して置き換えること（本クラスの2つの定数が
    命中率の予測精度を直接支配する）。
    """

    NIP_GAP_MM = 1.5
    RAG_T_NOM_MM = 3.0
    SLIP_COEF = 0.45          # 圧縮率の変化 → 初速の変化 の比【要実測】
    MAX_CORRECTION = 0.06     # ±6% を超える補正は異常値として無視する

    def __init__(self, roller_radius_m: float = 0.045):
        self.r = roller_radius_m
        self.sys_err = 0.0       # 系統誤差の推定値（EWMA）

    # 射出でローラーの運動エネルギーが雑巾へ移る。その回転数降下から初速を逆算できる。
    # 既存の C620 エンコーダ（1rpm 分解能）だけで **初速を 0.8% 精度で実測** できる。
    J_TOTAL = 2 * 6.9e-4      # kg·m² 上下2軸ぶんの慣性
    RAG_MASS = 0.048          # kg

    def measure_from_rpm_drop(self, omega_before: float, omega_after: float) -> float:
        """射出前後の角速度[rad/s]から実効初速[m/s]を推定する。

        ½·J·(ω1²−ω2²) = ½·m·v²  →  v = sqrt(J(ω1²−ω2²)/m)
        これを移動平均して、ニップの摩耗・発熱によるドリフトを追従補正する。
        """
        de = self.J_TOTAL * (omega_before ** 2 - omega_after ** 2)
        return math.sqrt(max(de, 0.0) / self.RAG_MASS)

    # --- 系統誤差の自動較正（EWMA）---------------------------------------
    # 射出ごとの実効初速を回転数降下から測り、系統ずれを推定して次弾に反映する。
    # α はシミュレーション（scripts/calibration_sim.py）で最適化した値。
    #   α=0.10 → 命中 73%（較正なし 67%）
    #   α=0.50 → 命中 65%（測定ノイズ 0.8% を拾って**較正なしより悪化**）
    CAL_ALPHA = 0.10

    def update_calibration(self, v_measured: float, v_commanded: float) -> float:
        """実測初速と指令初速から系統誤差を更新し、現在の推定値を返す。"""
        err = v_measured / v_commanded - 1.0
        if abs(err) > 0.10:          # 明らかな外れ値（測定失敗）は捨てる
            return self.sys_err
        self.sys_err += self.CAL_ALPHA * (err - self.sys_err)
        return self.sys_err

    def omega_for(self, v_target: float, thickness_mm: float) -> tuple[float, str]:
        """目標初速と実測厚みから、ローラーの目標角速度[rad/s]を返す。"""
        t = thickness_mm
        # 2枚重送（≈6mm）は必ず弾く。重ね投げは違反（FAQ 4.1 Q14）なので、
        # ここは「初速補正」ではなく **反則防止装置** でもある。
        if not (self.RAG_T_NOM_MM * 0.5 < t < self.RAG_T_NOM_MM * 1.6):
            return 0.0, f"厚み異常 {t:.1f}mm（重送/空送り）→ 逆転パージして再ピック"
        c_nom = (self.RAG_T_NOM_MM - self.NIP_GAP_MM) / self.RAG_T_NOM_MM
        c = (t - self.NIP_GAP_MM) / t
        rel = self.SLIP_COEF * (c - c_nom)          # 予測される初速のずれ（正=速くなる）
        rel = max(-self.MAX_CORRECTION, min(self.MAX_CORRECTION, rel))
        # 1枚ごとの厚み補正 と 系統誤差の較正 を両方打ち消す
        v_cmd = v_target / (1.0 + rel + self.sys_err)
        return v_cmd / self.r, ""


# ===========================================================================
@dataclass
class Interlocks:
    """機構破損・規定違反につながる組合せを機械的に禁止する。"""

    grabber_extended: bool = False
    moving: bool = False
    rollers_spinning: bool = False
    near_desk: bool = False

    def can_drive(self) -> tuple[bool, str]:
        if self.grabber_extended:
            return False, "グラバー全開中は走行禁止（後方転倒余裕 5.2m/s²・机接触）"
        return True, ""

    def can_extend_grabber(self) -> tuple[bool, str]:
        if self.moving:
            return False, "走行中はフォークを出さない（机を突く）"
        return True, ""

    def can_spinup(self) -> tuple[bool, str]:
        if self.moving:
            return False, "走行中のスピンアップは合計36A で規定違反（停止中に行う）"
        return True, ""

    def max_speed(self) -> float:
        return CREEP if self.near_desk else CRUISE

    def clamp_yaw(self, yaw: float, own_side_only: bool = True) -> float:
        y = max(-YAW_LIMIT, min(YAW_LIMIT, yaw))
        return y


# ===========================================================================
@dataclass
class MissionFSM:
    """装填〜射出の状態機械。遷移条件と、その状態で許される動作を持つ。

    STATES
      IDLE        待機（砲塔照準のみ）
      APPROACH    補充スポットへ後退（最後の200mmはクリープ）
      INSERT      フォーク挿入（機体停止）
      CLAMP       上押さえ下降
      RETRACT     引込（同時にローラースピンアップ＝電流が空いている唯一の窓）
      DUMP        最終引込でカムが櫛歯を傾け、山をホッパーへ落とす
      TRANSIT     射撃位置へ前進（ローラーは定速維持）
      ENGAGE      射出（1枚が機構を離れてから次：FAQ 4.1 Q14/Q15）
      RETRY       重送・挿入失敗からの復帰
    """

    state: str = "IDLE"
    t_state: float = 0.0
    interlocks: Interlocks = field(default_factory=Interlocks)
    budget: CurrentBudget = field(default_factory=CurrentBudget)
    shots: int = 0
    magazine: int = 0

    # 状態ごとに「許可される動作」を明示する（ここが安全仕様そのもの）
    ALLOW = {
        "IDLE": ("aim",),
        "APPROACH": ("drive", "aim"),
        "INSERT": ("grabber", "aim"),
        "CLAMP": ("press", "aim"),
        "RETRACT": ("grabber", "spinup", "aim"),
        "DUMP": ("grabber", "spinup", "aim"),
        "TRANSIT": ("drive", "aim", "feed"),
        "ENGAGE": ("aim", "feed", "shoot"),
        "RETRY": ("grabber", "aim"),
    }

    def allows(self, action: str) -> bool:
        return action in self.ALLOW[self.state]

    def step(self, dt: float, sensors: dict) -> str:
        """センサー入力で状態を進める。戻り値は遷移後の状態。"""
        self.t_state += dt
        s = self.state
        if s == "IDLE" and sensors.get("start"):
            self._go("APPROACH")
        elif s == "APPROACH" and sensors.get("desk_reached"):
            self._go("INSERT")
        elif s == "INSERT":
            if sensors.get("fork_jam"):
                self._go("RETRY")
            elif sensors.get("fork_extended"):
                self._go("CLAMP")
        elif s == "CLAMP" and sensors.get("press_down"):
            self._go("RETRACT")
        elif s == "RETRACT" and sensors.get("slide_near_home"):
            self._go("DUMP")
        elif s == "DUMP" and sensors.get("slide_home"):
            self.magazine = sensors.get("stack_count", 10)
            self._go("TRANSIT")
        elif s == "TRANSIT" and sensors.get("firing_position"):
            self._go("ENGAGE")
        elif s == "ENGAGE":
            if self.magazine <= 0:
                self._go("APPROACH")
            elif sensors.get("double_feed"):
                self._go("RETRY")
        elif s == "RETRY" and sensors.get("cleared"):
            self._go("INSERT" if self.magazine == 0 else "ENGAGE")

        # インターロックの更新
        self.interlocks.grabber_extended = sensors.get("slide_pos", 0.0) > 0.02
        self.interlocks.moving = abs(sensors.get("speed", 0.0)) > 0.02
        self.interlocks.near_desk = s in ("APPROACH", "INSERT", "CLAMP", "RETRY")
        return self.state

    def _go(self, nxt: str) -> None:
        self.state, self.t_state = nxt, 0.0

    def shoot(self) -> bool:
        """1枚撃つ。FAQ 4.1 Q14/Q15 に従い、機構を離れてからでないと次は撃てない。"""
        if not self.allows("shoot") or self.magazine <= 0:
            return False
        self.magazine -= 1
        self.shots += 1
        return True

    def current_demand(self, spinup: bool, accel: float) -> dict[str, float]:
        return {
            "shooter": CurrentBudget.spinup_demand() if spinup else 6.3,
            "turret": 2.3,
            "drive": CurrentBudget.drive_demand(accel) if self.allows("drive") else 0.0,
            "loader": 5.0 if self.allows("grabber") else 2.0,
        }


__all__ = ["MecanumKinematics", "TrapezoidProfile", "CurrentBudget", "ShotSolver",
           "MuzzleFeedForward", "Interlocks", "MissionFSM"]
