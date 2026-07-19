"""バケツ(2.4m)・机(4.8m)・旗(3.9m) を3パネル同時アニメする比較動画 (CUDA GPU シム版)。

    .venv/bin/python scripts/video.py [N] [engine=torch|newton]

各パネル: GPU最適param(out/opt-*[-newton].json)で N投(既定300)を1バッチ計算し、
物理判定の命中率を大表示。各パネルには「引き視点(弾道全景)」に加え、右上に
「雑巾拡大ビュー(重心追従)」を重ね、布の姿勢・変形を細かく見せる。
1本のmp4で4セグメント = 等倍(いちばん中心の成功例 → いちばん外した失敗例) → 低速(同)。
engine=newton は Warp/Newton の XPBD 接触ソルバ(ライブラリ品質)を使う。
出力 out/gpu-belt-3targets[-newton].mp4  (viewer/scripts/rag-mech.ts のGPU移植+旗追加)
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from ragsim import mesh, render
from ragsim.render import Cam, dot3, draw_rag, font, hsl, line3, make_cam, path3, project
from ragsim.targets import BucketTarget, FlagTarget, ShelfTarget

MECH_TITLE = {
    "belt": "ベルト式直動(対向ベルト)",
    "roller": "対向ローラー式(二段)",
    "yaw": "遠心式 yaw回転(ハンマー投げ式)",
    "sling": "遠心式 pitch回転(オーバースロー)",
    "trebuchet": "投石機式(重力駆動 / 装填はモーター)",
}

N = 300
ENGINE = "torch"
KIND = "belt"
for _a in sys.argv[1:]:
    if _a in ("torch", "newton"):
        ENGINE = _a
    elif _a in MECH_TITLE:
        KIND = _a
    else:
        N = int(_a)

if ENGINE == "newton":
    from ragsim.sim_newton import NewtonBeltSim as SimEngine
else:
    from ragsim.sim import BeltSim as SimEngine
OPT_SUFFIX = "-newton" if ENGINE == "newton" else ""
KIND_SUFFIX = "" if KIND == "belt" else f"-{KIND}"
OUT_NAME = f"gpu-{KIND}-3targets{OPT_SUFFIX}.mp4"

W, H = 1920, 900
HEAD_H = 92
PANEL_H = H - HEAD_H - 12  # 下部の凡例は廃止(見れば分かる) → パネルを下いっぱいまで
PW = W // 3
BG = (19, 28, 43)  # #131c2b 単色背景(グラデーション禁止)
HOLD = 12
SLOW = 2
REP_HUE = 28
FPS = 30
BELT_NIP = 0.35

PIVOT = np.array([0.0, 0.9, 0.0])
DIR = np.array([0.0, 0.0, 1.0])
LATERAL = np.array([-1.0, 0.0, 0.0])


def a_(rgb: tuple[int, int, int], alpha: float) -> tuple[int, int, int]:
    """疑似アルファ(単色背景へブレンド)"""
    return tuple(int(round(BG[i] + alpha * (rgb[i] - BG[i]))) for i in range(3))


# ---- M3508 駆動判定 (rag-mech.ts driveSpec 移植) ----
ARM_LEN, TETHER, CW_ARM_SHORT = 0.5, 0.12, 0.18
NIP_OF = {"belt": 0.35, "roller": 0.18}
SPIN_OF = {"belt": 0.0, "roller": 0.6}
M3508 = {"direct": (9250, 0.0156), "geared": (469, 0.3), "imax": 20, "pmax": 200}


def drive_spec(pa: float) -> str:
    rag_m = 0.048
    rho_cd_a = 0.5 * 1.225 * 1.5 * 0.06
    head = tail = ""
    if KIND in ("yaw", "sling"):
        r_eff = ARM_LEN + TETHER
        inertia = (1 / 3) * 0.25 * ARM_LEN ** 2 + rag_m * r_eff ** 2
        torque = inertia * (pa * pa / (2 * np.pi)) + rho_cd_a * (pa * r_eff) ** 2 * r_eff
        rpm = pa * 60 / (2 * np.pi)
        power = torque * pa
    elif KIND == "trebuchet":
        omega_cock = np.radians(160.0) / 2.0
        torque = pa * 9.81 * CW_ARM_SHORT + 0.25 * 9.81 * 0.3
        rpm = omega_cock / (2 * np.pi) * 60
        power = torque * omega_cock
        head, tail = "装填", "(射出は重力)"
    else:
        pulley_r = 0.02
        nip = NIP_OF[KIND]
        force = (rag_m + 0.15) * (pa * pa / (2 * nip)) + rho_cd_a * pa * pa
        rpm = pa / (2 * np.pi * pulley_r) * 60
        torque = force * pulley_r
        power = force * pa

    def assess(max_rpm, kt):
        if rpm > max_rpm * 1.03:
            return None
        motors = max(1, int(np.ceil(torque / (kt * M3508["imax"]))), int(np.ceil(power / M3508["pmax"])))
        return motors, torque / motors / kt

    for gear, (max_rpm, kt) in (("直結", M3508["direct"]), ("P19", M3508["geared"])):
        got = assess(max_rpm, kt)
        if got and got[0] <= 4:
            return f"{head}M3508{gear}×{got[0]}(各{got[1]:.0f}A) {power:.0f}W{tail}"
    return f"{head}M3508 成立せず {power:.0f}W{tail}"


@dataclass
class PanelCfg:
    key: str
    label: str
    D: float
    max_time: float
    hit_word: str
    make_target: object


CFGS = [
    PanelCfg("bucket", "固定バケツ②③  2.4m", 2.4, 4.0, "入った",
             lambda: BucketTarget(cx=0.0, cz=2.4, rim_y=0.55, depth=0.255, radius=0.137)),
    PanelCfg("desk", "机①(下棚)  4.8m", 4.8, 4.0, "入った",
             lambda: ShelfTarget(cx=0.0, cz=4.8, width=0.65, depth=0.45, top_y=0.5, desk_top_y=0.76)),
    PanelCfg("flag", "旗(横棒 y=3.0m)  3.9m", 3.92, 5.0, "掛かった",
             lambda: FlagTarget(bar_y=3.0, bar_z=3.92, bar_x0=-0.3, bar_x1=0.3)),
]


@dataclass
class Panel:
    cfg: PanelCfg
    target: object
    a: float
    b: float
    spin: float
    hits: int
    hit_pct: float
    arrivals: np.ndarray  # [N,3]
    hit_flags: np.ndarray
    rep: dict  # phase→ {frames [T,442,3], trail [T,3], is_hit, radial, steps, release_step}
    cam: Cam
    rect: tuple
    spec: str
    zoom_rect: tuple = (0, 0, 0, 0)  # 拡大ビューの矩形(パネル右上)
    arm_tips: object = None  # [S,3] 腕先端の軌跡(sling/yaw/trebuchet)


def make_sim(target, a: float, b: float, opt: dict, max_time: float, n: int):
    seeds = np.arange(1000, 1000 + n)
    if KIND in ("belt", "roller"):
        return SimEngine(target, speed=np.full(n, a), elev_deg=np.full(n, b), seeds=seeds,
                         nip_len=NIP_OF[KIND], spin_frac=opt.get("spin", SPIN_OF[KIND]) if KIND == "belt" else SPIN_OF[KIND],
                         max_time=max_time, record=True, record_stride=3)
    from ragsim.sim_mech import SlingSim, TrebuchetSim, rotate_yaw_dir
    if KIND == "sling":
        return SlingSim(target, omega_rel=np.full(n, a), angle_deg=np.full(n, b), seeds=seeds,
                        axis="pitch", max_time=max_time, record=True, record_stride=3)
    if KIND == "yaw":
        d0 = rotate_yaw_dir(np.array([0.0, 0.0, 1.0]), opt.get("azimuthRad", 0.0))
        return SlingSim(target, omega_rel=np.full(n, a), angle_deg=np.full(n, b), seeds=seeds,
                        axis="yaw", dir_vec=np.tile(d0, (n, 1)), max_time=max_time, record=True, record_stride=3)
    return TrebuchetSim(target, cw_kg=np.full(n, a), release_deg=np.full(n, b), seeds=seeds,
                        max_time=max_time, record=True, record_stride=3)


def build_panel(cfg: PanelCfg, rect: tuple) -> Panel:
    opt = json.loads((ROOT / "out" / f"opt-{cfg.key}{KIND_SUFFIX}{OPT_SUFFIX}.json").read_text())["best"]["belt"]
    a, b, spin = opt["a"], opt["b"], opt.get("spin", 0.0)
    target = cfg.make_target()
    t0 = time.time()
    sim = make_sim(target, a, b, opt, cfg.max_time, N)
    r = sim.run()
    hit = r["hit"]
    radial = r["radial"]
    print(f"  {cfg.key}: v={a} 仰角={b}° spin={spin} → {hit.sum()}/{N} = {hit.mean()*100:.0f}% ({time.time()-t0:.0f}s)")

    hit_idx = np.flatnonzero(hit)
    best = int(hit_idx[np.argmin(radial[hit_idx])]) if len(hit_idx) else int(np.argmin(radial))
    worst = int(np.argmax(radial))

    frames = r["frames"]  # [T,B,442,3]
    fsteps = r["frame_steps"]
    rep = {}
    for phase, idx in ((0, best), (1, worst)):
        cut = int(np.searchsorted(fsteps, r["settle_step"][idx])) + 4
        fr = frames[: max(cut, 8), idx].astype(np.float32)  # [T,442,3]
        rep[phase] = {"frames": fr, "trail": fr.mean(axis=1), "is_hit": bool(hit[idx]), "radial": float(radial[idx]),
                      "steps": fsteps[: max(cut, 8)], "release_step": int(r["release_step"][idx])}
    arm_tips = r.get("arm_tips")
    del frames

    # カメラ: 両代表軌道 + 散布を収める (rag-mech.ts buildPanel 移植)
    max_y, max_prog, min_prog = 1.0, cfg.D + 0.7, -0.4
    min_lat, max_lat = -0.35, 0.35
    for ph in rep.values():
        prog = ph["trail"][:, 2] - PIVOT[2]
        max_prog = max(max_prog, float(prog.max()))
        min_prog = min(min_prog, float(prog.min()))
        max_y = max(max_y, float(ph["trail"][:, 1].max()))
    lat = (r["arrival"][:, 0] - PIVOT[0]) * LATERAL[0]
    min_lat = min(min_lat, float(lat.min()))
    max_lat = max(max_lat, float(lat.max()))
    max_y += 0.3
    center = PIVOT + DIR * ((min_prog + max_prog) / 2) + np.array([0, max_y / 2 - 0.1, 0])
    fov = 40
    span = max(max_prog - min_prog, max_y, max_lat - min_lat, 1)
    # 焦点距離は vh 基準なので、縦長パネル(vw<vh)では横方向のはみ出しを防ぐため vh/vw 倍遠ざける
    cam_dist = span / 2 / np.tan(np.radians(fov) / 2) * 1.15 * max(1.0, rect[3] / rect[2] * 0.98)
    eye_dir = -LATERAL + DIR * -0.28 + np.array([0, 0.42, 0])
    eye_dir /= np.linalg.norm(eye_dir)
    eye = center + eye_dir * cam_dist
    cam = make_cam(eye, center, fov, rect[0], rect[1], rect[2], rect[3])

    # 拡大ビューの矩形(パネル右上、正方形)
    zsize = int(min(rect[2], rect[3]) * 0.42)
    zoom_rect = (rect[0] + rect[2] - zsize - 10, rect[1] + 82, zsize, zsize)

    return Panel(cfg, target, a, b, spin, int(hit.sum()), float(hit.mean() * 100),
                 r["arrival"], hit, rep, cam, rect, drive_spec(a), zoom_rect, arm_tips)


def zoom_cam(P: Panel, nodes: np.ndarray) -> Cam:
    """雑巾の重心を追い、布全体が収まるクローズアップカメラ。引き視点と同じ視線方向。"""
    c = nodes.mean(axis=0)
    r = P.zoom_rect
    eye_dir = -LATERAL + DIR * -0.28 + np.array([0, 0.42, 0])
    eye_dir /= np.linalg.norm(eye_dir)
    fov = 42
    span = 0.55  # 布(0.3×0.2)+変形の余裕。固定倍率で姿勢の大きさ感を保つ
    dist = span / 2 / np.tan(np.radians(fov) / 2)
    eye = c + eye_dir * dist
    return make_cam(eye, c, fov, r[0], r[1], r[2], r[3])


# ---------------- 3D 描画 ----------------
def ground_pt(prog: float, lat: float) -> np.ndarray:
    return PIVOT + DIR * prog + LATERAL * lat + np.array([0, -PIVOT[1], 0])


def draw_ground(d: ImageDraw.ImageDraw, P: Panel) -> None:
    lat_half = 0.9
    max_prog = P.cfg.D + 1.2
    line3(d, P.cam, ground_pt(-0.4, 0), ground_pt(max_prog, 0), a_((130, 150, 175), 0.55), 2.5)
    for m in range(int(np.ceil(max_prog)) + 1):
        line3(d, P.cam, ground_pt(m, -lat_half), ground_pt(m, lat_half), a_((120, 145, 175), 0.18), 1.5)
        sx, sy, fr = project(P.cam, ground_pt(m, -lat_half - 0.12))
        if fr and m > 0:
            d.text((sx - 10, sy - 6), f"{m}m", font=font(18), fill=a_((175, 195, 220), 0.92))


def draw_bucket(d: ImageDraw.ImageDraw, P: Panel) -> None:
    b: BucketTarget = P.target
    def ring(y: float, r: float, color, lw: float) -> None:
        th = np.linspace(0, 2 * np.pi, 49)
        pts = np.stack([b.cx + r * np.cos(th), np.full_like(th, y), b.cz + r * np.sin(th)], axis=-1)
        path3(d, P.cam, pts, color, lw)
    line3(d, P.cam, (b.cx, 0, b.cz), (b.cx, b.floor_y, b.cz), a_((150, 160, 175), 0.55), 4)
    for k in range(4):
        th = k / 4 * 2 * np.pi + np.pi / 4
        x, z = b.cx + b.radius * np.cos(th), b.cz + b.radius * np.sin(th)
        line3(d, P.cam, (x, b.floor_y, z), (x, b.rim_y, z), a_((200, 175, 120), 0.55), 2)
    ring(b.floor_y, b.radius, a_((200, 150, 80), 0.6), 2)
    ring(b.rim_y, b.radius, a_((255, 206, 84), 0.98), 4)


def draw_desk(d: ImageDraw.ImageDraw, P: Panel) -> None:
    s: ShelfTarget = P.target
    hw, hd = s.width / 2, s.depth / 2
    front, back = s.cz - hd, s.cz + hd
    def quad(pts, stroke, lw, fill=None) -> None:
        sx, sy, fr = project(P.cam, np.asarray(pts, dtype=np.float64))
        if not fr.all():
            return
        poly = list(zip(sx.tolist(), sy.tolist()))
        if fill:
            d.polygon(poly, fill=fill)
        d.line(poly + [poly[0]], fill=stroke, width=max(1, int(lw)))
    wall = a_((150, 165, 185), 0.5)
    cx = s.cx
    quad([(cx - hw, s.top_y, front), (cx + hw, s.top_y, front), (cx + hw, s.desk_top_y, front), (cx - hw, s.desk_top_y, front)], wall, 2, a_((120, 135, 155), 0.35))
    quad([(cx - hw, s.desk_top_y, front), (cx + hw, s.desk_top_y, front), (cx + hw, s.desk_top_y, back), (cx - hw, s.desk_top_y, back)], wall, 2, a_((140, 155, 175), 0.28))
    quad([(cx - hw, 0, back), (cx + hw, 0, back), (cx + hw, s.top_y, back), (cx - hw, s.top_y, back)], wall, 2, a_((90, 105, 125), 0.30))
    quad([(cx - hw, 0, front), (cx - hw, 0, back), (cx - hw, s.top_y, back), (cx - hw, s.top_y, front)], wall, 1.5)
    quad([(cx + hw, 0, front), (cx + hw, 0, back), (cx + hw, s.top_y, back), (cx + hw, s.top_y, front)], wall, 1.5)
    quad([(cx - hw, 0, front), (cx + hw, 0, front), (cx + hw, s.top_y, front), (cx - hw, s.top_y, front)], a_((255, 206, 84), 0.98), 4)


def draw_flag(d: ImageDraw.ImageDraw, P: Panel) -> None:
    f: FlagTarget = P.target
    # 旗布(横棒から垂れる布) — 文脈用の薄い矩形
    sx, sy, fr = project(P.cam, np.array([
        [f.bar_x0 + 0.02, f.bar_y - 0.02, f.bar_z], [f.bar_x1 - 0.02, f.bar_y - 0.02, f.bar_z],
        [f.bar_x1 - 0.02, f.bar_y - 0.40, f.bar_z], [f.bar_x0 + 0.02, f.bar_y - 0.40, f.bar_z]]))
    if fr.all():
        d.polygon(list(zip(sx.tolist(), sy.tolist())), fill=a_((200, 80, 80), 0.30))
    line3(d, P.cam, (f.pole_x, 0, f.bar_z), (f.pole_x, f.bar_y + 0.08, f.bar_z), a_((150, 160, 175), 0.8), 5)
    line3(d, P.cam, (f.bar_x0, f.bar_y, f.bar_z), (f.bar_x1, f.bar_y, f.bar_z), a_((255, 206, 84), 0.98), 6)


def draw_scatter(d: ImageDraw.ImageDraw, P: Panel) -> None:
    sx, sy, fr = project(P.cam, P.arrivals.astype(np.float64))
    for k in range(len(sx)):
        if not fr[k]:
            continue
        c = a_((120, 235, 150), 0.8) if P.hit_flags[k] else a_((255, 105, 105), 0.6)
        d.ellipse([sx[k] - 3.2, sy[k] - 3.2, sx[k] + 3.2, sy[k] + 3.2], fill=c)


def draw_belt(d: ImageDraw.ImageDraw, P: Panel) -> None:
    th = np.radians(P.b)
    nip = NIP_OF[KIND]
    track = DIR * np.cos(th) + np.array([0, np.sin(th), 0])
    line3(d, P.cam, PIVOT, PIVOT + track * nip, a_((150, 160, 175), 0.6), 4)
    n = -DIR * np.sin(th) + np.array([0, np.cos(th), 0])
    dot3(d, P.cam, PIVOT + n * 0.035, 12, (200, 120, 60))
    dot3(d, P.cam, PIVOT - n * 0.035, 12, (200, 120, 60))
    dot3(d, P.cam, PIVOT, 6, (142, 162, 184))


def draw_mech(d: ImageDraw.ImageDraw, P: Panel, rep: dict, idx: int) -> None:
    """機構の描画: ベルト系=ニップ / 腕系=ピボット→先端(離脱後はグレー)"""
    if KIND in ("belt", "roller") or P.arm_tips is None:
        draw_belt(d, P)
        return
    st = int(rep["steps"][min(idx, len(rep["steps"]) - 1)])
    tips = P.arm_tips
    tip = tips[min(st, len(tips) - 1)]
    released = st >= rep["release_step"]
    col = a_((150, 160, 175), 0.35) if released else (216, 192, 137)
    line3(d, P.cam, PIVOT, tip, col, 5)
    if KIND == "trebuchet":
        ad = tip - PIVOT
        n = np.linalg.norm(ad)
        if n > 1e-9:
            cw = PIVOT - ad / n * CW_ARM_SHORT
            line3(d, P.cam, PIVOT, cw, col, 5)
            dot3(d, P.cam, cw, 6 if released else 10, a_((180, 120, 90), 0.4) if released else (200, 120, 60))
    dot3(d, P.cam, PIVOT, 6, (142, 162, 184))


def param_text(P: Panel) -> str:
    if KIND == "yaw":
        return f"ω={P.a}rad/s tilt={P.b}°"
    if KIND == "sling":
        return f"ω={P.a}rad/s β={P.b}°"
    if KIND == "trebuchet":
        return f"CW={P.a}kg 離脱={P.b}°"
    t = f"速度={P.a}m/s 仰角={P.b}°"
    if KIND == "roller":
        t += f" スピン差={SPIN_OF['roller']}"
    elif P.spin:
        t += f" スピン差={P.spin}"
    return t


def _draw_zoom_inset(img: Image.Image, d: ImageDraw.ImageDraw, P: Panel, nodes: np.ndarray, trail_col, rep: dict, idx: int) -> None:
    """パネル右上に雑巾のクローズアップ(姿勢・変形確認用)を重ねる。標的・床・機構も描く。"""
    zx, zy, zw, zh = (int(v) for v in P.zoom_rect)
    zcam = zoom_cam(P, nodes)
    zcam = Cam(zcam.eye, zcam.right, zcam.up, zcam.fwd, zcam.focal, 0, 0, zw, zh)
    zlayer = Image.new("RGB", (zw, zh), (12, 18, 28))
    zd = ImageDraw.Draw(zlayer)
    # ズームカメラで同じシーン(床グリッド・標的・ベルト)を描く — 近接時に文脈が見える
    Pz = Panel(P.cfg, P.target, P.a, P.b, P.spin, P.hits, P.hit_pct, P.arrivals, P.hit_flags,
               P.rep, zcam, (0, 0, zw, zh), P.spec, arm_tips=P.arm_tips)
    draw_ground(zd, Pz)
    if P.cfg.key == "bucket":
        draw_bucket(zd, Pz)
    elif P.cfg.key == "desk":
        draw_desk(zd, Pz)
    else:
        draw_flag(zd, Pz)
    draw_mech(zd, Pz, rep, idx)
    # 向きの目安に薄いミニ軸(重心を通る鉛直線)
    c = nodes.mean(axis=0)
    line3(zd, zcam, (c[0], c[1] - 0.35, c[2]), (c[0], c[1] + 0.35, c[2]), (40, 52, 70), 1)
    draw_rag(zd, zcam, nodes.astype(np.float64), REP_HUE)
    img.paste(zlayer, (zx, zy))
    # 枠 + ラベル
    d.rectangle([zx, zy, zx + zw - 1, zy + zh - 1], outline=trail_col, width=2)
    d.rectangle([zx, zy, zx + 108, zy + 20], fill=(12, 18, 28))
    d.text((zx + 6, zy + 3), "雑巾 拡大", font=font(15), fill=(200, 212, 228))


def draw_panel(img: Image.Image, d: ImageDraw.ImageDraw, P: Panel, phase: int, t: int) -> None:
    rep = P.rep[phase]
    frames = rep["frames"]
    idx = min(t, len(frames) - 1)
    nodes = frames[idx]
    trail_col = a_((120, 220, 140), 0.95) if phase == 0 else a_((255, 130, 60), 0.95)

    # パネル内クリップ: PIL に clip がないのでパネル別レイヤに描いて貼る
    px, py, pw, ph = P.rect
    layer = Image.new("RGB", (int(pw), int(ph)), BG)
    ld = ImageDraw.Draw(layer)
    cam = Cam(P.cam.eye, P.cam.right, P.cam.up, P.cam.fwd, P.cam.focal, 0, 0, pw, ph)
    P2 = Panel(P.cfg, P.target, P.a, P.b, P.spin, P.hits, P.hit_pct, P.arrivals, P.hit_flags, P.rep, cam, (0, 0, pw, ph), P.spec, arm_tips=P.arm_tips)

    draw_ground(ld, P2)
    draw_scatter(ld, P2)
    if P.cfg.key == "bucket":
        draw_bucket(ld, P2)
    elif P.cfg.key == "desk":
        draw_desk(ld, P2)
    else:
        draw_flag(ld, P2)
    # 床への投影(点線) + 本体トレイル
    tr = rep["trail"][: idx + 1]
    ground_tr = tr.copy()
    ground_tr[:, 1] = 0
    path3(ld, cam, ground_tr, a_((120, 140, 165), 0.5), 1.5, dash=2)
    path3(ld, cam, tr, trail_col, 3)
    # 高さcue: 重心→床の鉛直線 + 影
    c = tr[-1]
    line3(ld, cam, c, (c[0], 0, c[2]), a_((120, 140, 165), 0.45), 1.5)
    sx, sy, fr = project(cam, np.array([c[0], 0.0, c[2]]))
    if fr:
        ld.ellipse([sx - 9, sy - 3.2, sx + 9, sy + 3.2], fill=a_((30, 40, 55), 0.7))
    draw_mech(ld, P2, rep, idx)
    draw_rag(ld, cam, nodes.astype(np.float64), REP_HUE)
    img.paste(layer, (int(px), int(py)))

    # ---- 雑巾 拡大ビュー(重心追従クローズアップ) ----
    _draw_zoom_inset(img, d, P, nodes, trail_col, rep, idx)

    # ---- パネル見出し ----
    tx = px + 18
    d.text((tx, py + 8), P.cfg.label, font=font(24), fill=(233, 239, 248))
    d.text((tx, py + 38), param_text(P), font=font(19), fill=(158, 195, 255))
    ok = (126, 224, 138) if P.hit_pct >= 60 else (255, 206, 84) if P.hit_pct >= 20 else (255, 122, 106)
    pct = f"{P.hit_pct:.0f}%"
    d.text((px + pw - 18 - d.textlength(pct, font=font(52)), py + 4), pct, font=font(52), fill=ok)
    sub = f"{P.cfg.hit_word} {P.hits}/{N}"
    d.text((px + pw - 18 - d.textlength(sub, font=font(15)), py + 56), sub, font=font(15), fill=(174, 185, 200))
    is_hit = rep["is_hit"]
    res = f"この回: {P.cfg.hit_word} (中心 {rep['radial']:.2f}m)" if is_hit else f"この回: 外れ (中心 {rep['radial']:.2f}m)"
    d.text((tx, py + ph - 52), res, font=font(18), fill=(126, 224, 138) if is_hit else (255, 122, 106))
    d.text((tx, py + ph - 26), P.spec, font=font(16), fill=(203, 214, 228))


def main() -> None:
    print(f"GPU 3標的比較動画 N={N} engine={ENGINE} kind={KIND}")
    panels = [build_panel(cfg, (i * PW, HEAD_H, PW, PANEL_H)) for i, cfg in enumerate(CFGS)]

    max_f = {ph: max(len(P.rep[ph]["frames"]) for P in panels) for ph in (0, 1)}
    plan: list[tuple[int, int, bool]] = []
    for slow in (False, True):
        for ph in (0, 1):
            reps = SLOW if slow else 1
            for t in range(max_f[ph]):
                plan.extend([(ph, t, slow)] * reps)
            plan.extend([(ph, max_f[ph] - 1, slow)] * HOLD)
    total = len(plan)
    print(f"4セグメント(等倍成功→等倍失敗→低速成功→低速失敗) {total}フレーム → エンコード")

    out = ROOT / "out" / OUT_NAME
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(out)],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    t0 = time.time()
    for i, (phase, t, slow) in enumerate(plan):
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        # ヘッダ
        d.text((22, 8), f"{MECH_TITLE[KIND]} × 3標的", font=font(34), fill=(242, 246, 252))
        ptxt = "① いちばん中心の回(成功例)" if phase == 0 else "② いちばん外した回(失敗例)"
        d.text((22, 52), ptxt, font=font(24), fill=(126, 224, 138) if phase == 0 else (255, 138, 106))
        sp = f"▶ スロー再生 (1/{SLOW}速)" if slow else "▶ 等倍"
        d.text((W - 24 - d.textlength(sp, font=font(22)), 52), sp, font=font(22), fill=(255, 206, 84) if slow else (159, 176, 196))

        for P in panels:
            draw_panel(img, d, P, phase, t)
        for k in (1, 2):
            d.line([(k * PW, HEAD_H), (k * PW, HEAD_H + PANEL_H)], fill=a_((255, 255, 255), 0.14), width=1)

        ff.stdin.write(img.tobytes())
        if i % 200 == 0:
            print(f"  frame {i}/{total} ({time.time()-t0:.0f}s)")
    ff.stdin.close()
    ff.wait()
    print(f"✅ 出力: {out} ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
