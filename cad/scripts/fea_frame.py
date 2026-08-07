"""TR 骨格の静解析（3次元ラーメンの梁要素FEM）.

アルミフレーム（HFS5-2020）で組んだ骨格を 3D Euler-Bernoulli 梁要素で解き、
各荷重ケースでのたわみ・部材応力・最も厳しい部材を出す。

    python scripts/fea_frame.py            # 全荷重ケースを解く
    python scripts/fea_frame.py --md       # Markdown 表で出力

モデル化の前提（意図的な単純化）
  * 節点は剛結（アルミフレームのブラケット結合は完全剛ではないので安全側ではない）。
    → 実機ではブラケットの回転剛性が支配的になり得るので、結果は「部材そのものの
      たわみ」の下限と読むこと。結合部の緩みは別途トルク管理で担保する。
  * 断面定数は MISUMI カタログ値: A=183mm², Ix=Iy=0.742e4 mm⁴。
    ねじり定数 J は溝付き中空断面のため実測困難 → 1.2e4 mm⁴ と仮定（保守側）。
  * 集中質量は最寄り節点に付け替える。板金・カバー類は等価質量として節点に配分。
  * 支持は4輪接地点の3方向拘束（回転自由＝ピン支持）。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_params as P  # noqa: E402

E = 70000.0          # MPa (N/mm²) A6005C-T5
G = 26000.0          # MPa
A = P.EXT_AREA       # mm²
I = P.EXT_I          # mm⁴（両軸）
J = 1.2e4            # mm⁴（仮定）
RHO = P.EXT_MASS_PER_M / 1000.0   # kg/mm
SIGMA_ALLOW = 90.0   # MPa 許容応力（A6005C-T5 の耐力 ~200MPa に安全率2.2）
G_ACC = 9810.0       # mm/s²

# 射出反力。雑巾を静止から v まで加速する間の反作用。
# 加速に使える時間は「雑巾がニップを通る時間」= L/v なので、
#     F = m·v/(L/v) = m·v²/L
# 時間は仮定値ではなく幾何から決まる（以前は 20ms という根拠のない値を使っていた）。
MUZZLE_V_MAX = 6.5                       # m/s 常用初速の上限（scripts/target_ev.py）
SHOT_N = P.RAG_MASS * MUZZLE_V_MAX ** 2 / (P.RAG_X / 1000.0)

Z_BASE = P.BASE_Z0 + P.EXT_W / 2       # ベース桁の中心高さ
Z_TOP = P.PEDESTAL_TOP_Z - P.EXT_W / 2  # 側面上桁の中心高さ
YS = P.SIDE_FRAME_Y
YO = P.FRAME_RAIL_Y_OUT
YI = P.FRAME_RAIL_Y_IN
XR = P.RAIL_X0


def build_model():
    """(節点座標, 部材リスト) を返す。CAD の部材配置をそのまま写す。"""
    nodes: list[tuple[float, float, float]] = []
    index: dict[tuple[float, float, float], int] = {}

    def nid(x, y, z):
        key = (round(x, 1), round(y, 1), round(z, 1))
        if key not in index:
            index[key] = len(nodes)
            nodes.append(key)
        return index[key]

    members: list[tuple[int, int, str]] = []

    def beam(a, b, name):
        members.append((nid(*a), nid(*b), name))

    # --- ベースフレーム ---
    for sy in (1, -1):
        beam((-P.BASE_X / 2, sy * YO, Z_BASE), (P.BASE_X / 2, sy * YO, Z_BASE), "ベース主桁")
        beam((-P.BASE_X / 2, sy * YI, Z_BASE), (P.BASE_X / 2, sy * YI, Z_BASE), "ベース内桁")
    for sx in (1, -1):
        beam((sx * 410, -YO, Z_BASE), (sx * 410, YO, Z_BASE), "ベース横桁")
        beam((sx * P.WHEELBASE_X / 2, -YI, Z_BASE), (sx * P.WHEELBASE_X / 2, YI, Z_BASE), "車軸横桁")
    # --- 側面トラス（後柱はマスト主柱を兼ねる） ---
    for sy in (1, -1):
        beam((XR, sy * YS, Z_BASE), (XR, sy * YS, Z_TOP), "側面後柱(下)")
        beam((XR, sy * YS, Z_TOP), (XR, sy * YS, P.MAST_TOP_Z), "マスト主柱(上)")
        beam((380, sy * YS, Z_BASE), (380, sy * YS, Z_TOP), "側面前柱")
        beam((-430, sy * YS, Z_TOP), (380, sy * YS, Z_TOP), "側面上桁")
        beam((XR, sy * YS, Z_BASE), (100, sy * YS, Z_TOP), "側面斜材")
    # --- 砲塔台座横梁 + 旋回リングプレート（等価梁） ---
    for x in (P.TURRET_X - 110, P.TURRET_X + 110):
        beam((x, -YS, Z_TOP), (x, YS, Z_TOP), "砲塔台座横梁")
    for y in (-110.0, 0.0, 110.0):
        beam((P.TURRET_X - 110, y, Z_TOP), (P.TURRET_X + 110, y, Z_TOP), "旋回リング板(等価梁)")
    # --- マスト頭部 ---
    beam((XR, -YS, P.MAST_BEAM_Z), (XR, YS, P.MAST_BEAM_Z), "マスト上部横梁")
    # バケツ受け板(t2 340角)の面内剛性を等価梁2本で表現（片持ちビームのねじれ止め）
    for x in (-70.0, 90.0):
        beam((x, -P.MAST_CANTILEVER_Y, P.MAST_BEAM_Z), (x, P.MAST_CANTILEVER_Y, P.MAST_BEAM_Z),
             "バケツ受け板(等価梁)")
    for sy in (1, -1):
        beam((XR, sy * P.MAST_CANTILEVER_Y, P.MAST_BEAM_Z),
             (135, sy * P.MAST_CANTILEVER_Y, P.MAST_BEAM_Z), "バケツ片持ちビーム")
        pass
    # --- 部材同士の交差点にも節点を立てる（端点でない交差は自動では繋がらない） ---
    raw = list(members)
    for ai in range(len(raw)):
        for bi in range(ai + 1, len(raw)):
            i1, j1, _ = raw[ai]
            i2, j2, _ = raw[bi]
            if len({i1, j1, i2, j2}) < 4:
                continue
            p1 = np.array(nodes[i1], dtype=float)
            d1 = np.array(nodes[j1], dtype=float) - p1
            p2 = np.array(nodes[i2], dtype=float)
            d2 = np.array(nodes[j2], dtype=float) - p2
            A_ = np.array([[d1 @ d1, -(d1 @ d2)], [d1 @ d2, -(d2 @ d2)]])
            if abs(np.linalg.det(A_)) < 1e-9:
                continue
            t, u_ = np.linalg.solve(A_, np.array([(p2 - p1) @ d1, (p2 - p1) @ d2]))
            if not (1e-3 < t < 1 - 1e-3 and 1e-3 < u_ < 1 - 1e-3):
                continue
            q1, q2 = p1 + t * d1, p2 + u_ * d2
            if np.linalg.norm(q1 - q2) < 1.0:
                nid(*((q1 + q2) / 2))

    pts = np.array(nodes, dtype=float)
    # --- 交点で部材を分割して確実に結合させる ---
    split: list[tuple[int, int, str]] = []
    for i, j, name in members:
        a, b = pts[i], pts[j]
        v = b - a
        Lv = np.linalg.norm(v)
        on = []
        for k, q in enumerate(pts):
            if k in (i, j):
                continue
            t = float(np.dot(q - a, v) / (Lv * Lv))
            if 1e-6 < t < 1 - 1e-6 and np.linalg.norm(a + t * v - q) < 1.0:
                on.append((t, k))
        on.sort()
        chain = [i] + [k for _, k in on] + [j]
        for u_, w_ in zip(chain[:-1], chain[1:]):
            split.append((u_, w_, name))
    return pts, split


# 集中質量 [kg] とその作用点（CAD の重心位置）
POINT_MASSES = [
    ("砲塔+射出ユニット", 4.94, (P.TURRET_X, 0.0, 900.0)),
    ("移動バケツ+受け", 1.50, (-70.0, 0.0, 1250.0)),
    ("ホッパー+雑巾14枚", 1.90, (-225.0, 0.0, 660.0)),
    ("斜路+シンギュレータ", 0.90, (90.0, 0.0, 790.0)),
    ("電装+バッテリー", 3.50, (-120.0, 0.0, 150.0)),
    ("椅子+マスコット", 3.10, (250.0, 160.0, 350.0)),
    ("駆動系(モーター4)", 1.80, (0.0, 0.0, 60.0)),
    ("カバー・配線・ねじ", 2.20, (0.0, 0.0, 400.0)),
]
GRABBER_MASS = 2.80


def beam_stiffness(L, e1):
    """3次元 12x12 梁要素の局所剛性行列と変換行列。"""
    k = np.zeros((12, 12))
    EA_L = E * A / L
    GJ_L = G * J / L
    a = 12 * E * I / L ** 3
    b = 6 * E * I / L ** 2
    c = 4 * E * I / L
    d = 2 * E * I / L
    k[0, 0] = k[6, 6] = EA_L
    k[0, 6] = k[6, 0] = -EA_L
    k[3, 3] = k[9, 9] = GJ_L
    k[3, 9] = k[9, 3] = -GJ_L
    for (u, r, sgn) in ((1, 5, 1.0), (2, 4, -1.0)):
        U, R, Un, Rn = u, r, u + 6, r + 6
        k[U, U] = k[Un, Un] = a
        k[U, Un] = k[Un, U] = -a
        k[R, R] = k[Rn, Rn] = c
        k[R, Rn] = k[Rn, R] = d
        k[U, R] = k[R, U] = sgn * b
        k[U, Rn] = k[Rn, U] = sgn * b
        k[Un, R] = k[R, Un] = -sgn * b
        k[Un, Rn] = k[Rn, Un] = -sgn * b
    # 方向余弦行列
    x = e1 / np.linalg.norm(e1)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x, up)) > 0.99:
        up = np.array([1.0, 0.0, 0.0])
    z = np.cross(x, up)
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    R3 = np.vstack([x, y, z])
    T = np.zeros((12, 12))
    for i in range(4):
        T[3 * i:3 * i + 3, 3 * i:3 * i + 3] = R3
    return T.T @ k @ T, T


def solve(nodes, members, extra_forces, accel=(0.0, 0.0, 0.0), grabber_x=-145.0):
    n = len(nodes)
    K = np.zeros((6 * n, 6 * n))
    Fv = np.zeros(6 * n)
    elems = []
    for i, j, name in members:
        vec = nodes[j] - nodes[i]
        L = float(np.linalg.norm(vec))
        ke, T = beam_stiffness(L, vec)
        dofs = np.r_[6 * i:6 * i + 6, 6 * j:6 * j + 6]
        K[np.ix_(dofs, dofs)] += ke
        elems.append((i, j, name, L, ke, T, dofs))
        # 部材自重と慣性力を両端へ等分（w[kg] × a[m/s²] = N）
        w = RHO * L
        for nd in (i, j):
            Fv[6 * nd + 0] += -w * accel[0] / 2
            Fv[6 * nd + 1] += -w * accel[1] / 2
            Fv[6 * nd + 2] += -w * 9.81 / 2

    def nearest(pt):
        return int(np.argmin(np.linalg.norm(nodes - np.array(pt), axis=1)))

    for label, m, pos in POINT_MASSES + [("グラバー", GRABBER_MASS, (grabber_x, 0.0, 780.0))]:
        nd = nearest(pos)
        Fv[6 * nd + 2] += -m * 9.81
        Fv[6 * nd + 0] += -m * accel[0]
        Fv[6 * nd + 1] += -m * accel[1]
    for pos, F in extra_forces:
        nd = nearest(pos)
        Fv[6 * nd:6 * nd + 3] += np.array(F)

    # 支持: 4輪接地点に最も近い節点を並進拘束
    fixed = []
    for sx in (1, -1):
        for sy in (1, -1):
            nd = nearest((sx * P.WHEELBASE_X / 2, sy * YI, Z_BASE))
            fixed.extend([6 * nd, 6 * nd + 1, 6 * nd + 2])
    # どの部材にも属さない節点は解かない（部材を抜いて試すときに孤立節点が出る）。
    live = {nd for i, j, _ in members for nd in (i, j)}
    dead = [d for nd in range(n) if nd not in live for d in range(6 * nd, 6 * nd + 6)]
    free = np.setdiff1d(np.arange(6 * n), np.array(sorted(set(fixed) | set(dead)), dtype=int))
    u = np.zeros(6 * n)
    Kf = K[np.ix_(free, free)]
    u[free] = np.linalg.solve(Kf + 1e-3 * np.eye(len(free)), Fv[free])

    # 部材応力
    worst = []
    for i, j, name, L, ke, T, dofs in elems:
        f_local = T @ (ke @ u[dofs])
        N = abs(f_local[0])
        My = max(abs(f_local[4]), abs(f_local[10]))
        Mz = max(abs(f_local[5]), abs(f_local[11]))
        sigma = N / A + (My + Mz) * (P.EXT_W / 2) / I
        worst.append((sigma, name, L, i, j))
    worst.sort(reverse=True)
    disp = np.linalg.norm(u.reshape(-1, 6)[:, :3], axis=1)

    def at(pt):
        nd = nearest(pt)
        return float(np.linalg.norm(u[6 * nd:6 * nd + 3]))

    key = {
        "砲塔取付面": at((P.TURRET_X, 0.0, Z_TOP)),
        "バケツ": at((-70.0, 0.0, P.MAST_BEAM_Z)),
        "レール前端": at((380.0, YS, Z_TOP)),
    }
    return u, disp, worst, key


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()
    nodes, members = build_model()

    muzzle = (P.TURRET_X, 0.0, P.NIP_Z)
    cases = [
        ("LC1 自重のみ（静止）", dict(extra_forces=[], accel=(0, 0, 0))),
        ("LC2 前後加速 1.5m/s²", dict(extra_forces=[], accel=(1500.0 / 1000.0, 0, 0))),
        ("LC3 横加速 1.5m/s²（旋回・被弾回避）", dict(extra_forces=[], accel=(0, 1500.0 / 1000.0, 0))),
        ("LC4 グラバー全開（片持ち最大）",
         dict(extra_forces=[], accel=(0, 0, 0), grabber_x=-700.0)),
        (f"LC5 射出反力（雑巾{P.RAG_MASS * 1000:.0f}g×{MUZZLE_V_MAX}m/s、ニップ通過中）",
         dict(extra_forces=[(muzzle, (-SHOT_N * math.cos(math.radians(P.PITCH_MAX)), 0.0,
                                      -SHOT_N * math.sin(math.radians(P.PITCH_MAX))))],
              accel=(0, 0, 0))),
        ("LC6 教壇へ接触停止（0.8m/s→0 を0.15s）",
         dict(extra_forces=[((410.0, 0.0, Z_BASE), (-186.0, 0.0, 0.0))], accel=(0, 0, 0))),
    ]
    rows = []
    for name, kw in cases:
        u, disp, worst, key = solve(nodes, members, **kw)
        s, mname, L, i, j = worst[0]
        rows.append((name, f"{key['砲塔取付面']:.2f}", f"{key['バケツ']:.2f}", f"{s:.1f}", mname,
                     "OK" if s < SIGMA_ALLOW else "NG"))
    if args.md:
        print("| 荷重ケース | 砲塔取付面の変位 [mm] | バケツ位置の変位 [mm] | 最大曲げ応力 [MPa] | 支配部材 | 判定 |")
        print("|---|---|---|---|---|---|")
        for r in rows:
            print("| " + " | ".join(r) + " |")
        print()
        print(f"許容応力 {SIGMA_ALLOW:.0f} MPa（A6005C-T5 耐力200MPa / 安全率2.2）、"
              f"節点数 {len(nodes)}、部材数 {len(members)}、総押出長 "
              f"{sum(np.linalg.norm(nodes[j] - nodes[i]) for i, j, _ in members) / 1000:.1f} m")
    else:
        print(f"節点 {len(nodes)} / 部材 {len(members)} / "
              f"押出長 {sum(np.linalg.norm(nodes[j] - nodes[i]) for i, j, _ in members) / 1000:.2f} m")
        for r in rows:
            print(f"[{r[5]:2s}] {r[0]:<34} 砲塔面 {r[1]:>6} mm  バケツ {r[2]:>6} mm  最大応力 {r[3]:>6} MPa ({r[4]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
