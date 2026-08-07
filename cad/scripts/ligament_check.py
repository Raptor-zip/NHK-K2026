#!/usr/bin/env python3
"""肉抜きした板の**荷重の通り道が細くなっていないか**を実体で確かめる。

⚠ 「分断」検査は板が 2 つに割れたときしか落ちない。**繋がってさえいれば
  幅 1mm の糸でも通る**ので、肉抜きを荷重無視で入れると
  「検査は通るのに実機では裂ける板」ができる。ここはその隙間を埋める。

やること（板 1 枚ごと）:
  1. **いちばん面積の大きい平面**の法線を板厚の向きとして、板の中央面で
     薄く切る（＝板の輪郭）。bbox の最小辺で決めると、耳を曲げた板が漏れる
  2. 輪郭（外周 + 穴）を 1mm 格子に焼いて、材料セルの**距離変換**を取る。
     距離変換の値 × 2 がその点での板の幅（＝リガメント幅）
  3. その板に固定を宣言している相手ごとに、**接触している領域**を格子へ
     落とす（＝荷重の出入口）
  4. 出入口どうしを結ぶ経路のうち、**最も細いところが最も太くなる経路**
     （ボトルネック最大化）を探し、その幅を「荷重経路の最小幅」とする
  5. 最小幅がしきい値を下回る板を出す

⚠ 4 は「最短経路」ではない。細い糸で最短に繋がっていても、太い迂回路が
  あるならそちらを通せばよい。max-min ダイクストラで解く。
"""
from __future__ import annotations

import heapq
import sys
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MplPath
from scipy import ndimage

_R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_R / "src"))
sys.path.insert(0, str(_R / "scripts"))

from build123d import Align, Box, Location  # noqa: E402

import assembly_check as AC  # noqa: E402
import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import validate as V  # noqa: E402

CTR = (Align.CENTER,) * 3
CELL = 1.0          # 格子の刻み [mm]
MAX_T = 8.0         # これより厚いものは板とみなさない
MIN_AREA = 4_000.0  # これより小さい板は見ない [mm²]
# 荷重経路の最小幅の下限 [mm]。アルミ板のリブとして成立する幅。
# ⚠ トラス肉抜き（`src/lattice.py`）の `rib` と同じ根拠（曲げと振動で
#   リブから裂ける）。ここはリブ単体ではなく**経路全体**の細りを見る。
MIN_LIG = 6.0


def _sections(shape, bb):
    """板の中央面で切った輪郭（外周ポリゴン, 穴ポリゴン列, 面内軸）を返す。

    ⚠ 板厚の向きを **bbox の最小辺**で決めると、耳（フランジ）を曲げた板が
      漏れる。car_beam は 40×623.8×**23** で、最小辺 23 が板厚（3）ではなく
      耳の高さなので「板ではない」と判定されていた。
      **いちばん面積の大きい平面**の法線を板厚の向きとする。
    """
    d = [bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z]
    ax = int(np.argmin(d))
    try:
        planar = [f for f in shape.faces()
                  if abs(abs(f.normal_at(f.center()).X)
                         + abs(f.normal_at(f.center()).Y)
                         + abs(f.normal_at(f.center()).Z) - 1.0) < 1e-6]
        big = max(planar, key=lambda f: f.area)
        nvec = big.normal_at(big.center())
        k = int(np.argmax([abs(nvec.X), abs(nvec.Y), abs(nvec.Z)]))
        if abs((nvec.X, nvec.Y, nvec.Z)[k]) > 0.999:
            ax = k
            # 板厚 = その向きの、いちばん面積の大きい平面から測った厚み
            c = big.center()
            d[ax] = 2.0 * min(
                abs((c.X, c.Y, c.Z)[ax] - (bb.min.X, bb.min.Y, bb.min.Z)[ax]),
                abs((bb.max.X, bb.max.Y, bb.max.Z)[ax] - (c.X, c.Y, c.Z)[ax])) \
                or d[ax]
    except Exception:                       # noqa: BLE001
        pass
    if d[ax] > MAX_T:
        return None
    c = [(bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2,
         (bb.min.Z + bb.max.Z) / 2]
    try:
        cc = big.center()
        # 板の中央面 = いちばん大きい平面から板厚の半分だけ内側
        inner = ((bb.min.X, bb.min.Y, bb.min.Z)[ax]
                 + (bb.max.X, bb.max.Y, bb.max.Z)[ax]) / 2
        cc_ax = (cc.X, cc.Y, cc.Z)[ax]
        c[ax] = cc_ax + (d[ax] / 2) * (1 if inner > cc_ax else -1)
    except Exception:                       # noqa: BLE001
        pass
    size = [d[0] + 2, d[1] + 2, d[2] + 2]
    size[ax] = 0.3
    sec = shape & Location(tuple(c)) * Box(*size, align=CTR)
    if sec is None or not sec.faces():
        return None
    face = max(sec.faces(), key=lambda f: f.area)
    if face.area < MIN_AREA:
        return None
    uv = [i for i in range(3) if i != ax]

    def poly(wire):
        # ⚠ 円弧は頂点だけ拾うと**多角形が潰れる**（穴が四角になる）。
        #   弧長で刻んで点列にする。
        n = max(16, int(wire.length / 2.0))
        pts = [wire @ (i / n) for i in range(n + 1)]
        return np.array([[(p.X, p.Y, p.Z)[uv[0]], (p.X, p.Y, p.Z)[uv[1]]]
                         for p in pts])

    return poly(face.outer_wire()), [poly(w) for w in face.inner_wires()], uv


def _raster(outer, holes):
    """輪郭を 1mm 格子へ焼く。戻りは (材料マスク, 原点, 形)。"""
    lo = outer.min(axis=0) - CELL
    hi = outer.max(axis=0) + CELL
    nu = int((hi[0] - lo[0]) / CELL) + 1
    nv = int((hi[1] - lo[1]) / CELL) + 1
    if nu * nv > 4_000_000:
        return None, lo, (0, 0)
    gu, gv = np.meshgrid(np.arange(nu) * CELL + lo[0],
                         np.arange(nv) * CELL + lo[1], indexing="ij")
    pts = np.stack([gu.ravel(), gv.ravel()], axis=1)
    mask = MplPath(outer).contains_points(pts).reshape(nu, nv)
    for h in holes:
        mask &= ~MplPath(h).contains_points(pts).reshape(nu, nv)
    return mask, lo, (nu, nv)


def _bottleneck(width, seeds):
    """seeds[0] から他の seed へ、**最も細いところが最も太い**経路を探す。

    max-min ダイクストラ。戻りは各 seed へのボトルネック幅の最小値。
    """
    nu, nv = width.shape
    best = np.zeros((nu, nv))
    pq = []
    for (i, j) in seeds[0]:
        if width[i, j] > best[i, j]:
            best[i, j] = width[i, j]
            heapq.heappush(pq, (-width[i, j], i, j))
    while pq:
        w, i, j = heapq.heappop(pq)
        w = -w
        if w < best[i, j]:
            continue
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if not (0 <= a < nu and 0 <= b < nv) or width[a, b] <= 0:
                continue
            nw = min(w, width[a, b])
            if nw > best[a, b]:
                best[a, b] = nw
                heapq.heappush(pq, (-nw, a, b))
    out = []
    for grp in seeds[1:]:
        vals = [best[i, j] for (i, j) in grp if best[i, j] > 0]
        out.append(max(vals) if vals else 0.0)
    return out


def main() -> int:
    pose = sys.argv[1] if len(sys.argv) > 1 else "match"
    sol = V.solids_with_bbox(A.build(AC.ALL_POSES[pose]()))
    by = {}
    for p, s, b in sol:
        by.setdefault(AC.part_name(p), []).append((s, b))
    rows, bad = [], []
    for nm, lst in sorted(by.items()):
        if len(lst) != 1:
            continue
        s, bb = lst[0]
        got = _sections(s, bb)
        if got is None:
            continue
        outer, holes, uv = got
        mask, lo, shape = _raster(outer, holes)
        if mask is None or not mask.any():
            continue
        # 距離変換 × 2 = その点での板の幅
        width = ndimage.distance_transform_edt(mask) * CELL * 2.0
        # 固定を宣言している相手ごとに、接触している領域を格子へ落とす
        seeds = []
        for t, _h, _q, _n in F.FIXINGS.get(nm, ()):
            for t2, (s2, b2) in ((t, x) for x in by.get(t, ())):
                inter = (bb.min.X <= b2.max.X and bb.max.X >= b2.min.X
                         and bb.min.Y <= b2.max.Y and bb.max.Y >= b2.min.Y
                         and bb.min.Z <= b2.max.Z and bb.max.Z >= b2.min.Z)
                if not inter:
                    continue
                q0 = (max(bb.min.X, b2.min.X), max(bb.min.Y, b2.min.Y),
                      max(bb.min.Z, b2.min.Z))
                q1 = (min(bb.max.X, b2.max.X), min(bb.max.Y, b2.max.Y),
                      min(bb.max.Z, b2.max.Z))
                i0 = int((q0[uv[0]] - lo[0]) / CELL)
                i1 = int((q1[uv[0]] - lo[0]) / CELL)
                j0 = int((q0[uv[1]] - lo[1]) / CELL)
                j1 = int((q1[uv[1]] - lo[1]) / CELL)
                cells = [(i, j)
                         for i in range(max(0, i0), min(shape[0], i1 + 1))
                         for j in range(max(0, j0), min(shape[1], j1 + 1))
                         if mask[i, j]]
                if cells:
                    seeds.append(cells)
        if len(seeds) < 2:
            continue
        w = _bottleneck(width, seeds)
        lig = min(w) if w else 0.0
        rows.append((lig, nm, len(seeds), float(width[mask].max())))
        if lig < MIN_LIG:
            bad.append((lig, nm, len(seeds)))
    rows.sort()
    print(f"[{pose}] 板として見た部品 {len(rows)}（固定先が 2 つ以上のもの）\n")
    print("  荷重経路の最小幅（細い順）")
    for lig, nm, ns, wmax in rows[:16]:
        mark = "  ← 細い" if lig < MIN_LIG else ""
        print(f"    {lig:6.1f}mm  {nm:22s} 固定先 {ns:2d} / 最大幅 {wmax:5.1f}{mark}")
    print()
    if bad:
        print(f"### 荷重の通り道が {MIN_LIG:.0f}mm を下回る板 {len(bad)} 件")
        for lig, nm, ns in bad:
            print(f"  {lig:6.1f}mm  {nm}（固定先 {ns} か所）")
        return 1
    print(f"荷重の通り道はどの板も {MIN_LIG:.0f}mm 以上")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
