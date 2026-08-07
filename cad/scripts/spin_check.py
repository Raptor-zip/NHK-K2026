"""回転部品の**1 回転ぶんの掃引**（外接回転体）が何かに当たらないか。

⚠ 車輪・射出ローラー・ハブは軸対称でない（スポークやメカナムのローラー）。
  「continuous」関節なので姿勢集合に角度が無く、**ある角度では当たらず
  別の角度で当たる**類の不良を今まで一度も見ていなかった。
  外接回転体は真の掃引を必ず包むので、これが当たらなければ全角度で安全と
  確定できる（当たったら角度を細かく標本にする必要がある）。
"""
import sys
from pathlib import Path
_R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_R / "src")); sys.path.insert(0, str(_R / "scripts"))
from build123d import Cylinder, Pos, Rot, Align
import tr_assembly as A, tr_fix as F, validate as V, assembly_check as AC

CTR = (Align.CENTER,) * 3
SPIN = {"roller_upper": (0, 1, 0), "roller_lower": (0, 1, 0),
        "singulator": (0, 1, 0), "wheel_fl": (0, 1, 0), "wheel_fr": (0, 1, 0),
        "wheel_rl": (0, 1, 0), "wheel_rr": (0, 1, 0)}

sh = A.build()
sol = V.solids_with_bbox(sh)
names = [AC.part_name(n) for n, _s, _b in sol]
spin_idx = [(k, AC.link_of(n)) for k, (n, _s, _b) in enumerate(sol)
            if AC.link_of(n) in SPIN]
print(f"回転部品 {len(spin_idx)} ソリッド")

# 軸の位置は同じリンクのソリッド全体の bbox 中心（軸上にある）
axis_c = {}
for k, lk in spin_idx:
    b = sol[k][2]
    axis_c.setdefault(lk, []).append(b)
axis_pt = {}
for lk, bs in axis_c.items():
    x0 = min(b.min.X for b in bs); x1 = max(b.max.X for b in bs)
    z0 = min(b.min.Z for b in bs); z1 = max(b.max.Z for b in bs)
    axis_pt[lk] = ((x0 + x1) / 2, (z0 + z1) / 2)

hits = []
for k, lk in spin_idx:
    nm = names[k]
    b = sol[k][2]
    cx, cz = axis_pt[lk]
    # ⚠ bbox の隅から半径を取ると円板でも R√2（41% 過大）になる。
    #   頂点から取る（円筒なら継ぎ目の頂点が外周にあるので実寸）。+1mm 余裕。
    vs = sol[k][1].vertices()
    r = 1.0 + max(((v.X - cx) ** 2 + (v.Z - cz) ** 2) ** 0.5 for v in vs)
    env = (Pos(cx, (b.min.Y + b.max.Y) / 2, cz) * Rot(90, 0, 0)
           * Cylinder(r, b.max.Y - b.min.Y, align=CTR))
    eb = env.bounding_box()
    for j, (_p, sb, bb) in enumerate(sol):
        if names[j] == nm or AC.link_of(sol[j][0]) == lk:
            continue
        if AC.bbox_overlap(eb, bb) <= AC.BOOL_MIN:
            continue
        dec = F.declared(nm, names[j])
        v = AC.overlap_volume(env, sb)
        if v > max(dec[1] if dec else 0.0, 1.0):
            hits.append((v, nm, names[j], r, dec[0] if dec else "宣言なし"))
hits.sort(reverse=True)
print(f"外接回転体が当たる組 {len(hits)}")
for v, na, nb, r, how in hits[:24]:
    print(f"  {v:9,.0f}mm³  {na}(掃引半径{r:.1f}) ↔ {nb}  [{how}]")

# ⚠ 外接回転体は**軸の近くも埋める**ので、車輪の中心にモーターの出力軸が
#   来るような「当たって当然」の組も出る。当たった組だけ**実体を 5° 刻みで
#   回して**確かめる（宣言のある組は対象外）。
print("\n■ 当たった組を 5° 刻みで実体確認（宣言の無い組だけ）")
import collections
by = collections.defaultdict(set)
for v, na, nb, r, how in hits:
    if how == "宣言なし":
        by[na].add(nb)
idx = {}
for k, (n_, s_, b_) in enumerate(sol):
    idx.setdefault(names[k], []).append(k)
real = []
for na, others in sorted(by.items()):
    k = idx[na][0]
    lk = AC.link_of(sol[k][0])
    cx, cz = axis_pt[lk]
    worst = 0.0; at = None
    for th in range(0, 360, 5):
        mv = Pos(cx, 0, cz) * Rot(0, float(th), 0) * Pos(-cx, 0, -cz) * sol[k][1]
        for nb in others:
            for j in idx[nb]:
                v = AC.overlap_volume(mv, sol[j][1])
                if v > worst:
                    worst, at = v, (th, nb)
    print(f"  {na:16s} 最悪 {worst:8,.0f}mm³" +
          (f" @ {at[0]}° ↔ {at[1]}" if at else "") +
          ("  ← 実体でも当たる" if worst > 1.0 else "  （外接体だけの見かけ）"))
    if worst > 1.0:
        real.append((worst, na, at))

print()
if real:
    print(f"### 回すと当たる部品 {len(real)} 件")
    for w, na, at in sorted(real, reverse=True):
        print(f"  {w:9,.0f}mm³  {na} ↔ {at[1]}  @ {at[0]}°")
    raise SystemExit(1)
print("回転部品はどの角度でも当たらない")
