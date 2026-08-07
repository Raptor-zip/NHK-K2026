"""突き合わせ継手の**内隅**を幾何から数え直し、L 金具の有無を検査する.

    python scripts/corner_bracket.py [--pose match] [--md] [--code] [--only 名前]

なぜ要るか
-----------
マスト上部横梁 ↔ 主柱は、長いあいだ

    「ここは L 金具が**入らない**継手。柱と横梁は X の幅が一致していて、
      当てられる面は柱の ±X 面と横梁の ±X 面──**同一平面**だから」

と註釈して、平板（ジョイントプレート）だけで留めてあった。註釈の前半は
正しいが、**面の組を 1 つしか見ていない**。T 継手の内隅の候補は
「横材の 4 つの側面 × 縦材の側面」で 4 通りあり、このときは
横梁の**下面**と柱の内面が直交して空いていた。そこが本来の入隅で、
バケツ＋水の鉛直荷重は本来そこを通る。

人が一度「入らない」と判断すると、その判断は図にも台帳にも残らないまま
固定される。**判断のほうを毎回検査する**ためにこの表を作る。

何を見るか
-----------
押出材どうしの突き合わせ（一方の**端面**が他方の**側面**に当たる継手）を
実体の bbox から拾い、継手ごとに内隅の候補を 4 つ数える。候補が
「使える」条件は 1 つだけ:

    縦材（当てられる側）が、横材のその側面より **腕の長さ以上はみ出している**

はみ出しが 0 なら 2 つの面は同一平面で、金具の腕は相手から外れて
座面 0mm² になる（`L.bracket_tee` の註にある「マスト 4 か所」がこれ）。

使える内隅ごとに
  * すでに金具（両方の材に留まると宣言された部品）が入っているか
  * 入っていないなら、そこに `L.bracket()` を置いて他の部品に当たらないか
を調べ、置ける場合は**そのまま貼れるコード**を出す（`--code`）。

⚠ **見ていない継手がある: 重ね継手（ラップ）。**
`joints_of()` が拾うのは「A の**端面**が B の側面に当たっている」組だけ。
2 本の押出材が**面で重なっている**（どちらも端面ではない）組はここに出ない。
2026-08-07 に表示器の横材（`disp_beam`）がそうだった。柱の -X 面に重ねて置き、
`BRACKET` と宣言しながら金具もねじも 1 つも無く、この表にも 1 行も出ていない。
重ね継手には**標準の L 金具が原理的に入らない**（金具の穴は内隅から 11mm、
溝の芯は材の縁から 10mm。腕をそれぞれの材の軸方向へ伸ばして初めて穴が芯に
乗るが、重ね継手では 2 本の溝が材幅ぶん＝ 20mm 離れていて、幅 20 の金具で
両方に届く向きが存在しない）。**重ね継手を見つけたら T 継手に作り替える**のが
正解で、ここに検査を足すならその方針で。いまは
`scripts/assembly_check.py` の F 項（BRACKET と書いたのに金具が無い）が
症状のほうを拾っている。
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

from build123d import Pos, Rot  # noqa: E402

import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402
import validate as V  # noqa: E402

POSES = {"match": lambda: P.POSE_MATCH,
         "stowed": lambda: P.POSE_STOWED,
         "loading": lambda: P.POSE_LOADING}

AXIS = "xyz"

# 押出材とみなす断面 [mm]（長辺, 短辺）。HFS5-2020 / HFS5-2010
EXT_SECTIONS = ((20.0, 20.0), (20.0, 10.0))
SEC_TOL = 0.8           # 断面寸法の許容差
MIN_LEN = 40.0          # これより短いものは部材と見ない
TOUCH_TOL = 0.6         # 端面が相手の面に「当たっている」とみなすすきま [mm]
LAP_MIN = 3.0           # 直交する 2 軸で、この幅以上重なっていること [mm]

# L 金具（`L.bracket()` の既定値）
ARM = 17.0
PLATE_T = 6.0
SIZE = 20.0
# 提案した金具が他の部品に食い込む体積の上限 [mm³]。ねじ 1 本ぶん未満は
# 「溝ナットに噛む」ぶんなので許す（`tr_fix.HOW` の TNUT と同じ考え方）。
DIG_MAX = 1.0
# bbox の重なりがこれ未満ならブーリアンを走らせない [mm³]
BOOL_MIN = 20.0


def part_name(path: str) -> str:
    return path.split("/")[-1].split("#")[0]


def rng(bb, ax):
    return ((bb.min.X, bb.max.X), (bb.min.Y, bb.max.Y),
            (bb.min.Z, bb.max.Z))[ax]


def face(bb, ax, side):
    lo, hi = rng(bb, ax)
    return hi if side > 0 else lo


def lap(ba, bb, ax):
    """軸 ax の重なり幅 [mm]（負なら離れている）。"""
    la, ha = rng(ba, ax)
    lb, hb = rng(bb, ax)
    return min(ha, hb) - max(la, lb)


def bbox_overlap(ba, bb):
    v = 1.0
    for ax in range(3):
        v *= max(0.0, lap(ba, bb, ax))
    return v


def member_axis(bb):
    """押出材なら長手方向の軸（0/1/2）、そうでなければ None。"""
    d = [rng(bb, i)[1] - rng(bb, i)[0] for i in range(3)]
    order = sorted(range(3), key=lambda i: d[i])
    ax = order[2]
    if d[ax] < MIN_LEN:
        return None
    sec = sorted((d[order[0]], d[order[1]]), reverse=True)
    for w, h in EXT_SECTIONS:
        if abs(sec[0] - w) <= SEC_TOL and abs(sec[1] - h) <= SEC_TOL:
            return ax
    return None


# 局所基底 (d1, d2, d3) を作る回転 Rot(rx,ry,rz) を総当たりで探す。
# `L.bracket()` は 原点＝内隅・腕は +X と +Y・幅は Z なので、
#   X_local → d1（突き当てた材の中へ戻る向き）
#   Y_local → d2（横材の側面から外へ出る向き）
# に写す回転が要る。
_ANGLES = (0, 90, 180, -90)


def _image(rot, v):
    p = (rot * Pos(*v)).position
    return (round(p.X, 6), round(p.Y, 6), round(p.Z, 6))


def find_rot(d1, d2):
    d3 = (d1[1] * d2[2] - d1[2] * d2[1],
          d1[2] * d2[0] - d1[0] * d2[2],
          d1[0] * d2[1] - d1[1] * d2[0])
    for rx in _ANGLES:
        for ry in _ANGLES:
            for rz in _ANGLES:
                r = Rot(rx, ry, rz)
                if (_image(r, (1, 0, 0)) == d1 and _image(r, (0, 1, 0)) == d2
                        and _image(r, (0, 0, 1)) == d3):
                    return (rx, ry, rz)
    raise AssertionError(f"軸に沿わない基底 {d1} {d2}")


def unit(ax, sgn):
    v = [0.0, 0.0, 0.0]
    v[ax] = float(sgn)
    return tuple(v)


class Joint:
    """A の端面（軸 na・符号 sgn）が B の側面に当たっている継手。"""

    def __init__(self, a, ba, na, sgn, b, bb_, nb):
        self.a, self.ba, self.na, self.sgn = a, ba, na, sgn
        self.b, self.bb, self.nb = b, bb_, nb
        self.plane = face(ba, na, sgn)

    def key(self):
        return (self.a, self.b)

    def label(self):
        return (f"{self.a}[{AXIS[self.na]}{'+' if self.sgn > 0 else '-'}端] "
                f"↔ {self.b}[{AXIS[self.nb]}材]")


def joints_of(members):
    """(名前 -> (bbox, 軸)) から突き合わせ継手を拾う。"""
    out = []
    names = sorted(members)
    for a in names:
        ba, na = members[a]
        for b in names:
            if a == b:
                continue
            bb_, nb = members[b]
            if na == nb:
                continue                       # 平行な材どうしは継手にしない
            for sgn in (1, -1):
                # A の端面が B の手前面と一致していること
                if abs(face(ba, na, sgn) - face(bb_, na, -sgn)) > TOUCH_TOL:
                    continue
                # 残る 2 軸で実際に重なっていること（角で線が触れるだけは除く）
                if any(lap(ba, bb_, m) < LAP_MIN for m in range(3) if m != na):
                    continue
                out.append(Joint(a, ba, na, sgn, b, bb_, nb))
    return out


def corners_of(j):
    """継手 j の内隅の候補。(m, s, はみ出し, 使えるか, 理由) の並び。"""
    out = []
    for m in range(3):
        if m == j.na:
            continue
        k = 3 - j.na - m                       # 残る軸（金具の幅方向）
        for s in (1, -1):
            fa = face(j.ba, m, s)
            fb = face(j.bb, m, s)
            stick = (fb - fa) * s              # 縦材が横材の側面より出ている量
            wide = min(lap(j.ba, j.bb, k), rng(j.ba, k)[1] - rng(j.ba, k)[0])
            if stick < ARM - 0.01:
                why = ("同一平面（腕が相手から外れる）" if stick <= 0.01
                       else f"はみ出し {stick:.1f}mm < 腕 {ARM:.0f}mm")
                out.append((m, s, k, stick, False, why))
            elif wide < SIZE - 0.01:
                out.append((m, s, k, stick, False,
                            f"幅 {wide:.1f}mm < 金具 {SIZE:.0f}mm"))
            else:
                out.append((m, s, k, stick, True, ""))
    return out


def corner_loc(j, m, s, k):
    """内隅に `L.bracket()` を置く配置（Pos * Rot）と、その (x,y,z,rx,ry,rz)。"""
    d1 = unit(j.na, -j.sgn)                    # 突き当てた材の中へ戻る向き
    d2 = unit(m, s)                            # 横材の側面から外へ出る向き
    rot = find_rot(d1, d2)
    p = [0.0, 0.0, 0.0]
    p[j.na] = j.plane
    p[m] = face(j.ba, m, s)
    lo, hi = max(rng(j.ba, k)[0], rng(j.bb, k)[0]), min(rng(j.ba, k)[1],
                                                        rng(j.bb, k)[1])
    p[k] = (lo + hi) / 2.0
    return Pos(*p) * Rot(*rot), (p[0], p[1], p[2]) + rot


def seat_area(ba, bb):
    """接触面積の目安 [mm²]。bbox の重なりのうち大きい 2 辺の積。"""
    e = sorted(max(0.0, lap(ba, bb, ax)) for ax in range(3))
    return e[1] * e[2]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="match", choices=list(POSES))
    ap.add_argument("--md", action="store_true")
    ap.add_argument("--code", action="store_true",
                    help="金具の無い内隅について、貼り付けられるコードを出す")
    ap.add_argument("--only", default="",
                    help="この文字列を含む部品が絡む継手だけ見る")
    args = ap.parse_args()

    sol = V.solids_with_bbox(A.build(POSES[args.pose]()))
    by_name: dict[str, list] = {}
    for path, s, bb_ in sol:
        by_name.setdefault(part_name(path), []).append((s, bb_))

    # 1 部品が複数ソリッドのときは代表（最大）を使う
    members: dict[str, tuple] = {}
    for nm, lst in by_name.items():
        if nm.startswith("scr_") or nm.startswith("riv_"):
            continue
        s, bb_ = max(lst, key=lambda t: t[1].size.X * t[1].size.Y * t[1].size.Z)
        ax = member_axis(bb_)
        if ax is not None:
            members[nm] = (bb_, ax)

    js = joints_of(members)
    if args.only:
        js = [j for j in js if args.only in j.a or args.only in j.b]

    # 継手ごとに 1 行。A↔B と B↔A の両方が出るので、名前の組で畳む
    seen = set()
    rows = []
    for j in js:
        kk = tuple(sorted((j.a, j.b)))
        if kk in seen:
            continue
        seen.add(kk)
        rows.append(j)
    rows.sort(key=lambda j: (j.a, j.b))

    ng, ok, blocked, zero = [], [], [], []
    lines = []
    for j in rows:
        cs = corners_of(j)
        good = [c for c in cs if c[4]]
        # その継手の「金具」＝ A と B の**両方**に留まると宣言された部品
        conns = [n for n in by_name
                 if n not in (j.a, j.b) and not n.startswith("scr_")
                 and F.declared(n, j.a) and F.declared(n, j.b)]
        note = []
        filled = []
        for (m, s, k, stick, okc, why) in cs:
            if not okc:
                note.append(f"{AXIS[m]}{'+' if s > 0 else '-'}: {why}")
                continue
            loc, code = corner_loc(j, m, s, k)
            brk = loc * L.bracket()
            bbrk = brk.bounding_box()
            here = [n for n in conns
                    if any(bbox_overlap(bbrk, b2) > 1.0 for _s2, b2 in by_name[n])]
            if here:
                filled.append((m, s, here))
                note.append(f"{AXIS[m]}{'+' if s > 0 else '-'}: {'/'.join(here)} が入っている")
                continue
            # 空いている内隅。実際に置けるか（他の部品に当たらないか）を見る
            # ⚠ ねじは**置き直せる**（位置は screw_place.py が幾何から決める）。
            #   金具より先にねじが置いてあるだけの所を「置けない」と言うと、
            #   順序の問題が構造の問題に見える。分けて数える。
            hit, scr = [], []
            for n, lst in by_name.items():
                if n in (j.a, j.b):
                    continue
                for s2, b2 in lst:
                    if bbox_overlap(bbrk, b2) < BOOL_MIN:
                        continue
                    r = brk & s2
                    v = 0.0 if r is None else r.volume
                    if v > DIG_MAX:
                        (scr if n.startswith("scr_") else hit).append((v, n))
            if hit:
                hit.sort(reverse=True)
                blocked.append((j, m, s, hit[:3]))
                note.append(f"{AXIS[m]}{'+' if s > 0 else '-'}: 空きだが "
                            f"{hit[0][1]} に {hit[0][0]:.0f}mm³ 当たる")
            else:
                if scr:
                    scr.sort(reverse=True)
                    note.append(f"{AXIS[m]}{'+' if s > 0 else '-'}: "
                                f"（先に置かれたねじ {scr[0][1]} と "
                                f"{scr[0][0]:.0f}mm³ ← screw_place.py を回し直す）")
                # 当て面が本当に材の上に載っているかを面積で確かめる
                sa = seat_area(bbrk, j.ba)
                sb = seat_area(bbrk, j.bb)
                ng.append((j, m, s, code, sa, sb))
                note.append(f"{AXIS[m]}{'+' if s > 0 else '-'}: **空き**"
                            f"（当て面 {j.a} {sa:.0f}mm² / {j.b} {sb:.0f}mm²）")
        if filled:
            ok.append(j)
        else:
            # ⚠ **ここが本題。** 内隅が空いていること自体は不良ではない
            #   （軽荷重の T 継手は片側 1 個で足りる、と `_butt_brackets` が
            #   決めている）。不良なのは「使える内隅があるのに金具が
            #   **1 つも**入っていない継手」で、これは実機では
            #   手を離した瞬間に落ちる。
            zero.append(j)
        lines.append((j, good, note))

    w = (lambda s: print(s))
    w(f"# 内隅ブラケットの検査（姿勢 {args.pose}）" if args.md
      else f"内隅ブラケットの検査（姿勢 {args.pose}）")
    w("")
    w(f"押出材 {len(members)} 本・突き合わせ継手 {len(rows)} 組")
    w("")
    if args.md:
        w("| 継手 | 使える内隅 | 内訳 |")
        w("|---|---|---|")
    for j, good, note in lines:
        if args.md:
            w(f"| {j.label()} | {len(good)}/4 | " + "<br>".join(note) + " |")
        else:
            w(f"  {j.label()}  使える内隅 {len(good)}/4")
            for t in note:
                w(f"      {t}")
    w("")
    w(f"金具が入っている継手 {len(ok)} 組 / **金具が 1 つも無い継手 {len(zero)} 組** / "
      f"増やせる空き内隅 {len(ng)} 箇所 / 物が当たる内隅 {len(blocked)} 箇所")
    for j in zero:
        good = [c for c in corners_of(j) if c[4]]
        w(f"  NG    {j.label()} ← 使える内隅 {len(good)} 箇所すべてが空。"
          f"{'置ける' if any(t[0] is j for t in ng) else '要検討'}")
    for j, m, s, code, sa, sb in ng:
        w(f"  空き  {j.label()} の {AXIS[m]}{'+' if s > 0 else '-'} 側"
          f"（当て面 {sa:.0f} / {sb:.0f} mm²）")
    for j, m, s, hit in blocked:
        w(f"  当たり {j.label()} の {AXIS[m]}{'+' if s > 0 else '-'} 側 ← "
          + "・".join(f"{n} {v:.0f}mm³" for v, n in hit))

    if args.code and ng:
        w("")
        w("```python")
        for j, m, s, code, _sa, _sb in ng:
            x, y, z, rx, ry, rz = code
            nm = f"brk_{j.a}_{AXIS[m]}{'p' if s > 0 else 'm'}"
            w(f"    # {j.label()} の {AXIS[m]}{'+' if s > 0 else '-'} 側の内隅")
            w(f"    loc = Pos({x:.4g}, {y:.4g}, {z:.4g}) * Rot({rx}, {ry}, {rz})")
            w(f"    put(parts, loc * L.bracket(), {nm!r},")
            w(f"        to=({j.a!r}, {j.b!r}), how=\"TSLOT\","
              f" note=\"1-M5（腕 1 枚に 1 本）\")")
            w(f"    LEDGER.add(\"ブラケット HBLFSN5\", 0.012,"
              f" \"MISUMI カタログ\", \"シャシー\")")
            w(f"    bracket_screws(parts, loc, {nm!r}, ({j.a!r}, {j.b!r}))")
        w("```")
    # 合否は「金具ゼロの継手」だけで決める。空き内隅は情報として出す。
    return 1 if zero else 0


if __name__ == "__main__":
    raise SystemExit(main())
