"""骨格結合の**荷重の受け方**を検査する — 「留まってはいるが、その受け方でいいのか」.

    python scripts/joint_load.py [--pose flat] [--md] [--limit N]

`assembly_check.py` は「留まっているか（接触・座面・段数）」を見る。
`fasteners.py` は手で選んだ 13 か所のボルト本数を計算する。
どちらも**荷重がどの向きで座面に入るか**は見ていない。
実機で壊れる継手は、たいてい本数ではなく**受け方**が悪い:

  1. edge      板の**切り口**で受けている（座面は板厚 × 幅しか無い）
  2. pry       座面の外に荷重が乗っていて、ねじが**引張＋てこ**で効いている
  3. pullout   アルミフレームの溝ナットが**引き抜き**方向に効いている
  4. cantilev  片持ちが長く、受けの幅に対して腕が長すぎる
  5. nogusset  押出材どうしの直角結合にガセット（三角補強）が無い

既存の検査との切り分け
  * `scripts/fea_frame.py` は押出材を**剛結**の梁として解く。継手そのものの
    受け方（ボルトが引張かせん断か）は、剛結を仮定した時点で消える。
    そのファイルの冒頭にも「結合部の緩みは別途担保する」と書いてある。
    ここがその「別途」に当たる。
  * `scripts/fasteners.py` は荷重を人が書いた 13 か所だけ。ここは
    **宣言されている全部の締結**を幾何から見る（280 組以上）。
  * `scripts/assembly_check.py` の座面検査は「面積が閾値以上か」だけで、
    その面が板の側面（切り口）かどうかは区別しない。ここは向きを見る。

⚠ 荷重は「その部品にぶら下がる質量 × 動的倍率」で見積もる。射出反力や
  走行の突き上げは `fea_frame.py` の荷重ケースに任せる（重複させない）。
  ここで出したいのは絶対値ではなく**受け方の良し悪しの順位**なので、
  同じ見積もりで全部の継手を並べれば足りる。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import plate_audit as PA  # noqa: E402  幾何と材質の下ごしらえを共有する

G = 9.81
DYN = 3.0                 # 動的倍率（plate_audit と同じ）
GROW = 1.0                # mm 接触とみなす隙間

# --- しきい値 ---------------------------------------------------------------
EDGE_SEAT_MAX = 400.0     # mm² 切り口の座面がこれ以下なら曲げに耐えない
EDGE_M_MIN = 300.0        # N·mm 切り口に掛かる曲げがこれ未満なら実害なし
PRY_RATIO = 1.0           # 偏心 ÷ 座面の半幅。1 を超えるとねじが引張になる
PRY_M_MIN = 500.0         # N·mm これ未満のモーメントは無視
CANTI_RATIO = 4.0         # 腕 ÷ 受けの幅
CANTI_KG = 0.15           # これ未満の質量の片持ちは無視
# ⚠ **「1 か所で持っている部材」だけを、てこ・片持ちで見る。**
#   最初の版は骨格の桁まで並べてきた（ベース主桁が「てこ比 66.8」）。
#   桁は長手の全域で受けているので、受けの広がりを見れば区別できる。
#   受けが部材の長さの半分以上に広がっていれば、それは分布支持であって
#   片持ちではない（そちらの曲げは fea_frame.py が梁で解いている）。
LOCALIZED = 0.5           # 受けの広がり ÷ 部材の長さ。これ未満なら 1 か所留め
# ⚠ ぶら下がり質量は「固定の連鎖の木」で数えるので、骨格の**根に近い部材**
#   ほど機体の大半を背負う形になる。それは正しいが、継手 1 か所の話ではない。
#   ここより重い部材は骨格そのものなので fea_frame.py に任せる。
HUB_KG = 6.0
# MISUMI 5 シリーズ 後入れナット（HNTP5-5）の**引き抜き**の目安 [N]。
# ⚠ せん断（滑り）と桁が違う。カタログの推奨締付 5N·m で得られる軸力は
#   5000N あるが、それは「溝に押し付ける力」であって、溝から抜く方向の
#   強さではない。溝の口（幅 6mm）にナットの肩が掛かっているだけなので、
#   引き抜きはアルミの肩が変形して抜ける。実務では 1 個 500N 見当。
TNUT_PULLOUT_N = 500.0
EXT_SEC = (20.0, 10.0)    # 押出材の断面寸法（HFS5-2020 / 2010）

STRUCT = ("BOLT", "TSLOT", "BRACKET", "RIVET", "WELD")
# ガセット・ブラケットとみなす部品名の頭
BRK_PREFIX = ("brk_", "gus_", "mount_brk", "car_brk", "beltbrk_", "odo_brk")

# 検査から外す組と理由。**理由を書かせる**運用は assembly_check.REST_OK と同じ。
SKIP_PAIR = {
    # ⚠ **バケツ ↔ 受け板は外さない（2026-08-07）。** 前は「バンドで抱える
    #   ので座面の話ではない」と書いて検査から外していたが、そのバンドは
    #   図にも無く、質量台帳に 150g が載っているだけだった。いまは底面
    #   4-M5 で受け板に留まる（ポリカ t2 の底に φ5.5・押さえリングで面圧を
    #   散らす）ので、座面も本数も普通に評価できる。
    ("chair_5go", "chair_mount_0"): "規定で載せる椅子。ベルト固定",
}


# ⚠ ボルトの本数は**宣言の備考から読む**。図に描いてあるねじ（`scr…`）は
#   代表的な締結だけで、ほとんどの継手は「4-M5」のように備考にしか無い。
#   描かれたねじを数えると、6 本で受けている継手が 1 本扱いになり、
#   1 本あたりの荷重が 6 倍に出る（実際 レール取付板が 670N/本 と出て、
#   本当は 112N/本 だった）。備考の書式は全部「n-Mx」か「各 n-Mx」。
BOLT_RE = __import__("re").compile(r"(\d+)\s*[-−–]\s*M(\d+)")


def bolts_of(note: str) -> int:
    m = BOLT_RE.search(note or "")
    return int(m.group(1)) if m else 1


def contact_box(ba, bb):
    """2 つの bbox の重なり（すきま GROW ぶん膨らませて取る）。無ければ None。"""
    out = []
    for i in range(3):
        lo = max(ba[2 * i] - GROW, bb[2 * i] - GROW)
        hi = min(ba[2 * i + 1] + GROW, bb[2 * i + 1] + GROW)
        if hi <= lo:
            return None
        out.append((lo, hi))
    return out


def thin_axis(box):
    ext = [box[1] - box[0], box[3] - box[2], box[5] - box[4]]
    k = min(range(3), key=lambda i: ext[i])
    return k, ext


def center(box):
    return [(box[0] + box[1]) / 2, (box[2] + box[3]) / 2, (box[4] + box[5]) / 2]


def is_ext(info, nm) -> bool:
    """HFS5 の押出材か（断面が 20×20 か 20×10 で、残る 1 辺が長い）。"""
    b = info[nm]["box"]
    ext = sorted([b[1] - b[0], b[3] - b[2], b[5] - b[4]])
    return (any(abs(ext[0] - s) < 1.0 for s in EXT_SEC)
            and abs(ext[1] - 20.0) < 1.0 and ext[2] > 60.0)


def long_axis(box):
    ext = [box[1] - box[0], box[3] - box[2], box[5] - box[4]]
    return max(range(3), key=lambda i: ext[i])


def subtree_cog(info, F):
    """(部品 -> (ぶら下がる質量 kg, その重心)) を返す。片持ちの腕を測るのに要る。"""
    adj: dict[str, set[str]] = {}
    for name, lst in F.FIXINGS.items():
        for t, h, _q, _n in lst:
            if h in PA.NO_LOAD:
                continue
            adj.setdefault(name, set()).add(t)
            adj.setdefault(t, set()).add(name)
    parent: dict[str, str | None] = {r: None for r in F.ROOTS}
    order: list[str] = []
    frontier = list(F.ROOTS)
    while frontier:
        nxt = []
        for cur in frontier:
            for n in sorted(adj.get(cur, ())):
                if n not in parent:
                    parent[n] = cur
                    order.append(n)
                    nxt.append(n)
        frontier = nxt
    m = {nm: info.get(nm, {}).get("mass", 0.0) for nm in parent}
    mom = {nm: [m[nm] * c for c in center(info[nm]["box"])] if nm in info else [0.0] * 3
           for nm in parent}
    for nm in reversed(order):
        p = parent[nm]
        if p is None:
            continue
        m[p] = m.get(p, 0.0) + m[nm]
        for i in range(3):
            mom[p][i] = mom.get(p, [0.0] * 3)[i] + mom[nm][i]
    out = {}
    for nm in parent:
        if m[nm] > 1e-9:
            out[nm] = (m[nm], [mom[nm][i] / m[nm] for i in range(3)])
        else:
            out[nm] = (0.0, center(info[nm]["box"]) if nm in info else [0.0] * 3)
    return out, parent


def audit(info, F):
    cog, parent = subtree_cog(info, F)
    # ⚠ **「支えている相手」と「載っている相手」を混ぜてはいけない。**
    #   接触を全部まとめて座面にすると、上に載る部品の接触まで受けに数えて
    #   しまい、どの部材も「全長で受けている」ことになる（実際、片持ちの
    #   マスト腕が spread 1.10 になって片持ち検出が 0 件になった）。
    #   受けは「自分にぶら下がっていない相手」。段数の浅い相手だけに絞ると、
    #   斜材のように**両端を同じ深さのガセットで持つ部材**が片側だけで
    #   持っていることになり、これも嘘の片持ちになる。
    def hangs_on(x: str, top: str) -> bool:
        """x は top にぶら下がっているか（top が x の祖先か）。"""
        cur, n = parent.get(x), 0
        while cur is not None and n < 200:
            if cur == top:
                return True
            cur, n = parent.get(cur), n + 1
        return False
    edge, pry, pullout, canti, nogus = [], [], [], [], []
    seen = set()

    # --- 部品ごとの「構造的な座面」をまとめる ---------------------------
    seats: dict[str, list] = {}
    for nm, lst in F.FIXINGS.items():
        if nm not in info:
            continue
        for tgt, how, _q, note in lst:
            if how not in STRUCT or tgt not in info:
                continue
            cb = contact_box(info[nm]["box"], info[tgt]["box"])
            if cb is None:
                continue
            seats.setdefault(nm, []).append((tgt, how, cb, note))
            seats.setdefault(tgt, []).append((nm, how, cb, note))

    for nm, lst in sorted(seats.items()):
        if nm.startswith(("scr", "cab_", "wire_")):
            continue
        m_kg, cg = cog.get(nm, (0.0, center(info[nm]["box"])))
        m_kg = max(m_kg, info[nm]["mass"])
        w_n = m_kg * G * DYN
        k, ext = thin_axis(info[nm]["box"])
        t_own = ext[k]

        # 1. 板の切り口で受けている締結 --------------------------------
        for tgt, how, cb, note in lst:
            if (nm, tgt) in SKIP_PAIR or (tgt, nm) in SKIP_PAIR:
                continue
            ce = [cb[i][1] - cb[i][0] for i in range(3)]
            kc = min(range(3), key=lambda i: ce[i])       # 接触の法線軸
            if t_own > PA.PLATE_T_MAX or ext[k] >= 0.5 * min(
                    e for i, e in enumerate(ext) if i != k):
                continue                                  # 板ではない
            # 法線が板厚の軸と違い、かつ板厚ぶんが丸ごと接触に入っている
            # ＝ 相手は板の**切り口**に当たっている
            if kc == k or ce[k] < 0.7 * t_own:
                continue
            other = [i for i in range(3) if i not in (k, kc)][0]
            seat = t_own * ce[other]
            arm = abs(cg[kc] - (cb[kc][0] + cb[kc][1]) / 2)
            mom = w_n * arm
            if seat <= EDGE_SEAT_MAX and mom >= EDGE_M_MIN and (nm, tgt) not in seen:
                seen.add((nm, tgt))
                seen.add((tgt, nm))
                edge.append((mom, nm, tgt, how, seat, t_own, arm, m_kg))

        # 2. 座面の外に荷重が乗っている（てこ）--------------------------
        #    構造的な座面をひとまとめにした矩形と、ぶら下がる重心の位置を比べる。
        own = info[nm]["box"]
        sup = [x for x in lst if not hangs_on(x[0], nm)] or lst
        allb = [(min(c[i][0] for _t, _h, c, _o in sup),
                 max(c[i][1] for _t, _h, c, _o in sup)) for i in range(3)]
        # 軸ごとに「その向きは全長で受けているか」を見る。ブラケットは
        # 底面の X 方向は全長で受けていても、Y 方向へ張り出していれば
        # そこがてこになる。軸をまとめて 1 つの指標にすると両方見落とす。
        spread = [(allb[i][1] - allb[i][0]) / max(own[2 * i + 1] - own[2 * i], 1e-6)
                  for i in range(3)]
        # ⚠ 押出材だからといって外さない。**片持ちの押出材こそ本命**
        #   （バケツを持つマスト腕は 410mm の片持ち）。分布支持かどうかは
        #   軸ごとの `spread` で判る。外すのは「機体の大半を背負う部材」だけ。
        local = m_kg <= HUB_KG
        # 座面の法線＝座面の広がりが一番小さい軸。**この軸は腕にも座面幅にも
        # 使えない**（重なりの厚みしか無いので、必ず「受け幅 2mm」になる）。
        kn = min(range(3), key=lambda i: allb[i][1] - allb[i][0])
        if lst and m_kg >= 0.05 and local:
            ecc, half, axis = 0.0, 1e9, None
            for i in range(3):
                if i == kn or spread[i] >= LOCALIZED:
                    continue
                c0 = (allb[i][0] + allb[i][1]) / 2
                hw = max((allb[i][1] - allb[i][0]) / 2, 1e-6)
                e = abs(cg[i] - c0)
                if e / hw > ecc / max(half, 1e-9):
                    ecc, half, axis = e, hw, i
            if axis is not None and ecc > PRY_RATIO * half:
                mom = w_n * ecc
                hows = {h for _t, h, _c, _o in sup}
                # 偶力で受ける: 引張側のボルトに掛かる合計 = M ÷ 座面の幅
                n_b = max(1, sum(bolts_of(o) for _t, _h, _c, o in sup) // 2)
                if mom >= PRY_M_MIN:
                    pry.append((mom, nm, ecc, half, "XYZ"[axis], m_kg,
                                sorted(hows), mom / (2 * half) / n_b))

        # 3. 溝ナットが引き抜き方向に効いている --------------------------
        for tgt, how, cb, note in lst:
            if how != "TSLOT" or not is_ext(info, tgt):
                continue
            ce = [cb[i][1] - cb[i][0] for i in range(3)]
            kc = min(range(3), key=lambda i: ce[i])
            # 溝の面の法線が Z（＝押出材の上面/下面に留めた）で、部品が
            # **下側**にあるなら、自重がそのまま引き抜きになる
            cc = (cb[kc][0] + cb[kc][1]) / 2
            below = cg[kc] < cc
            n_nut = bolts_of(note)
            if kc == 2 and below:
                f = w_n
            else:
                # 面内でも、座面の外に重心があれば偶力で片側が引き抜かれる
                span = max(ce[i] for i in range(3) if i != kc)
                arm = math.dist(cg, center([cb[0][0], cb[0][1], cb[1][0], cb[1][1],
                                            cb[2][0], cb[2][1]]))
                f = w_n * arm / max(span, 1.0) if arm > span / 2 else 0.0
            if f / n_nut > TNUT_PULLOUT_N * 0.5:
                pullout.append((f / n_nut, nm, tgt, n_nut, m_kg,
                                "自重が直接" if kc == 2 and below else "てこで"))

        # 4. 片持ちが長い ---------------------------------------------
        # ⚠ 腕は**その部材が受けから先へどれだけ出ているか**で測る。
        #   ぶら下がる荷重の重心までの距離で測ると、軸受のような小さい部品が
        #   「腕 216mm の片持ち」として並ぶ（実際は先の質量が離れているだけで、
        #   部材そのものは短い）。直せるのは部材の形と受けの位置なので、
        #   部材の張り出しで測るほうが指摘として使える。
        if lst and m_kg >= CANTI_KG and local:
            sc = [(allb[i][0] + allb[i][1]) / 2 for i in range(3)]
            arm, width = 0.0, 0.0
            for i in range(3):
                if i == kn or spread[i] >= LOCALIZED:
                    continue                 # 法線方向 / 全長で受けている向き
                a = max(abs(own[2 * i] - sc[i]), abs(own[2 * i + 1] - sc[i]))
                if a > arm:
                    arm, width = a, max(allb[i][1] - allb[i][0], 1e-6)
            if width > 1e-6 and arm / width > CANTI_RATIO:
                canti.append((w_n * arm, nm, arm, width, m_kg, len(sup)))

    # 5. 押出材どうしの直角結合にガセットが無い ------------------------
    for nm, lst in sorted(F.FIXINGS.items()):
        if nm not in info or not is_ext(info, nm):
            continue
        for tgt, how, _q, _n in lst:
            if tgt not in info or not is_ext(info, tgt) or how not in STRUCT:
                continue
            if long_axis(info[nm]["box"]) == long_axis(info[tgt]["box"]):
                continue                     # 平行材（継ぎ足し）は直角結合ではない
            if (nm, tgt) in seen or (tgt, nm) in seen:
                continue
            helpers = [o for o in PA.partners_of(F, nm)
                       if o[0].startswith(BRK_PREFIX)
                       and tgt in [x[0] for x in PA.partners_of(F, o[0])]]
            if not helpers:
                seen.add((nm, tgt))
                m_kg = max(cog.get(nm, (0.0,))[0], cog.get(tgt, (0.0,))[0])
                nogus.append((m_kg, nm, tgt, how))

    for lst in (edge, pry, pullout, canti, nogus):
        lst.sort(reverse=True)
    return edge, pry, pullout, canti, nogus


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="flat", choices=list(PA.POSES))
    ap.add_argument("--limit", type=int, default=12)
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    info, F, _unknown, _claims = PA.build_parts(args.pose)
    edge, pry, pullout, canti, nogus = audit(info, F)

    if args.md:
        print("\n### 荷重的に悪い固定\n")
        print("| 種類 | 継手 | 症状 | 数値 | 直し方 |")
        print("|---|---|---|---|---|")

    def row(kind, joint, sym, num, fix):
        if args.md:
            print(f"| {kind} | {joint} | {sym} | {num} | {fix} |")
        else:
            print(f"[{kind}] {joint:<40s} {sym} {num}  → {fix}")

    for mom, nm, tgt, how, seat, t, arm, m in edge[:args.limit]:
        row("切口", f"{nm} ↔ {tgt}", f"座面 {seat:.0f}mm²(t{t:.1f})",
            f"腕 {arm:.0f}mm・曲げ {mom / 1000:.1f}N·m",
            "留め代を曲げて面で当てる / ガセットを足す")
    for mom, nm, ecc, half, ax, m, hows, tb in pry[:args.limit]:
        row("てこ", nm, f"偏心 {ecc:.0f}mm / 座面半幅 {half:.0f}mm ({ax})",
            f"てこ比 {ecc / half:.1f}・曲げ {mom / 1000:.1f}N·m・引張 {tb:.0f}N/本",
            f"座面を偏心の向きへ広げる（いまの留め方 {'+'.join(hows)}）")
    for f, nm, tgt, n, m, why in pullout[:args.limit]:
        row("抜け", f"{nm} → {tgt}", f"溝ナット {n} 個に {why}",
            f"1 個あたり引抜 {f:.0f}N（目安 {TNUT_PULLOUT_N:.0f}N）",
            "受けを側面の溝へ移す / ブラケットでせん断に変える")
    for mom, nm, arm, w, m, n in canti[:args.limit]:
        row("片持", nm, f"腕 {arm:.0f}mm / 受け幅 {w:.0f}mm",
            f"比 {arm / w:.1f}・{m:.2f}kg で {mom / 1000:.1f}N·m",
            "受けを離して 2 点で支える / 斜材を足す")
    for m, nm, tgt, how in nogus[:args.limit]:
        row("無筋", f"{nm} ⊥ {tgt}", f"直角結合が {how} だけ",
            f"ぶら下がり {m:.2f}kg", "ガセットかブラケットを足す")

    n = len(edge) + len(pry) + len(pullout) + len(canti) + len(nogus)
    print(f"\n切り口受け {len(edge)} / てこ {len(pry)} / 溝ナット引抜 {len(pullout)}"
          f" / 長い片持ち {len(canti)} / ガセット無し {len(nogus)} — 計 {n} 件")
    return 1 if n else 0


if __name__ == "__main__":
    raise SystemExit(main())
