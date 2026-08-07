"""TR 設計検証 — 規定適合・干渉・質量を機械的にチェックする.

    python scripts/validate.py            # 全ポーズを検証してレポート出力
    python scripts/validate.py --md       # Markdown 表で出力（DESIGN.md へ貼る用）
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from build123d import Location  # noqa: E402

import tr_assembly as A  # noqa: E402
import tr_fix as F  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402

OK, NG = "OK", "NG"


def _solids(shape, skip=()):
    """Compound を再帰的に辿って (ラベルパス, **世界座標の** Solid) を列挙する。

    ⚠ ここが本ファイルで一番間違えやすい。
    build123d の子ノードは**親に付いた Location を自分の形状に持っていない**。
    砲塔・グラバー・車輪・シンギュレータは `Pos(...) * Rot(...) * link_xxx()` の形で
    グループ単位に位置を与えているので、葉をそのまま取り出すと**局所座標**になる。
    （例: フォークの葉は z=-2..0 で返る。実際は 761..763）

    局所座標のまま干渉判定すると、bbox が重ならないので**必ず「干渉なし」になる**。
    つまりチェックが素通りする。ここで祖先の Location を掛けてから返すこと。
    """
    出 = []
    ident = Location()

    def walk(node, path):
        label = getattr(node, "label", "") or ""
        newpath = f"{path}/{label}" if label else path
        # ⚠ **判定は「末尾一致」。部分一致にしてはいけない。**
        #   グループ名にも部品名にも "bucket" が入るので、`"bucket" in path`
        #   で外形から除くと、**バケツを留める金具まで丸ごと消える**
        #   （受け板・位置決めタブ・押さえリング）。規定 3.2.2 が高さから
        #   除いてよいのは**バケツそのもの**だけで、留める金具は機体なので
        #   1200mm に効く。実際 2026-08-07 まで、受け板を厚くしても
        #   タブを立てても「スタート時 高さ」は 1 mm も動かなかった。
        if any(newpath.endswith(s) for s in skip):
            return
        children = list(getattr(node, "children", []) or [])
        if node.location != ident:
            # 位置を持つノードに来たら、そこで `.solids()` を呼んで下を畳む。
            # OCC は Location を遅延合成するので**形状のコピーが起きない**。
            # （葉ごとに `moved()` を掛けると 345 個で 9.0GB / 96 秒。実測済み）
            #
            # ただし畳むと**葉のラベルが失われる**。部品ごとの固定宣言
            # （tr_fix.py）と突き合わせるには名前が要るので、
            # 局所座標のまま葉を辿って**展開順のラベル列**を作り、
            # `node.solids()` の並びと対応づける。
            # 局所形状に `.solids()` を呼ぶだけならコピーは起きない。
            ws = node.solids()
            labels = []

            def collect(n, p):
                lab = getattr(n, "label", "") or ""
                np_ = f"{p}/{lab}" if lab else p
                ch = list(getattr(n, "children", []) or [])
                if ch:
                    for c in ch:
                        collect(c, np_)
                else:
                    labels.extend([np_] * len(n.solids()))

            collect(node, newpath)
            if len(labels) != len(ws):
                # 対応が取れないときは黙って落とさず、番号名で出す
                labels = [newpath] * len(ws)
            seen: dict[str, int] = {}
            for lab, s in zip(labels, ws):
                k = seen.get(lab, 0)
                seen[lab] = k + 1
                出.append((f"{lab}#{k}", s))
            return
        if children:
            for c in children:
                walk(c, newpath)
        else:
            出.append((newpath, node))

    walk(shape, "")
    return 出


def _bbox_of(parts):
    xs = [p.bounding_box() for _, p in parts]
    return (
        min(b.min.X for b in xs), max(b.max.X for b in xs),
        min(b.min.Y for b in xs), max(b.max.Y for b in xs),
        min(b.min.Z for b in xs), max(b.max.Z for b in xs),
    )


def _subtree(shape, label):
    """ラベルで部分木を探す。

    `_solids()` は葉をそのまま返すので、**親に付いた Location が乗らない**。
    グループ単位で位置を与えている部分（フォークなど）を世界座標で測るときは、
    その Compound を取り出して `.solids()` を呼ぶこと。
    """
    if getattr(shape, "label", "") == label:
        return shape
    for c in getattr(shape, "children", None) or []:
        got = _subtree(c, label)
        if got is not None:
            return got
    return None


# 外形（規定 3.2.2）から除く部品。**バケツ本体と、その 2L 目盛りのデータムだけ。**
# ルールブック 3.2.2 の脚注「※スタート時も競技中も移動バケツの高さは含まない」。
# ⚠ 除くのはバケツで、**バケツを留める金具ではない**。受け板・位置決めタブ・
#   押さえリングは機体の一部なので高さに数える。
SKIP_ENVELOPE = ("/bucket/bucket", "/bucket/bucket_2l_datum")


def envelope(pose, skip=SKIP_ENVELOPE):
    """指定ポーズの外形寸法（バケツ本体を除外できる）。"""
    shape = A.build(pose)
    parts = _solids(shape, skip=skip)
    x0, x1, y0, y1, z0, z1 = _bbox_of(parts)
    return dict(size=(x1 - x0, y1 - y0, z1), box=(x0, x1, y0, y1, z0, z1), shape=shape)


# 干渉を見る組み合わせと、**必要なすきま [mm]**。
#   0.0 = ボルトで留める取付面。**接触しているのが正しい**ので、
#         むしろ「離れている＝浮いている」ほうを NG にする
#   >0  = 動く物どうし、または動く物と固定物。この値だけ離れていなければならない
#
# ⚠ ここを全部「接触=NG」で見ていたせいで、取付面が 12 件も NG に見えていた。
#   逆に、取付面が浮いていても（＝設計ミス）これまで気付けなかった。
PAIRS = [
    # 砲塔は ±30° 旋回する。固定物とは組立公差ぶん離れていなければならない
    ("turret", "base_link/side_frame", 3.0),
    ("turret", "base_link/chair", 3.0),
    ("turret", "base_link/feed_ramp", 3.0),
    ("turret", "base_link/mast", 3.0),
    # グラバーは P.GRAB_STROKE（316mm）走る。レールの能力は 424.6 だが
    # 駆動が出せるのは 316（src/tr_params.py の GRAB_STROKE 参照）
    ("grabber", "base_link/hopper", 3.0),
    ("grabber", "base_link/feed_ramp", 3.0),
    ("grabber", "base_link/side_frame", 3.0),
    ("slide_rails", "grabber", 2.0),        # レール内部の摺動すきま。2mm あれば十分
    ("slide_rails", "base_link/feed_ramp", 3.0),
    # 配線は動く物に触れてはいけない（擦れて被覆が破れる）
    ("base_link/cables", "turret", 3.0),
    ("base_link/cables", "grabber", 3.0),
    ("base_link/cables", "slide_rails", 3.0),
    ("base_link/cables", "base_link/feed_ramp", 2.0),
    # --- ここから下は取付面。接しているのが正しい ---
    # ⚠ レールは上桁に**直接**は留まらない。取付プレート（rail_plate、
    #   slide_rails_fixed グループ）を介す。ここを「レール ↔ 上桁が接触」で
    #   見ていたので、最小すきま 5.8mm が「浮いている」と 4 姿勢ぶん NG に
    #   出ていた。実際に面で当たるのはプレートと上桁。
    ("slide_rails_fixed", "base_link/side_frame", 0.0),
    ("base_link/mast", "base_link/side_frame", 0.0),  # マスト主柱は側面後柱を兼用
    # ⚠ 椅子は側面トラスではなく**ベース骨格のマウント脚 4 本**で留める
    #   （tr_fix の chair_mount_0..3）。ここに残しておくと「接触していない」
    #   と出るが、それは古い前提のほう。締結の検証は assembly_check が持つ。
    ("base_link/chair", "base_link/base_frame", 0.0),
    ("base_link/chair", "base_link/feed_ramp", 3.0),   # 椅子と斜路は別物。離すこと
]


def _in_group(path: str, group: str) -> bool:
    """グループ名は**パスの区切りで**照合する。

    ⚠ 部分文字列で見ていたので `cab_turret` が `turret` に、`cab_grabber` が
      `grabber` に入っていた。配線が可動部として扱われ、
      「grabber × base_link/cables で cab_grabber ↔ cab_grabber が 0.00mm」
      という**同じ部品どうしの干渉**が NG に出ていた（4 姿勢 ×3 組 = 12 件）。
      本当の 1 件（車体側板と斜路座金具のすきま 1.9mm）が埋もれていた。
    """
    segs = [x for x in path.split("/") if x]
    g = [x for x in group.split("/") if x]
    return any(segs[i:i + len(g)] == g for i in range(len(segs) - len(g) + 1))


def _pn(path: str) -> str:
    """ソリッドのパスから部品名（tr_fix の宣言で使う名前）を取り出す。"""
    return path.split("/")[-1].split("#")[0]


def _where(path, solids):
    """干渉相手を「名前 + 実座標」で表す。名前だけだと次の実行で追えない。"""
    for n, s, b in solids:
        if n == path:
            return (f"{n.split('/')[-1]}"
                    f"[X{b.min.X:.0f}..{b.max.X:.0f} "
                    f"Y{b.min.Y:.0f}..{b.max.Y:.0f} "
                    f"Z{b.min.Z:.0f}..{b.max.Z:.0f}]")
    return path.split("/")[-1]


def solids_with_bbox(shape):
    """(ラベルパス, 世界座標 Solid, その bbox) の一覧。姿勢ごとに一度だけ作る。"""
    return [(n, s, s.bounding_box()) for n, s in _solids(shape)]


def interference(solids, group_a, group_b, tol=1.0):
    """2グループ間の干渉を調べ、(当たっている組, 最小すきま[mm]) を返す。

    判定は**ブーリアンではなく最小距離**（BRepExtrema）で行う。理由は2つ。

      * ブーリアンは重い。座標バグを直して実際に比較が走るようになった途端、
        メモリを 18GB 食って止まった。最小距離は新しい形状を作らないので桁違いに軽い
      * 「干渉していない」だけでは設計の役に立たない。
        **あと何mm で当たるのか**が分かるほうが、公差や組立誤差の判断に使える

    bbox の粗ふるい → bbox の重なり体積で早期打ち切り → 最小距離、の順。
    bbox 同士が離れていれば実体も必ず離れているので、この打ち切りは安全側。
    """
    # solids は**姿勢ごとに一度だけ**作って渡すこと。ペアごとに `_solids()` を
    # 呼び直すと、祖先 Location を掛けた形状のコピーが 17 回作られてメモリを食う。
    a = [(n, s, bx) for n, s, bx in solids if _in_group(n, group_a)]
    b = [(n, s, bx) for n, s, bx in solids if _in_group(n, group_b)]
    hits = []
    gap = float("inf")
    for na, sa, ba in a:
        for nb, sb, bb in b:
            # bbox 同士の距離（各軸の離れ量の二乗和）。実体の距離の下限になる。
            dx = max(0.0, ba.min.X - bb.max.X, bb.min.X - ba.max.X)
            dy = max(0.0, ba.min.Y - bb.max.Y, bb.min.Y - ba.max.Y)
            dz = max(0.0, ba.min.Z - bb.max.Z, bb.min.Z - ba.max.Z)
            lower = (dx * dx + dy * dy + dz * dz) ** 0.5
            if lower >= min(gap, 30.0):
                continue          # 既知の最小すきまより遠いので、測るまでもない
            try:
                d = sa.distance_to(sb)
            except Exception:
                continue
            gap = min(gap, d)
            if d < tol:
                hits.append((na, nb, d))
    return hits, (None if gap == float("inf") else gap)


def report(as_md: bool = False):
    rows = []

    def chk(name, value, limit, ok, unit="mm"):
        rows.append((name, f"{value}", f"{limit}", OK if ok else NG))

    # --- 外形 ---
    st = envelope(P.POSE_STOWED)
    sx, sy, sz = st["size"]
    lim = P.RULE_START_BOX
    chk("スタート時 外形 縦×横×高さ",
        f"{sx:.0f} × {sy:.0f} × {sz:.0f}",
        f"≤ {lim[0]:.0f} × {lim[1]:.0f} × {lim[2]:.0f}",
        sx <= lim[0] and sy <= lim[1] and sz <= lim[2])

    ld = envelope(P.POSE_LOADING)
    lx, ly, lz = ld["size"]
    lim = P.RULE_MATCH_BOX
    chk("競技中 最大展開（グラバー全開）",
        f"{lx:.0f} × {ly:.0f} × {lz:.0f}",
        f"≤ {lim[0]:.0f} × {lim[1]:.0f} × {lim[2]:.0f}",
        lx <= lim[0] and ly <= lim[1] and lz <= lim[2])

    yaw_pose = dict(P.POSE_MATCH, yaw=P.YAW_LIMIT, pitch=P.PITCH_MAX)
    yw = envelope(yaw_pose)
    yx, yy, yz = yw["size"]
    chk("競技中 砲塔ヨー+30°・仰角60°",
        f"{yx:.0f} × {yy:.0f} × {yz:.0f}",
        f"≤ {lim[0]:.0f} × {lim[1]:.0f} × {lim[2]:.0f}",
        yx <= lim[0] and yy <= lim[1] and yz <= lim[2])

    # --- バケツ ---
    chk("移動バケツ上面高さ", f"{P.BUCKET_TOP_Z:.0f}",
        f"{P.RULE_BUCKET_TOP_MIN:.0f}〜{P.RULE_BUCKET_TOP_MAX:.0f}",
        P.RULE_BUCKET_TOP_MIN <= P.BUCKET_TOP_Z <= P.RULE_BUCKET_TOP_MAX)

    robot_top = max(st["size"][2], ld["size"][2], yw["size"][2])
    chk("ロボット最高点 < バケツ2L目盛り (3.2.3c)",
        f"{robot_top:.0f}", f"< {P.BUCKET_2L_Z:.0f}", robot_top < P.BUCKET_2L_Z)

    # --- 質量 ---
    A.build(P.POSE_MATCH)
    total = L.LEDGER.total_kg
    chk("重量（椅子・バケツ・電池込み）", f"{total:.2f} kg", f"≤ {P.RULE_MASS_MAX:.1f} kg",
        total <= P.RULE_MASS_MAX)

    # --- 機能寸法 ---
    # ⚠ 先端は RAIL_X0 ではなく **RAIL_X0 − FORK_TINE_EXT**。歯だけを
    #   延ばしたので、この式を直さないと延ばした効果が出ない。
    fork_tip = P.RAIL_X0 - P.FORK_TINE_EXT - P.GRAB_STROKE
    chk("フォーク最大到達（机エッジ-30mm基準）", f"{fork_tip:.0f}", "≤ -680（山の遠端に届く）",
        fork_tip <= -680)
    chk("櫛歯 下面高さ vs 机上面760", f"{P.FORK_Z - P.FORK_T:.1f}", "760.5〜762（山の下に入る）",
        760.5 <= P.FORK_Z - P.FORK_T <= 762.0)
    # 歯先を含めた**実形状の最下点**を CAD から取る。パラメータではなく形状を見るので、
    # 将来テーパを付け直しても検出できる。天板より下に来ると机の**縁に正面衝突**して
    # 机を押す（4.1.4a 反則）。scripts/fork_clearance.py の積み上げ参照。
    fork_z_min = min(s.bounding_box().min.Z
                     for s in _subtree(A.build(P.POSE_LOADING), "fork").solids())
    chk("フォーク実形状の最下点 vs 机上面760",
        f"{fork_z_min:.2f}（上面テーパ {P.FORK_TIP_ANGLE:.1f}°・先端 t{P.FORK_TIP_T}）",
        "760.5〜762（下面は水平のまま＝縁に当てない）",
        P.DESK_H + 0.5 <= fork_z_min <= P.DESK_H + 2.0)
    chk("ホッパー内寸 vs スーパー雑巾400×600",
        f"{P.HOP_X1 - P.HOP_X0:.0f} × {2 * P.HOP_Y:.0f}", "≥ 400 × 600",
        (P.HOP_X1 - P.HOP_X0) >= 400 and 2 * P.HOP_Y >= 600)
    chk("射出ローラー有効幅 vs スーパー雑巾600",
        f"{max(P.ROLLER_Y) * 2 + P.ROLLER_W:.0f}", "≥ 520（中央部把持）",
        max(P.ROLLER_Y) * 2 + P.ROLLER_W >= 520)
    chk("モーター本数", f"M3508×7 / M2006×4 = {len(P.MOTORS)}", "11軸（戦略書§4.0と一致）",
        len(P.MOTORS) == 11)

    # --- スーパー雑巾（400×600・140g）のハンドリング ---
    fork_w = (P.FORK_TINES - 1) * P.FORK_PITCH + P.FORK_TINE_W
    chk("スーパー雑巾 幅600 vs 櫛歯全幅", f"{fork_w:.0f}", "600に対し中央支持（布は垂れる）",
        fork_w >= 400)
    tail = P.SUPER_X - P.FORK_LEN
    drag = 0.2 * P.SUPER_MASS * 9.81                    # 布→メラミン天板の摩擦
    desk_break = 0.5 * 8.0 * 9.81                       # 机(約8kg)を滑らせるのに要る力
    chk("スーパー雑巾 奥行400 のはみ出し量", f"{tail:.0f} mm",
        f"引きずり摩擦 {drag:.2f}N ≪ 机が動く {desk_break:.0f}N", drag < desk_break * 0.05)
    chk("上押さえのクランプ面積 vs スーパー雑巾",
        f"{P.PRESS_PLATE[0]:.0f}×{P.PRESS_PLATE[1]:.0f}", "中央 220×400 を押さえる",
        P.PRESS_PLATE[0] >= 200 and P.PRESS_PLATE[1] >= 380)
    chk("ホッパー内寸 vs スーパー雑巾（再掲・余裕）",
        f"+{(P.HOP_X1 - P.HOP_X0) - P.SUPER_X:.0f} / +{2 * P.HOP_Y - P.SUPER_Y:.0f} mm",
        "各方向 +10mm 以上", (P.HOP_X1 - P.HOP_X0) - P.SUPER_X >= 10
        and 2 * P.HOP_Y - P.SUPER_Y >= 10)

    # --- 干渉 ---
    inter_rows = []
    for pose_name, pose in (("競技中(ヨー+30/仰角60)", yaw_pose),
                            ("競技中(ヨー-30/仰角20)", dict(P.POSE_MATCH, yaw=-P.YAW_LIMIT,
                                                            pitch=P.PITCH_MIN)),
                            ("装填時(フォーク全開)", P.POSE_LOADING),
                            ("スタート時", P.POSE_STOWED)):
        shape = A.build(pose)
        pose_solids = solids_with_bbox(shape)
        for ga, gb, need in PAIRS:
            hits, gap = interference(pose_solids, ga, gb, tol=max(need, 0.01))
            # ⚠ **締結を宣言してある組は除く。** 取付面（ブラケットを桁に
            #   ボルト留め）や圧入は触れているのが正しいのに、ここは距離の
            #   しきい値だけで見るので NG に出る。実際 27 件がそれで、
            #   本当の 1 件（フォーク到達不足）がノイズに埋もれていた。
            #   宣言と実体の突き合わせは assembly_check が持つ。
            hits = [h for h in hits
                    if F.declared(_pn(h[0]), _pn(h[1])) is None]
            label = f"{ga} × {gb}"
            if need == 0.0:
                # 取付面。触れているのが正しいので、**離れている**ほうを疑う
                ok = gap is not None and gap < 1.0
                got = ("接触（取付面）" if ok else
                       "十分離れている（>30mm）" if gap is None else
                       f"すきま {gap:.1f}mm ⚠ 浮いている")
                inter_rows.append((pose_name, label + "〈取付〉", got, OK if ok else NG))
            elif hits:
                worst = min(hits, key=lambda h: h[2])
                # ⚠ 部品番号（turret#37 など）は形状を1つ足すたびに振り直される。
                #   番号だけ出しても次の実行では追跡できないので、**実座標**を併記する。
                inter_rows.append((pose_name, label,
                                   f"{len(hits)}箇所 最小すきま{worst[2]:.2f}mm "
                                   f"{_where(worst[0], pose_solids)} ↔ "
                                   f"{_where(worst[1], pose_solids)}",
                                   NG))
            elif gap is None:
                inter_rows.append((pose_name, label, "十分離れている（>30mm）", OK))
            else:
                inter_rows.append((pose_name, label,
                                   f"すきま {gap:.1f}mm（要 {need:.1f}）", OK))

    if as_md:
        print("| 検証項目 | 設計値 | 規定/目標 | 判定 |")
        print("|---|---|---|---|")
        for n, v, l, s in rows:
            print(f"| {n} | {v} | {l} | {s} |")
        print()
        print("| ポーズ | 干渉ペア | 結果 | 判定 |")
        print("|---|---|---|---|")
        for r in inter_rows:
            print("| " + " | ".join(r) + " |")
    else:
        w = max(len(r[0]) for r in rows)
        for n, v, l, s in rows:
            print(f"[{s:2s}] {n:<{w}}  {v:>28}   ({l})")
        print()
        for r in inter_rows:
            print(f"[{r[3]:2s}] {r[0]:<24} {r[1]:<40} {r[2]}")

    ng = [r for r in rows if r[3] == NG] + [r for r in inter_rows if r[3] == NG]
    return len(ng)


def mass_table(as_md: bool = False):
    A.build(P.POSE_MATCH)
    groups = L.LEDGER.by_group()
    if as_md:
        print("| 系統 | 質量 [kg] | 比率 |")
        print("|---|---|---|")
        for g, m in sorted(groups.items(), key=lambda kv: -kv[1]):
            print(f"| {g} | {m:.2f} | {m / L.LEDGER.total_kg * 100:.0f}% |")
        print(f"| **合計** | **{L.LEDGER.total_kg:.2f}** | 規定 {P.RULE_MASS_MAX:.0f}kg |")
    else:
        for g, m in sorted(groups.items(), key=lambda kv: -kv[1]):
            print(f"  {g:<10} {m:6.2f} kg")
        print(f"  {'合計':<10} {L.LEDGER.total_kg:6.2f} kg / 規定 {P.RULE_MASS_MAX:.0f} kg")


if __name__ == "__main__":
    md = "--md" in sys.argv
    n = report(md)
    print()
    mass_table(md)
    sys.exit(1 if n else 0)
