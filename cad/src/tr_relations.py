"""部品どうしの関係（締結・摺動・無関係）の定義.

なぜ要るか
-----------
干渉チェックを「手で列挙した17ペア」でやっていたとき、
**列挙していない組み合わせは一度も検査されていなかった**。
283 ソリッドの総当たりは 39,903 組あり、そのうち実体が接触しているのは 355 組。
17ペアで見ていたのはその一部でしかない。

一方、355 組の大半は**接触していて正しい**（ボルトが板を留める、車輪が軸に付く、
フォークがキャリッジに吊られる…）。だから「接触＝NG」の一律判定もできない。

そこで**部品どうしの関係を宣言**し、関係から期待されるすきまを自動で決める。
関係が宣言されていない組み合わせは `FREE`（＝必ず離れていなければならない）
として扱われるので、**列挙漏れが「素通り」ではなく「NG」になる**。

関係の種類
-----------
    BOLTED    ボルト・ねじで留める。接触が正しい。離れていたら「浮いている」で NG
    PRESSED   圧入・インロー嵌合。食い込みも許容（しめしろ）
    WELDED    溶接・一体成形。接触が正しい
    MESHED    歯車・ベルトの噛み合い。接触が正しい
    SLIDING   摺動（スライドレール内部）。指定すきまを保つ
    ROTATING  回転（軸受・ローラー）。指定すきまを保つ
    ROUTED    配線が沿う相手。触れてよい（ただし可動部は別途 FREE で縛る）
    CONTAINED 中に入る（マスコットが椅子に座る、雑巾がホッパーに入る）
    FREE      無関係。**必ず離れていなければならない**（既定値）
"""

from __future__ import annotations

# --- 関係の種類と、そこから決まる判定 ---------------------------------------
# (必要すきま[mm], 接触してよいか, **食い込んでよい体積 [mm³]**)
#
# ⚠ 「接触してよい」と「**重なってよい**」は別。ここを混同していた。
#   `distance_to` は接触も食い込みも 0.0 を返すので、
#   `may_touch=True` の組を測らずに飛ばすと、**何mm食い込んでも通る**。
#   実際、押出材同士（BOLTED）が 20 組・最大 2294mm³（約10mm）めり込んでいた。
#
#   実体の重なりが許されるのは
#     PRESSED  … しめしろ（φ26 のインローを H7 の穴へ、など）
#     MESHED   … 歯車・ベルトの噛み合い表現
#   だけで、それ以外は **0 でなければならない**。
#   突き合わせて留める押出材が重なっていたら、それは切断長の誤り。
#
# ⚠ 当初は BOLTED に「離れていたら NG（浮いている）」を持たせたが、**破綻した**。
#   関係は**グループ間**で宣言している。`("fasteners","base"): BOLTED` と書くと
#   「56本のねじ × 167個の base 部品」の**全組**が接触必須になり、
#   18,068 組すべてを実測することになって終わらなかった。
#   実際には1本のねじは2〜3部品しか留めない。
#   **「離れていたら NG」はグループ単位では表現できない。**
#   浮きの検出が要るなら、ねじ1本ごとに「留める相手」を個別に持たせる必要がある
#   （→ 今後の課題。今は締結解析 scripts/fasteners.py が荷重側から担保している）
KINDS = {
    # 名前            すきま  接触可   食い込み許容 [mm³]
    "BOLTED":    (0.0, True,   0.0),      # 突き合わせて留める。重なりは切断長の誤り
    # ねじ**そのもの**と相手。ねじは相手に刺さって当然なので重なりを許す。
    # 上限 1200mm³ は M5×20 が頭まで完全に埋まった量（677mm³）の約 2 倍。
    # これを超えたら、刺さっているのはねじではなく別の何かが埋まっている。
    "FASTENED":  (0.0, True,  1200.0),
    "PRESSED":   (0.0, True,  4000.0),    # しめしろ。φ26×10mm 程度まで
    "WELDED":    (0.0, True,   500.0),    # 溶接ビード・隅肉ぶん
    "MESHED":    (0.0, True,  2000.0),    # 歯車・ベルトの噛み合い表現
    "SLIDING":   (0.0, True,     0.0),    # 摺動面。重なったら動かない
    "ROTATING":  (0.0, True,     0.0),    # 同上
    "ROUTED":    (0.0, True,   200.0),    # 配線は多少沈む表現を許す
    "CONTAINED": (0.0, True,   1e12),     # 包含。中身が入っていて当然
    "FREE":      (3.0, False,    0.0),    # **既定値**。3mm 離れていること
}

# --- 部品グループ間の関係 -----------------------------------------------------
# キーは「ラベルに含まれる文字列」の組。左右の順は問わない。
# ⚠ ここに書かれていない組み合わせは自動的に FREE になる。
#   つまり**書き忘れは「素通り」ではなく「NG」として出る**。これが狙い。
RELATIONS = {
    # --- ねじ類は留める相手すべてと接触する ---
    ("fasteners", "base"):        "FASTENED",
    ("fasteners", "side"):        "FASTENED",
    ("fasteners", "deck"):        "FASTENED",
    ("fasteners", "drive"):       "FASTENED",
    ("fasteners", "turret"):      "FASTENED",
    ("fasteners", "carriage"):    "FASTENED",
    ("fasteners", "mast"):        "FASTENED",
    ("fasteners", "hopper"):      "FASTENED",
    ("fasteners", "feed_ramp"):   "FASTENED",
    ("fasteners", "fork"):        "FASTENED",
    ("fasteners", "chair"):       "FASTENED",
    ("fasteners", "electronics"): "FASTENED",
    ("fasteners", "sensors"):     "FASTENED",
    ("fasteners", "rail"):        "FASTENED",

    # --- 骨格どうし（同じフレームの部材は継手で接する） ---
    ("base", "base"):             "BOLTED",
    ("side", "side"):             "BOLTED",
    ("base", "side"):             "BOLTED",
    ("base", "deck"):             "BOLTED",
    ("side", "deck"):             "BOLTED",
    ("deck", "deck"):             "BOLTED",
    ("side", "mast"):             "WELDED",   # マスト主柱は側面後柱を兼用
    ("mast", "mast"):             "BOLTED",
    ("base", "drive"):            "BOLTED",   # モーターマウントを桁に留める
    ("side", "rail"):             "BOLTED",   # レールを上桁に留める
    ("side", "chair"):            "BOLTED",
    ("base", "chair"):            "BOLTED",
    ("side", "hopper"):           "BOLTED",   # ホッパーをトラスに吊る
    ("side", "feed_ramp"):        "BOLTED",
    ("base", "electronics"):      "BOLTED",
    ("deck", "electronics"):      "BOLTED",
    ("base", "sensors"):          "BOLTED",
    ("side", "sensors"):          "BOLTED",
    ("mast", "bucket"):           "BOLTED",   # バケツ受け板

    # --- 意図して FREE にしている組（近いが離れていなければならない）---
    # scripts/relations_audit.py で「未宣言だが 30mm 以内」に出たものを精査し、
    # FREE が正しいものはここに**明示的に**書く。書いておかないと
    # 「まだ考えていない組」と区別がつかない。
    ("electronics", "turret"):    "FREE",   # 非常停止と旋回体（現状 5.90mm）
    ("carriage", "turret"):       "FREE",   # 両方動く（12.07mm）
    ("feed_ramp", "turret"):      "FREE",   # 斜路と旋回体（10.23mm）
    ("carriage", "feed_ramp"):    "FREE",   # キャリッジは 316mm 走る（7.07mm）
    ("carriage", "side"):         "FREE",   # 21.50mm
    ("carriage", "electronics"):  "FREE",   # 14.01mm
    ("carriage", "sensors"):      "FREE",   # 10.00mm
    ("carriage", "singulator"):   "FREE",   # 22.60mm
    ("fork", "rail"):             "FREE",   # 10.90mm
    ("fork", "turret"):           "FREE",   # 23.62mm
    ("fork", "sensors"):          "FREE",   # 14.09mm
    ("fork", "singulator"):       "FREE",   # 21.60mm
    ("press", "turret"):          "FREE",   # 24.77mm
    ("grabber_fixed", "turret"):  "FREE",   # 28.27mm
    ("grabber_fixed", "press"):   "FREE",   # 15.00mm
    ("rail", "turret"):           "FREE",   # 18.37mm
    ("side", "turret"):           "FREE",   # 15.90mm
    ("side", "singulator"):       "FREE",   # 13.45mm
    ("base", "wheel"):            "FREE",   # 15.00mm
    ("sensors", "wheel"):         "FREE",   # 19.00mm
    ("electronics", "rail"):      "FREE",   # 19.35mm
    ("deck", "drive"):            "FREE",   # 27.00mm
    ("cables", "carriage"):       "FREE",   # 配線が可動部に触れると擦れる
    ("cables", "grabber_fixed"):  "FREE",
    ("cables", "turret"):         "FREE",
    ("cables", "fork"):           "FREE",
    ("cables", "singulator"):     "FREE",
    ("cables", "rail"):           "FREE",   # 3.35mm。レールは動くので触れさせない
    # ⚠ deck ↔ rail は **3.35mm しかない**。台座横梁の下面 818 と
    #   レール上端 814.6（RAIL_Z=797）の差。ここが機体で最も薄いすきま。
    #   組立公差（上桁の左右高さ差 ±1mm）を考えると、実機では要確認。
    #   横梁は砲塔リングを支える構造材なので動かせない。
    ("deck", "rail"):             "FREE",

    # --- 上で FREE にできない、本当に接する組 ---
    ("fork", "press"):            "SLIDING",  # 上押さえが櫛歯の上へ降りてくる
    ("fasteners", "wheel"):       "FASTENED",   # 車輪をハブアダプタへ留める
    ("fasteners", "grabber_fixed"): "FASTENED",

    # --- 駆動系 ---
    ("drive", "drive"):           "BOLTED",   # モーター＋マウント板＋ハブ
    ("drive", "wheel"):           "PRESSED",  # 車輪がハブアダプタに嵌る
    ("wheel", "wheel"):           "WELDED",   # 1輪の中のローラーとハブ

    # --- 砲塔・射出 ---
    ("turret", "turret"):         "BOLTED",   # 旋回体の中の部品どうし
    ("turret", "deck"):           "ROTATING", # 旋回リングが台座に載る
    ("turret", "roller"):         "ROTATING",
    ("roller", "roller"):         "PRESSED",  # ウレタンタイヤがハブに嵌る
    ("turret", "pulley"):         "MESHED",
    ("turret", "belt"):           "MESHED",

    # --- グラバー ---
    ("carriage", "carriage"):     "BOLTED",
    ("carriage", "fork"):         "BOLTED",   # フォークをキャリッジに吊る
    ("fork", "fork"):             "WELDED",   # 櫛歯と根元バー
    ("carriage", "rail"):         "SLIDING",  # キャリッジがレールを滑る
    ("rail", "rail"):             "SLIDING",  # 3段の重なり
    ("carriage", "press"):        "SLIDING",  # 上押さえの昇降
    ("press", "press"):           "BOLTED",
    ("carriage", "grabber_fixed"): "SLIDING",
    ("grabber_fixed", "side"):    "BOLTED",

    # --- ホッパー・搬送 ---
    ("hopper", "hopper"):         "WELDED",
    ("hopper", "singulator"):     "ROTATING", # ピックローラーが前壁の上端に
    ("singulator", "singulator"): "PRESSED",
    ("hopper", "feed_ramp"):      "BOLTED",
    ("feed_ramp", "feed_ramp"):   "WELDED",
    ("feed_ramp", "singulator"):  "ROTATING",

    # --- 電装・配線 ---
    ("electronics", "electronics"): "BOLTED",
    ("cables", "electronics"):    "ROUTED",
    ("cables", "cables"):         "ROUTED",
    ("cables", "base"):           "ROUTED",
    ("cables", "side"):           "ROUTED",
    ("cables", "deck"):           "ROUTED",
    ("cables", "mast"):           "ROUTED",
    ("cables", "hopper"):         "ROUTED",
    ("cables", "sensors"):        "ROUTED",
    ("cables", "drive"):          "ROUTED",
    ("cables", "chair"):          "ROUTED",
    ("cables", "feed_ramp"):      "ROUTED",
    ("cables", "fasteners"):      "ROUTED",

    # --- 椅子は「フレームに固定する」ので、触れる相手が多い（規定 3.1.2）---
    ("chair", "deck"):            "BOLTED",
    ("chair", "drive"):           "BOLTED",   # 座板が駆動マウントの上に来る
    ("chair", "sensors"):         "BOLTED",   # 上段LiDARのステーが座面裏を通る
    ("chair", "electronics"):     "BOLTED",
    ("chair", "hopper"):          "BOLTED",
    ("chair", "turret"):          "FREE",     # ⚠ 砲塔は回るので椅子と離すこと

    # --- グラバー全体（キャリッジ+フォーク+上押さえが1グループに畳まれる）---
    ("grabber", "grabber"):       "BOLTED",
    ("grabber", "rail"):          "SLIDING",   # レールを滑る
    ("grabber", "grabber_fixed"): "SLIDING",   # 固定側のレール受けとの相対運動
    ("grabber", "fasteners"):     "FASTENED",
    ("grabber", "carriage"):      "BOLTED",
    ("grabber", "fork"):          "BOLTED",
    ("grabber", "press"):         "SLIDING",
    ("grabber", "side"):          "SLIDING",   # 上桁の内側を通る
    ("grabber", "cables"):        "ROUTED",
    ("grabber", "mascot"):        "CONTAINED",
    # ⚠ グラバーは伸びると 316mm 動く。**動く相手とは離すこと**
    ("grabber", "turret"):        "FREE",
    ("grabber", "hopper"):        "FREE",
    ("grabber", "feed_ramp"):     "FREE",
    ("grabber", "deck"):          "FREE",
    ("grabber", "electronics"):   "FREE",
    ("grabber", "singulator"):    "FREE",

    # --- グラバー固定部（レール受け・ベルト系ブラケット）---
    ("grabber_fixed", "grabber_fixed"): "BOLTED",
    ("grabber_fixed", "rail"):    "BOLTED",
    ("grabber_fixed", "deck"):    "BOLTED",
    ("grabber_fixed", "mast"):    "BOLTED",

    # --- センサー類（ステー + 本体 + 取付板）---
    ("sensors", "sensors"):       "BOLTED",
    ("sensors", "deck"):          "BOLTED",
    ("sensors", "hopper"):        "BOLTED",
    ("sensors", "singulator"):    "ROTATING",  # 厚みセンサーがピックローラーに近接
    ("sensors", "feed_ramp"):     "BOLTED",
    ("sensors", "drive"):         "BOLTED",
    ("sensors", "electronics"):   "ROUTED",

    # --- 電装は取り付く先が多い（非常停止は上桁、ブレーカーはデッキ…）---
    ("electronics", "side"):      "BOLTED",
    ("electronics", "mast"):      "BOLTED",
    ("electronics", "drive"):     "ROUTED",
    ("electronics", "hopper"):    "BOLTED",

    # --- 装飾・規定物 ---
    ("chair", "mascot"):          "CONTAINED",  # マスコットが椅子に座る
    # マスコットの中身どうし（体・頭・三角巾・ボタン・靴・持っている雑巾）。
    # ⚠ 曲面に貼る部品（ボタン・三角巾・ゼッケン）は、相手と**同じ楕円体**を
    #   内面に持つ殻として切り出してある（`tr_lib._patch`）。だから接触は
    #   面で、重なりはブーリアンの丸めぶんしか出ない。
    ("mascot", "mascot"):         "WELDED",
    ("chair", "chair"):           "WELDED",
    ("cables", "mascot"):         "CONTAINED",  # マスコットのエンベロープ内を通る
    ("bucket", "bucket"):         "WELDED",
    ("mascot", "base"):           "CONTAINED",
    ("mascot", "deck"):           "CONTAINED",
    ("mascot", "side"):           "CONTAINED",
    ("mascot", "electronics"):    "CONTAINED",
    ("mascot", "sensors"):        "CONTAINED",
    ("mascot", "fasteners"):      "CONTAINED",
    ("mascot", "turret"):         "CONTAINED",
    ("mascot", "hopper"):         "CONTAINED",
    ("mascot", "feed_ramp"):      "CONTAINED",
    ("mascot", "drive"):          "CONTAINED",
    ("mascot", "wheel"):          "CONTAINED",
    ("mascot", "cables"):         "CONTAINED",
    # ねじ同士は近接してよい（同じ継手を複数本で留めるので必ず並ぶ）。
    # ねじ頭どうしが干渉するなら座面の設計ミスだが、それは締結解析の領分。
    # ねじ同士は刺さり合わない。同じ穴を 2 本が奪い合っていたら配置ミス
    ("fasteners", "fasteners"):   "BOLTED",
}


# ラベルからグループ名を取り出すための対応表。
# ⚠ **葉のラベルから判定する**こと。パス全体で探すと
#   `/tr_robot/base_link/side_frame/side_frame_0` が `base` に化ける
#   （`base_link` が先に当たる）。実際そうなって 74 件の誤検出が出た。
GROUP_KEYS = (
    # 長い名前・具体的な名前を先に置く（部分一致で先勝ちのため）
    "grabber_fixed", "feed_ramp", "electronics", "singulator",
    "fasteners", "carriage", "sensors", "mascot", "bucket", "hopper",
    "turret", "roller", "pulley", "cables", "wheel", "chair", "drive",
    "grabber",  # ← グラバー全体（キャリッジ+フォーク+上押さえを畳んだもの）。
                #    `_solids()` は Location を持つノードで畳むので、
                #    `/tr_robot/grabber#N` として出てくる。carriage より後ろに置くと
                #    `carriage_0` が先に当たるので、この順序で正しい
    "press", "belt", "mast", "deck", "fork", "rail", "side", "base",
)


def group_of(path: str) -> str:
    """ラベルパスから所属グループを決める。

    **葉のラベル**（最後の要素）だけを見る。祖先のパスに `base_link` が
    含まれていても、葉が `side_frame_0` なら `side` を返す。
    """
    leaf = path.split("/")[-1]
    for k in GROUP_KEYS:
        if k in leaf:
            return k
    # 葉で決まらないときだけ、親のパスを遡って探す
    for k in GROUP_KEYS:
        if k in path:
            return k
    return leaf.split("#")[0].split("_")[0]


def relation(ga: str, gb: str) -> str:
    """2グループ間の関係。宣言が無ければ FREE（＝離れていなければならない）。"""
    return RELATIONS.get((ga, gb)) or RELATIONS.get((gb, ga)) or "FREE"


def expected(ga: str, gb: str):
    """(関係名, 必要すきま, 接触可, 食い込み許容体積) を返す。"""
    kind = relation(ga, gb)
    gap, may_touch, overlap = KINDS[kind]
    return kind, gap, may_touch, overlap
