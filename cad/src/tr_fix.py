"""部品ごとの**固定関係**の台帳.

なぜグループ間の関係では駄目だったか
--------------------------------------
`tr_relations.py` は「グループ A とグループ B は BOLTED」という粒度だった。
これだと

  * `("base","base"): BOLTED` と書くだけで、ベース骨格 68 個の**どの組**が
    留まっているのかは何も言っていない。実際には全組が留まるわけがない
  * 「宣言が無い組は FREE（3mm 離す）」という既定値は、**離れてさえいれば通る**。
    空中に浮いた部品ほど通りやすい、という逆立ちした基準になっていた
  * 「接触してよい」関係にした途端、その組は**検査から外れた**

結果、STEP には浮いた部品と食い込んだ部品が同居していた。

ここで持つもの
---------------
部品 1 個ごとに

    その部品は **どの部品に** **どうやって** 留まっているか

を宣言する。宣言は形状と同じ場所（組立コード）に書くので、部品を足したのに
留め方を書かなければ、その時点で検査に落ちる。

これで 3 つの検査が同時に成立する:

    1. 固定先が無い部品 → NG（浮き。距離ではなく**宣言の有無**で判定）
    2. 宣言した固定先と接触していない → NG（図と宣言の食い違い）
    3. **宣言していない部品どうしが接触している** → NG（意図しない干渉）

3 が最も効く。従来は「離れているべき組」を人が列挙していたので、
列挙漏れがそのまま素通りだった。ここでは**接触しているほうを列挙させる**ので、
書いていない接触はすべて異常になる。
"""

from __future__ import annotations

# --- 固定方法 ---------------------------------------------------------------
#   接触必須   … 宣言した相手と実体が接していなければ NG
#   食い込み許容 [mm³]
HOW = {
    "BOLT":     (True, 0.0,     "ボルト・ねじで締結（突き合わせ）"),
    "TSLOT":    (True, 0.0,     "アルミフレームの溝ナット締結"),
    "BRACKET":  (True, 0.0,     "ブラケット経由の締結"),
    "WELD":     (True, 300.0,   "溶接・一体成形（ビードぶんの重なりを許す）"),
    "RIVET":    (True, 0.0,     "リベット・かしめ"),
    # ⚠ 許容は**実測の 2 倍**で置く。4,000 のように大きく取ると、
    #   「穴を開け忘れた」「内径が軸径と違う」といった描き間違いが宣言の陰に
    #   隠れる。実際 hinge_shaft ↔ fork_root が 3,646mm³、プーリの内径 8 に
    #   軸 φ10 で 452mm³ ×4 隠れていた（本物のしめしろは φ8×28 で 0.007mm³）。
    #   いま 20mm³ を超える PRESS の組は無い。
    "PRESS":    (True, 300.0,   "圧入・インロー（しめしろを許す）"),
    # モーターの**出力軸**に固定する。軸そのものが関節なので、
    # ハウジング側と回転側でリンクが分かれるのが正しい。
    "SHAFT":    (True, 300.0,   "出力軸への固定（この結合が関節そのもの）"),
    "SCREW_IN": (True, 600.0,   "ねじ本体が相手に刺さる（実測最大 281）"),
    # ⚠ ねじが**貫通穴を通る**のは干渉ではない。BOLT（座面を要求する）で
    #   宣言すると「座面が足りない」「板を貫いている」と二重に出る。
    #   ねじの座面は頭が当たる相手にだけ要る。通り抜ける相手は THRU。
    "THRU":     (True, 400.0,   "貫通穴を通す（穴あけ前提・実測最大 170）"),
    # ⚠ TSLOT は「板を溝ナットで押出材に留める」宣言なので、板と押出材は
    #   面で当たる（食い込み 0）。**ねじ自身**が溝ナットに刺さるのは別で、
    #   ねじ 1 本ぶん材料の中に入る。混ぜると板の食い込みを見逃す。
    "TNUT":     (True, 400.0,   "ねじが溝ナットに刺さる（実測最大 177）"),
    "SLIDE":    (True, 0.0,     "摺動面。接するが重なってはいけない"),
    # ⚠ **接触を要求してはいけない唯一の締結。** レベリング座は「ねじで
    #   引き、ばねで押し戻す」ので、2 枚の板のあいだは*空いているのが
    #   正しい*（すきまが調整代そのもの）。BOLT で宣言すると
    #   「宣言した固定先と接していない」が必ず出るし、逆に接するように
    #   描くと調整代が 0 になる＝機構が無いのと同じになる。
    "ADJUST":   (False, 0.0,    "ねじとばねで浮かせて姿勢を出す（**離れているのが正**）"),
    "ROTATE":   (True, 0.0,     "軸受・転がり接触"),
    # ⚠ **3,200 は「ベルトを角パイプ形で描いている」前提の値だった。**
    #   ベルトをプーリに巻き付くスタジアム形で描き直したら、重なりは
    #   1 組 95mm³（歯の噛み合いぶん）まで落ちた。3,200 のままだと、
    #   平板や角枠で描いた誤りが 20,849mm³ 分そのまま隠れる（実際に隠れていた）。
    #   実測の 2 倍で 300 に締める。
    "MESH":     (True, 300.0,   "歯車・ベルトの噛み合い（歯 1 山ぶんの重なり）"),
    # 配線は結束バンド・配線サドル・アセテートテープで留める。
    # ねじ穴を用意する必要はないが、**どの部材に沿わせるかは決まっていなければ**
    # いけない（沿う相手が無い配線は、実機では垂れて可動部に巻き込まれる）。
    "ROUTE":    (True, 200.0,   "配線を結束バンド／サドル／テープで沿わせる"),
    # ⚠ 配線には**沿わせる**と**差し込む**の 2 種がある。
    #   コネクタは機器へ実際に入り込むので、ROUTE の許容（200）では足りない。
    #   逆に許容を上げると、機器を貫いている配線まで見逃す。分けて宣言する。
    "CONNECT":  (True, 600.0,   "コネクタで機器に差し込む（実測最大 264）"),
    "BUNDLE":   (True, 800.0,   "配線どうしを結束バンドで束ねる（実測最大 371）"),
    "SIT":      (True, 0.0,     "載せて固定（バケツ・椅子など）"),
    # ⚠ ベルトクランプは歯付きベルトを**噛み込んで**掴む。ゴムは潰れる
    #   ので、剛体どうしと同じ「食い込み 0」では検査できない。
    #   ただし 1,500 は大きすぎた。「挟む板 2 枚」と書いて 1 本の柱でベルトを
    #   貫いていた 1,350mm³ が隠れていた。挟む形に直したら 20mm³ 未満。
    "CLAMP":    (True, 300.0,   "バンド・面ファスナー・ベルトクランプで掴む"),
    "CONTAIN":  (True, 1e12,    "中に入る（マスコット・雑巾）"),
    # マスコットは布と発泡でできていて、ねじも溶接も使わない。
    # ⚠ WELD（一体成形）と混ぜないこと。300mm³ まで重なってよい理由が
    #   「縫い代とフェルトの厚み」で、溶接ビードとは別物。分けておかないと、
    #   どちらの許容を締めればよいか後から分からなくなる。
    "SEW":      (True, 300.0,   "縫い付け・接着（フェルト／発泡）"),
}

# 部品名 -> [(固定先, 方法, 個数, 備考), ...]
FIXINGS: dict[str, list[tuple[str, str, int, str]]] = {}

# 部品名 -> その部品の bounding box（組立中に put() が記録する）。
# **座標を手で書くのをやめて、相手の面から決める**ために使う。
# 「だいたいこの辺」で書いた数字は、相手を動かした瞬間に浮くか食い込む。
# 実際、板の位置を手で書いていたせいで
#   ・レール取付板が上桁から 3.4mm 浮く
#   ・キャリッジ側金具がインナーレールに 9,474mm³ 食い込む
#   ・ブラケットが長手材の無い側に置かれて角の線でしか当たらない
# といった不具合が繰り返し出た。面を参照すれば、相手が動いても追従する。
BOX: dict[str, object] = {}


def face(name: str, axis: str, side: int) -> float:
    """`name` の bbox の面座標を返す。axis は "x"/"y"/"z"、side は +1/-1。"""
    b = BOX.get(name)
    if b is None:
        raise KeyError(f"{name} はまだ組み立てられていない（面を参照する前に置くこと）")
    lo, hi = ((b.min.X, b.max.X), (b.min.Y, b.max.Y),
              (b.min.Z, b.max.Z))["xyz".index(axis)]
    return hi if side > 0 else lo


def touch_point(name: str, axis: str, side: int, other=(0.0, 0.0), clear: float = 0.0):
    """`name` の面に**触れる位置**の (x, y, z) を返す。配線の終端に使う。

    axis で指定した軸は面の座標（+clear だけ離す）、残る 2 軸は `other` の値。
    配線の終点を手で書くと、相手の中に埋まるか宙に浮くかのどちらかになる。
    実際「モーター後端に結束」と宣言した 4 本が本体に 1,060mm³ 埋まっていた。
    """
    v = face(name, axis, side) + side * clear
    i = "xyz".index(axis)
    out = [other[0], other[1]]
    out.insert(i, v)
    return tuple(out)


def seat_on(name: str, axis: str, side: int, t: float) -> float:
    """`name` の面に厚み `t` の板を**載せた**ときの、板の中心座標。

    side=+1 なら面の外側（座標が増える側）に載る。
    """
    return face(name, axis, side) + side * t / 2.0

# 形状を持たない基準（床・競技用品など、ここから固定の連鎖が始まる点）
ROOTS: set[str] = set()

# 配線の折れ線。name -> (points, dia)
# ⚠ ケーブルには**最小曲げ半径**がある（外径の 4 倍が目安）。折れ線で
#   描いていると、節点で 90° 以上曲げても図の上では成立してしまう。
#   実機では被覆が潰れて芯線が切れるか、反発して可動部に押し出される。
#   節点の角度と、そこを丸めるのに必要な直線長を突き合わせる。
CABLE: dict[str, tuple] = {}

# ねじの**工具アクセス**。name -> (頭の中心 x,y,z, 工具を差す向き dx,dy,dz)
# ⚠ 締結を宣言できていても、六角レンチが入らなければ実機では組めない。
#   「設計上は留まっている」と「組み立てられる」は別の話。
#   組めない締結は、部品を作り直すか順序を変えるしかないので、
#   図面を出す前に気づく必要がある。
TOOL: dict[str, tuple] = {}

# 「接触してよいが固定関係ではない」組。**明示したものだけ**許す。
# 例: 可動部が動作範囲の端で当たる、雑巾がガイドに触れる。
TOUCH_OK: set[tuple[str, str]] = set()

# ねじ 1 本ごとの**規格**。name -> (種類, 呼び径, 長さ, 付属品の並び)
# ⚠ **員数は宣言（note）ではなく実体から数える。** note の "4-M4 PCD35" は
#   人が書いた文字列で、ねじを 1 本消しても 4 のまま残る。実際
#   「4-M4 + スペーサ」と注記して実体が 1 本も無い箇所が大半だった。
#   ここに載るのは `put_screw()` で**実体を置いたねじだけ**。宣言だけで
#   実体の無い締結は scripts/fastener_bom.py が別枠で数え、
#   「未実体」として一覧に出す。買う数を出すのはこの 2 つの合計。
FASTENER: dict[str, tuple] = {}

# **傾いた押出材の溝の芯**。name -> (軸上の 1 点 x,y,z, 軸方向 dx,dy,dz)
# ⚠ 押出材の溝は 1 面に 1 条、面の中心線を長手方向に走る。軸に平行な
#   部材なら bbox の中心線がそのまま溝の芯になるが、**斜めに寝た部材では
#   bbox が断面より太くなる**ので bbox からは出せない（`screw_place.ext_axes`
#   はそこで None を返し、呼ぶ側は面のどこにでもねじを散らしていた）。
#   出せないものは**置いた側が教える**。ここに載っている部材は、ねじが
#   芯に乗っているかを `scripts/screw_check.py` の D 項が検査する。
SLOT_AXIS: dict[str, tuple] = {}


def reset() -> None:
    FIXINGS.clear()
    ROOTS.clear()
    TOUCH_OK.clear()
    BOX.clear()
    TOOL.clear()
    CABLE.clear()
    FASTENER.clear()
    SLOT_AXIS.clear()


def snapshot() -> dict:
    """いまの固定宣言をまるごと控える。

    ⚠ 姿勢を変えても形の変わらない部分（車体・車輪）は作り直さずに
      使い回している。そのとき**宣言だけが再登録されない**と、
      固定の連鎖が切れて全部品が「浮き」になる。実際、掃引検査の
      2 姿勢目以降で 423 部品すべてが浮き判定になっていた。
      形をキャッシュするなら、宣言も一緒にキャッシュすること。
    """
    return {"fix": {k: list(v) for k, v in FIXINGS.items()},
            "roots": set(ROOTS), "touch": set(TOUCH_OK), "box": dict(BOX),
            "tool": dict(TOOL), "cable": dict(CABLE),
            "fastener": dict(FASTENER), "slot_axis": dict(SLOT_AXIS)}


def restore(snap: dict) -> None:
    for k, v in snap["fix"].items():
        FIXINGS.setdefault(k, []).extend(v)
    ROOTS.update(snap["roots"])
    TOUCH_OK.update(snap["touch"])
    BOX.update(snap["box"])
    TOOL.update(snap.get("tool", {}))
    CABLE.update(snap.get("cable", {}))
    FASTENER.update(snap.get("fastener", {}))
    SLOT_AXIS.update(snap.get("slot_axis", {}))


def fix(name: str, to, how, qty: int = 1, note: str = "") -> None:
    """`name` を `to` に `how` で固定する、と宣言する。

    `to` は 1 つでも複数でもよい（横材が両端で 2 本の長手材に留まる等）。
    `how` も相手ごとに変えてよい。**同じ部品でも相手ごとに留め方は違う**
    ので、配線を桁に「沿わせ」ながらモーターへ「差し込む」ような場合は
    `to=(桁, モーター), how=("ROUTE", "CONNECT")` と書く。
    """
    targets = (to,) if isinstance(to, str) else tuple(to)
    hows = (how,) * len(targets) if isinstance(how, str) else tuple(how)
    if not targets:
        raise ValueError(f"{name}: 固定先が空")
    if len(hows) != len(targets):
        raise ValueError(f"{name}: 固定先 {len(targets)} 個に対し方法 {len(hows)} 個")
    for h in hows:
        if h not in HOW:
            raise ValueError(f"未知の固定方法 {h!r}（{', '.join(HOW)} のいずれか）")
    FIXINGS.setdefault(name, []).extend(
        (t, h, qty, note) for t, h in zip(targets, hows))


def root(name: str) -> None:
    """固定の連鎖の起点（基準部材）として登録する。"""
    ROOTS.add(name)


def touch_ok(a: str, b: str, note: str = "") -> None:
    """固定ではないが接触してよい組。**書いたものだけ**許される。"""
    TOUCH_OK.add((a, b))
    TOUCH_OK.add((b, a))


def declared(a: str, b: str):
    """a と b のあいだに宣言があれば (方法, 食い込み許容) を返す。無ければ None。"""
    for src, dst in ((a, b), (b, a)):
        for t, how, _qty, _note in FIXINGS.get(src, ()):
            if t == dst:
                return how, HOW[how][1]
    if (a, b) in TOUCH_OK:
        return "TOUCH_OK", 0.0
    return None


def targets_of(name: str) -> list[str]:
    return [t for t, _h, _q, _n in FIXINGS.get(name, ())]


def unresolved(known: set[str]) -> list[tuple[str, str]]:
    """存在しない部品を固定先にしている宣言を返す（打ち間違い検出）。"""
    bad = []
    for name, lst in FIXINGS.items():
        for t, _h, _q, _n in lst:
            if t not in known and t not in ROOTS:
                bad.append((name, t))
    return bad


def depth_of(known: set[str]) -> dict[str, int]:
    """ROOTS から何段の締結を経て届くか（組立の段数）。

    段数は**公差の積み上がり**そのもの。1 段ごとに位置決めの誤差が
    足されるので、段が深い部品は実機で位置が出ない。
    治具で直接位置を決めるか、基準面を作り直すかの判断に使う。
    """
    # ⚠ **配線の宣言は段数の経路にしてはいけない。** 配線は位置決めをしない
    #   （束ねてある・沿わせてあるだけ）ので、公差は伝わらない。
    #   実際 ケーブルベアの帯を car_side_L へ ROUTE で宣言したとき、
    #   上押さえの段数が 16 → 14 に「改善」した。配線を通る近道ができた
    #   だけで、組立の公差は何も変わっていない。
    NO_STACK = ("ROUTE", "BUNDLE", "CONNECT")
    adj: dict[str, set[str]] = {}
    for name, lst in FIXINGS.items():
        for t, h, _q, _n in lst:
            if h in NO_STACK:
                continue
            adj.setdefault(name, set()).add(t)
            adj.setdefault(t, set()).add(name)
    out = {r: 0 for r in ROOTS}
    frontier = list(ROOTS)
    d = 0
    while frontier:
        d += 1
        nxt = []
        for cur in frontier:
            for n in adj.get(cur, ()):
                if n not in out:
                    out[n] = d
                    nxt.append(n)
        frontier = nxt
    return {k: v for k, v in out.items() if k in known}


def reachable(known: set[str]) -> set[str]:
    """ROOTS から固定の宣言をたどって到達できる部品。

    到達できない部品は、実機では**どこにも留まっていない**。
    接触しているかどうかとは別の話で、こちらのほうが厳しい。
    """
    adj: dict[str, set[str]] = {}
    for name, lst in FIXINGS.items():
        for t, _h, _q, _n in lst:
            adj.setdefault(name, set()).add(t)
            adj.setdefault(t, set()).add(name)
    seen = set(ROOTS)
    stack = list(ROOTS)
    while stack:
        cur = stack.pop()
        for nxt in adj.get(cur, ()):
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return seen & known
