"""TR — 部品ジェネレータ・ライブラリ.

すべての部品は「取付基準（データム）を原点に置く」規約で作る:

* アルミフレーム : 長手方向の中心を原点、長手を +X に取る
* モーター       : 取付面を z=0（フランジ前面）、出力軸を +Z に取る
* 板金/切削板    : 板中心を原点、板厚を Z に取る
* 車輪           : 回転軸を Z、車輪中心を原点に取る

質量は MassLedger に逐次記録する。購入品はカタログ値、製作品は
体積×密度（tr_params.DENSITY）で算出する。
"""

from __future__ import annotations

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))


import copy as _copy
import functools as _functools
from dataclasses import dataclass, field
from math import atan2, cos, degrees, hypot, radians, sin, sqrt

from build123d import (
    Align,
    Color,
    Compound,
    Axis,
    Box,
    Circle,
    Cone,
    Cylinder,
    Location,
    Plane,
    Pos,
    Rectangle,
    Rot,
    Sphere,
    Torus,
    extrude,
    import_step,
    scale,
)

import tr_params as P

MM = 1.0

# ---------------------------------------------------------------------------
# 材質別カラー（STEP / GLB / ビューアで材質が一目でわかるようにする）
#   ピンク = 3Dプリント部品（PETG）
# ---------------------------------------------------------------------------
MAT_COLOR = {
    "PETG": (0.98, 0.42, 0.66),      # ピンク: 3Dプリント
    "A5052": (0.78, 0.80, 0.83),     # 明るい銀: CNC切削アルミ板
    "A6005C": (0.62, 0.66, 0.70),    # ややくすんだ銀: アルミフレーム（アルマイト）
    "ADC12": (0.46, 0.49, 0.53),     # 濃い銀: ダイカストブラケット
    "SUS304": (0.55, 0.58, 0.62),    # 青みの銀: ステンレス板金
    "STEEL": (0.35, 0.37, 0.40),     # 濃灰: スチール（スライドレール）
    "URETHANE": (0.25, 0.22, 0.24),  # 黒褐色: ウレタンローラー
    "RUBBER": (0.18, 0.35, 0.72),    # 青: 輪ゴム（消耗品。試合ごとに掛け替える）
    "SILICONE": (0.90, 0.35, 0.20),  # 朱: シリコン（リタードパッド＝消耗品）
    "POM": (0.95, 0.95, 0.92),       # 白: POM（無給油ブッシュ）
    "PC": (0.72, 0.86, 0.92),        # 淡い水色: ポリカーボネート（透明部品）
    "SCREEN": (0.13, 0.22, 0.30),    # 暗い青緑: 表示器の画面（点いている面）
    "PP_DANPLA": (0.86, 0.90, 0.84),  # 乳白: プラダン（PP 中空板）
    "TEKCELL": (0.93, 0.89, 0.76),    # 生成り: テクセル（PP ハニカム板）
    "PLYWOOD": (0.80, 0.66, 0.44),   # 木色: 椅子の座板
    "MOTOR": (0.20, 0.20, 0.22),     # 黒: DJI モーター本体
    "MOTOR_SHAFT": (0.85, 0.72, 0.30),  # 金: 出力軸・フランジ
    "PCB": (0.10, 0.45, 0.25),       # 基板の緑
    "BATTERY": (0.15, 0.18, 0.35),   # 紺: LiPo
    "SENSOR": (0.30, 0.32, 0.36),    # LiDAR/カメラ
    "ESTOP": (0.85, 0.12, 0.12),     # 赤: 非常停止
    "MASCOT": (0.95, 0.85, 0.55),    # マスコットのエンベロープ／体のフェルト
    "MASCOT_SUIT": (0.20, 0.36, 0.62),  # つなぎ（藍）
    "MASCOT_TRIM": (0.85, 0.26, 0.28),  # 三角巾・ボタン（赤）
    "MASCOT_RAG": (0.70, 0.73, 0.68),   # マスコットが持つ雑巾
    "MASCOT_DARK": (0.14, 0.14, 0.17),  # 靴・瞳
    "SPONGE": (0.94, 0.90, 0.86),    # 上押さえのスポンジパッド
    "RUBBER": (0.18, 0.18, 0.18),    # ベルト
    "CABLE": (0.12, 0.12, 0.14),     # 配線束
}


def mat(shape, material: str):
    """材質に応じた色を設定して返す。"""
    rgb = MAT_COLOR.get(material)
    if rgb is not None:
        try:
            shape.color = Color(*rgb)
        except Exception:
            pass
    return shape


# ---------------------------------------------------------------------------
# 同じ形は 1 度しか作らない（プロトタイプの使い回し）
# ---------------------------------------------------------------------------
# ねじ・ブラケット・押出材・車輪は、**同じ引数で何百回も呼ばれる**。
# 実測（build1 41.6 秒の内訳）:
#     mecanum_wheel  8.16s（4 回 = 2 種類）
#     screw_tnut     3.92s / screw 2.42s（数百本、規格は十数通り）
#     bracket        3.23s（46 個、寸法は 1〜2 通り）
#     topo_plate     2.60s（同じ板を姿勢ごとに作り直す）
# 形はブーリアンで作るので 1 個あたり数十ms〜1.7秒かかるが、
# **引数が同じなら結果は必ず同じ**。1 度作って複製すればよい。
#
# ⚠ **プロトタイプそのものを返してはいけない。** build123d の Shape は
#   `.label` / `.color` と木の親子（children / topo_parent）を Python 側の
#   属性で持つ。同じオブジェクトを 2 か所へ置くと、
#     * put() の label_shape が前に付けた名前を上書きし、
#     * Compound(children=[...]) に入れた時点で**親が付け替わって元の木が壊れる**
#   （tr_assembly.build() が「ソリッド数が変わった」で止めるあの事故）。
#   `copy.copy` は TShape（実体の幾何）だけ共有した**新しいラッパ**を返す。
#   build123d 自身が「多数の締結具を持つアセンブリ向け」と書いている使い方で、
#   幾何を作り直さないので M5 ねじ 1 本あたり 16.6ms → 0.28ms（60 倍）。
def proto_cache(fn):
    """引数が同じなら形を作り直さず、幾何を共有した複製を返す。

    ⚠ 引数に**形状や可変オブジェクトを取る関数には掛けない**こと
      （`lighten(part, …)` のように「渡された板に穴をあける」ものは、
      同じ引数でも相手が違えば結果が違う）。掛けてよいのは
      **数値・文字列だけを引数に取り、毎回同じ形を返す**関数だけ。
    """
    store: dict = {}

    @_functools.wraps(fn)
    def wrapped(*args, **kwargs):
        key = (args, tuple(sorted(kwargs.items())))
        got = store.get(key)
        if got is None:
            got = store[key] = fn(*args, **kwargs)
        return _copy.copy(got)

    wrapped.cache_clear = store.clear
    wrapped.cache_size = lambda: len(store)
    return wrapped


# ---------------------------------------------------------------------------
# ブーリアンを「まとめて 1 回」にするのは**やめた**（測った結果の記録）
# ---------------------------------------------------------------------------
# `part - [穴, 穴, …]` のように道具を並べて 1 回で渡すと、逐次に引くより
# 速くなる──というのは**矩形の板でだけ**成り立つ話だった。
#     矩形 400×600 に φ40 の穴 28 個 … 逐次 1216ms → 一括 165ms（7.4 倍）
# ところが実物の板（`topo_plate` が読む最適化輪郭）で測ると逆転する:
#     穴  8 個 … 逐次   758ms / 一括   519ms （一括が 1.46 倍速い）
#     穴 24 個 … 逐次  3790ms / 一括  5548ms （一括が 0.68 倍）
#     穴 48 個 … 逐次  5787ms / 一括 27257ms （一括が 0.21 倍＝5 倍遅い）
# OCCT は 1 回のブーリアンに渡した**道具どうしの交差も全部解く**ので、
# 道具の数に対して二次で効いてくる。この設計の板は最適化輪郭で穴も数十個
# あるほうが普通なので、一括にすると遅くなる側に必ず落ちる。
# （体積はどちらでも 1e-9 まで一致した。速さだけの話。）
#
# → 高速化は「ブーリアンの回し方」ではなく「**同じ形を作り直さない**」
#   （`proto_cache`）でやる。そちらは形が 1 ビットも変わらない。


CTR = (Align.CENTER, Align.CENTER, Align.CENTER)
BASE = (Align.CENTER, Align.CENTER, Align.MIN)


# ---------------------------------------------------------------------------
# 質量台帳
# ---------------------------------------------------------------------------
@dataclass
class MassItem:
    label: str
    qty: int
    unit_kg: float
    source: str
    group: str


@dataclass
class MassLedger:
    items: list[MassItem] = field(default_factory=list)

    def add(self, label: str, unit_kg: float, source: str, group: str, qty: int = 1) -> None:
        self.items.append(MassItem(label, qty, unit_kg, source, group))

    def add_solid(self, label: str, shape, material: str, group: str, qty: int = 1) -> None:
        """体積×密度で質量を記録する（製作品用）。"""
        grams = shape.volume * P.DENSITY[material]
        self.add(label, grams / 1000.0, f"{material} 体積計算", group, qty)

    @property
    def total_kg(self) -> float:
        return sum(i.qty * i.unit_kg for i in self.items)

    def by_group(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for i in self.items:
            out[i.group] = out.get(i.group, 0.0) + i.qty * i.unit_kg
        return out


LEDGER = MassLedger()


# ---------------------------------------------------------------------------
# アルミフレーム HFS5-2020
# ---------------------------------------------------------------------------
VENDOR_EXT = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                          "..", "vendor", "misumi", "HFS5-2020-840.stp")


def _ext_profile():
    """HFS5-2020 の断面（XY平面、20×20、4面溝、中心穴φ4.3）。

    **MISUMI 公式 STEP があればその端面を使う。** 溝の実形状はカタログの
    規格表に載っていないので、手で起こすと必ずずれる。実際、推定で作った
    プロファイルは断面積 195.5mm² で、公式の 182.8mm²（カタログ 183）に対して
    **6.9% 太かった**（溝を 12.7mm² 彫り足りていなかった）。

    公式データが無い環境でも動くよう、フォールバックは残す。
    """
    if _os.path.exists(VENDOR_EXT):
        try:
            solid = import_step(VENDOR_EXT)
            # 押し出し方向の端面（面積が断面積に一致する平面）を取る
            faces = sorted(solid.faces().filter_by(Plane.YZ), key=lambda f: f.center().X)
            if faces:
                return Plane.YZ.to_local_coords(faces[0])
        except Exception:
            pass
    # --- フォールバック: 規格表からの推定（断面積が 6.9% 過大になる） ---
    half = P.EXT_W / 2
    sk = Rectangle(P.EXT_W, P.EXT_W)
    for ang in (0, 90, 180, 270):
        c, s = cos(radians(ang)), sin(radians(ang))
        # 溝口 6mm 幅 × 深さ4mm
        mouth = Pos(c * (half - 2.0), s * (half - 2.0)) * Rot(0, 0, ang) * Rectangle(4.0, 6.0)
        # 溝底チャンバ 11mm 幅 × 深さ2.5mm
        cham = Pos(c * (half - 5.25), s * (half - 5.25)) * Rot(0, 0, ang) * Rectangle(2.5, 11.0)
        sk = sk - mouth - cham
    sk = sk - Circle(4.3 / 2)
    return sk


_EXT_FACE = None


# 直前に作った形が押出材か。`put()` が拾って「名前が EXT_PREFIX に載っているか」を
# 検査する。⚠ 載せ忘れると `screw_place` が中実材として扱い、**2020 の中空に
#   六角ナットを入れる**締結が図に出る（`disp_beam` が実際そうなっていた）。
LAST_EXT = False


def ext2020(length: float):
    """HFS5-2020（押出材）。中身は `_ext2020` のキャッシュ。

    ⚠ 印を付けるのは**キャッシュの外**。`proto_cache` の中で立てると、
      2 本目以降は関数本体が呼ばれないので印が付かない。
    """
    global LAST_EXT
    LAST_EXT = True
    return _ext2020(length)


@proto_cache
def _ext2020(length: float):
    """HFS5-2020 を長手 +X 方向・中心原点で作る。

    ⚠ 押し出しの**向きは断面の法線で決まる**。`Rectangle` から作った面は +Z へ、
    STEP から取り出した端面は法線の向き次第で -Z へ伸びる。
    `Pos(0, 0, -length/2)` のように向きを決め打ちすると、公式データに
    差し替えた途端に中心が length だけずれる（実際にずれて外形検証が壊れた）。
    → **押し出した後に bbox から中心を合わせる。**
    """
    global _EXT_FACE
    if _EXT_FACE is None:
        _EXT_FACE = _ext_profile()
    solid = extrude(_EXT_FACE, amount=length)
    b = solid.bounding_box()
    solid = Pos(0, 0, -(b.min.Z + b.max.Z) / 2) * solid   # 長手方向の中心を原点へ
    # 断面は XY にあり長手が Z → 長手を X に倒す
    return mat(Rot(0, 90, 0) * solid, "A6005C")


def _ext1020_profile():
    """HFS5-2010（20×10）の断面。**公式 CAD が無いので規格表からの推定**。

    2020 は vendor/misumi の STEP を使えるが、2010 は未取得。
    溝形は 2020 と同じ 5 シリーズなので、外形 20×10 に同じ溝を切って作る。
    質量は推定断面積ではなく**カタログ値 0.25 kg/m** を使う（`ext_mass_2010`）。
    形は逃げの確認用、質量はカタログ、と役割を分ける。
    """
    # ⚠ 20 を **Y** に、10 を Z に取る。逆にすると斜材の Y 幅が 10 しかなくなり、
    #   柱の側面（|Y| 差 10）に当てたガセットが斜材まで届かず 5mm 浮く。
    #   軸力部材なので断面の向きは剛性より**継手が成立するか**で決める。
    sk = Rectangle(P.EXT_W / 2, P.EXT_W)
    for ang, d in ((0, P.EXT_W / 4), (180, P.EXT_W / 4), (90, P.EXT_W / 2), (270, P.EXT_W / 2)):
        c, s_ = cos(radians(ang)), sin(radians(ang))
        sk = sk - (Pos(c * (d - 2.0), s_ * (d - 2.0)) * Rot(0, 0, ang) * Rectangle(4.0, 6.0))
    return sk


_EXT1020_FACE = None


@proto_cache
def ext2010(length: float):
    """HFS5-2010 を長手 +X・断面 20(Y)×10(Z)・中心原点で作る。

    斜材のように**軸力しか受けない部材**に使う。2020 との差は 0.25 kg/m、
    斜材 707mm × 2 本で 354g。35kg 規定に対して無視できない差になる。
    """
    global _EXT1020_FACE
    if _EXT1020_FACE is None:
        _EXT1020_FACE = _ext1020_profile()
    solid = extrude(_EXT1020_FACE, amount=length)
    b = solid.bounding_box()
    solid = Pos(0, 0, -(b.min.Z + b.max.Z) / 2) * solid
    return mat(Rot(0, 90, 0) * solid, "A6005C")


def add_ext2010(label: str, length: float, group: str, qty: int = 1) -> None:
    LEDGER.add(f"HFS5-2010 {label} L{length:.0f}", 0.250 * length / 1000.0,
               "MISUMI カタログ 0.25kg/m", group, qty)


def ext_seg(y0: float, y1: float, cuts, axis="y"):
    """`y0`〜`y1` を走る押出材を、`cuts`（横切る部材の中心座標）で**切り分ける**.

    アルミフレームの井桁を「桁を端から端まで通す」で描くと、交差部で
    実体が 20mm 素通しに重なる。CAD では描けるが実機では組めない。
    実際に 20 組・最大 2294mm³ の食い込みがこれで生まれていた。

    実機では横材を**縦材のあいだの寸法に切って**、ブラケットで留める。
    ここではその切断長を計算し、(中心座標, 長さ) の一覧を返す。

    返り値が複数になる＝**部品点数が増える**。それが正しい姿で、
    「1本の長い部材」で済ませていたのは実機を無視した簡略化だった。
    """
    edges = [y0]
    for c in sorted(cuts):
        if y0 + 1.0 < c < y1 - 1.0:
            edges += [c - P.EXT_W / 2, c + P.EXT_W / 2]
    edges.append(y1)
    out = []
    for a, b in zip(edges[::2], edges[1::2]):
        if b - a > 5.0:            # 5mm 未満の切れ端は作らない（実際には作れない）
            out.append(((a + b) / 2, b - a))
    return out


# ---------------------------------------------------------------------------
# ブラケット（MISUMI HBLFSN5 相当 / 3Dプリント補強ガセット）
# ---------------------------------------------------------------------------
# L 字ブラケットのねじ穴位置（局所座標・腕の付け根から）。
# ⚠ 穴が無いブラケットにねじを通すと、軸が金具の中に丸ごと入る
#   （M5×t6 で 118mm³）。それが 46 個 ×2 本ぶん溜まると、
#   「宣言の陰の重なり」の合計が 1 万 mm³ 増える。**穴は実物にある**
#   ので、描かないほうが間違い。ここを開けておけば重なりは 0 になる。
BRACKET_HOLE = 11.0        # 内隅からの距離（腕 17・板厚 6 の中央）
BRACKET_HOLE_DIA = 5.5     # M5 のバカ穴
# ⚠ **腕 a のねじは皿ねじにする。** 2 本とも六角穴付き（頭 φ8.5×5）に
#   すると、内隅で頭どうしが 102mm³ 重なる（46 個すべてで起きた）。
#   2 本の軸線は内隅の上（h, h, 0）で**交差**していて、頭はその交点へ
#   向かって伸びる。離すには軸間距離 √2·|h−11| ≥ 8.5、つまり h ≥ 17 が
#   要るが腕は 17mm しかない。
#   幅方向（±Z）へ振るのも駄目だった。**幅方向は溝と直角**なので、
#   5mm ずらすとねじが溝から外れて材を削る（実測 24,084mm³ に増えた）。
#   → 片方の頭を材の中へ沈める。皿もみは φ10 深さ 2.25、腕 t6 に対して
#     残肉 3.75mm。相手の頭（最も近い縁で y=6.75）とは 0.75mm 空く。
BRACKET_CSK_DIA = 10.0
BRACKET_CSK_DEPTH = (10.0 - 5.5) / 2      # 90° 皿なので深さ =（径差）/2


@proto_cache
def bracket(size: float = 20.0, t: float = 6.0, arm: float = 17.0):
    """L字ブラケット HBLFSN5 相当。**原点＝内隅**、腕は +X と +Y、幅は Z 方向。

    突き合わせた押出材はブラケットが無ければ**ただ接しているだけ**で、
    実機では手を離した瞬間に落ちる。整合性チェックの「浮き」は
    接触の有無しか見ないので、ここを描かないと「接触＝組める」と誤読する。

    腕を X-Y に置いてあるのは、水平材どうしの T 継手（横材の端が
    長手材の側面に突き当たる）が最も多いから。Z 回りに 0/90/180/-90 を
    掛ければ 4 象限どれにでも向く。柱脚のような縦継手は Rot(90,…) で倒す。

    ねじ穴は各腕に 1 つずつ（幅 20mm に M5 の座金は 1 列しか並ばない）。
    `BRACKET_HOLE` の位置に `screw_tnut()` を差せば芯が合う。
    """
    a = Box(arm, t, size, align=(Align.MIN, Align.MIN, Align.CENTER))
    b = Box(t, arm, size, align=(Align.MIN, Align.MIN, Align.CENTER))
    # 隅の三角リブ（ダイカストの肉）
    rib = Box(arm, arm, size - 8.0, align=(Align.MIN, Align.MIN, Align.CENTER))
    rib -= Pos(t, t, 0) * Rot(0, 0, 45) * Box(arm * 2, arm * 2, size,
                                              align=(Align.MIN, Align.CENTER, Align.CENTER))
    body = a + b + rib
    r = BRACKET_HOLE_DIA / 2
    # 腕 a（板厚は Y）を Y 方向に貫く穴。外面（Y=t）から皿もみを付ける
    body -= (Pos(BRACKET_HOLE, t / 2, 0) * Rot(90, 0, 0)
             * Cylinder(r, t + 2, align=CTR))
    body -= (Pos(BRACKET_HOLE, t - BRACKET_CSK_DEPTH / 2, 0) * Rot(90, 0, 0)
             * Cone(BRACKET_CSK_DIA / 2, r, BRACKET_CSK_DEPTH, align=CTR))
    # 腕 b（板厚は X）を X 方向に貫く穴（こちらは六角穴付きの頭が載る）
    body -= (Pos(t / 2, BRACKET_HOLE, 0) * Rot(0, 90, 0)
             * Cylinder(r, t + 2, align=CTR))
    return mat(body, "ADC12")


# ブラケットを内隅の象限へ向ける Z 回転 [deg]。(x の符号, y の符号) で引く
BRACKET_ROT = {(1, 1): 0.0, (-1, 1): 90.0, (-1, -1): 180.0, (1, -1): -90.0}


@proto_cache
def bracket_m(size: float = 20.0, t: float = 6.0, arm: float = 17.0,
              flip_y: bool = False):
    """`bracket()` の Y 鏡映つき版。左右対称な位置に同じ金具を置くため。"""
    b = bracket(size, t, arm)
    if flip_y:
        from build123d import mirror, Plane
        b = mirror(b, Plane.XZ)
    return b


@proto_cache
def bracket_tee(size: float = 20.0, t: float = 6.0, arm: float = 17.0,
                flip_y: bool = False):
    """T 継手用の L 金具。**原点＝当てる 2 面の交線**、幅は Z 方向。

    板 A: X -arm..0、Y 0..t   （法線 +Y の面に当て、腕は -X へ）
    板 B: X 0..t、Y -arm..0   （法線 +X の面に当て、腕は -Y へ）

    ⚠ `bracket()` は腕と板厚が**同じ象限**に出る（内隅にぴったり入る形）。
      ところが T 継手──横材の**端**が縦材の**側面**に突き当たる継手──では、
      板厚は材から離れる側、腕は材のある側に出るので、象限が逆になる。
      これは回転でも鏡映でも `bracket()` からは作れない。無理に回すと
      腕が縦材の幅から外れて座面が **0mm²** になる（マスト 4 か所が
      そうだった。「BRACKET と宣言したのに実体が無い」より質が悪く、
      実体はあるのに効いていない）。
      Rot(0,0,0/90/180/-90) で 4 通りの向きが得られる。
    """
    a = Box(arm, t, size, align=(Align.MAX, Align.MIN, Align.CENTER))
    b = Box(t, arm, size, align=(Align.MIN, Align.MAX, Align.CENTER))
    # 隅の三角リブ（ダイカストの肉）
    rib = Box(arm, arm, size - 8.0, align=(Align.MAX, Align.MAX, Align.CENTER))
    rib -= Pos(-t, -t, 0) * Rot(0, 0, 45) * Box(arm * 2, arm * 2, size,
                                                align=(Align.MAX, Align.CENTER, Align.CENTER))
    out = a + b + rib
    if flip_y:
        from build123d import mirror, Plane
        out = mirror(out, Plane.XZ)
    return mat(out, "ADC12")


@proto_cache
def tabbed_plate(w: float, h: float, t: float = 3.0, tab: float = 25.0,
                 sy: float = 1.0, tab_w: float | None = None,
                 tab_x: float = 0.0):
    """⚠ **使わないこと（自校では曲げられない）。** 呼び出し元は 0 件。

    ⚠ 2026-08-04 に加工能力を確認した結果、**アルミ板の板金（曲げ）はできない**。
      できるのは 2D の切り抜きと、板の**端面への横穴**（ドリル / タップ）まで。
      この関数が作るのは 90° 曲げのフランジ付き板なので、そのままでは
      実物にならない。同じ役目は**平板 2 枚を直交させ、片方の端面に
      タップを立てて留める**（`L.plate` ×2 + 端面タップ）で果たせる。
      消さずに残してあるのは「なぜ曲げ板を使っていないか」を残すため。

    立て板の上端に**留め代**（90° 曲げのフランジ）を付けた板金。

    原点＝垂直部の中心。垂直部は XZ 面（板厚は Y）、留め代は上端で
    sy 方向へ曲げる。

    ⚠ 細い板の**端面**で押出材に留めても、座面は板厚ぶんしか出ない
      （t3 × 幅30 = 90mm²）。M5 の座金すら収まらない面積で
      「ボルト 2 本で留める」と宣言していた。曲げて面で当てること。

    ⚠ 板 2 枚で車輪を挟む構成では、両方の留め代を内向きに折ると
      互いに 1,620mm³ 重なる。`tab_w` と `tab_x` で X 方向にずらして
      互い違いに配置すること。
    """
    v = Box(w, t, h, align=CTR)
    f = Box(tab_w if tab_w else w, tab, t, align=CTR)
    return v + Pos(tab_x, sy * (tab + t) / 2, (h - t) / 2) * f


@proto_cache
def angle_hanger(w: float, run: float, rise: float, t: float = 4.0,
                 sy: float = 1.0):
    """L 字の吊り金具。原点＝水平部の始端（取付面の側）・上面。

    水平部は sy 方向へ `run` 伸び、その先端から `rise` ぶん立ち上がる。
    立ち上がりの**面**を相手（柱など）に当てるための金具。
    """
    h = Box(w, run, t, align=(Align.CENTER, Align.MIN, Align.MAX))
    # ⚠ 立ち上がりは水平部の**先端の内側**に置く。先端から先へ出すと
    #   相手（柱）の中へ 1,788mm³ 入る。当てるのは相手の手前の面。
    v = Box(w, t, rise, align=(Align.CENTER, Align.MAX, Align.CENTER))
    if sy < 0:
        h = Rot(0, 0, 180) * h
        v = Rot(0, 0, 180) * v
    return mat(h + Pos(0, sy * run, 0) * v, "A5052")


def angle_hanger_parts(w: float, run: float, rise: float, t: float = 4.0,
                       sy: float = 1.0):
    """`angle_hanger` を**曲げずに作る**ための 2 枚（水平板, 立板）を返す。

    ⚠ 自校ではアルミ板を曲げられない（`export_fab.CAN_BEND = False`）。
      L 字を 1 部品で描くと**そのままでは実物にならない**ので、水平板と
      立板を別々に置き、立板を水平板の**端面のタップ**へ留める
      （横穴加工まではできる）。
    ⚠ 端面にタップを立てる側は `TAP_MIN` を満たす厚みが要る
      （M4 なら 6mm、M5 なら 7.5mm）。t4 のままだと M3 しか立たない。
      呼ぶ側で板厚を選ぶこと。
    """
    # ⚠ **水平板は立板のぶんだけ短くする。** 前は水平板を `run` いっぱいまで
    #   伸ばしたうえで立板を `run-t..run` に置いていたので、2 枚が
    #   w×t×t（60×8×8 = 3,840mm³）**同じ場所を占めていた**。図では L 字に
    #   見えるが、実物は 2 枚とも切り出す独立した板なので組めない。
    #   水平板を `0..run-t`、立板を `run-t..run` にして端面どうしを突き合わせる。
    #   （外形寸法 run は変えない。立板は相手＝柱の面に接したままにする）
    h = Box(w, run - t, t, align=(Align.CENTER, Align.MIN, Align.MAX))
    v = Box(w, t, rise, align=(Align.CENTER, Align.MAX, Align.CENTER))
    if sy < 0:
        h = Rot(0, 0, 180) * h
        v = Rot(0, 0, 180) * v
    return mat(h, "A5052"), mat(Pos(0, sy * run, 0) * v, "A5052")


@proto_cache
def cable_saddle(w: float = 24.0, t: float = 3.0, rise: float = 20.0,
                 run: float = 27.0, dia: float = 8.0):
    """配線サドル。**原点＝取付面の下端**、立ち上がりは +Z、腕は +Y。

    配線は「桁に沿わせる」と宣言していても、実体は桁の面から 20mm 以上
    離れたところを走っていることが多い（骨格や機器を避けた結果そうなる）。
    宣言だけあって実体が無ければ、実機ではケーブルが垂れる。
    面から配線までの距離を埋める金具を、宣言のたびに描くこと。

    腕の先はケーブルの外径ぶんを抱く U 字にして、載せるだけでなく
    結束バンドが通る形にする。
    """
    v = Box(w, t, rise, align=(Align.CENTER, Align.MIN, Align.MIN))
    h = Box(w, run, t, align=(Align.CENTER, Align.MIN, Align.MIN))
    lip = Box(w, t, dia + 2 * t, align=(Align.CENTER, Align.MIN, Align.MIN))
    return mat(v + Pos(0, 0, rise) * (h + Pos(0, run, 0) * lip), "A5052")


@proto_cache
def fork_tine(length: float, width: float, t: float, tip_len: float, tip_t: float):
    """櫛歯 1 本。歯先は**上面だけ**を削ってナイフエッジにする（下面は水平）。

    ⚠ 傾斜は「箱を回して引く」では作れない。以前は長さ tip_len*1.2 の箱を
      3.44° 回して引いていたが、箱の**端面も一緒に傾く**ので、
      歯先より根元側で厚み全部を削ってしまい、
      **歯先 30mm が本体から 3.1mm 離れた別部品になっていた**。
      STEP では 5 本の歯先が宙に浮いた状態で出力されていた。
      断面の輪郭をそのまま多角形で描いて引く。角度が形の定義に直接現れる。
    """
    from build123d import Polyline, make_face
    body = Pos(-(length + tip_len) / 2, 0, -t / 2) * Box(length + tip_len, width, t, align=CTR)
    # 下面 z=-t 固定。上面は x=-length で z=0、x=-(length+tip_len) で z=-(t-tip_t)
    prof = make_face(Polyline(
        (-(length + tip_len) - 1.0, -(t - tip_t)),
        (-length, 0.0),
        (-length, t),
        (-(length + tip_len) - 1.0, t),
        (-(length + tip_len) - 1.0, -(t - tip_t)),
    ))
    wedge = extrude(Plane.XZ * prof, amount=width + 2, both=True)
    tine = body - wedge
    if len(tine.solids()) != 1:
        raise ValueError(f"櫛歯が {len(tine.solids())} 個に分かれた（歯先の傾斜カットが深すぎる）")
    return mat(tine, "SUS304")


@proto_cache
def gusset(a: float, b: float, t: float = 4.0):
    """三角ガセット板。原点＝直角の頂点、+X に `a`、+Z に `b`。斜材の付け根用。"""
    from build123d import Polyline, make_face, Plane
    prof = Plane.XZ * make_face(Polyline((0, 0), (a, 0), (0, b), (0, 0)))
    return mat(extrude(prof, amount=t, dir=(0, 1, 0)) .moved(Pos(0, -t / 2, 0)), "A5052")


def ext_mass(length_mm: float) -> float:
    return P.EXT_MASS_PER_M * length_mm / 1000.0


def add_ext(label: str, length: float, group: str, qty: int = 1) -> None:
    LEDGER.add(f"HFS5-2020 {label} L{length:.0f}", ext_mass(length), "MISUMI カタログ 0.5kg/m", group, qty)


# ---------------------------------------------------------------------------
# 板材（CNC切削 A5052）
# ---------------------------------------------------------------------------
def plate(size_x: float, size_y: float, t: float):
    """板中心を原点、板厚を Z 方向に取った矩形板。"""
    return mat(Box(size_x, size_y, t, align=CTR), "A5052")


# トポロジー最適化した外形を読み込むところ。⚠ 最適化そのものはここで
# 走らせない（1 枚 10〜60 秒かかるので、STEP を作るたびに解いていられない）。
# `scripts/topo_opt.py` が `out/topo/<板名>.json` に落とした輪郭を読むだけ。
_TOPO_DIR = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                          "..", "out", "topo")
_topo_cache: dict[str, object] = {}


def topo_plate(name: str, t: float, material: str = "A5052",
               fallback: tuple[float, float] | None = None):
    """トポロジー最適化した外形の板。板中心が原点、板厚は Z。

    `name`     … `scripts/topo_opt.py` が出した輪郭の名前（部品名と同じ）
    `fallback` … 輪郭がまだ無いときに使う矩形 (size_x, size_y)

    ⚠ **輪郭が無いのに黙って矩形を返してはいけない。** 「最適化した」と
      いう名前の関数が実際には何もしていない状態が、いちばん見つけにくい。
      `fallback` を渡したときだけ矩形に落ち、そのことを標準エラーに出す。
      渡さなければ例外にする。
    ⚠ 輪郭 JSON は**組立より古くなりうる**。相手を動かしたら
      `python scripts/topo_cache.py && python scripts/topo_opt.py` を
      やり直すこと。古さは `scripts/topo_check.py` が見る。
    """
    import json
    import sys

    if name not in _topo_cache:
        path = _os.path.join(_TOPO_DIR, f"{name}.json")
        _topo_cache[name] = json.load(open(path, encoding="utf-8")) \
            if _os.path.exists(path) else None
    d = _topo_cache[name]
    if d is None:
        if fallback is None:
            raise FileNotFoundError(
                f"topo_plate({name}): 輪郭が無い（{_TOPO_DIR}/{name}.json）。"
                f"`python scripts/topo_opt.py --only {name}` で作ること")
        print(f"⚠ topo_plate({name}): 輪郭が無いので矩形 "
              f"{fallback[0]:.0f}×{fallback[1]:.0f} で代用した", file=sys.stderr)
        return plate(fallback[0], fallback[1], t)

    # ⚠ `src/` が sys.path に入っているとは限らない。検査スクリプトは
    #   自分で入れているが、**STEP の生成ランチャーは入れない**（生成器の
    #   ファイルだけを読み込む）。実際それで STEP だけ
    #   `ModuleNotFoundError: No module named 'topo'` になった。
    return _topo_solid(name, t, material)


# ⚠ **形だけを別関数に分けてキャッシュする。** `topo_plate` 本体は
#   「輪郭が無い」ときに例外を投げたり標準エラーへ警告を出したりする
#   （＝呼ぶたびに起きてほしい）ので、そこごとキャッシュすると
#   **警告が 1 度しか出なくなる**。黙って矩形に落ちるのがいちばん怖い、
#   というのがこの関数の趣旨なので、判定は毎回通し、押し出しだけ省く。
#   実測 build2 で 2.36 秒（姿勢を変えるたびに同じ板を押し出し直していた）。
@proto_cache
def _topo_solid(name: str, t: float, material: str):
    """輪郭 JSON から押し出した板（`topo_plate` の形の部分だけ）。"""
    # ⚠ `src/` が sys.path に入っているとは限らない。検査スクリプトは
    #   自分で入れているが、**STEP の生成ランチャーは入れない**（生成器の
    #   ファイルだけを読み込む）。実際それで STEP だけ
    #   `ModuleNotFoundError: No module named 'topo'` になった。
    _here = _os.path.dirname(_os.path.abspath(__file__))
    if _here not in _sys.path:
        _sys.path.insert(0, _here)
    from topo import shape as _shape
    from build123d import extrude
    face = _shape.to_face(_shape.from_json(_topo_cache[name]))
    return mat(extrude(face, amount=t / 2) + extrude(face, amount=-t / 2),
               material)


# ねじの呼び径 → (頭部径, 頭部高さ)  ISO 4762 六角穴付きボルト
SCREW_HEAD = {2: (3.8, 2.0), 3: (5.5, 3.0), 4: (7.0, 4.0), 5: (8.5, 5.0),
              6: (10.0, 6.0)}

# 皿ねじ ISO 10642 の頭部径（90°）。頭の高さは（径差）/2 で出る
FLAT_HEAD_DIA = {2: 4.0, 3: 6.0, 4: 8.0, 5: 10.0, 6: 12.0}


@proto_cache
def screw(size: int = 5, length: float = 12.0, washer: bool = False):
    """六角穴付きボルト。頭部の座面を z=0 に置き、軸は -Z 方向へ伸びる。

    ねじ山は表現しない（STEP が重くなるだけで設計判断に効かないため）。
    座面 z=0 が「締結する板の上面」に一致するので、板の上面に置けばよい。
    """
    # ⚠ ワッシャは座面と頭の**あいだ**に入る。以前は Z -1.2..0 に描いて
    #   いたので、相手の板に必ず 1.2mm 埋まっていた（washer=True の
    #   すべてのねじで発生）。板の上に載せ、頭はその上へずらす。
    dh, hh = SCREW_HEAD[size]
    wt = 1.2 if washer else 0.0
    head = Pos(0, 0, wt + hh / 2) * Cylinder(dh / 2, hh, align=CTR)
    shank = Pos(0, 0, -length / 2) * Cylinder(size / 2, length, align=CTR)
    body = head + shank
    if washer:
        body = body + Pos(0, 0, wt / 2) * (Cylinder(dh / 2 + 1.0, wt, align=CTR)
                                           - Cylinder(size / 2 + 0.2, wt + 1.0,
                                                      align=CTR))
    return mat(body, "STEEL")


@proto_cache
def screw_flat(size: int = 5, length: float = 12.0):
    """皿ねじ（90°）。**頭の上面が z=0**（＝板の表面と面一）、軸は -Z へ伸びる。

    ⚠ 六角穴付きの `screw()` と原点の意味が違う。あちらは頭の**座面**が
      z=0（頭は板の上に出る）、こちらは頭の**上面**が z=0（頭は板の中に
      沈む）。同じ原点にすると、皿ねじが板の上に浮いて座らない。
    """
    dh = FLAT_HEAD_DIA[int(size)]
    hh = (dh - size) / 2                     # 90° 皿の頭の高さ
    head = Pos(0, 0, -hh / 2) * Cone(size / 2, dh / 2, hh, align=CTR)
    shank = Pos(0, 0, -(hh + (length - hh) / 2)) * \
        Cylinder(size / 2, length - hh, align=CTR)
    return mat(head + shank, "STEEL")


def screws_at(points, size: int, length: float, pos, rot=(0, 0, 0), washer: bool = False):
    """points（局所XY）に並べたボルト群を、pos/rot の座標系へ配置して返す。"""
    out = []
    for x, y in points:
        out.append(Pos(*pos) * Rot(*rot) * Pos(x, y, 0) * screw(size, length, washer))
    return out


# アルミフレーム 5 シリーズの溝（vendor/misumi の公式 STEP を実測）。
#   深さ 0.0..1.9  … 開口 6.0mm
#   深さ 1.9..2.5  … 12.0mm に広がる（ナットの掛かる段）
#   深さ 2.5..6.0  … 45° で絞られて閉じる（半幅 = 8.5 − 深さ）
# ⚠ この実測が無いと**溝ナットの置き場所が決められない**。推定プロファイル
#   （溝口 4mm 深さ + チャンバ 2.5mm）では底が 6.5mm になり、0.5mm ぶん
#   ねじが空振りして「離れ」になる。形は公式データから取ること。
EXT_SLOT_MOUTH = 1.9
EXT_SLOT_BOTTOM = 6.0
POST_NUT_DIA = 11.0        # 後入れナット（HNTT5-5 相当）の外形
POST_NUT_H = 3.4
POST_NUT_TOP = 2.4         # 押出材の表面からナット上面までの深さ


@proto_cache
def post_nut(size: int = 5):
    """アルミフレームの**後入れナット**。溝の中に収まる円板で表す。

    ⚠ 実物は角ばった鋼片だが、**向きを持たせると使えない**。ねじは
      軸まわりに回転自由（`screw()` は回転対称）なので、角ナットを付けると
      「溝の長手方向がどちらか」を置く側が知っていなければならなくなる。
      ブラケットは 4 象限へ回るうえ柱脚では倒れるので、そこを間違えると
      ナットが材を突き抜ける。円板なら向きが要らない。
    ⚠ 溝は 45° で絞られているので、円板は縁が少し材に食い込む（実測 40mm³
      以下）。これは TNUT の許容（400）の内側で、「ナットが溝に噛んでいる」
      ことの実体そのもの。逆に食い込まない寸法にすると溝と接触しなくなり、
      「宣言したのに接していない」になる。
    """
    # ⚠ 下穴はねじ軸より**細く**する。同径にすると円筒どうしの面が重なり、
    #   和が 1 つのソリッドにならずに薄皮の面が残ることがある。
    return Cylinder(POST_NUT_DIA / 2, POST_NUT_H, align=CTR) - \
        Cylinder((size - 0.6) / 2, POST_NUT_H + 1.0, align=CTR)


@proto_cache
def screw_tnut(size: int = 5, grip: float = 6.0, washer: bool = False,
               flat: bool = False):
    """溝ナット締結の 1 セット（ボルト + 後入れナット）を 1 つの形にする。

    `grip` は**ねじが通る板の厚み**。原点 z=0 は板の表面、押出材の表面は
    z=-grip。ナットはその下 `POST_NUT_TOP` から `POST_NUT_H` ぶん。

    ⚠ ナットを別部品にすると、46 個のブラケットで 92 個ソリッドが増え、
      組立検査（総当たり）が目に見えて遅くなる。買うときも「ボルトと
      ナットは 1 組」なので、**1 つの締結具**として扱うほうが台帳とも合う。
    """
    length = grip + POST_NUT_TOP + POST_NUT_H
    body = screw_flat(size, length) if flat else screw(size, length, washer)
    nut = Pos(0, 0, -(grip + POST_NUT_TOP + POST_NUT_H / 2)) * post_nut(size)
    return mat(body + nut, "SUS304")


# 六角穴付きボルトの呼び長さ（規格品として流通している寸法だけ）
SCREW_LENGTHS = (4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 35, 40, 45, 50)


def screw_len(grip: float, engage: float) -> float:
    """「板厚 + かみ合い長さ」を、実在する呼び長さへ切り上げる。

    ⚠ 中途半端な長さ（13mm・17mm）を図面に書くと、買えないので現場で
      別の長さに置き換わる。置き換わった時点で図面と実機がずれる。
    """
    need = grip + engage
    for ln in SCREW_LENGTHS:
        if ln >= need - 1e-6:
            return float(ln)
    return float(SCREW_LENGTHS[-1])

# --- 締結具 1 個の質量 [kg] -------------------------------------------------
# ⚠ **概算 700g の中身を人が想像で決めていた**のが問題だった。呼び径と
#   長さが決まれば質量は形から出る。ここでは規格の外形から体積を出し、
#   材質は A2-70 ステンレス（7.93 g/cm³, scripts/fasteners.py と同じ前提）
#   で統一する。カタログ実測（M5×12 で 4.1g）と 5% 以内で一致する。
_NUT_S = {2: 4.0, 3: 5.5, 4: 7.0, 5: 8.0, 6: 10.0}      # 二面幅 ISO 4032
_NUT_M = {2: 1.6, 3: 2.4, 4: 3.2, 5: 4.7, 6: 5.2}       # 高さ
_WASH = {2: (5.0, 2.2, 0.3), 3: (7.0, 3.2, 0.5),        # 平座金 ISO 7089
         4: (9.0, 4.3, 0.8), 5: (10.0, 5.3, 1.0), 6: (12.0, 6.4, 1.6)}
RHO_FASTENER = 7.93e-3      # g/mm³ SUS304 / A2-70

# 皿頭ブラインドリベット（120°）の頭部径。POP NSD/NTD 相当のカタログ値。
# ⚠ 皿リベットは**板に 120° の座ぐりを掘って**沈める。座ぐりの深さぶん
#   板が薄くなるので、`rivet_flat_head_h` が板厚の半分を超えるなら
#   その板には使えない（`screw_check` が「皿が板を貫く」で落とす）。
RIVET_FLAT_HEAD = {3.0: 6.0, 3.2: 6.5, 4.0: 8.0, 4.8: 9.5}

# ブラインドリベットの**買える全長の上限** [mm]。呼び径 φd に対し
# 掴み代の上限が約 4d、それに頭と軸残りを足した値（POP のカタログ範囲）。
# ⚠ **上限が無いと、成立しない締結からありえない長さのリベットが生まれる。**
#   実際 `brk_lift ↔ lift_guide` で **φ3.2 × 43mm** が出ていた。原因は
#   2 枚が**斜面で接している**こと。`screw_place.place_pair` は接触の法線を
#   「bbox の重なりがいちばん薄い軸」で選ぶので、座標軸に乗らない斜めの
#   接触では法線を取り違える。取り違えた向きに材料を測るとガイド板の
#   長手 562mm を貫くことになり、深さ 30mm → 全長 43mm になった。
#   その 43mm の尾が旋回時に台座横梁を 46mm³ 削っていた（`sweep_fine`）。
#   長さで弾けば、**成立しない締結が「置けなかった」として表に出る**。
def rivet_len_max(d: float) -> float:
    return 6.0 * float(d) + 6.0


def rivet_flat_head(d: float) -> float:
    """皿リベットの頭部径 [mm]。表に無い呼び径は 2d で見積もる。"""
    return RIVET_FLAT_HEAD.get(round(float(d), 1), 2.0 * float(d))


def rivet_flat_head_h(d: float) -> float:
    """皿リベットの頭の高さ [mm]（120° 皿）。= (D − d) / 2 ÷ tan60°"""
    from math import tan, radians
    return (rivet_flat_head(d) - float(d)) / 2 / tan(radians(60.0))


def fastener_mass(kind: str, size: float, length: float = 0.0) -> float:
    """締結具 1 個の質量 [kg]。`kind` は員数表の種類名と同じ。"""
    from math import pi, sqrt
    d = float(size)
    if kind == "CAP":                      # 六角穴付きボルト ISO 4762
        dh, hh = SCREW_HEAD[int(d)]
        v = pi * (dh / 2) ** 2 * hh + pi * (d / 2) ** 2 * length
    elif kind == "FLAT":                   # 皿ねじ ISO 10642（頭は円錐台）
        dh = FLAT_HEAD_DIA[int(d)]
        hh = (dh - d) / 2                  # 90° 皿なので頭の高さ =（径差）/2
        v = pi * hh / 3 * ((dh / 2) ** 2 + (dh / 2) * (d / 2) + (d / 2) ** 2)
        v += pi * (d / 2) ** 2 * max(0.0, length - hh)
    elif kind == "HEXNUT":                 # 六角ナット ISO 4032
        s, m = _NUT_S[int(d)], _NUT_M[int(d)]
        v = (sqrt(3) / 2 * s * s - pi * (d / 2) ** 2) * m
    elif kind == "WASHER":                 # 平座金 ISO 7089
        od, idd, t = _WASH[int(d)]
        v = pi * ((od / 2) ** 2 - (idd / 2) ** 2) * t
    elif kind == "TNUT":                   # 後入れナット（形は post_nut）
        v = pi * (POST_NUT_DIA / 2) ** 2 * POST_NUT_H - pi * (d / 2) ** 2 * POST_NUT_H
    elif kind == "RIVET":                  # ブラインドリベット（軸残りを含む）
        v = pi * (d / 2) ** 2 * (length + 6.0) + pi * (d * 1.0) ** 2 * 1.0
    elif kind == "RIVET_FLAT":             # 皿頭ブラインドリベット（120°）
        dh = rivet_flat_head(d)
        hh = rivet_flat_head_h(d)
        v = pi * (d / 2) ** 2 * (length + 6.0)
        v += pi * hh / 3 * ((dh / 2) ** 2 + (dh / 2) * (d / 2) + (d / 2) ** 2)
    elif kind == "SPACER":                 # 六角スペーサ（黄銅・中空）
        v = (sqrt(3) / 2 * 5.5 ** 2 - pi * (d / 2) ** 2) * length
    else:
        raise ValueError(f"未知の締結具 {kind!r}")
    return v * RHO_FASTENER / 1000.0


# ---------------------------------------------------------------------------
# 締結具の員数（宣言 ↔ 実体の突き合わせ）
# ---------------------------------------------------------------------------
# ⚠ **数え方は 1 か所にしか置かない。** 質量台帳（tr_assembly）と員数表
#   （scripts/fastener_bom.py）が別々に数えると、片方だけ直したときに
#   「図の本数」「買う本数」「質量」が三者三様になる。実際 700g の概算が
#   図の 56 本とも note の 478 組とも無関係な数字のまま残っていた。

# 締結具を消費する固定方法。これ以外（PRESS/ROTATE/WELD/…）はねじを使わない
# ⚠ ADJUST は接触を要求しない締結だが、**ねじは要る**（むしろねじだけで
#   姿勢が決まる機構）。ここから外すと 4 本の調整ねじが「どの宣言にも
#   紐づかないねじ」として員数表から落ちる。
FASTENER_HOW = ("BOLT", "TSLOT", "THRU", "SCREW_IN", "TNUT", "RIVET", "ADJUST")

# ⚠ BRACKET は「金具で留まっている」という**要約**であって、ねじの本数の
#   宣言ではない。金具自身が brk_* として TSLOT で宣言されているので、
#   両方数えると重複する（38 組・152 本ぶん）。
SUMMARY_HOW = ("BRACKET",)

KIND_JA = {"CAP": "六角穴付きボルト", "FLAT": "皿ねじ",
           "HEXNUT": "六角ナット", "WASHER": "平座金",
           "TNUT": "後入れナット（溝ナット）", "RIVET": "ブラインドリベット",
           "RIVET_FLAT": "皿頭ブラインドリベット", "SPACER": "六角スペーサ"}
KIND_ORDER = ("CAP", "FLAT", "HEXNUT", "WASHER", "TNUT", "RIVET",
              "RIVET_FLAT", "SPACER")

# 購入単位（1 袋の入り数）。
# ⚠ 必要数ちょうどで発注すると必ず足りなくなる（落とす・舐める・仮組みで
#   ばらす）。袋単位で数えておけば予備が自然に入る。
PACK = {"CAP": 100, "FLAT": 100, "HEXNUT": 100, "WASHER": 100,
        "TNUT": 50, "RIVET": 100, "RIVET_FLAT": 100, "SPACER": 50}

# ねじが刺さる相手にタップを立てられる最小厚み（1.5d）。
# scripts/assembly_check.py の TAP_MIN と同じ根拠。
TAP_MIN = {2: 3.0, 3: 4.5, 4: 6.0, 5: 7.5, 6: 9.0}

# アルミフレーム（溝ナットで留める相手）の名前の頭。
# ⚠ 同じ "rail_" で始まってもスライドレールは押出材ではない。
# ⚠ **押出材なのにここに載っていない部材があった**（`disp_beam`）。載せ忘れると
#   `screw_place` は「中身の詰まった材」として扱い、板厚が足りれば**タップ**、
#   足りなければ**貫通させて六角ナット**を選ぶ。2020 の中空にナットは入らない
#   ので、どちらも実機では組めない締結が図に出る。押出材を足したらここも足す。
EXT_PREFIX = ("rail_L", "rail_R", "post_", "cross_", "topbeam", "mast_cross",
              "mast_arm", "brace_", "pedestal_beam", "disp_beam")


def is_screw_part(name: str) -> bool:
    """その部品名は**ねじ本体**か。"""
    return name.startswith("scr_") or name.startswith("nip_screw")


def is_extrusion(name: str) -> bool:
    return name.startswith(EXT_PREFIX)


def parse_note(note: str):
    """note から (本数, 呼び径, リベットか, 皿か) を読む。読めなければ None。

    "4-M3 PCD26" → (4, 3.0, False, False) / "8-φ3.2" → (8, 3.2, True, False)
    "2-φ3 皿リベット" → (2, 3.0, True, True)

    ⚠ **「皿」は頭が沈むという指定**で、締結の強さの話ではない。相手の面が
      **面一でなければならない**とき（フォークの櫛歯の下面のように、
      1.2mm の頭が出るだけで机の天板に当たる）にここへ書く。
    """
    import re
    m = re.search(r"(\d+)\s*-\s*([MmφΦ])\s*([\d.]+)", note or "")
    if not m:
        return None
    return (int(m.group(1)), float(m.group(3)), m.group(2) in "φΦ",
            "皿" in (note or ""))


def part_thickness(name: str) -> float:
    """部品の**板厚**の目安 = bbox の最小辺 [mm]。

    ⚠ 長さの見当を付けるための近似。押出材（20 角）は 20 になるが、
      押出材へはタップを立てず溝ナットで留めるので別の分岐に落ちる。
    """
    import tr_fix as _F
    b = _F.BOX.get(name)
    if b is None:
        return 0.0
    return min(b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z)


def estimate_fastener(a: str, b: str, how: str, size: float, rivet: bool,
                      flush: bool = False):
    """実体の無い締結 1 本ぶんの (種類, 呼び径, 長さ, 付属品) を見積もる。

    ⚠ ここは**推定**。実体を置いた締結は `F.FASTENER` の値をそのまま使う。
      どちらから来た数字かは表に出し続けること（推定が既定になると
      「図に無いから短くしておこう」という調整が黙って入る）。
    """
    if rivet or how == "RIVET":
        d = size or 3.2
        if flush:
            # 皿頭ブラインドリベット（120°）。頭が板に沈むので面から出ない。
            grip = min(part_thickness(a), 6.0) + min(part_thickness(b), 6.0)
            return ("RIVET_FLAT", d, max(4.0, round(grip + 1.0)), ())
        # ⚠ 板厚は bbox の最小辺で見ているので、曲げ板や L 形の部品では
        #   「最小辺」が板厚ではなく形の高さになる。そのまま足すと
        #   **φ3.2 × 73mm のリベット**という買えないものが表に載った
        #   （実際 10 種類出た）。リベットが留めるのは薄板どうしなので、
        #   片側 6mm で頭打ちにする。
        grip = min(part_thickness(a), 6.0) + min(part_thickness(b), 6.0)
        return ("RIVET", d, max(4.0, round(grip + 1.0)), ())
    d = int(size) if size else 5
    ta, tb = part_thickness(a), part_thickness(b)
    if how in ("TSLOT", "TNUT") or is_extrusion(a) or is_extrusion(b):
        # 溝ナット締結。押出材でないほうの板を通して後入れナットに入る
        grip = min(ta if is_extrusion(b) else tb, 10.0)
        return ("CAP", d, screw_len(grip, 6.0), (("TNUT", d),))
    if how == "THRU":
        return ("CAP", d, screw_len(ta + tb, 6.0),
                (("HEXNUT", d), ("WASHER", d)))
    # BOLT / SCREW_IN: 相手にタップが立つ厚みがあるか
    # ⚠ **`flush` はボルトにも効かせる（2026-08-05）。** `flush` はリベットの
    #   分岐でしか見ていなかったので、注記に「皿」と書いた**ボルト**が
    #   黙って `CAP`（六角穴付き）のまま置かれていた。実際
    #   `brk_yaw_Lf → brk_yaw_Lfv` を「2-M4 皿」に直しても頭は 4mm 出たままで、
    #   側板とのすきま 0.22mm が消えなかった。
    #   ⚠ **注記と実体が食い違うのがいちばん悪い。**図面には皿と書いてあり、
    #     モデルは丸頭で、干渉検査は丸頭で通っている状態になる。
    #   皿は座金を使わない（頭が沈むので座面に当たらない）。
    kind = "FLAT" if flush else "CAP"
    if tb >= TAP_MIN.get(d, 6.0):
        return (kind, d, screw_len(min(ta, 10.0), 1.5 * d),
                () if flush else (("WASHER", d),))
    return (kind, d, screw_len(min(ta, 10.0) + min(tb, 10.0), 6.0),
            (("HEXNUT", d),) if flush else (("HEXNUT", d), ("WASHER", d)))


def fastener_joints():
    """(宣言の組, 実体の入った組, 不足の組, 紐づかないねじ, 要約の組) を返す。

    **`build()` の後に呼ぶこと**（F.BOX / F.FIXINGS / F.FASTENER を読む）。
    """
    import tr_fix as _F
    known = set(_F.BOX)
    # 実体のあるねじを「留めている 2 部品」で索引する
    served: dict[frozenset, list[str]] = {}
    for nm in _F.FASTENER:
        tg = frozenset(t for t in _F.targets_of(nm) if not is_screw_part(t))
        served.setdefault(tg, []).append(nm)

    joints: dict[tuple, tuple] = {}
    summary = []
    for name, lst in _F.FIXINGS.items():
        if is_screw_part(name) or name not in known:
            continue
        for t, how, _qty, note in lst:
            if is_screw_part(t) or t not in known:
                continue
            if how in SUMMARY_HOW:
                summary.append((name, t, how))
                continue
            if how not in FASTENER_HOW:
                continue
            joints.setdefault(tuple(sorted((name, t))), (how, note))

    placed, missing = [], []
    used: set[str] = set()
    for (a, b), (how, note) in sorted(joints.items()):
        got = sorted({nm for tg, names in served.items()
                      if a in tg and b in tg for nm in names})
        used.update(got)
        pn = parse_note(note)
        need = pn[0] if pn else 2         # note に本数が無ければ 2 本と読む
        size = pn[1] if pn else 0.0
        rivet = pn[2] if pn else (how == "RIVET")
        flush = pn[3] if pn else False
        if got:
            placed.append((a, b, how, note, need, len(got)))
        if len(got) < need:
            missing.append((a, b, how, note, need - len(got), size, rivet, flush))
    stray = sorted(set(_F.FASTENER) - used)
    return joints, placed, missing, stray, summary


def fastener_tally(missing):
    """(種類, 呼び径, 長さ) ごとの個数と、実体/推定の内訳を返す。"""
    import collections
    import tr_fix as _F
    cnt: collections.Counter = collections.Counter()
    src: collections.Counter = collections.Counter()
    for _nm, (kind, size, length, extras) in _F.FASTENER.items():
        cnt[(kind, float(size), float(length))] += 1
        src["実体"] += 1
        for ek, es in extras:
            cnt[(ek, float(es), 0.0)] += 1
            src["実体"] += 1
    for a, b, how, _note, n, size, rivet, flush in missing:
        kind, d, ln, extras = estimate_fastener(a, b, how, size, rivet, flush)
        cnt[(kind, float(d), float(ln))] += n
        src["推定"] += n
        for ek, es in extras:
            cnt[(ek, float(es), 0.0)] += n
            src["推定"] += n
    return cnt, src


def fastener_ledger_rows():
    """質量台帳に積む締結具の行 [(ラベル, 個数, 単重kg)] を返す。"""
    _j, _p, missing, _s, _sm = fastener_joints()
    cnt, _src = fastener_tally(missing)
    rows = []
    for kind in KIND_ORDER:
        for k in sorted((k for k in cnt if k[0] == kind),
                        key=lambda k: (k[1], k[2])):
            d = f"M{k[1]:.0f}" if kind != "RIVET" else f"φ{k[1]:g}"
            ln = f"×{k[2]:.0f}" if k[2] else ""
            rows.append((f"{KIND_JA[kind]} {d}{ln}", cnt[k], fastener_mass(*k)))
    return rows


def cable_to(points, targets, dia: float = 10.0, clear: float = 0.0):
    # ⚠ `clear` は**面から中心線までの距離**。ケーブルは中心線で置かれるので
    #   半径（dia/2）ちょうどにすると、相手の面に**接して沈まない**。
    #   小さくすると沈み（7,458mm³ 埋まった）、大きくすると浮く（離れになる）。
    #   既定は半径そのもの＝結束バンドで面に密着させた状態。
    """折れ線の**両端を相手の表面に着地させた**ケーブル束を作る。

    `targets` は (部品名, 軸, 側) の並び。端点だけを相手の面へ寄せる。

    ⚠ 配線の端点を手で書くと、相手に埋まるか宙に浮くかのどちらかになる。
      実際「モーター後端に結束」と宣言した 4 本が本体に 1,060mm³ 埋まり、
      別の 4 本は内桁まで 20mm 届いていなかった。
      **面から決めれば、相手が動いても追従する。**
    """
    import tr_fix as _F
    pts = [list(p) for p in points]
    gap = dia / 2.0 if clear == 0.0 else clear
    for idx, tgt in targets:
        if tgt is None:
            continue
        # ⚠ 経路に点を 1 つ足すたびに、後ろの添字が全部ずれる。
        #   終点は -1、その手前は -2 と**末尾から**書けるようにしておく。
        if not -len(pts) <= idx < len(pts):
            raise IndexError(f"経路は {len(pts)} 点しかないのに添字 {idx}")
        name, axis, side = tgt
        i = "xyz".index(axis)
        pts[idx][i] = _F.face(name, axis, side) + side * gap
    return cable([tuple(p) for p in pts], dia)


# 直前に作った配線の折れ線。put() が拾って F.CABLE に控える。
LAST_CABLE = None


@proto_cache
def belt_loop(span: float, r_pulley: float, thick: float = 3.0,
              width: float = 15.0):
    """同径 2 プーリに**巻き付く**ベルト。XY 平面に作り、幅は Z。

    直線部 2 本と、両端のプーリに接する半円でできた輪（スタジアム形の環）。
    内周がちょうどプーリ半径なので、プーリとは接するだけで重ならない。

    ⚠ 矩形の枠（`Box(span+80, 2r, w) - Box(...)`）で描くと、プーリの外周が
      枠の**平らな端を突き抜ける**。φ40 のプーリで 1 組 2,872mm³、機体
      全体では 2 万 mm³ 以上が「宣言で許容された重なり」として溜まり、
      ビューアでは全部干渉に見えていた。実物は巻き付くので円弧で描く。
    """
    def stadium(hw):
        return (Box(span, 2 * hw, width, align=CTR)
                + Pos(span / 2, 0, 0) * Cylinder(hw, width, align=CTR)
                + Pos(-span / 2, 0, 0) * Cylinder(hw, width, align=CTR))
    return stadium(r_pulley + thick) - stadium(r_pulley)


def cable(points, dia: float = 10.0):
    """折れ線に沿ったケーブル束。各区間を円柱、節点を球でつなぐ。

    実配線の取り回しを CAD 上で見えるようにして、可動部との干渉を検査できるようにする。
    質量は「配線・コネクタ・結束」の概算に含まれるので個別計上しない。
    """
    from build123d import Sphere
    global LAST_CABLE
    LAST_CABLE = (tuple(tuple(p) for p in points), dia)
    parts = []
    for a, b in zip(points[:-1], points[1:]):
        ax, ay, az = a
        bx, by, bz = b
        dx, dy, dz = bx - ax, by - ay, bz - az
        ln = (dx * dx + dy * dy + dz * dz) ** 0.5
        if ln < 1e-6:
            continue
        # +Z の円柱を (dx,dy,dz) 方向へ向ける
        yaw = degrees(atan2(dy, dx))
        pitch = degrees(atan2((dx * dx + dy * dy) ** 0.5, dz))
        seg = Pos(ax, ay, az) * Rot(0, 0, yaw) * Rot(0, pitch, 0) * Cylinder(
            dia / 2, ln, align=(Align.CENTER, Align.CENTER, Align.MIN))
        parts.append(seg)
    # ⚠ 球は**両端にも**置く。端点に球が無いと、そこは円柱の平らな端面に
    #   なり、斜めに入る区間では相手の面まで届かない（2mm 手前で止まる）。
    #   cable_to が端点を面に着地させても、届かなければ意味がない。
    for pt in points:
        parts.append(Pos(*pt) * Sphere(dia / 2))
    # ⚠ ここも**1 個ずつ融合する**。`lighten` の註に書いた理由と同じで、
    #   節点 9 個の配線は円柱 8 + 球 9 = 17 個になり、一括ブーリアンが
    #   割に合わなくなる境目（十数個）に乗っている。速さが読めない書き方は
    #   選ばない。
    body = parts[0]
    for x in parts[1:]:
        body = body + x
    return mat(body, "CABLE")


def bolt_circle(n: int, pcd: float, start_deg: float = 0.0):
    return [
        (pcd / 2 * cos(radians(start_deg + i * 360 / n)), pcd / 2 * sin(radians(start_deg + i * 360 / n)))
        for i in range(n)
    ]


def drill(part, points, dia: float, depth: float, z: float = 0.0):
    """z を中心に貫通/座ぐり穴をあける。"""
    for x, y in points:
        part = part - Pos(x, y, z) * Cylinder(dia / 2, depth, align=CTR)
    return part


# ---------------------------------------------------------------------------
# DJI RoboMaster モーター
# ---------------------------------------------------------------------------
@proto_cache
def m3508(with_gearbox: bool = True):
    """M3508 P19。取付面 z=0、出力軸 +Z。寸法はデータシート外形図より。"""
    shaft = Pos(0, 0, P.M3508_SPIGOT_H) * Cylinder(
        P.M3508_SHAFT_DIA / 2, P.M3508_SHAFT_LEN, align=BASE
    )
    spigot = Cylinder(P.M3508_SPIGOT_DIA / 2, P.M3508_SPIGOT_H, align=BASE)
    flange = Pos(0, 0, -4.3) * Cylinder(P.M3508_FLANGE_DIA / 2, 4.3, align=BASE)
    body_len = P.M3508_BODY_LEN if with_gearbox else 48.0
    body = Pos(0, 0, -4.3 - body_len) * Cylinder(P.M3508_BODY_DIA / 2, body_len, align=BASE)
    # 全長 98.4 = 後端キャップ 4.4 + 本体 66.7 + 前端 27.3（データシート側面図）
    cap = Pos(0, 0, -4.3 - body_len - 4.4) * Cylinder(P.M3508_BODY_DIA / 2 - 4.0, 4.4, align=BASE)
    return mat(shaft + spigot + flange + body + cap, "MOTOR")


@proto_cache
def m2006(with_shaft: bool = True):
    """M2006 P36。取付面 z=0、出力軸 +Z。

    ⚠ `with_shaft=False` は**ハウジングだけ**を返す。出力軸は回るので、
      関節をまたぐところでは軸を回る側のリンクに分けて描かなければ
      「回るリンクの部品を車体にボルト留め」という宣言になる。
    """
    body = Pos(0, 0, -P.M2006_BODY_LEN) * Cylinder(P.M2006_BODY_DIA / 2, P.M2006_BODY_LEN, align=BASE)
    if not with_shaft:
        return mat(body, "MOTOR")
    shaft = Cylinder(P.M2006_SHAFT_DIA / 2, P.M2006_SHAFT_LEN, align=BASE)
    return mat(shaft + body, "MOTOR")


@proto_cache
def motor_mount_plate(size_x: float, size_z: float, t: float = 8.0, lighten_it: bool = False,
                      axle_z: float = 0.0, path: bool = False):
    """M3508 用 CNC 切削マウント板（板中心原点・板厚 Z）。

    φ26 インロー座 + 4-M4 PCD35 の取付穴を持つ。
    """
    # ⚠ 穴はインロー径（φ16.7）ではなく、**モーター本体の張り出しが通る径**。
    #   取付面より先には インロー φ16.7 と 出力軸 φ10 しか無いので φ17 で足りる
    #   が、取付面は板の**内面**に来るため、板厚ぶんは軸だけが通る。
    #   ここを軸径で開けるとインローが入らず、逆にインロー径だと
    #   本体の面取りが当たる。実測値どおり φ16.8 で開ける。
    if path:
        # ⚠ 肉抜きは**軸穴を開ける前**に掛ける。後から掛けると
        #   格子の穴が軸穴の縁を削り、軸の通り道が塞がる形になる。
        #
        # この板の荷重は「モーターの取付ボルト円（PCD35）で受けた 1 輪ぶんの
        # 接地反力とトルク」を「上端の耳（フレームの溝ナット）」へ渡すだけ。
        # ⚠ 固定点は**上端の辺**（耳が生えている辺）。ここは相手の面から
        #   取れない（耳は呼ぶ側が別に作る）ので、辺の上に 3 点置く。
        #   荷重は 1 輪 86N（35kg/4）に加速と衝撃を見て 250N。
        #
        # ⚠ **径のばらばらな丸穴（旧 `lighten_path`）をやめた（2026-08-07）。**
        #   荷重経路を避けて穴を置く考え方は正しかったが、出来上がりは
        #   「大小の丸がランダムに散った板」で、経路が見えず、事情を知らない
        #   人には手抜きに見える。同じ考え方のまま**帯は帯として無垢で残し、
        #   その外を三角格子で抜く**形に変える。板を見れば力の通り道が
        #   そのまま読める（＝丸穴では出せなかった説明力がある）。
        #   帯の半幅は旧実装と同じ根拠: せん断で決まる幅（250N / (2·t·54MPa)
        #   = 0.6mm）よりリブの下限のほうが太いので、`BAND_MIN` を使う。
        anc = [(-size_x / 2 + 18.0, size_z / 2 - 8.0, 7.0),
               (0.0, size_z / 2 - 8.0, 7.0),
               (size_x / 2 - 18.0, size_z / 2 - 8.0, 7.0)]
        # 座はモーターのフランジ（φ42）+ 1mm。取付ボルト（PCD35・頭 φ8）も入る
        bore_r = P.M3508_FLANGE_DIA / 2 + 1.0
        # ⚠ 帯は**両端の 2 本だけ**。耳へは 2-M5 で留まる（中央は座ではない）。
        #   3 本にすると扇が板を覆い尽くして 1,343mm² しか抜けず、丸穴の
        #   ころ（2,791mm²）より重い板になった。
        p, _rep = plate_iso(
            size_x, size_z, t, cell=48.0, rib=7.0, edge=8.0, fillet=5.0,
            guards=[(0.0, axle_z, bore_r)] + list(anc),
            bands=[(ax_, ay_, 0.0, axle_z, BAND_MIN)
                   for ax_, ay_, _r in anc if abs(ax_) > 1e-6],
            label="motor_mount_plate")
    elif lighten_it:
        p, _rep = plate_iso(size_x, size_z, t, cell=48.0, rib=7.0, edge=8.0,
                            fillet=5.0,
                            guards=[(0.0, axle_z,
                                     P.M3508_FLANGE_DIA / 2 + 1.0)],
                            label="motor_mount_plate")
    else:
        p = plate(size_x, size_z, t)
    # 取付面より先には インロー φ16.7（1.0）と 出力軸 φ10（24.8）しか無い。
    # 板厚ぶんを軸が通れるよう φ16.9 で開ける。
    # ⚠ 軸穴は**板の中心ではなく車軸の位置**に開ける。板は上端の耳で
    #   フレームに留まるので中心が車軸より 37.5mm 上にあり、
    #   中心に開けると軸が板の無垢部を貫く（314mm³ ×4）。
    p = p - Pos(0, axle_z, 0) * Cylinder(P.M3508_SPIGOT_DIA / 2 + 0.1, t + 2, align=CTR)
    p = drill(p, [(bx, by + axle_z) for bx, by in bolt_circle(4, P.M3508_BOLT_PCD, 45.0)],
              4.5, t + 2)
    return p


# ---------------------------------------------------------------------------
# メカナムホイール φ127（購入品：外形エンベロープ＋ローラー表現）
# ---------------------------------------------------------------------------
@proto_cache
def mecanum_wheel(hand: int = 1):
    """回転軸 Z、中心原点。hand=+1/-1 でローラー傾き（左右輪）を切替。

    ⚠ 4 輪あるが**形は 2 種類しかない**（左右の傾きだけ）。1 枚 1.7 秒
      かかるので、作り直していたぶん（8.16 秒）がまるごと無駄だった。
    ⚠ **ここのブーリアンは一括にしない。** ローラー 12 本を 1 回で融合すると
      速くなるどころか遅くなり（12 本が互いに交わらないので交差グラフを
      作るだけ損）、そのうえ切り揃えの結果が 292705.47 → 292705.66mm³ と
      1e-6 だけ動いた。**この設計は重なり体積を mm³ で追っている**ので、
      速さのために数字を動かすのは割に合わない。形は 1 文字も変えず、
      「同じ形を作り直さない」だけで 8.16 秒 → 1.7 秒にする。
    """
    r_roller = 13.0
    l_roller = 78.0
    r_center = P.WHEEL_DIA / 2 - r_roller  # 50.5

    hub = Cylinder(38.0, 18.0, align=CTR)
    for i in range(2):
        z = (P.WHEEL_WIDTH / 2 - 4.0) * (1 if i else -1)
        hub = hub + Pos(0, 0, z) * Cylinder(42.0, 8.0, align=CTR)
    # ⚠ 抜くのは軸径 φ10 ではなく**ハブアダプタの外径 φ44**。
    #   軸径で抜いていたので、外側にあるアダプタ（φ44 × 22）が
    #   ホイールのハブに 11,536mm³ 丸ごと埋まっていた。
    #   片側だけ座ぐると**左右で入る向きが逆になる**ので（左輪は -Y 側、
    #   右輪は +Y 側からアダプタが入る）、貫通穴にして両側から入るようにする。
    # ⚠ アダプタ外径ちょうど（r=22）で抜くと接線になり、ブーリアンが
    #   場所によって「重なり 11,467mm³」を返したり 0 を返したりした
    #   （前輪は 0、後輪は 11,467。形は同一なのに結果が違う）。
    #   接線は避けて 1mm のすきま嵌めにする。実機でも挿入代が要る。
    # すきま嵌め 0.2mm。1.0 も空けるとアダプタと**接触しなくなり**、
    # 「ボルト留めなのに離れている」で検出される（締結は面で当たること）。
    hub = hub - Cylinder(22.2, P.WHEEL_WIDTH + 4, align=CTR)

    rollers = None
    for i in range(P.WHEEL_ROLLERS):
        ang = 360.0 / P.WHEEL_ROLLERS * i
        # ⚠ 傾斜は**半径方向の軸まわり**に取る。`Rot(0,0,90)` を挟むと
        #   傾斜軸が接線方向になり、バレルの両端が**半径方向に振れて中心へ
        #   潜り込む**。実測すると φ44 の穴の中に 30,386mm³ の材料が残り、
        #   ハブアダプタが車輪に 11,467mm³ 食い込んでいた（左右で出方が違い、
        #   前輪 0 / 後輪 11,467 という不可解な結果になっていた）。
        #   実物のメカナムはバレルの内端が r=30 付近までしか来ない。
        r = Rot(0, 0, ang) * Pos(r_center, 0, 0) * Rot(45 * hand, 0, 0) * Cylinder(
            r_roller, l_roller, align=CTR
        )
        rollers = r if rollers is None else rollers + r
    # 転動エンベロープ φ127 × 幅60 で切り揃える
    clip = Cylinder(P.WHEEL_DIA / 2, P.WHEEL_WIDTH, align=CTR)
    return mat(hub + (rollers & clip), "URETHANE")


# ---------------------------------------------------------------------------
# MISUMI スライドレール SRX3616（3段引・中荷重・スチール）
# ---------------------------------------------------------------------------
@proto_cache
def rail_stage(i: int, extension: float, hand: int = 1):
    """3段引きスライドレールの **1 段だけ** を返す（i = 0 アウター / 1 中間 / 2 インナー）。

    ⚠ 以前は 3 段＋保持器を 1 つの形状に融合して返していた。すると
      **固定段と可動段が同じ「部品」になり**、レールを上桁に留める
      ねじが可動段にも留まっている、という宣言になってしまう。
      リンクをまたぐ固定の検査（assembly_check）でこれが 12 件出た。
      段ごとに分ければ、アウターは base_link、インナーはキャリッジ側、
      と正しいリンクに属せる。部品も 2 本 → 6 本 + 保持器 8 個に増える。

    アウターレールの -X 端を原点、長手 +X、断面高さ 35.3 を Z、
    厚み 19.1 を Y に取る。extension はインナーの -X 方向への繰り出し量。

    ⚠ **これは干渉検査用の外形近似**であって、実物の断面ではない。
      実物は板厚 1.75/2.58/1.75mm の3枚を C 形に曲げたもの。
      **質量は P.RAIL_MASS（カタログ値）を使うので設計には影響しない。**
      板厚と Z 位置（P.RAIL_PLATE_Z）と取付穴（P.RAIL_HOLES）は公式 CAD の実測値。
    """
    ln = P.RAIL_LEN - 8.0 * i
    tv = -extension * (0.0, 0.5, 1.0)[i]
    z0, z1 = P.RAIL_PLATE_Z[i]
    t_sec = z1 - z0
    y0 = hand * (z0 + t_sec / 2)
    # C 形材は断面の周を材料が回っている。厚み方向の逃げは持たない近似なので
    # **平板 1 枚**として扱う（高さ方向の中央を Y 全厚で抜くと段が 2 本に割れる）。
    body = Pos(ln / 2 + tv, y0, 0) * Box(ln, t_sec, P.RAIL_H, align=CTR)
    if i == 0:
        # アウターレールの取付穴（公式 CAD 実測。P.RAIL_HOLES）
        for x0, x1 in P.RAIL_HOLES:
            r = P.RAIL_HOLE_DIA / 2
            for xs in (x0, x1):
                body = body - Pos(xs, y0, P.RAIL_HOLE_Z) * Rot(90, 0, 0) * Cylinder(
                    r, t_sec + 2, align=CTR)
            if x1 > x0:      # 長穴の中間を角穴でつなぐ
                body = body - Pos((x0 + x1) / 2, y0, P.RAIL_HOLE_Z) * Box(
                    x1 - x0, t_sec + 2, P.RAIL_HOLE_DIA, align=CTR)
    return mat(body, "STEEL")


@proto_cache
def rail_retainer(k: int, sz: int, extension: float, hand: int = 1):
    """段と段のあいだのボール保持器（k = 0 外↔中 / 1 中↔内、sz = 上下）。

    段の隙間は 6.51mm。実物はここに鋼球と保持器が入って荷重を渡す。
    描かないと 3 段が互いに触れず、レールが「浮いた板の集まり」になる。
    保持器は段の中間速度で動くので、繰り出しの 1/4・3/4 の位置に置く。
    """
    ya = P.RAIL_PLATE_Z[k][1]
    yb = P.RAIL_PLATE_Z[k + 1][0]
    gap = yb - ya
    yc = hand * (ya + gap / 2)
    rt_len = P.RAIL_LEN * 0.62
    rt_tv = -extension * (0.25 + 0.5 * k)
    z = sz * (P.RAIL_H / 2 - 5.0)
    return mat(Pos(rt_len / 2 + rt_tv, yc, z) * Box(rt_len, gap, 7.0, align=CTR), "STEEL")


# ---------------------------------------------------------------------------
# 移動バケツ（エンテック PO-24A）と、その固定 [RULE 3.2.3]
# ---------------------------------------------------------------------------
@proto_cache
def bucket():
    """底面 z=0、開口上向き。外形はカタログ φ273 × H255。"""
    r_top = P.BUCKET_DIA / 2
    r_bot = P.BUCKET_R_BOT
    w = P.BUCKET_WALL_T
    outer = Cone(r_bot, r_top, P.BUCKET_H, align=BASE)
    inner = Pos(0, 0, w) * Cone(r_bot - w, r_top - w, P.BUCKET_H, align=BASE)
    rim = Pos(0, 0, P.BUCKET_H - 4.0) * (
        Cylinder(r_top + 3.0, 4.0, align=BASE) - Cylinder(r_top - w, 6.0, align=BASE)
    )
    return mat((outer - inner) + rim, "PC")


def bucket_wall_r(z: float) -> float:
    """バケツ底から高さ z における**胴の外半径**（テーパー 22/255＝4.93°）。"""
    return P.BUCKET_R_BOT + P.BUCKET_TAPER * z / P.BUCKET_H


@proto_cache
def bucket_2l_ring():
    """2L目盛り線（規定 3.2.3c の高さ基準を可視化するデータムリング）。"""
    r = bucket_wall_r(P.BUCKET_2L_FROM_BOTTOM)
    return Pos(0, 0, P.BUCKET_2L_FROM_BOTTOM) * Torus(r + 1.5, 1.5)


def _poly_face(poly):
    """shapely の Polygon を build123d の面（板ローカル XY・Z=0）にする。

    `topo.shape.to_face` と同じ約束（`align=None` を必ず付ける。既定の
    align は図形を原点へ**寄せ直す**ので、座標をそのまま渡したつもりが
    板ごと平行移動して穴の位置が全部ずれる）。
    """
    from build123d import Polygon as _BdPolygon
    sk = _BdPolygon(*[(float(x), float(y)) for x, y in poly.exterior.coords[:-1]],
                    align=None)
    for ring in poly.interiors:
        sk = sk - _BdPolygon(*[(float(x), float(y)) for x, y in ring.coords[:-1]],
                             align=None)
    return sk


# 受け板の入隅の丸め [mm]。⚠ 丸めないと円環と帯の合流点が鋭い入隅になる。
# アルミ板はそこから裂けるし、レーザーの経路としても角が残る。
SEAT_FILLET = 12.0
# 丸めた輪郭を間引く許容 [mm]。R12 の 1/60。レーザーのカーフ（0.2mm）より
# 細かいので切った形は変わらない。⚠ 間引かないと輪郭が 3,000 点を超えて
# STEP と DXF が桁違いに重くなる（topo の to_outline と同じ理由）。
SEAT_SIMPLIFY = 0.2


def _bucket_seat_outline():
    """バケツ受け板の外形（shapely・板中心＝バケツ中心）。

    形の理由:
      * **受け円環**（r 95..118）… バケツ底 φ229 の縁を全周で受ける。
        中央は抜く。バケツの底は円錐台の**縁**で立つので、中の材料は
        荷重を受けていない
      * **帯 2 本**（|Y|=150・長さ 190）… 片持ちビーム（2020）の上面に
        そのまま載る。⚠ 円環の外径 118 とビームの内面 140 は 22mm しか
        離れていないので、四隅へ腕を伸ばす必要はない（前の 340 角の板は
        隅の 4 か所で留めていたので、腕が 100mm ずつ要っていた）
      * **桁 4 本** … 円環と帯をつなぐ。入隅は R12
      * **耳 4 つ** … 位置決めタブの座。±Y は帯に含まれるので ±X だけ張り出す
    """
    import math as _m
    from shapely.geometry import Point as _Pt, box as _box
    from shapely.ops import unary_union as _uu

    def _rrect(cx, cy, w, h, r):
        return _box(cx - w / 2 + r, cy - h / 2 + r,
                    cx + w / 2 - r, cy + h / 2 - r).buffer(r, 32)

    ro, ri = P.BUCKET_SEAT_RO, P.BUCKET_SEAT_RI
    ry, rl, rh = P.MAST_CANTILEVER_Y, P.BUCKET_SEAT_RAIL_L, P.BUCKET_SEAT_RAIL_H
    parts = [_Pt(0, 0).buffer(ro, 160)]
    for sy in (1, -1):
        parts.append(_rrect(0, sy * ry, rl, rh, 10.0))
        y0, y1 = ro - 22.0, ry - rh / 2 + 4.0        # 桁は円環と帯へ食い込ませる
        for sx in (1, -1):
            parts.append(_rrect(sx * 55.0, sy * (y0 + y1) / 2, 28.0, y1 - y0, 6.0))
    # ⚠ **耳は円ではなく矩形にすること。** 円の耳は先へ行くほど細るので、
    #   タブの足（r=136 で幅 47mm の帯）の**両端が板からはみ出す**。
    #   実測で足の 44% が空中に出ていた（`plate.intersection(足)` で数えた）。
    #   ±Y のタブは帯と円環のあいだの材料に載るので耳は要らない。
    ear_r0, ear_r1 = ro - 12.0, P.BUCKET_TAB_R + 10.0
    for sx in (1, -1):
        parts.append(_rrect(sx * (ear_r0 + ear_r1) / 2, 0.0,
                            ear_r1 - ear_r0, 56.0, 8.0))
    sk = _uu(parts)
    sk = sk.buffer(+SEAT_FILLET, 64).buffer(-SEAT_FILLET, 64)
    sk = sk.difference(_Pt(0, 0).buffer(ri, 160))
    sk = sk.simplify(SEAT_SIMPLIFY, preserve_topology=True).buffer(0)
    if sk.geom_type != "Polygon":
        raise ValueError(f"受け板の外形が 1 枚にならなかった: {sk.geom_type}")
    return sk


def bucket_seat_holes():
    """受け板にあける穴 [(x, y, 径)]。**穴の位置は組立側と共有する**。

    ⚠ ここで返す位置に、組立側（`tr_assembly`）が実際のねじを置く。
      板の穴とねじを別々に書くと、片方を動かしたときに芯がずれる。
    """
    out = []
    for x, y in bolt_circle(P.BUCKET_BOLT_N, 2 * P.BUCKET_BOLT_R, 45.0):
        out.append((x, y, 5.5))              # バケツ固定 M5 皿（バカ穴）
    for x, y in bolt_circle(P.BUCKET_TAB_N, 2 * P.BUCKET_TAB_R, 0.0):
        out.append((x, y, 5.5))              # 位置決めタブ M5
    return out


@proto_cache
def bucket_seat_plate(t: float):
    """バケツ受け板（A5052・2D 切り抜き）。板中心が原点、板厚は Z 中心。

    ⚠ **等間隔の丸穴（`lighten`）はここでは使わない。** 前はこの板（340 角）に
      φ44 を 58 ピッチで並べていたが、中央 φ210 の抜きと重なって三日月形の
      切れ端が並び、円環から締結へ向かう荷重の通り道を横切って穴が開いていた。
      「板を先に決めて、あとから穴で軽くする」から、そうなる。
      **板の形そのものを荷重の通り道にすれば、穴で軽くする必要がない**
      （340 角 t2 + 丸穴 12 = 310g → 円環 + 帯 t3 = 297g）。
    """
    from build123d import extrude
    face = _poly_face(_bucket_seat_outline())
    sol = extrude(face, amount=t / 2) + extrude(face, amount=-t / 2)
    for x, y, d in bucket_seat_holes():
        sol -= Pos(x, y, 0) * Cylinder(d / 2, t + 2, align=CTR)
    return mat(sol, "A5052")


@proto_cache
def bucket_hold_ring():
    """バケツの**中**に入る押さえリング（PETG）。底面 z=0（＝バケツの内底）。

    規定 3.2.3d は「バケツの固定**のみ**を目的とした部品」だけ中に入れて
    よいとしている。この輪はまさにそれで、
      * 皿ねじの頭を沈めて、投げ込まれた雑巾が引っかからない面にする
      * φ5.5 の穴 4 か所に集中する力を、底の全周（φ176..222）へ散らす
        （バケツの底は t2 のポリカ。座金だけだと縁から裂ける）
    ⚠ 板厚 4 は「皿頭 M5 の沈み 2.5mm ＋ 残肉 1.5mm」で決まっている。
      薄くすると頭が突き抜ける。
    """
    t = P.BUCKET_HOLD_T
    ring = (Cylinder(P.BUCKET_HOLD_RO, t, align=BASE)
            - Cylinder(P.BUCKET_HOLD_RI, t + 2, align=BASE))
    for x, y in bolt_circle(P.BUCKET_BOLT_N, 2 * P.BUCKET_BOLT_R, 45.0):
        ring -= Pos(x, y, -1.0) * Cylinder(5.5 / 2, t + 2, align=BASE)
        # 皿ざぐり（90°）。上面で φ10、2.5mm 下で φ5
        ring -= Pos(x, y, t - 2.5) * Cone(2.5, FLAT_HEAD_DIA[5] / 2, 2.5, align=BASE)
    return mat(ring, "PETG")


@proto_cache
def bucket_tab():
    """位置決めタブ（PETG・4 個）。バケツ底面 z=0、+X 方向を中心に置く。

    バケツの胴（片側 4.93° のテーパー）に沿った内面を持つ爪で、
      * 設置のとき、バケツを落とし込むだけで底のボルト穴が合う
      * 走行中の横荷重を、ポリカの底にあけた φ5.5 の穴から逃がす
    ⚠ 内面はバケツから `BUCKET_TAB_GAP`（0.3mm）逃がす。ぴったりに描くと
      組立検査の接触判定（0.5mm）には入るが、実機では入らない。
    ⚠ 高さは 16mm。**規定 3.2.2 のスタート時 1200mm に効く**（バケツ本体の
      高さは規定から除かれるが、留める金具は機体）。1175 + 16 = 1191 が
      **機体の最高点**になる（前は 1179）。規定まで 9mm しかない。
    """
    from build123d import Axis, Plane, Polyline, make_face, revolve

    h = P.BUCKET_TAB_H
    g = P.BUCKET_TAB_GAP
    foot_t, foot_ro = 5.0, P.BUCKET_TAB_R + 7.0
    wall_ro = P.BUCKET_SEAT_RO + 3.0
    lead = 2.5                                   # 上端の面取り（呼び込み）
    pts = [
        (bucket_wall_r(0.0) + g, 0.0),
        (foot_ro, 0.0),
        (foot_ro, foot_t),
        (wall_ro, foot_t),
        (wall_ro, h),
        (bucket_wall_r(h) + g + lead, h),
        (bucket_wall_r(h - lead) + g, h - lead),
    ]
    prof = Plane.XZ * make_face(Polyline(*pts, pts[0]))
    # 1 個あたりの角度。⚠ **受け板の耳が支えられる幅で決まる**（胴に当たる
    #   長さで決めない）。20° なら r=136 で幅 47mm、耳（56mm）に収まる。
    span = 20.0
    tab = revolve(prof, Axis.Z, revolution_arc=span).rotate(Axis.Z, -span / 2)
    tab -= Pos(P.BUCKET_TAB_R, 0, -1.0) * Cylinder(5.5 / 2, foot_t + 2, align=BASE)
    return mat(tab, "PETG")


# ---------------------------------------------------------------------------
# 椅子（新JIS 教室用椅子 5号・脚切除）／マスコット
# ---------------------------------------------------------------------------
@proto_cache
def chair_5go():
    """座板下面 z=0、背もたれは +X 側。座面中心が原点。"""
    seat = Pos(0, 0, P.CHAIR_SEAT_T / 2) * Box(
        P.CHAIR_SEAT_D, P.CHAIR_SEAT_W, P.CHAIR_SEAT_T, align=CTR
    )
    back_x = P.CHAIR_SEAT_D / 2 - P.CHAIR_BACK_T / 2
    back = Pos(back_x, 0, P.CHAIR_SEAT_T + P.CHAIR_BACK_H / 2) * Box(
        P.CHAIR_BACK_T, P.CHAIR_SEAT_W, P.CHAIR_BACK_H, align=CTR
    )
    # 座面-背もたれ接続部（原型維持部）
    link = None
    for y in (-P.CHAIR_SEAT_W / 2 + 30, P.CHAIR_SEAT_W / 2 - 30):
        b = Pos(back_x - 20, y, P.CHAIR_SEAT_T + 40) * Box(40, 24, 80, align=CTR)
        link = b if link is None else link + b
    return mat(seat + back + link, "PLYWOOD")


@proto_cache
def mascot_envelope():
    """マスコットの規定最小外形 300×300×600（下面 z=0）。"""
    x, y, z = P.MASCOT_BOX
    return mat(Box(x, y, z, align=BASE), "MASCOT")


# ---------------------------------------------------------------------------
# マスコット「シボリ」— 規定 3.1.3 の実体
# ---------------------------------------------------------------------------
# ⚠ **外形の箱を置いただけでは「製作」ではない。** 規定 3.1.3 は
#   「マスコットを製作して座らせること」で、300×300×600 はその下限。
#   箱のままだと、
#     ・当日そこに置く実物が無い（実際に作るまで誰も形を知らない）
#     ・「服を着せる」（3.1.3）を満たす面が決まらない
#     ・重心と固定点が出ない。競技中に落ちれば減点（3.1.3 最後の項）
#   ので、形を決めて実体で持つ。
#
# デザイン — **他所からの流用ではないこと**が規定の要件（著作権が絡む
# キャラクターは使用不可）。そこで、造形の要素を**この競技のものだけ**で
# 作った。設定は「絞った雑巾がそのまま生きものになった」:
#     ・体の斜めのうねり … 雑巾を絞ったときのねじれ。3 本巻いてある
#     ・頭のてっぺんの輪 … 絞り終わりのねじり目
#     ・左右へ張り出す耳 … 雑巾の四隅
#     ・目               … 縫い付けたボタン（糸穴 4 つ）
#     ・手               … 布のミトン。両手で雑巾を持っている
#   服は掃除当番のつなぎと三角巾で、胸のゼッケンにチーム名を入れる
#   （「チームメンバーの一員とわかるような服」＝規定 3.1.3）。
#
# 実機の作り方: 芯は EPP ブロックの切り出し（30kg/m³）、表面はフェルト張り。
#   質量はおよそ 1.2kg。**規定の重量制限には入らない**ので質量台帳には
#   載せない（載せると 35kg 規定の集計が狂う）。ただし椅子マウントの
#   荷重には効くので、`scripts/fea_frame.py` は「椅子+マスコット 3.10kg」
#   として見込んである。
#   固定は尻に埋めた M6 鬼目ナット 2 個 + 座板の下から蝶ボルト。
#   工具無しで外せる＝規定「計量計測時には速やかに取り外せるように」。
def _ell(rx: float, ry: float, rz: float):
    """楕円体（中心原点）。球を軸ごとに伸ばして作る。"""
    return scale(Sphere(1.0), by=(rx, ry, rz))


def _rod(p0, p1, r: float):
    """2 点を結ぶ丸棒。

    ⚠ `Rot(0, ry, rz)` ひとつでは向かない。`Rot` は X→Y→Z の順に
      掛かるので、+Z を倒したあとの Z 回転が効かない（左右の腕が
      同じ方向を向く）。倒す回転と振る回転は**別の Location として**掛ける。
    """
    d = [p1[i] - p0[i] for i in range(3)]
    ln = sqrt(sum(v * v for v in d))
    mid = [(p0[i] + p1[i]) / 2 for i in range(3)]
    ry = degrees(atan2(hypot(d[0], d[1]), d[2]))
    rz = degrees(atan2(d[1], d[0]))
    return Pos(*mid) * Rot(0, 0, rz) * Rot(0, ry, 0) * Cylinder(r, ln, align=CTR)


# 頭の芯（この楕円体の**外面がそのまま**目・三角巾の内面になる）
MASCOT_HEAD_C = (8.0, 0.0, 475.0)
MASCOT_HEAD_R = (110.0, 112.0, 120.0)
MASCOT_BODY_C = (34.0, 0.0, 232.0)
MASCOT_BODY_R = (76.0, 116.0, 112.0)


def _patch(ctr, rad, grow: float, keep):
    """楕円体の表面に貼る「殻の切れ端」。

    ⚠ **曲面に平らな板を当てても面では触れない。** ボタンや三角巾を
      Box や Cylinder で作ると、頭とは点でしか接しないか、逆に深く
      埋まる。相手と**同じ楕円体**を内面に持つ殻を切り出せば、内面が
      頭の外面そのものなので、隙間も食い込みも出ない。
    """
    inner = Pos(*ctr) * _ell(*rad)
    outer = Pos(*ctr) * _ell(*(r + grow for r in rad))
    return (outer - inner) & keep


@_functools.lru_cache(maxsize=1)
def mascot_parts():
    """マスコット「シボリ」の部品 {名前: 形状}。

    原点は椅子の座面の上・マスコットの中心。+X が背もたれ側（背中）、
    -X が正面（顔の向き）、下面 z=0。
    **規定の 300×300×600 にちょうど内接する**（「以上」が要件なので、
    外形はぴったりで満たし、周りの部品はエンベロープだけ避ければよい）。
    """
    hc, hr = MASCOT_HEAD_C, MASCOT_HEAD_R
    bc, br = MASCOT_BODY_C, MASCOT_BODY_R

    # --- つなぎ（尻・胴・首・脚・腕・手）--------------------------------
    suit = (Pos(62, 0, 92) * _ell(88, 138, 98)          # 尻   X -26..150
            + Pos(*bc) * _ell(*br)                       # 胴   Z 120..344
            + Pos(8, 0, 342.5) * Cylinder(46, 55, align=CTR))   # 首 Z 315..370
    for sy in (1, -1):
        # 太もも（前へ）。前端 X=-98 が靴の座面になる
        suit += Pos(-28, sy * 74, 78) * Rot(0, 90, 0) \
            * Cylinder(54, 140, align=CTR)
        # 上腕（下向き）→ ひじ → 前腕（前かつ内向き）→ 布のミトン
        # ⚠ 腕の**外側が Y=±150**。規定の 300mm はここで満たしている
        suit += Pos(26, sy * 118, 251) * Cylinder(32, 118, align=CTR)
        suit += _rod((26, sy * 118, 196), (-108, sy * 78, 190), 30)
        # ⚠ ミトンは**前腕の太さ（φ60）を包める高さ**にすること。44 では
        #   腕の下半分がミトンからはみ出し、下に置いた雑巾へ 8,886mm³
        #   食い込んだ（持っているのではなく、腕が刺さっている状態）。
        suit += Pos(-108, sy * 78, 190) * Box(80, 70, 64, align=CTR)
    # 絞り目のうねり。**この子の由来そのもの**なので形として持たせる。
    # ⚠ 引くのではなく足す。彫るには「胴を 5mm 内側に縮めた形」が要る
    #   （オフセットは重く、失敗すると体が輪切りになる）。巻き付けた
    #   ねじれとして足せば、体積が増えるだけで壊れようがない。
    # ⚠ 胸（Z 235..300）は**空けておく**。ゼッケンはそこに貼る。
    for z, spin in ((170.0, 0.0), (215.0, 55.0), (320.0, 110.0)):
        s = sqrt(max(0.05, 1.0 - ((z - bc[2]) / br[2]) ** 2))
        suit += (Pos(bc[0], 0, z) * Rot(0, 3, spin)
                 * scale(Torus(1.0, 0.09),
                         by=(br[0] * s + 1.0, br[1] * s + 1.0, 88.0)))
    # 椅子の座面から下は無い（尻の楕円体を切る）
    suit -= Pos(0, 0, -500) * Box(600, 600, 1000, align=CTR)

    # --- 頭（耳・絞りの結び目つき）--------------------------------------
    head = Pos(*hc) * _ell(*hr)
    for sy in (1, -1):
        # 耳＝雑巾の四隅。Rot(±90) で円錐を Y 方向へ倒す
        head += Pos(hc[0], sy * 126, 500) * Rot(sy * -90, 0, 0) \
            * Cone(32, 3, 44, align=CTR)
    # 頭のてっぺんの「絞り終わりのねじり」。ここが Z=600（規定の高さ）
    head += Pos(hc[0], 0, 586) * Torus(26, 14)
    head -= Pos(0, 0, 370 - 500) * Box(600, 600, 1000, align=CTR)  # 首から下
    # 口（ステッチ）。表面から 11mm 彫る
    head -= Pos(-92, 0, 438) * Box(14, 62, 9, align=CTR)

    parts = {"mascot_suit": mat(suit, "MASCOT_SUIT"),
             "mascot_head": mat(head, "MASCOT")}

    # --- 目（縫い付けたボタン）------------------------------------------
    for sy, sd in ((1, "L"), (-1, "R")):
        cyl = Pos(-155, sy * 48, 502) * Rot(0, 90, 0) \
            * Cylinder(26, 270, align=CTR)
        eye = _patch(hc, hr, 4.0, cyl)
        for dy, dz in ((9, 9), (9, -9), (-9, 9), (-9, -9)):
            eye -= Pos(-155, sy * 48 + dy, 502 + dz) * Rot(0, 90, 0) \
                * Cylinder(2.5, 300, align=CTR)
        parts[f"mascot_eye_{sd}"] = mat(eye, "MASCOT_TRIM")

    # --- 三角巾（頭に巻く帯 + 後ろの結び目）------------------------------
    # ⚠ 帯の高さは**目（Z 476..528）と絞りの結び（Z 572..）のあいだ**にしか
    #   置けない。頭のてっぺんまで覆うと、別部品どうしが同じ場所を取る。
    band = _patch(hc, hr, 5.0,
                  Pos(0, 0, 550) * Box(600, 600, 40, align=CTR))
    for sy in (1, -1):
        # ⚠ 結び目は**帯の外面に食い込ませる**こと。頭の外面を基準に置くと、
        #   帯（外へ 5mm）の高さでは楕円体が細っていて 9mm 離れ、
        #   三角巾が 3 つのソリッドに分断された。
        band += Pos(114, sy * 22, 548) * Sphere(18)
        # 結び目から垂れる端。⚠ **結び目の中心を通す**こと。結び目の球の
        #   外側に置くと、球は帯と繋がっているのに垂れだけ離れる
        #   （三角巾が 3 ソリッドに分断された）。
        # ⚠ 垂れは**頭の外**に出すこと。結び目の球に届かせようと内側へ
        #   寄せたら、頭の後ろの膨らみへ 19,803mm³ 刺さった（結び目を
        #   通しさえすれば繋がるので、内側へ入れる必要は無い）。
        band += Pos(130, sy * 24, 545) * Rot(0, 10, 0) \
            * Box(16, 32, 60, align=CTR)
        # ⚠ 耳は帯の高さを突き抜けている。**帯から耳を抜く**（同じ場所に
        #   materials が 2 つあると、別部品どうしの食い込みになる）
        band -= Pos(hc[0], sy * 126, 500) * Rot(sy * -90, 0, 0) \
            * Cone(34, 5, 48, align=CTR)
    parts["mascot_bandana"] = mat(band, "MASCOT_TRIM")

    # --- 胸のゼッケン（チーム名を書く板）---------------------------------
    parts["mascot_badge"] = mat(
        _patch(bc, br, 3.0, Pos(-30, 0, 267) * Box(60, 110, 62, align=CTR)),
        "MASCOT_RAG")

    # --- 靴（太ももの前端に面で座る）------------------------------------
    for sy, sd in ((1, "L"), (-1, "R")):
        parts[f"mascot_foot_{sd}"] = mat(
            Pos(-124, sy * 74, 78) * Box(52, 76, 68, align=CTR), "MASCOT_DARK")

    # --- 手に持つ雑巾（上面がミトンの下面に接する）-----------------------
    parts["mascot_rag"] = mat(Pos(-108, 0, 151) * Box(78, 190, 14, align=CTR),
                              "MASCOT_RAG")
    return parts


# ---------------------------------------------------------------------------
# センサー・電装（購入品エンベロープ）
# ---------------------------------------------------------------------------
@proto_cache
def lidar():
    """北陽電機 UST-20LX の外形。**原点＝検出面**（走査面 z=0）、取付は底面。

    ⚠ **検出面は本体の中央ではない。** 外形図の「47.4（検出面）」のとおり、
      底面から 47.4mm、天面から 22.6mm の位置にある。ここを中央（35）で
      描くと、走査面の高さを合わせたつもりで胴が 12.4mm ずれる
      （そのぶん取付座も上下する）。原点を検出面に取れば、置く側は
      `Pos(x, y, LIDAR_LOW_Z)` と書くだけで狂わない。
    ⚠ **円筒ではなく角柱**（50×50×70）。前は φ60 の円筒で描いていたので、
      平板に載せるとき「円筒の側面と板は線でしか当たらない」という
      本来ありえない当たり方になっていた。角柱なら底面が面で載る。
    """
    z0 = -P.LIDAR_PLANE_Z                    # 底面
    body = Pos(0, 0, z0) * Box(P.LIDAR_W, P.LIDAR_D, P.LIDAR_H, align=BASE)
    return mat(body, "SENSOR")


def box_part(dims):
    x, y, z = dims[0], dims[1], dims[2]
    return Box(x, y, z, align=BASE)


@proto_cache
def estop():
    """非常停止スイッチ（きのこ形）。取付面 z=0、押しボタンが +Z。"""
    base = Cylinder(P.ESTOP_DIA / 2 - 6, 14.0, align=BASE)
    head = Pos(0, 0, 14.0) * Cylinder(P.ESTOP_DIA / 2, 12.0, align=BASE)
    return mat(base + head, "ESTOP")


# ---------------------------------------------------------------------------
# 汎用: プーリ / ベアリング / ローラー
# ---------------------------------------------------------------------------
@proto_cache
def pulley(dia: float, width: float, bore: float):
    body = Cylinder(dia / 2, width, align=CTR)
    flange = None
    for z in (-width / 2 - 1.0, width / 2 + 1.0):
        f = Pos(0, 0, z) * Cylinder(dia / 2 + 3.0, 2.0, align=CTR)
        flange = f if flange is None else flange + f
    return mat((body + flange) - Cylinder(bore / 2, width + 8, align=CTR), "A5052")


@proto_cache
def bearing(od: float, idia: float, width: float):
    outer = Cylinder(od / 2, width, align=CTR) - Cylinder(od / 2 - 3.0, width + 2, align=CTR)
    inner = Cylinder(idia / 2 + 3.0, width, align=CTR) - Cylinder(idia / 2, width + 2, align=CTR)
    mid = Cylinder(od / 2 - 3.0, width - 3.0, align=CTR) - Cylinder(idia / 2 + 3.0, width, align=CTR)
    return mat(outer + inner + mid, "STEEL")


# ---------------------------------------------------------------------------
# ウォームギヤ（仰角の減速）
#
# ⚠ **歯は実体で作る。** ウォームを「ただの円柱」、ホイールを「ただのプーリ」で
#   描いていたときは、宣言だけ MESH にしてあって図の上では 2 つの円筒が
#   1 点で接しているだけだった。そのため
#     ・中心距離が基準円の和（42）ではなく円筒半径の和（46）になっていて、
#       実物なら 4mm 浮いて噛まない
#     ・ウォームの軸線がホイールの歯幅の中心から 4mm ずれていた
#       （ウォーム軸はホイールの中心平面に乗っていなければ噛まない）
#     ・歯の無い円筒なので、歯先と歯底のすきま・バックラッシが検査に出ない
#   という 3 つの誤りが、どの検査にも引っかからずに残っていた。
#   歯を体積で描くと、この 3 つは全部「食い込み」か「離れ」で出る。
#
# 歯形は厳密なインボリュートではなく、**軸方向断面が台形の ZA 形ウォーム**で
# 近似する（干渉検査が目的なので、噛み合いの体積が正しければ足りる）。
# ホイール側は、その台形をラックとみなして歯溝を彫る。歯溝は進み角ぶん
# 傾ける（傾けないと歯幅の端で 0.375mm 分ねじ山に当たる）。
# ---------------------------------------------------------------------------
# ⚠ 歯を作るのは 1 回 0.65 秒かかる（螺旋の掃引 + 歯溝 40 個のブーリアン）。
#   link_pitch() は姿勢ごとに組み直されるので、連続掃引の 46 姿勢で
#   30 秒を捨てることになる。形は姿勢に依らないのでキャッシュする。
#   **使う側は必ず Location を掛けること**（`Pos(...) * L.worm(...)`）。
#   Location を掛けると build123d は新しい Shape を返すので、木を壊さない。
#   キャッシュした実体をそのまま Compound の children に渡すと、親が
#   付け替わって前の姿勢の木が壊れる（tr_assembly.build の見張りが拾う）。
_WORM_CACHE: dict = {}


def _thread_half_t(r: float, m: float, d1: float, bl: float, ta: float) -> float:
    """ウォーム半径 r における、ねじ山の軸方向半歯厚。

    基準円（r = d1/2）で p/4 − bl/2、そこから歯元へ向かって圧力角ぶん太る。
    """
    from math import pi
    return pi * m / 4.0 - bl / 2.0 + (d1 / 2.0 - r) * ta


@proto_cache
def worm(m: float, z1: int, d1: float, length: float, bl: float,
         alpha_deg: float = 20.0):
    """円筒ウォームのねじ部（軸 Z・中心原点）。歯底円筒 + ねじ山。

    位相の規約: **+X 方向（＝ホイールを向く側）に、ねじ山の中心が z=0 で来る。**
    ホイール側は歯溝の中心を −X 方向・z=0 に置くので、この 2 つを守れば
    そのまま噛み合う。位相をどこかに書いておかないと、片方を作り直した
    ときに黙って半ピッチずれる（歯の頂どうしが当たる）。
    """
    from math import pi, tan
    from build123d import Helix, Polyline, make_face, sweep

    key = ("worm", m, z1, d1, length, bl, alpha_deg)
    if key in _WORM_CACHE:
        return _WORM_CACHE[key]
    ta = tan(radians(alpha_deg))
    lead = pi * m * z1
    ra = d1 / 2.0 + m               # 歯先半径
    rf = d1 / 2.0 - 1.25 * m        # 歯底半径
    r_in = rf - 1.0                 # 歯底円筒より内側から起こして融合を確実にする
    prof = make_face(Polyline(
        (r_in, 0, -_thread_half_t(r_in, m, d1, bl, ta)),
        (ra, 0, -_thread_half_t(ra, m, d1, bl, ta)),
        (ra, 0, _thread_half_t(ra, m, d1, bl, ta)),
        (r_in, 0, _thread_half_t(r_in, m, d1, bl, ta)),
        (r_in, 0, -_thread_half_t(r_in, m, d1, bl, ta))))
    # ⚠ 掃引の高さは**リードの偶数倍**にする。半端な高さで中心へ寄せると、
    #   z=0 に来るのがねじ山の中心ではなく谷になり、位相が半ピッチずれる。
    n = 2 * int(length / (2.0 * lead) + 2)
    h = n * lead
    path = Pos(0, 0, -h / 2) * Helix(pitch=lead, height=h, radius=d1 / 2.0)
    thread = sweep(prof.moved(Location((0, 0, -h / 2))), path=path, is_frenet=True)
    body = (thread + Cylinder(rf, h, align=CTR)) & Cylinder(ra + 1.0, length, align=CTR)
    _WORM_CACHE[key] = body
    return body


@proto_cache
def worm_wheel(m: float, z1: int, z2: int, face: float, bore: float, a: float,
               bl: float, d1: float, alpha_deg: float = 20.0):
    """ウォームホイールの歯部（軸 Y・中心原点、歯溝の中心が −X／z=0）。

    `a` は中心距離。歯溝の幅は「その半径に来るウォームのねじ山の厚み + bl」で
    決めるので、中心距離を渡さないと決まらない。
    """
    from math import atan2, degrees, pi, tan
    from build123d import Polyline, make_face

    key = ("wheel", m, z1, z2, face, bore, a, bl, d1, alpha_deg)
    if key in _WORM_CACHE:
        return _WORM_CACHE[key]
    ta = tan(radians(alpha_deg))
    r2 = m * z2 / 2.0
    ra2 = r2 + m
    rf2 = r2 - 1.25 * m

    def gap_half(rho: float) -> float:
        """半径 rho における歯溝の軸方向半幅（＝そこに来るねじ山の半厚 + bl）。"""
        return _thread_half_t(a - rho, m, d1, bl, ta) + bl

    lo, hi = rf2 - 0.5, ra2 + 1.0
    cut = make_face(Polyline((lo, 0, -gap_half(lo)), (hi, 0, -gap_half(hi)),
                             (hi, 0, gap_half(hi)), (lo, 0, gap_half(lo)),
                             (lo, 0, -gap_half(lo))))
    # 歯溝は進み角 γ だけ傾ける。ウォームのねじ山は歯幅方向（Y）へ進むと
    # z が tanγ·Y だけ進むので、まっすぐ彫ると歯幅の端でフランクに当たる。
    gam = degrees(atan2(pi * m * z1, pi * d1))
    # ⚠ `both=True` で両側へ伸ばす。片側だけだと押し出しの向き（面の法線）が
    #   歯幅の外へ出てしまい、ブランクを一切削らない（削れていないのに
    #   ブーリアンは成功するので、体積を見ないと気づけない）。
    one = Rot(gam, 0, 0) * extrude(cut, amount=face / 2.0 + 3.0, both=True)
    # Rot(0,β,0) は角度 α を β だけ**減らす**。β を 9°刻みで回せば
    # 180°（= 9°×20）も並びに含まれるので、−X 方向にも歯溝が来る。
    cutters = [Rot(0, k * 360.0 / z2, 0) * one for k in range(z2)]
    blank = Rot(90, 0, 0) * Cylinder(ra2, face, align=CTR)
    body = blank.cut(*cutters) - Rot(90, 0, 0) * Cylinder(bore / 2, face + 4, align=CTR)
    _WORM_CACHE[key] = body
    return body


@proto_cache
def shooter_roller():
    """射出ローラー1個（回転軸 Z、中心原点）: 3Dプリントハブ + ウレタンタイヤ t3。"""
    tire = Cylinder(P.ROLLER_DIA / 2, P.ROLLER_W, align=CTR) - Cylinder(
        P.ROLLER_DIA / 2 - 3.0, P.ROLLER_W + 2, align=CTR
    )
    # ハブは幅20mmのスポークディスク（軽量化）
    hub = Cylinder(P.ROLLER_DIA / 2 - 3.0, 20.0, align=CTR) - Cylinder(
        P.ROLLER_SHAFT_DIA / 2, 24.0, align=CTR
    )
    for x, y in bolt_circle(6, 54.0):
        hub = hub - Pos(x, y, 0) * Cylinder(11.0, 24.0, align=CTR)
    return Compound(children=[mat(tire, "URETHANE"), mat(hub, "PETG")])


# ---------------------------------------------------------------------------
# 板の肉抜き
#
# ⚠ **丸穴を並べる方式（`lighten` / `lighten_path`）は 2026-08-07 に捨てた。**
#   等間隔のグリッドも、荷重経路を避けて径を変える方式も、出来上がりは
#   「穴が主役の板」で、残る材料の幅が径とピッチの引き算の余りで決まる。
#   いま板を抜くのは次の 3 つだけ:
#       `topo_plate`   … トポロジー最適化した外形（構造板。`scripts/topo_opt.py`）
#       `plate_iso`    … アイソグリッド（幅のある板）
#       `plate_slots`  … 長穴（帯のように細長い板）
#   考え方と、丸穴で何が駄目だったかは `src/lattice.py` の冒頭にまとめた。
#
# 荷重の通り道（帯）の考え方だけは `plate_iso(bands=...)` に引き継いだ。
# 帯の半幅はせん断で決まる幅 F/(2·t·τ) とリブの下限の大きいほう。
# ---------------------------------------------------------------------------
TAU_ALLOW = 54.0        # MPa 板のせん断許容（A5052 の許容 90MPa × 0.6）
BAND_MIN = 6.0          # mm 荷重経路の帯の**半幅**の下限（リブ 1 本ぶん）
#
# ⚠ この機体の荷重（数十〜数百 N）では、帯の幅は**せん断ではなくリブの
#   下限**で決まる（t4 で 100N なら必要幅 0.5mm）。応力で決まるように
#   なるのは 2.6kN を超えてから。だから下限を持たせている。
#   ここを外すと帯が線になり、板が抜け殻になる。


# ---------------------------------------------------------------------------
# トラス肉抜き — `lattice` が出した 2D の輪郭を板にする層
#
# ⚠ **穴をブーリアンで抜かない。** 板の輪郭（外形 − 切り欠き − 穴）を 2D で
#   組み立ててから、**1 回だけ押し出す**。丸穴のころの註（「穴は 1 個ずつ
#   抜く。まとめて渡すと OCCT が道具どうしの交差を全部解いて二次で遅く
#   なる」）は、そもそも道具を渡すのをやめれば消える問題だった。
# ⚠ さらに、**輪郭も `Sketch` の引き算で作ってはいけない**。build123d の
#   スケッチ演算は 1 回ごとにブーリアンを回す。実測（motor_mount の板・
#   穴 15 個・1 個 30 頂点）:
#       Sketch から穴を 15 回引く …  9.11 秒
#       Face + make_holes（1 回）  …  0.05 秒（180 倍）
#   押し出しも「t/2 を 2 回押して融合」は融合に 5.7 秒かかる。
#   **t を 1 回押して t/2 下げる**と 0.05 秒（体積は 41,588mm³ で一致）。
# ---------------------------------------------------------------------------
def _truss_solid(profile: dict, t: float, material: str = "A5052"):
    """`lattice` が返した輪郭（外周 + 穴）を、板厚 `t` に押し出す。"""
    from build123d import Face, Wire

    def _wire(ring):
        return Wire.make_polygon([(float(x), float(y), 0.0) for x, y in ring],
                                 close=True)

    face = Face(_wire(profile["outer"]))
    if profile["holes"]:
        face = face.make_holes([_wire(h) for h in profile["holes"]])
    # ⚠ **押し出す向きを `dir` で明示すること。** `amount` だけ渡すと
    #   **面の法線**へ押し出すが、法線は輪郭の巻き方向で決まる。
    #   `lattice` の輪郭は shapely の差集合から取っているので、切り欠きの
    #   有無で外周の向きが変わる。実際 rail_plate（t4）は法線が −Z になり、
    #   z 0..4 のつもりが −4..0 に出て、そこから t/2 下げたので
    #   **板が 4mm ずれた**（組み上げると y 336..340 が 340..344 になり、
    #   凍結してある M5 16 本の座面が全部 0/12 になった）。
    #   `_topo_solid` が「t/2 を 2 回押して融合」しているのは、
    #   （遅いかわりに）この向きの問題を踏まない書き方だったため。
    return mat(extrude(face, amount=t, dir=(0, 0, 1)).moved(Pos(0, 0, -t / 2)),
               material)


def plate_iso(size_x: float, size_y: float, t: float, *, cell: float,
              rib: float = 8.0, edge: float | None = None,
              fillet: float | None = None, guards=(), bands=(), cuts=(),
              angle: float = 0.0, origin=(0.0, 0.0),
              material: str = "A5052", label: str = ""):
    """アイソグリッドで肉抜きした板（板中心が原点・板厚 Z）。

    返り値は `(板, 内訳 dict)`。内訳は `lattice.iso_holes` のもの。

    `cell`   … 三角形の一辺。板の短辺のおよそ 1/3 が目安（3 列入る）
    `rib`    … リブ幅。板のどこでもこの幅が残る。アルミ板は 6mm が下限
               （`scripts/ligament_check.py` の `MIN_LIG`）
    `edge`   … 縁の残肉。既定は max(rib, 2×板厚)
    `guards` … 抜いてはいけない円 [(x, y, r)]。座面・軸穴・ボルトの座
    `bands`  … 抜いてはいけない帯 [(x0, y0, x1, y1, 半幅)]。荷重の通り道
    `cuts`   … 先に切り欠いてある矩形 [(cx, cy, w, h)]

    ⚠ `guards` は**相手の面から取ること**（`F.face` / `F.seat_on`）。板の
      局所座標で手打ちすると、相手を動かしたとき保護円だけ取り残されて
      座面の真ん中に穴が開く（`lighten_path` と同じ註）。
    """
    import lattice as _LT
    edge = max(rib, 2.0 * t) if edge is None else edge
    prof, rep = _LT.iso_holes(size_x, size_y, cell=cell, rib=rib, edge=edge,
                              fillet=fillet, guards=guards, bands=bands,
                              cuts=cuts, angle=angle, origin=origin,
                              label=label or "plate_iso")
    if rep["parts"] != 1:
        raise ValueError(
            f"plate_iso({label}): 肉抜きで板が {rep['parts']} 個に分断された")
    return _truss_solid(prof, t, material), rep


def plate_slots(size_x: float, size_y: float, t: float, *, length: float,
                width: float, rib: float = 8.0, edge: float | None = None,
                along: str = "x", rows: int = 1, offset: float = 0.0,
                guards=(), bands=(), cuts=(), material: str = "A5052",
                label: str = ""):
    """長穴（スタジアム形）で肉抜きした帯板（板中心が原点・板厚 Z）。

    幅がリブ 3 本ぶんも無い帯板ではアイソグリッドの三角形が成立しない
    （内接円が `rib/2 + 丸め` より小さくなる）。そういう板はこちら。
    `along` は長穴を並べる向き（板の長手）。
    """
    import lattice as _LT
    edge = max(rib, 2.0 * t) if edge is None else edge
    prof, rep = _LT.slot_holes(size_x, size_y, length=length, width=width,
                               rib=rib, edge=edge, along=along, rows=rows,
                               offset=offset, guards=guards, bands=bands,
                               cuts=cuts, label=label or "plate_slots")
    if rep["parts"] != 1:
        raise ValueError(
            f"plate_slots({label}): 肉抜きで板が {rep['parts']} 個に分断された")
    return _truss_solid(prof, t, material), rep


# ---------------------------------------------------------------------------
# シンギュレータ（1 枚ずつに捌く機構）の部品
# ---------------------------------------------------------------------------
# ⚠ 以前は「タイヤ + 芯」を 1 つの Compound で返していた（`pick_roller()`）。
#   形は同じでも、**部品としては 1 個**なので
#     ・摩擦材が減ったときに何を交換するのかが図に無い
#     ・タイヤとハブの嵌め合い（どちらが受けでどちらが入るか）が宣言できない
#   という状態だった。摩擦材は布と擦れて必ず減る**消耗品**なので、
#   ハブと別部品にして単独で抜けるようにする。
PICK_TIRE_T = 3.0                   # ウレタンタイヤの肉厚
PICK_HUB_OD = P.PICK_ROLLER_DIA - 2 * PICK_TIRE_T   # 34: タイヤの内径


@proto_cache
def pick_tire():
    """分離ローラーの摩擦タイヤ（ウレタン t3・回転軸 Z）。

    内径 φ34 のスリーブ。ハブへ**圧入**して摩擦だけを担う消耗品。
    """
    return mat(Cylinder(P.PICK_ROLLER_DIA / 2, P.PICK_ROLLER_W, align=CTR)
               - Cylinder(PICK_HUB_OD / 2, P.PICK_ROLLER_W + 2, align=CTR),
               "URETHANE")


@proto_cache
def pick_hub():
    """分離ローラーのハブ（3Dプリント PETG・回転軸 Z）。

    ⚠ 無垢の円盤だと 1 個 15.4g ×5 = 77g ある。**タイヤを支えるのは
      外周のリムだけ**なので、内側は抜いてよい。φ7 の穴を PCD22 に 5 つで
      12.0g／個（-3.4g）。リブは 6.9mm 残るので PETG でも裂けない。
    ⚠ 穴を φ9 まで広げるとリムが 2mm・リブが 3.5mm になり、圧入の締めしろで
      リムが内側へ逃げてタイヤが空転する。φ7 で止める。
    """
    hub = (Cylinder(PICK_HUB_OD / 2, P.PICK_ROLLER_W, align=CTR)
           - Cylinder(P.SING_SHAFT_DIA / 2, P.PICK_ROLLER_W + 2, align=CTR))
    for x, y in bolt_circle(5, 22.0):
        hub -= Pos(x, y, 0) * Cylinder(3.5, P.PICK_ROLLER_W + 2, align=CTR)
    return mat(hub, "PETG")


@proto_cache
def flange_bush(bore: float, od: float, ln: float, flange_od: float,
                flange_t: float = 1.5):
    """フランジブッシュ（回転軸 Z）。原点＝胴の中心、鍔は **-Z 側**に出る。

    ⚠ 「フランジブッシュ 2 個」と注記だけ書いて実体が無いと、軸は
      軸受ブロックの穴（＝切りっぱなしのアルミの縁）で直接受けることに
      なる。実機では軸が削れるか、穴が広がって隙間が出る。
      注記に書いた部品は描く。描けないなら注記のほうを消す。
    """
    body = Cylinder(od / 2, ln, align=CTR)
    fl = Pos(0, 0, -(ln + flange_t) / 2) * Cylinder(flange_od / 2, flange_t,
                                                    align=CTR)
    return mat((body + fl) - Cylinder(bore / 2, ln + 2 * flange_t + 2, align=CTR),
               "POM")


@proto_cache
def retard_clip(w: float, wall_t: float, t: float, run: float, leg: float):
    """リタードパッドの台。前壁の**上端に馬乗りで被せる** U 字クリップ。

    原点 = 壁の上端の面・壁厚の中心。板厚 `t` の脚が壁の両面を挟み、
    web は -X 端から +X へ `run` 伸びる。その上面がパッドの座。

    ⚠ プラダン（中空板）の**切り口**にパッドを直接貼ることはできない。
      板の中はリブと空洞で、接着できる実肉は 1mm も無い。
      縁に金具を被せて、板の**両面**（リブの通っている面）で受ける。
    ⚠ 山側（-X）の脚は壁の内面より板厚ぶん（1mm）出っ張る。布はここを
      登ってくるので、実機では脚の下端に面取りかテープを入れて段を消すこと。
      1mm なら引っかからないが、脚を厚くするとここが引っかかりの元になる。
    """
    x_lo = -(wall_t / 2 + t)
    body = Pos(x_lo + run / 2, 0, t / 2) * Box(run, w, t, align=CTR)
    for sx in (-1, 1):
        # ⚠ 脚と web を「面で突き合わせる」と、稜線だけで繋がった 2 個の
        #   ソリッドになりかねない（組立検査の「分断」で出る）。
        #   脚を web の高さまで通して**体積で重ねる**。
        body += Pos(sx * (wall_t + t) / 2, 0, (t - leg) / 2) * Box(
            t, w, leg + t, align=CTR)
    return mat(body, "A5052")


@proto_cache
def retard_pad(w: float, ln: float, t: float, land: float = 1.0):
    """リタードパッド（シリコン）。原点 = 底面・-X 端。

    上面（Z = t）が「1 枚だけ通す門」の高さ。-X 側（雑巾が入ってくる側）は
    45° の逆傾斜にして、乗り上げた 2 枚目をここで噛ませる。

    ⚠ 斜面は**入口側にだけ**付ける。出口側にも付けると、乗り越えた 2 枚目が
      押し戻されずそのまま滑り出る（斜面が坂を下る向きになる）。
    ⚠ 入口の角を 0mm まで削ぐと、シリコンが刃のように薄くなって欠ける。
      `land` だけ平らな縁を残す。
    """
    pad = Pos(ln / 2, 0, t / 2) * Box(ln, w, t, align=CTR)
    # z = x + land より上を削ぐ = 45° の斜面（入口に land の縁が残る）
    pad -= Pos(0, 0, land) * Rot(0, -45, 0) * Box(
        400.0, w + 2, 400.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return mat(pad, "SILICONE")

# ---------------------------------------------------------------------------
# 計測輪（オドメトリ輪）— 回転軸 Z・中心原点
# ---------------------------------------------------------------------------
# ⚠ **オムニは「円柱 + 材質 URETHANE」では描けない。**
#   もとのウレタンタイヤは中実の円柱で描いてあったが、オムニに替えると
#   「横に転がれる」ことが図の上のどこにも現れない。ローラーを実際に
#   並べておくと、幅（双列ぶん 20mm）も、列を半ピッチずらしてあることも、
#   ローラーがスカートやメカナムとどれだけ離れているかも、そのまま出る。
@proto_cache
def odo_omni():
    """計測輪の双列オムニホイール φ50×W20（購入品の外形）。

    回転軸 Z、中心原点。ローラーの軸は**車輪の面内で円周に接する向き**で、
    半径 `a = R - r` の円上に並ぶ（`a + r = R` なので、ローラーの腹が
    ちょうど φ50 の包絡線に乗る）。

    ⚠ **ローラーを 1 列にしない。** 単列だとローラーとローラーのあいだで
      接地半径が落ち、転動径が周期的に変わる = 距離のスケールが波打つ。
      2 列を半ピッチ（18°）ずらして、片方の谷を相手の腹で埋める。
    ⚠ **ブーリアンは一括にしない**（メカナムと同じ理由）。ローラー 20 本を
      1 回で融合するとカーネルが落ちる。1 本ずつ足してから包絡線で切る。
    """
    R = P.ODO_OMNI_OD / 2
    r = P.ODO_OMNI_ROLLER_R
    a = R - r                       # ローラー軸の半径（腹が φ50 に乗る）
    n = P.ODO_OMNI_ROLLERS
    step = 360.0 / n
    rollers = None
    for row in (-1, 1):
        z = row * P.ODO_OMNI_ROW_OFF
        for i in range(n):
            deg = i * step + (step / 2 if row > 0 else 0.0)
            c = (Rot(0, 0, deg) * Pos(a, 0, z) * Rot(90, 0, 0)
                 * Cylinder(r, P.ODO_OMNI_ROLLER_L, align=CTR))
            rollers = c if rollers is None else rollers + c
    # 外板 2 枚 + 中板。ローラーの内端（半径 a-r）まで届かせて融合させる
    plate_r = a - r + 2.0
    hub = (Pos(0, 0, P.ODO_OMNI_W / 2 - 1.0) * Cylinder(plate_r, 2.0, align=CTR)
           + Pos(0, 0, -(P.ODO_OMNI_W / 2 - 1.0))
           * Cylinder(plate_r, 2.0, align=CTR)
           + Cylinder(plate_r, 3.0, align=CTR)
           + Cylinder(P.ODO_OMNI_BORE / 2 + 4.0, P.ODO_OMNI_W, align=CTR))
    # 包絡線で切る。ローラーは幅方向にも半径 r だけ張り出すので、
    # ここで Z も φ50 も同時に落ちる（切らないと幅が 20 に収まらない）。
    clip = Cylinder(R, P.ODO_OMNI_W, align=CTR)
    # ⚠ 材質は URETHANE（黒褐色）。**PETG（ピンク＝3Dプリント）にしない。**
    #   色は「自校で作る部品かどうか」を一目で見せるための符丁で、
    #   オムニは購入品（`export_fab` でも購入扱い）。
    return mat((hub + rollers) & clip
               # 変換ハブのボスが通るボア（+0.2 のすきま）
               - Cylinder(P.ODO_OMNI_BORE / 2 + 0.1, P.ODO_OMNI_W + 2, align=CTR),
               "URETHANE")


@proto_cache
def odo_hub():
    """オムニ ⇄ φ5 軸の変換ハブ（3Dプリント PETG）。つばが +Z 側。

    ⚠ **市販のオムニは φ5 のボアでは売っていない。** かといって軸を太く
      すると軸受が MR105ZZ（外径 10）から外れ、ボスがアーム板からはみ出す。
      あいだに変換ハブを 1 つ入れるのがいちばん影響が小さい。
    ⚠ **つばだけで持たない。** ボスをオムニのボア（φ12）に通し切って、
      幅 20mm ぜんぶで芯を出す。つば 1 枚に 3-M3 で留めただけだと、
      車輪が首を振って転動径が変わる。
    """
    # ⚠ ボスは**外側の面をオムニとツラ**にし、内側だけ 3mm 伸ばす。外へ
    #   出すとスカート内面（車輪の外面から 5mm）に当たり、内へ伸ばしすぎると
    #   回らない軸受ボス（d=14 から）と擦る。
    t = P.ODO_HUB_FLG_T                         # つば厚
    ln = P.ODO_OMNI_W + t
    boss = Pos(0, 0, t / 2) * Cylinder(P.ODO_OMNI_BORE / 2 - 0.1, ln, align=CTR)
    flange = Pos(0, 0, P.ODO_OMNI_W / 2 + t / 2) \
        * Cylinder(P.ODO_OMNI_PCD / 2 + 4.0, t, align=CTR)
    return mat((boss + flange)
               # 軸は圧入。0.1 のしめしろ（ちょうどにすると接線になる）
               - Cylinder(P.ODO_SHAFT_D / 2 - 0.05, P.ODO_OMNI_W + 12, align=CTR),
               "PETG")


@proto_cache
def odo_lm_rail(length: float | None = None):
    """ミニチュアリニアガイドのレール MGN9（購入品）。

    原点＝レール断面の中心・**長さ方向は Z**、断面の幅は X、高さは Y で、
    取付面（板に当たる面）が Y = +H/2 に来る向き。
    """
    ln = P.ODO_LM_RAIL_L if length is None else length
    return mat(Box(P.ODO_LM_RAIL_W, P.ODO_LM_RAIL_H, ln, align=CTR), "STEEL")


@proto_cache
def odo_lm_block():
    """ミニチュアリニアガイドのブロック MGN9C（購入品）。

    レールと同じ向き（長さ方向 Z）。原点＝**レール断面の中心**に合わせて
    あるので、レールとブロックは同じ位置に置けば嵌まる。

    ⚠ **溝はレールの実寸ちょうどに開ける。** 摺動面は「接するが重ならない」
      （`tr_fix.HOW["SLIDE"]`）。すきまを付けると検査が「離れ」で落ちるし、
      食い込ませると当然「食い込み」で落ちる。
    """
    # 取付面（レール断面の +Y 側の面）からブロック上面まで H=10。
    # レール高さ 6.5 は**そのうちの内数**なので、ブロックが取付面から
    # 反対側へ張り出すのは 3.5mm しかない。ここを 10mm と読み違えると、
    # 車輪外周との逃げが 6.5mm 甘く出る。
    # ⚠ **袖の端は取付面まで届かない。** 実物は 1mm 手前で止まる（届いたら
    #   レールを留めるボルトの頭を舐める）。ここを取付面ツラで描くと、
    #   ブロックが取付板と面で当たり、`assembly_check` に「未宣言接触」で
    #   出る（可動部が固定部に触れているのだから、指摘のほうが正しい）。
    gap = 1.0
    h = P.ODO_LM_BLOCK_H - gap
    body = Pos(0, P.ODO_LM_RAIL_H / 2 - gap - h / 2, 0) \
        * Box(P.ODO_LM_BLOCK_W, h, P.ODO_LM_BLOCK_L, align=CTR)
    groove = Box(P.ODO_LM_RAIL_W, P.ODO_LM_RAIL_H, P.ODO_LM_BLOCK_L + 2,
                 align=CTR)
    return mat(body - groove, "STEEL")


@proto_cache
def odo_band(span: float):
    """予圧の輪ゴム 1 本。原点＝2 本のピンの中点、張る向きは Z。

    ⚠ **輪ゴムは「円柱」ではなく、閉じた輪**として描く。1 本のロープで
      描くと、ピンに掛かっている（＝外れない）ことも、ピンの間隔が
      伸び率そのものであることも図に出ない。断面は切幅 1.1 × 厚 1.1。
    ⚠ **2 条の帯を並べただけにしない。** 端をつながずに置くと、
      `assembly_check` が「2 ソリッドが 2 塊に離れている（分断）」と出す
      ——そしてそれは正しい。実物は端でピンを回り込んでいる。
      内側の輪郭を「直線 span + 両端 R」のスタジアム形にして、
      外側へ肉厚ぶん太らせた**輪**を作る。
    """
    t = 1.1                                     # 帯の厚み（ピン → 外側へ）
    w = 1.1                                     # 切幅（ピンの軸方向）
    # ピンへ 0.2mm 食い込ませる。**掛かっていることを図に出す**ためで、
    # 接線で置くと「宣言した相手と接していない」と「接している」の
    # あいだで判定が揺れる（`how="CLAMP"` の許容 300mm³ に対して 20mm³）。
    r_in = P.ODO_BAND_PIN_D / 2 - 0.2
    r_out = r_in + t

    def stadium(r):
        return (Pos(0, 0, 0) * Box(w, 2 * r, span, align=CTR)
                + Pos(0, 0, span / 2) * Rot(0, 90, 0) * Cylinder(r, w, align=CTR)
                + Pos(0, 0, -span / 2) * Rot(0, 90, 0) * Cylinder(r, w, align=CTR))

    return mat(stadium(r_out) - stadium(r_in), "RUBBER")


@proto_cache
def odo_bearing():
    """計測輪の玉軸受 MR105ZZ（5×10×4）の外形。

    ⚠ 汎用の `bearing()` は外輪・内輪を肉厚 3mm で描くので、
      外径 10 / 内径 5 では「外輪 r2.5..5.5・内輪 r2.5..5.5」と重なって
      中間輪が空になる（形が壊れる）。小径軸受は 1 個の輪で近似する。
    """
    return mat(Cylinder(P.ODO_BRG_OD / 2, P.ODO_BRG_W, align=CTR)
               # 内径は軸径 + 0.2（片側 0.1 のすきま）。ちょうどにすると接線。
               - Cylinder(P.ODO_SHAFT_D / 2 + 0.1, P.ODO_BRG_W + 2, align=CTR),
               "STEEL")


@proto_cache
def lvl_spring():
    """LiDAR レベリング座の押し戻しばね。原点＝ばねの中心、軸は Z。

    **素線までは描かない**（干渉に効くのはコイル外径のエンベロープだけで、
    素線を描いても設計判断は 1 つも変わらない。STEP が重くなるだけ）。
    高さは中立位置のすきま（`LIDAR_LVL_GAP`）そのもの。調整で ±3mm
    伸び縮みするが、**縮む側で密着しないこと**が効く条件。
    自由長 20 → 中立 12 → 最短 9 で、密着高さ 8.8 に当たらない
    （当たると、そこから先はばねではなく板が突っ張る＝調整代が名目だけになる）。
    """
    assert P.LIDAR_LVL_GAP - P.LIDAR_LVL_ADJ > P.LIDAR_LVL_SPRING_SOLID, \
        "いちばん締めた隅でばねが密着する（調整代が実質ゼロ）"
    return mat(Cylinder(P.LIDAR_LVL_SPRING_OD / 2, P.LIDAR_LVL_GAP, align=CTR)
               - Cylinder(P.LIDAR_LVL_SPRING_ID / 2, P.LIDAR_LVL_GAP + 2, align=CTR),
               "STEEL")
