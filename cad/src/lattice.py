"""トラス肉抜きの 2D 生成器（アイソグリッド / スロット）.

なぜ丸穴をやめるのか
--------------------
2026-08-07 まで、板の肉抜きは `tr_lib.lighten()`（等間隔グリッド）と
`lighten_path()`（荷重経路を避けて径を変える）で**丸穴**を並べていた。
丸は「穴どうしを rib 以上離せば必ず一続きになる」ことが自明なので実装が
楽で、最初の一手としては正しかった。ただし出来上がりは
**穴が主役の板**になる:

* 残る材料が「穴と穴のすきま」でしかないので、幅が場所ごとにばらつく。
  いちばん細いところで強度が決まるのに、そこは設計値ではなく
  ピッチと径の**引き算の余り**として決まっている
* 丸穴どうしのあいだにできる材料は、**荷重の向きと無関係**な曲がった帯に
  なる。板の中で力が通る向き（面内せん断なら ±45°、曲げなら長手）に
  沿っていないので、同じ質量なら三角格子のほうが確実に硬い
* 見た目が「等間隔の水玉」になり、切削品というより穴あきパンチング板に見える

ここで作るのは**リブが主役**の肉抜き。まずリブの網（＝トラス）を決め、
その隙間を穴として抜く。だから

* リブ幅は**どこでも一定**（`rib`）。最小残肉が設計値そのものになる
* リブは直線で、格子の向きは指定できる。荷重の向きに合わせられる
* 三角形は面内で**筋交いが入った状態**なので、同じ肉でせん断に強い
  （四角格子は平行四辺形に潰れる。丸穴の残肉はその中間）

対応する 2 つの格子
-------------------
`iso_holes`  … 正三角形の格子（アイソグリッド）。**幅のある板**用。
               ロケットの燃料タンクや航空機の胴体パネルで使う定番。
`slot_holes` … 長穴（スタジアム形）を並べる。**帯のように細長い板**用。
               三角形の内接円が入らない幅（およそ 4×rib 以下）では
               三角格子が成立しないので、こちらを使う。

共通の作り方（どちらも同じ関数を通る）
  1. 板の輪郭から**縁の残肉** `edge` を削った領域を「抜いてよい範囲」にする
  2. 座面・軸穴・ボルトなど**抜いてはいけない円** `guards` を、
     半径 + `rib` だけ膨らませて範囲から除く。荷重の通り道 `bands` は
     膨らませずに（帯の幅がそのまま残るように）除く
  3. 格子のセル（三角形 / 長方形）を並べ、各セルを `rib/2 + fillet` だけ
     内側へ縮め、`fillet` だけ膨らませて**角を丸めた穴**にする。
     隣り合うセルどうしは rib/2 + rib/2 = `rib` だけ離れる
  4. 範囲からはみ出す穴は**範囲で切り詰める**（縁のリブでスパッと切る）。
     切り口は丸め直し、元の半分未満に痩せた穴は開けない
  5. 抜いたあとの材料が一続きであることを確かめる

⚠ 穴が 1 つも入らない組み合わせは例外にする。`lighten()` と同じ理由で、
  黙って何もしないと**呼んだこと自体が効果の証拠に見えてしまう**。
"""

from __future__ import annotations

from math import cos, radians, sin, sqrt

Ring = list[tuple[float, float]]

# 角の丸めを何本の直線で近似するか（1/4 円あたり）。円弧のままでは
# build123d の `Wire.make_polygon` に渡せないので、ここで折れ線にする。
# ⚠ 3 では丸めが多角形に見える。6 なら弦の誤差は半径の 0.86%
#   （φ16 の角で 0.07mm）で、切削の公差に埋まる。
QUAD_SEGS = 6
MIN_HOLE_AREA = 120.0     # mm² これより小さい穴は開けない（切削の意味が無い）


def _sh():
    """shapely を遅延 import する（読み込みが重い）。"""
    from shapely import geometry as g
    from shapely.ops import unary_union
    return g, unary_union


# ---------------------------------------------------------------------------
# 抜いてよい範囲
# ---------------------------------------------------------------------------
def free_region(size_x: float, size_y: float, *, edge: float, grow: float = 0.0,
                guards=(), bands=(), cuts=(), outer: Ring | None = None):
    """穴を置いてよい領域（shapely の Polygon / MultiPolygon）を返す。

    `outer` … 板の外形。省略すると `size_x × size_y` の矩形（板中心が原点）
    `cuts`  … 板から先に切り欠いてある矩形 [(cx, cy, w, h)]。
              ⚠ 切り欠きの**縁**にも残肉が要るので、ここも `edge` を見る
    `guards`… 抜いてはいけない円 [(x, y, r)]。座面・軸穴・ボルトの頭。
              半径そのものを渡す（clearance は `grow` で足す）
    `bands` … 抜いてはいけない**帯** [(x0, y0, x1, y1, 半幅)]。締結の座から
              荷重点へ力が流れる道。`tr_lib.lighten_path` が楕円で近似して
              いたものと同じ役割で、ここでは線分の周りのカプセルにする。
              格子は帯を避けて切り詰められるので、板には**荷重の通り道が
              無垢の筋として残り、その外がトラスになる**
    `grow`  … **保護円**を膨らませる量。リブ 1 本ぶん（＝ `rib`）を渡す

    ⚠ **帯は `grow` で膨らませない。** 帯そのものが残したい材料なので、
      穴が帯の縁に接して初めて幅が設計値（2×半幅）になる。ここに
      `rib` を足すと帯が 2×rib だけ太り、板が抜けなくなる。実際
      mount_brk（130×120 t4）で足していたときは 1,343mm² しか抜けず、
      丸穴のころ（2,791mm²）より**重い板**になっていた。
    """
    g, _ = _sh()
    dom = (g.Polygon(outer) if outer
           else g.box(-size_x / 2, -size_y / 2, size_x / 2, size_y / 2))
    for cx, cy, w, h in cuts:
        dom = dom.difference(g.box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))
    free = dom.buffer(-edge, quad_segs=QUAD_SEGS)
    for gx, gy, gr in guards:
        free = free.difference(
            g.Point(gx, gy).buffer(gr + grow, quad_segs=QUAD_SEGS * 2))
    for x0, y0, x1, y1, hw in bands:
        free = free.difference(
            g.LineString([(x0, y0), (x1, y1)]).buffer(hw,
                                                      quad_segs=QUAD_SEGS * 2))
    return dom, free


def _rounded(cell_ring: Ring, rib: float, fillet: float):
    """セルを rib/2 縮め、角を `fillet` で丸めた穴にする。"""
    g, _ = _sh()
    poly = g.Polygon(cell_ring)
    inner = poly.buffer(-(rib / 2.0 + fillet), quad_segs=QUAD_SEGS)
    if inner.is_empty:
        return None
    return inner.buffer(fillet, quad_segs=QUAD_SEGS)


MIN_FRAC = 0.45           # 切り詰めた穴は元の面積のこれ以上を保つこと


def _place(cells, free, rib: float, fillet: float, clip: bool = True,
           smooth: float | None = None, min_frac: float = MIN_FRAC):
    """セル列を穴にして、`free` に収まるものだけ返す。

    `clip=True` … `free` からはみ出すセルは**捨てずに切り詰める**。
      捨てるだけにすると板の縁に格子 1 つ分の無地の帯が残り、
      「格子を貼り忘れた板」に見える（実物のアイソグリッドパネルは
      パターンが縁のリブでスパッと切られている）。

    切り詰めた穴には 2 つの後処理をかける。どちらも**見た目の問題**だが、
    ここを通さないと丸穴をやめた意味が半分無くなる:

      * 開き（`smooth` だけ縮めてから膨らませる）… 切り口の角を丸め直し、
        同時に幅が 2×smooth 未満のヒゲを消す
      * 面積が元のセルの `min_frac` 未満になったら**開けない** … 縁に
        三日月やかけらが並ぶのを止める。かけらは軽量化にもほぼ効かない

    ⚠ 切り詰めた穴が**内側に島を抱えた**ら、その穴ごと捨てる。保護円が
      セルの内側に丸ごと入るとそうなり、島（＝座面）が板から切り離される。
    """
    smooth = (fillet if fillet else rib / 2.0) if smooth is None else smooth
    holes = []
    for ring in cells:
        h = _rounded(ring, rib, fillet)
        if h is None or h.is_empty:
            continue
        full = h.area
        # ⚠ `within` は境界の接触を許す。縁の残肉ちょうどで接するのは
        #   設計どおりなので `contains` ではなくこちらを使う。
        if not h.within(free):
            if not clip:
                continue
            h = h.intersection(free)
            if h.is_empty:
                continue
            h = h.buffer(-smooth, quad_segs=QUAD_SEGS) \
                 .buffer(smooth, quad_segs=QUAD_SEGS)
        for p in (h.geoms if h.geom_type == "MultiPolygon" else [h]):
            if p.geom_type != "Polygon" or p.is_empty:
                continue
            if p.area < max(MIN_HOLE_AREA, min_frac * full) or p.interiors:
                continue
            holes.append(p)
    return holes


def _ring(coords) -> Ring:
    return [(round(float(x), 4), round(float(y), 4)) for x, y in coords[:-1]]


def _profile(dom, holes, label: str) -> dict:
    """板の輪郭を `{"outer": 外周, "holes": 穴}` にする。

    ⚠ 切り欠き（`cuts`）が板の内側に丸ごと入っていると、外形のほうに
      穴ができる。それも `holes` に混ぜて返す（呼ぶ側は「外周 1 本 +
      穴 n 本」だけを見ればよくなる）。
    """
    if dom.geom_type != "Polygon":
        raise ValueError(f"{label}: 切り欠きで板が {len(dom.geoms)} 枚に割れている")
    return {"outer": _ring(dom.exterior.coords),
            "holes": [_ring(h.exterior.coords) for h in holes]
                     + [_ring(i.coords) for i in dom.interiors]}


def _connected(dom, holes) -> int:
    """穴を抜いたあと、材料がいくつの塊になるかを返す。"""
    _, unary_union = _sh()
    left = dom.difference(unary_union(holes))
    return 1 if left.geom_type == "Polygon" else len(left.geoms)


# ---------------------------------------------------------------------------
# アイソグリッド（正三角形の格子）
# ---------------------------------------------------------------------------
def iso_cells(size_x: float, size_y: float, cell: float,
              angle: float = 0.0, origin=(0.0, 0.0)) -> list[Ring]:
    """一辺 `cell` の正三角形で平面を敷き詰め、板の bbox を覆う分だけ返す。

    行 j は y ∈ [j·h, (j+1)·h]（h = cell·√3/2）を上向き三角と下向き三角で
    埋める。次の行を cell/2 ずらすと、行の境目で三角形の底辺どうしが
    重なり、**水平のリブが 1 本の直線につながる**（＝アイソグリッド）。
    ずらさないと境目でリブが半ピッチ食い違い、格子が崩れて見える。

    ⚠ 格子は**局所座標の x = 0 について左右対称**になるよう、行をまるごと
      h/2 下げてある。下げないと板の中心線に行の境目（＝水平リブ）が来て、
      パターンが左右で半ピッチずれる。板は左右対称に作るものが多く、
      そこだけ非対称だと「格子の割り付けを間違えた板」に見える。
      `angle=90` を渡せば、この対称軸が板の X 軸（＝上下対称）になる。
    """
    h = cell * sqrt(3) / 2.0
    # 回転させても bbox を覆えるように、対角線ぶん広く取る
    reach = (size_x ** 2 + size_y ** 2) ** 0.5 / 2.0 + cell
    nj = int(reach / h) + 2
    ni = int(reach / cell) + 2
    ca, sa = cos(radians(angle)), sin(radians(angle))
    ox, oy = origin

    def xf(x, y):
        return (ox + x * ca - y * sa, oy + x * sa + y * ca)

    cells: list[Ring] = []
    for j in range(-nj, nj + 1):
        y0, y1 = j * h - h / 2, (j + 1) * h - h / 2
        off = (j % 2) * cell / 2.0
        for i in range(-ni, ni + 1):
            x0 = i * cell + off
            cells.append([xf(x0, y0), xf(x0 + cell, y0),
                          xf(x0 + cell / 2, y1)])
            cells.append([xf(x0 + cell / 2, y1), xf(x0 + cell * 1.5, y1),
                          xf(x0 + cell, y0)])
    return cells


def iso_holes(size_x: float, size_y: float, *, cell: float, rib: float,
              edge: float, fillet: float | None = None, guards=(), bands=(),
              cuts=(), angle: float = 0.0, origin=(0.0, 0.0), clip: bool = True,
              min_frac: float = MIN_FRAC, outer: Ring | None = None,
              label: str = ""):
    """アイソグリッドで肉抜きした板の輪郭を返す。

    返り値は `(輪郭, 内訳 dict)`。輪郭は `{"outer": 外周, "holes": 穴}`、
    内訳は `{"n": 穴の数, "area": 抜いた面積 mm², "rib": リブ幅,
    "cell": 一辺, "parts": 残った材料の塊の数}`。
    """
    fillet = rib if fillet is None else fillet
    # 三角形の内接円半径は cell/(2√3)。rib/2 + fillet がそれ以上だと
    # 縮めた時点で消える＝穴が 1 つも開かない。呼ぶ前に弾く。
    if rib / 2.0 + fillet >= cell / (2.0 * sqrt(3)):
        raise ValueError(
            f"iso_holes({label}): 一辺 {cell:.0f} の三角形に リブ {rib:.0f} ／"
            f"丸め {fillet:.0f} は入らない（内接円 {cell / (2 * sqrt(3)):.1f}mm）")
    dom, free = free_region(size_x, size_y, edge=edge, grow=rib, guards=guards,
                            bands=bands, cuts=cuts, outer=outer)
    if free.is_empty:
        raise ValueError(
            f"iso_holes({label}): 縁の残肉 {edge:.0f}mm と保護円 "
            f"{len(guards)} 個で、抜いてよい範囲が残らない")
    holes = _place(iso_cells(size_x, size_y, cell, angle, origin), free,
                   rib, fillet, clip=clip, min_frac=min_frac)
    if not holes:
        raise ValueError(
            f"iso_holes({label}): 穴が 1 つも入らない（板 {size_x:.0f}×"
            f"{size_y:.0f}, 一辺 {cell:.0f}, リブ {rib:.0f}, 縁 {edge:.0f}）。"
            f"cell を小さくするか edge を詰めること")
    return _profile(dom, holes, f"iso_holes({label})"), \
        {"n": len(holes), "area": sum(h.area for h in holes),
         "rib": rib, "cell": cell, "parts": _connected(dom, holes)}


# ---------------------------------------------------------------------------
# 長穴（帯のように細長い板用）
# ---------------------------------------------------------------------------
def slot_holes(size_x: float, size_y: float, *, length: float, width: float,
               rib: float, edge: float, along: str = "x", rows: int = 1,
               guards=(), bands=(), cuts=(), clip: bool = True,
               offset: float = 0.0, min_frac: float = MIN_FRAC,
               outer: Ring | None = None, label: str = ""):
    """長手方向に並べた長穴（スタジアム形）で肉抜きした板の輪郭を返す。

    返り値は `iso_holes` と同じ `(輪郭, 内訳 dict)`。

    `length`/`width` … 穴そのものの長さ・幅（丸めた後の寸法）
    `rows`            … 短手方向に何列並べるか
    `offset`          … 板中心から最初の長穴の中心までの距離。0 なら
                        中心に 1 本置いてそこから両側へ並べる

    ⚠ 三角格子は**内接円が rib/2 + fillet より大きい**ことが要る。板幅が
      リブ 3 本ぶんも無い帯では成立しないので、こちらを使う。長穴の丸みは
      端の半円（半径 width/2）で、これも切削で素直に出せる。
    ⚠ 長穴の位置は**板中心について必ず左右対称**に振る（`i·pitch` を
      -n..n で回すのではなく、± の対で作る）。板の中心を挟んで保護円が
      対称にあるのに穴だけ片側へ寄ると、割り付けを間違えたように見える。
    """
    dom, free = free_region(size_x, size_y, edge=edge, grow=rib, guards=guards,
                            bands=bands, cuts=cuts, outer=outer)
    if free.is_empty:
        raise ValueError(
            f"slot_holes({label}): 縁の残肉 {edge:.0f}mm と保護円 "
            f"{len(guards)} 個で、抜いてよい範囲が残らない")
    long_x = along == "x"
    span = size_x if long_x else size_y
    across = size_y if long_x else size_x
    pitch = length + rib
    n = int((span + rib) / pitch) + 2
    # 長手方向の中心。0 を挟んで ± の対で並べる（左右対称）
    longs = [0.0] if offset == 0.0 else []
    for i in range(0 if offset else 1, n + 1):
        c_ = offset + i * pitch
        longs += [c_, -c_]
    # ⚠ スタジアム形は「矩形を width/2 縮めてから膨らませる」では作らない。
    #   縮め量が高さのちょうど半分になるので、shapely の精度しだいで
    #   空になったり残ったりする（幅 28 では通り、幅 20 では
    #   「穴が 1 つも入らない」で落ちた）。**線分を width/2 で膨らませる**
    #   のが定義そのもので、精度にも寄りかからない。
    g, _ = _sh()
    half = max(0.0, (length - width) / 2.0)
    cy_step = width + rib          # 短手方向の並び。rows=1 なら中心 1 本
    cells: list[Ring] = []
    for k in range(rows):
        c = (k - (rows - 1) / 2.0) * cy_step
        for c_long in longs:
            a, b = (c_long, c) if long_x else (c, c_long)
            seg = ([(a - half, b), (a + half, b)] if long_x
                   else [(a, b - half), (a, b + half)])
            stad = g.LineString(seg).buffer(width / 2.0, quad_segs=QUAD_SEGS)
            cells.append(_ring(stad.exterior.coords))
    # セル自体が最終寸法なので、`_place` には縮め代も丸めも渡さない
    holes = _place(cells, free, 0.0, 0.0, clip=clip,
                   smooth=width * 0.3, min_frac=min_frac)
    if not holes:
        raise ValueError(
            f"slot_holes({label}): 穴が 1 つも入らない（板 {size_x:.0f}×"
            f"{size_y:.0f}, 長穴 {length:.0f}×{width:.0f}, 縁 {edge:.0f}、"
            f"短手の空き {across:.0f}）")
    return _profile(dom, holes, f"slot_holes({label})"), \
        {"n": len(holes), "area": sum(h.area for h in holes),
         "rib": rib, "cell": length, "parts": _connected(dom, holes)}
