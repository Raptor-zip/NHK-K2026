"""表示器（Elecrow SH080 / 1280×800）に映す画を作る → `tex/screen.png`.

    cd cad && .venv/bin/python promo/screen.py

⚠ **表示内容を CAD の形として入れない。** 画面に出ているのは画素で、
  厚みのある部品ではない。ロゴを 0.3mm の板として組立に足すと、STEP にも
  部品表にも「実在しない部品」が 1 個増え、質量・干渉・製作データの全部が
  それを本物として扱う。形として持つのは**有効表示領域（`disp_screen`）まで**で、
  そこに何が映っているかはテクスチャの側の話にする。

⚠ **画素数は実品に合わせる（1280×800）。** 表示器のドットピッチより細かい
  文字を描いても実機では出ない。ここで読めない大きさは実機でも読めない。

映すのはピット・立ち上げ用の画面（`DESIGN.md` §4.5.7 の機上計算機が出す）。
数値は**表示例**であって、実機の値をここに焼き込んでいるわけではない。
"""

from __future__ import annotations

import math
import os

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "tex", "screen.png")

W, H = 1280, 800
SS = 2                      # 描くときだけ 2 倍（PIL の図形は縁が階段になる）

# 機体の色に合わせる。シアンは画面の地の黒に対して最も明るく見える色で、
# ピンクは 3D プリント部品（PETG）＝機体の差し色と同じ。
BG_TOP = (7, 10, 14)
BG_BOT = (14, 20, 27)
CYAN = (63, 216, 255)
PINK = (255, 107, 166)
WHITE = (232, 240, 245)
GREY = (120, 136, 150)
LINE = (34, 46, 58)

FONTS_JA = ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/ipaexfont-gothic/ipaexg.ttf")
FONTS_EN = ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",)


def font(size: int, ja: bool = False) -> ImageFont.FreeTypeFont:
    """使える最初のフォントを返す。⚠ 和文は CJK 対応でないと豆腐になる。"""
    for path in (FONTS_JA if ja else FONTS_EN):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    raise FileNotFoundError(f"フォントが無い: {FONTS_JA if ja else FONTS_EN}")


def gradient(w: int, h: int, top, bot) -> Image.Image:
    img = Image.new("RGB", (1, h))
    px = img.load()
    for y in range(h):
        t = y / max(1, h - 1)
        px[0, y] = tuple(round(a + (b - a) * t) for a, b in zip(top, bot))
    return img.resize((w, h), Image.BILINEAR)


def track(dr: ImageDraw.ImageDraw, xy, text: str, fnt, fill, spacing: float):
    """字間を空けて描く（PIL に letter-spacing は無い）。返すのは右端 x。"""
    x, y = xy
    for ch in text:
        dr.text((x, y), ch, font=fnt, fill=fill)
        x += dr.textlength(ch, font=fnt) + spacing
    return x - spacing


def mark(dr: ImageDraw.ImageDraw, cx: int, cy: int, r: int) -> None:
    """ロゴマーク — 「絞った雑巾が的まで飛ぶ弧」.

    競技（雑巾を投げて当てる）そのものを、的・弧・雑巾の 3 つだけで描く。
    ⚠ 著作権の絡む意匠は使えない（規定 3.1.3）ので、競技の要素から起こす。
    """
    # 的（同心円 3 本）。右上に置く
    tx, ty = cx + int(r * 0.46), cy - int(r * 0.42)
    for rad, wid, col in ((r * 0.52, r * 0.085, CYAN),
                          (r * 0.33, r * 0.075, CYAN),
                          (r * 0.13, 0, PINK)):
        box = (tx - rad, ty - rad, tx + rad, ty + rad)
        if wid:
            dr.ellipse(box, outline=col, width=int(wid))
        else:
            dr.ellipse(box, fill=col)

    # 投擲の弧（2 次ベジェ）。的の中心へ入る。先へ行くほど細くする
    p0 = (cx - r * 0.92, cy + r * 0.78)
    p1 = (cx - r * 0.30, cy - r * 1.10)      # 制御点（山）
    p2 = (tx - r * 0.30, ty + r * 0.30)
    n = 160
    for i in range(n + 1):
        t = i / n
        u = 1 - t
        x = u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0]
        y = u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1]
        rad = r * (0.075 - 0.045 * t)
        dr.ellipse((x - rad, y - rad, x + rad, y + rad), fill=WHITE)

    # 雑巾（弧の根元）。絞ったねじれが分かるように少し傾けた四角
    s = r * 0.30
    ang = 0.38
    ca, sa = math.cos(ang), math.sin(ang)
    quad = [(p0[0] + (dx * ca - dy * sa), p0[1] + (dx * sa + dy * ca))
            for dx, dy in ((-s, -s), (s, -s), (s, s), (-s, s))]
    dr.polygon(quad, fill=PINK)


def build() -> Image.Image:
    w, h = W * SS, H * SS
    base = gradient(w, h, BG_TOP, BG_BOT)

    # 光る要素（マーク・見出し）は別レイヤに描いてぼかし、地に足してから
    # もう一度くっきり描く。⚠ これが無いと「点いている画面」に見えない。
    glow = Image.new("RGB", (w, h), (0, 0, 0))
    gd = ImageDraw.Draw(glow)
    layer = Image.new("RGB", (w, h), (0, 0, 0))
    dr = ImageDraw.Draw(layer)

    f_head = font(30 * SS, ja=True)
    f_word = font(104 * SS)
    f_sub = font(27 * SS, ja=True)
    f_lbl = font(21 * SS)
    f_val = font(38 * SS)
    f_unit = font(22 * SS)

    m = 62 * SS

    # --- 上の帯 ----------------------------------------------------------
    track(dr, (m, 44 * SS), "TR — 投擲ロボット", f_head, WHITE, 2 * SS)
    right = "READY"
    fx = w - m - dr.textlength(right, font=f_head)
    dr.text((fx, 44 * SS), right, font=f_head, fill=CYAN)
    dot = 9 * SS
    dr.ellipse((fx - 26 * SS - dot, 58 * SS - dot, fx - 26 * SS + dot, 58 * SS + dot),
               fill=CYAN)
    dr.line((m, 100 * SS, w - m, 100 * SS), fill=LINE, width=2 * SS)

    # --- ロゴ ------------------------------------------------------------
    mr = 118 * SS
    mcx, mcy = m + mr + 10 * SS, 330 * SS
    for d in (gd, dr):
        mark(d, mcx, mcy, mr)

    wx = mcx + mr + 74 * SS
    for d in (gd, dr):
        track(d, (wx, 262 * SS), "FLAGSHIP", f_word, WHITE, 6 * SS)
    track(dr, (wx + 4 * SS, 392 * SS), "高専ロボコン2026 — 高専杯 雑巾投擲選手権",
          f_sub, GREY, 1 * SS)

    # --- 下の状態表示 ----------------------------------------------------
    # ⚠ 表示例。実機の値ではない
    dr.line((m, 566 * SS, w - m, 566 * SS), fill=LINE, width=2 * SS)
    cells = (("BATT", "24.1", "V"), ("YAW", "+12.0", "°"),
             ("PITCH", "42.0", "°"), ("RAG", "3", "/3"))
    cw = (w - 2 * m) / len(cells)
    for i, (lbl, val, unit) in enumerate(cells):
        x = m + cw * i
        dr.text((x, 606 * SS), lbl, font=f_lbl, fill=GREY)
        vx = x + dr.textlength(val, font=f_val)
        dr.text((x, 640 * SS), val, font=f_val, fill=WHITE)
        dr.text((vx + 7 * SS, 651 * SS), unit, font=f_unit, fill=CYAN)
        if i:
            dr.line((x - 26 * SS, 606 * SS, x - 26 * SS, 690 * SS),
                    fill=LINE, width=2 * SS)

    glow = glow.filter(ImageFilter.GaussianBlur(radius=22 * SS))
    img = ImageChops.add(base, Image.eval(glow, lambda v: int(v * 0.55)))
    img = ImageChops.screen(img, layer)
    return img.resize((W, H), Image.LANCZOS)


def main() -> None:
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    build().save(OUT)
    print(f"表示器の画 {W}×{H} → {OUT}")


if __name__ == "__main__":
    main()
