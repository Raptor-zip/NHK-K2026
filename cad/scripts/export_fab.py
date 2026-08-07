"""製作データを書き出す — 加工屋・プリンタにそのまま渡せるファイルにする.

    python scripts/export_fab.py            # out/fab/ 以下へ全部書き出す
    python scripts/export_fab.py --md       # 製作リスト（Markdown）
    python scripts/export_fab.py --only tine0

なぜ要るか
----------
`BOM.md` には**部品名と数量しか無い**。CNC もレーザーも 3D プリンタも、
要るのは名前ではなく形なので、あの表だけでは 1 点も発注できない。
STEP は 3 コンフィグ**まるごと 1 ファイル**（tr_match.step）で出ていて、
部品 1 個を取り出す手段が無かった。

出すもの
--------
    out/fab/dxf/<部品>.dxf   平板の外形 + 穴（板ローカルの 2D、単位 mm）
    out/fab/stl/<部品>.stl   3D プリント（PETG）
    out/fab/step/<部品>.step **作れない部品**（直すための参考。発注用ではない）
    out/fab/manifest.json    部品 → 工程・材質・板厚・寸法・ファイル（機械が読む）
    FAB.md                   製作リストと板取り（`--md` の出力。人が読んで発注に使う）

⚠ `FAB.md` は `BOM.md` と同じくリポジトリ直下に置く（`check_all` が書く）。
  `cad/out/*` は .gitignore 対象なので、そこへ置くと**発注に使った表が
  履歴に残らない**。ファイル本体（DXF/STL/STEP）は再生成できるので out/ でよい。

⚠ **姿勢は POSE_FLAT（可動軸を全部 0 にした姿勢）で採る。** 可動部の板は
  姿勢のぶんだけ回っているので、競技姿勢のまま投影すると板が斜めになり、
  厚み方向が軸に乗らない。`plate_audit` が板の寸法を測るときと同じ理由。

平板かどうかの判定
------------------
**体積 ÷ 厚み = 投影面積**（かつ、その軸に垂直な面の面積の合計 = 投影面積 × 2）
が成り立てば、その軸方向に一様な板。角度の付いた面・座ぐり・段差があると
必ず崩れるので、「切り抜いて済む形」だけが通る。板厚は bbox の最小辺では
決めない（曲げ板や L 形は最小辺が板厚ではない）。

⚠ **判定が付かないものを平板として出してはいけない。** 外形だけそれらしい
  DXF が出ると、加工屋はそれを信じて切る。

作れないものは「作れない」と出す
--------------------------------
平板でも 3D プリントでも購入でもないものは `NG` にする。前は `MILL`
（削り出し）という受け皿があって 93 部品がそこへ落ち、「STEP を渡せば作れる」
ように見えていた。**自校では 3D の削り出しも曲げもできない**（上の `CAN_*`）ので、
その受け皿は嘘だった。作れないものを作れないと出さないと、設計が制約に寄らない。

曲げ品も同じ。展開図は出さない（そもそも曲げられないし、展開長は曲げる機械の
内 R と K ファクタで決まる値で、こちらで仮定すると穴の位置を間違えたまま
確定させることになる）。⚠ 一度自動展開を試して失敗している。`bend_hint()` の註。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

from build123d import (Location, Plane, Solid, Vector,  # noqa: E402
                       export_step, export_stl)
from build123d.exporters import ColorIndex, ExportDXF, Unit  # noqa: E402

import plate_audit as PA  # noqa: E402
import tr_params as P  # noqa: E402

OUT = os.path.join(ROOT, "out", "fab")

# ---------------------------------------------------------------------------
# 自校の加工能力（2026-08-04 に確認）
# ---------------------------------------------------------------------------
# ⚠ **アルミ板は曲げられない（板金ができない）。**
#   できるのは
#     ・平板の 2D 切り抜き（外形と通し穴）
#     ・板の**端面への横穴**（ドリル / タップ）
#   ここまで。3D の削り出し・ポケット・段差は**できない**。
#   PETG は 3D プリントなので形は自由。それ以外は買う。
#
# だから設計は「平板を直交させ、端面タップで留める」に寄せる。曲げ耳
# （`L.tabbed_plate`）や L 金具（`L.angle_hanger`）は**そのままでは作れない**。
# ⚠ この制約を知らずに図を出すと、**作れない部品の DXF が加工屋に渡る**。
#   平板でもプリントでも購入でもないものは `作れない` として落とす。
CAN_BEND = False          # 板金（曲げ）
CAN_MILL_3D = False       # 3D の削り出し（ポケット・段差・自由曲面）
CAN_EDGE_HOLE = True      # 板の端面への横穴（ドリル / タップ）

# --- 平板の判定 -------------------------------------------------------------
# ⚠ **作れるかどうかを決めるのは「板厚」ではなく「一様な断面か」。**
#   2D で切り抜ける形なら、板厚が 20mm でも 30mm でも切れる（厚板から
#   輪郭を切り出すだけ）。逆に t3 でも段差やポケットがあれば削り出しになる。
#   ここを 14mm で切っていたので、`hinge_blk`（24×24×20 の単純な角材）まで
#   「作れない」に落ちていた。判定は `PRISM_T_MAX` まで許す。
PRISM_T_MAX = 60.0    # 一様断面とみなす厚みの上限（これ以上は材料が無い）
T_MAX = 14.0          # 「板金の板」と呼ぶ上限（板取りの表で使う）
FLAT_TOL = 0.02       # 体積 ÷ 厚み と 投影面積 のずれの許容（2%）
AREA_MIN = 100.0      # mm² これ未満は DXF にしても意味が無い

# --- 材質ごとの工程と、買える板厚 -------------------------------------------
SHEET_MAT = {
    "A5052": ("CNC切削 / レーザー", (0.8, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)),
    "SUS304": ("レーザー / ウォータージェット", (1.0, 1.5, 2.0, 2.5, 3.0)),
    "PC": ("レーザー（ポリカ）", (0.8, 1.0, 1.5, 2.0, 3.0, 5.0)),
    "PP_DANPLA": ("カッター / トムソン", (2.5, 3.0, 4.0, 5.0)),
    "TEKCELL": ("カッター", (5.0, 10.0)),
    "SILICONE": ("トムソン / ハサミ", (1.0, 2.0, 3.0, 5.0)),
    "URETHANE": ("ウォータージェット", (2.0, 3.0, 5.0, 10.0)),
    "POM": ("CNC切削", (2.0, 3.0, 5.0, 10.0)),
}
# 標準の板取り寸法（発注単位）。A5052 は 1000×2000 の 1/4 サイズが一般的。
SHEET_SIZE = {"A5052": (500.0, 1000.0), "SUS304": (500.0, 1000.0),
              "PC": (600.0, 900.0), "PP_DANPLA": (910.0, 1820.0),
              "TEKCELL": (910.0, 1820.0), "SILICONE": (300.0, 300.0),
              "URETHANE": (300.0, 300.0), "POM": (300.0, 500.0)}

PRINT_MAT = ("PETG",)

# --- 対象外（買うもの・実体ではないもの）------------------------------------
# ⚠ **理由を書かせる。** 面倒になってここへ足すのを防ぐ（`plate_audit.SKIP`
#   と同じ運用）。ここに入るものは「発注リストに部品図が要らない」という
#   宣言なので、間違えると**図の無い部品が黙って消える**。
BUY_MAT = {
    "MOTOR": "DJI M3508 / M2006（購入）",
    "MOTOR_SHAFT": "モーターの出力軸（購入品の一部）",
    "PCB": "制御基板（購入 or 別途設計）",
    "BATTERY": "6S LiPo（購入）",
    "SENSOR": "LiDAR / カメラ（購入）",
    "ESTOP": "非常停止スイッチ（購入）",
    "RUBBER": "タイミングベルト（購入）",
    "CABLE": "配線束（実体ではなく取り回しの表示）",
    "PLYWOOD": "椅子の座板（規定で載せる購入品）",
    "ADC12": "MISUMI 5 シリーズ ブラケット（購入）",
    "A6005C": "MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト）",
    "MASCOT": "マスコット（EPP + フェルト。規定 3.1.3 で重量制限外）",
    "MASCOT_SUIT": "マスコットのつなぎ", "MASCOT_TRIM": "マスコットの三角巾",
    "MASCOT_RAG": "マスコットが持つ雑巾", "MASCOT_DARK": "マスコットの靴・瞳",
}
BUY_PREFIX = {
    "scr": "ねじ・リベット（fasteners_bom.md の員数表で発注）",
    "cab_": "配線束", "wire_": "配線", "rag_": "雑巾（競技で配られる）",
    # ⚠ `rail_` で丸ごと外してはいけない。**レール取付プレート
    #   `rail_plate_L/R`（A5052 t4・肉抜き）は自校加工品**で、これを落とすと
    #   BOM に載っている板が製作データから消える。段と玉だけを外す。
    "rail_in_": "MISUMI SRX3616 インナーレール（購入）",
    "rail_mid_": "MISUMI SRX3616 中間レール（購入）",
    "rail_out_": "MISUMI SRX3616 アウターレール（購入）",
    "rail_ball_": "SRX3616 のボールリテーナ（購入品の一部）",
    "odo_wheel": "計測輪の双列オムニホイール φ50×W20（購入）",
    "odo_brg": "MR105ZZ 玉軸受（購入）", "odo_mag": "エンコーダ用磁石（購入）",
    "odo_rail": "リニアガイド MGN9 レール L64（購入）",
    "odo_blk": "リニアガイド MGN9C ブロック（購入）",
    # ⚠ 輪ゴムは**消耗品**。図面には出さないが、部品表からも落とすと
    #   「予圧をどうやって掛けるのか」が製作データから消える。
    "odo_band": "予圧の輪ゴム #10（消耗品。試合ごとに掛け替える）",
    # ⚠ ばねは**外径のエンベロープ**（中空円筒）で描いてある。素線は描かない
    #   ので、そのままだと「板厚 0.8mm のスチール板」に見えて NG に落ちる。
    "lidar_lvl_spr": "LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3）",
    "beltbush": "POM フランジブッシュ（購入）",
    "pitch_wbrg": "6001ZZ 玉軸受（購入）", "pitch_wcol": "スラストカラー（購入）",
    "pitch_worm": "ウォーム m1.5 1条（購入）",
    # ⚠ 軸は「削り出し」ではなく**丸棒を切るだけ**。図を出しても加工屋は
    #   困る（長さの表だけでよい）。BOM の「軸受・軸・止め輪」に入っている。
    "sing_shaft": "φ8 中空軸（切断のみ）",
    "beltshaft_": "φ10 鋼軸（切断のみ）",
    "lift_shaft_": "φ5 鋼軸（切断のみ）",
    "roller_shaft_": "φ12 中空軸 A2017（切断のみ）",
    "odo_shaft": "φ5 鋼軸（切断のみ）",
    # ⚠ **タイミングプーリは買うもの**。歯形は削れないし、BOM §3 に
    #   「HTD5M 20T/60T」「24T ×4」として**予算に入っている**。A5052 の
    #   ソリッドで描いてあるのは外形を見るためで、製作品ではない。
    #   これを「削り出し」に出していたので、作れない部品が 18 個水増しされていた。
    "beltpul_": "HTD5M タイミングプーリ（購入。BOM §3）",
    "grab_pul_": "HTD5M タイミングプーリ（購入。BOM §3）",
    "lift_pul_": "HTD5M タイミングプーリ（購入。BOM §3）",
    "ramp_pul_": "HTD5M タイミングプーリ（購入。BOM §3）",
    "roller_pul_": "HTD5M 24T タイミングプーリ（購入。BOM §3）",
    "yaw_pulley": "HTD5M タイミングプーリ 20T/60T（購入。BOM §3）",
    # 軸継手も歯形と同じで買うもの（BOM §3「φ8-φ12 カップリング」）。
    "pitch_wcplg": "クランプ式軸継手 φ8-φ12（購入。BOM §3）",
    "sing_cplg": "クランプ式軸継手 φ8-φ8（購入。BOM §3）",
    # 軸受まわり。ブロックごと買う（ミニチュアピロー / フランジ軸受）。
    "lift_brg_": "フランジ軸受ユニット（購入）",
    "sing_brg_": "フランジ軸受ユニット（購入）",
    "odo_boss": "軸受ボス（購入 or PETG。φ14 の小物で切り抜きにならない）",
    # --- 回転体・軸・軸受・ばね類。どれも削れないし、買うのが前提 ---------
    "wheel_": "メカナムホイール φ100（購入。BOM §3）",
    "hub_": "メカナムのハブアダプタ（ホイールに付属 or 購入）",
    "roller_brg_": "深溝玉軸受（購入。BOM §3）",
    "roller_d": "射出ローラーのウレタンタイヤ t3（シートから抜く消耗品）",
    "roller_u": "射出ローラーのウレタンタイヤ t3（シートから抜く消耗品）",
    "sing_tire": "分離ローラーのウレタンタイヤ t3（シートから抜く消耗品）",
    "sing_bush": "POM フランジブッシュ（購入）",
    "sing_pad": "リタードパッド シリコン t3（シートから抜く消耗品）",
    "press_pad": "上押さえのスポンジパッド（シートから抜く）",
    "press_bush": "フランジブッシュ（購入）",
    "nip_screw": "ニップ調整ねじ（購入。M8 全ねじ + ロックナット）",
    "ramp_shaft_": "φ8 鋼軸（切断のみ）",
    # ⚠ 「曲げ板の可能性」と出ていたが実体は**中空のガイド軸**（φ10×φ6）。
    #   丸パイプを切るだけ。板ではない。
    "press_guide_": "φ10×φ6 アルミ中空軸（切断のみ。上押さえのガイド）",
    # ⚠ **軸受ホルダは買う。** 内径 φ28 の座と、スラストを受ける段（φ24 / φ26）
    #   を持つ筒で、旋盤の中ぐりが要る。自校は 2D 切り抜きまでなので作れない。
    #   MISUMI の軸受ホルダ（BGRAB/BGRB 系、6001 用）がそのまま使える。
    #   図の形は購入品の**外形**（モーターや LiDAR と同じ扱い）。
    "pitch_wbrk_": "軸受ホルダ 6001 用（購入。BOM §3「ウォーム軸まわり」）",
    # 割締めブロック＝市販のシャフトクランプ / セットカラー。
    "hinge_blk_": "φ8 シャフトクランプ（購入）",
    "nip_snut_": "ニップ調整ねじの受けナット（購入。M8 フランジナット）",
    "press_clamp_": "φ10 シャフトクランプ（購入）",
    "fork_clamp": "φ8 シャフトクランプ（購入）",
    # ⚠ 実体は外径φ30・内径φ20・幅 9 の**カラー**。板から切り出すものでは
    #   なく、市販のスペーサ／カラーがそのまま使える（BOM §3「スラストカラー」）。
    #   平板判定は通るが、円環の輪郭を 2D へ落とすと平面から浮く
    #   （`write_dxf` の非平面ガードが捕まえた）。
    "pitch_pivot_": "仰角軸受カラー 外径φ30×内径φ20×幅9（購入）",
    "yaw_ext_shaft": "φ18 延長軸（切断のみ）",
    "yaw_ring": "旋回 V リング φ220（購入。BOM §3）",
}
BUY_NAME = {
    "bucket": "移動バケツ（審判が置く購入品）",
    "bucket_2l_datum": "規定 3.2.3c の目盛り位置（実体ではない）",
    "chair_5go": "新JIS 教室用椅子 5号（購入）",
    "mascot_envelope": "規定 3.1.3 の外形（実体ではない）",
    "mascot_badge": "マスコットのゼッケン",
    "desk": "補充スポットの机（会場設備）",
    "pc_mini": "制御 PC（購入）",
    "hinge_shaft": "φ8 鋼軸（切断のみ）",
    "aim_camera": "照準カメラ（購入）", "power_led": "電源表示 LED（購入）",
    # ⚠ 表示器のパネルは t14 のポリカ「板」に見えるが、実体は**表示モジュールの
    #   外形**。板として切り出すものではない（規格厚にも無い）。
    "disp_panel": "表示器モジュール（購入。t14 はモジュールの厚み）",
}


# ---------------------------------------------------------------------------
# 平板・曲げ板の判定
# ---------------------------------------------------------------------------
AXES = ("X", "Y", "Z")


def _ext(bb):
    return (bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z)


def _plane_normals(sol):
    """その部品が持つ**平面の法線**を重複無しで返す（板厚の候補）。

    ⚠ **世界の X/Y/Z だけを試してはいけない。** 板が斜めに置いてあると
      どの軸にも乗らず、平らな板が「作れない」に落ちる。実際
      ホッパーの傾斜底（`hop_floor`）と斜路のガイド板がそうだった。
      可動部の回転は `POSE_FLAT` で消せるが、**設計として傾いている板**は
      消せない。部品自身の面の向きを候補にする。
    """
    out = []
    for f in sol.faces():
        try:
            nv = f.normal_at()
        except Exception:
            continue                     # 曲面（円筒側面など）は法線が一定でない
        v = Vector(nv.X, nv.Y, nv.Z)
        if v.length < 1e-9:
            continue
        v = v.normalized()
        if v.Z < -1e-9 or (abs(v.Z) <= 1e-9 and
                           (v.Y < -1e-9 or (abs(v.Y) <= 1e-9 and v.X < 0))):
            v = Vector(-v.X, -v.Y, -v.Z)   # ±を畳む
        if not any(abs(v.dot(u) - 1.0) < 1e-6 for u in out):
            out.append(v)
    return out


def _extent_along(sol, n: Vector):
    """法線 n 方向の厚み（頂点を投影した幅）。"""
    ds = [n.dot(Vector(v.X, v.Y, v.Z)) for v in sol.vertices()]
    return (min(ds), max(ds)) if ds else (0.0, 0.0)


def slab_axis(sol, t_max: float = T_MAX, tol: float = FLAT_TOL):
    """「一様な断面の板」なら (板厚, 法線, 投影面積) を返す。無ければ None。

    判定は **体積 ÷ 厚み == 投影面積**。投影面積はその法線に垂直な面の
    面積の合計 ÷ 2 から採る。座ぐり・段差・曲げがあると必ず崩れるので、
    「そのまま切り抜けば済む形」だけが通る。
    """
    best = None
    for n in _plane_normals(sol):
        lo, hi = _extent_along(sol, n)
        e = hi - lo
        if e > t_max or e <= 0.2:
            continue
        a_face = 0.0
        for f in sol.faces():
            try:
                nv = f.normal_at()
            except Exception:
                continue
            if abs(abs(Vector(nv.X, nv.Y, nv.Z).normalized().dot(n)) - 1.0) < 1e-6:
                a_face += f.area
        if a_face <= 0:
            continue
        area = sol.volume / e
        if abs(a_face / 2 - area) / max(area, 1e-9) < tol:
            if best is None or e < best[0]:
                best = (e, n, area)
    return best


def profile_face(sol, n: Vector):
    """法線 n の**下面**（＝切り抜く輪郭）を 1 枚返す。取れなければ None。

    ⚠ 面が複数に割れているものは返さない。割れているのは段差・座ぐりが
      ある印で、1 枚の輪郭では表せない（DXF にすると嘘になる）。
    """
    faces = []
    for f in sol.faces():
        try:
            nv = f.normal_at()
        except Exception:
            continue
        if abs(abs(Vector(nv.X, nv.Y, nv.Z).normalized().dot(n)) - 1.0) < 1e-6:
            faces.append(f)
    if not faces:
        return None
    off = [n.dot(Vector(f.center().X, f.center().Y, f.center().Z)) for f in faces]
    lo = min(off)
    bot = [f for f, o in zip(faces, off) if abs(o - lo) < 1e-6]
    return bot[0] if len(bot) == 1 else None


def bend_hint(sol, mat: str, t_max: float = T_MAX) -> bool:
    """「板金の曲げ品かもしれない」という**目安**。数値は出さない。

    ⚠ **ここで展開長を出そうとして 1 度失敗している。** 「同じ板厚のスラブが
      2 枚、直交して繋がっている」を面積と体積の式だけで判定したところ、
      6 件が曲げ板と判定され、その中身が

          roller_shaft_d  辺 1574.0 + 3.2 / 展開長 1573.8   ← 実物は φ12 の中空軸
          press_guide_L   辺  513.2 + 3.0 / 展開長  512.8   ← 辺 3.0 は曲げではない

      と、**部品の全長 602mm より長い辺**が出るような値だった。逆に本当の
      曲げ板（`mount_brk_*` = モーターマウント）は 1 枚も引っかからなかった。
      面積の式だけでは、円筒の端面や小さな段差が「もう 1 枚のスラブ」に化ける。

    そして**展開長は本来こちらで決める値ではない**。内 R と K ファクタは
    曲げる機械（V 幅・パンチ R）で決まるので、板金屋は自分の値で STEP から
    展開する。推測した K で作った DXF を渡すのは、**穴の位置を間違えたまま
    確定させる**ことになる（t4・K±0.1 で 0.6mm ずれる）。

    → 曲げ品には **STEP を渡す**。ここでは「板金材料で、板厚が薄く、
      平板判定が付かなかった」という**人が見るための目安**だけを出す。
    """
    if mat not in SHEET_MAT:
        return False
    ext = sorted(_ext(sol.bounding_box()))
    return ext[0] <= t_max and ext[2] > 3 * ext[0]


# ---------------------------------------------------------------------------
# DXF / STL / STEP を書く
# ---------------------------------------------------------------------------
def face_to_xy(face, n: Vector):
    """法線 n の面を、XY 平面のローカル座標へ移す。

    ⚠ **どちら側から見た図かを必ず持ち回ること。** 板は左右対称の対で
      作ってあるものが多く（`*_L` / `*_R`）、裏返しに切ると**鏡像の部品**が
      できる。外形だけ見ても気づかない（穴の位置で初めて分かる）。
      ここは常に **法線 +n の側から見た図**にし、`manifest.json` と
      `FAB.md` にその向きを書く。
    """
    c = face.center()
    # n に垂直な適当な軸を面内 X に取る（n と平行でないものを選ぶ）
    seed = Vector(0, 0, 1) if abs(n.dot(Vector(0, 0, 1))) < 0.9 else Vector(1, 0, 0)
    x_dir = seed.cross(n).normalized()
    return Plane(origin=(c.X, c.Y, c.Z), z_dir=(n.X, n.Y, n.Z),
                 x_dir=(x_dir.X, x_dir.Y, x_dir.Z)).to_local_coords(face)


def normal_str(n: Vector) -> str:
    """法線を人が読める向きにする（軸に乗っていれば軸名）。"""
    for i, ax in enumerate(AXES):
        u = Vector(*(1.0 if j == i else 0.0 for j in range(3)))
        if abs(abs(n.dot(u)) - 1.0) < 1e-6:
            return f"+{ax} から見た図"
    return f"法線 ({n.X:+.3f}, {n.Y:+.3f}, {n.Z:+.3f}) から見た図"


PLANAR_TOL = 0.01     # mm 2D へ落とした輪郭が平面に載っているとみなす許容


def assert_planar(face2d, name: str):
    """XY へ移した輪郭が本当に平面に載っているか（頂点の Z の幅）。

    ⚠ **頂点だけでは足りない。** 円弧は端点しか頂点を持たないので、
      弧の途中が平面から浮いていても 0 と出る。実測でも、この値が
      0 なのに `ExportDXF` は「6 points found outside the XY plane」と
      警告していた。**判定は書き出しの警告そのものを捕まえる**
      （`write_dxf`）。ここは補助的な目安。
    """
    zs = [v.Z for v in face2d.vertices()]
    return (max(zs) - min(zs)) if zs else 0.0


def write_dxf(path: str, face):
    """外形を OUTLINE、穴を HOLE レイヤに分けて書く。

    ⚠ レイヤを分けるのは**加工屋が外形と穴を区別できるようにする**ため。
      1 レイヤに混ぜると、どれが外周でどれが内側かは形からしか分からず、
      入れ子の判定を相手にさせることになる。
    """
    ex = ExportDXF(unit=Unit.MM)
    # ⚠ 色は `ColorIndex` の**実在する名前**しか受け取らない（WHITE は無い）。
    #   存在しない名前を渡すと AttributeError で 1 枚も書けないまま
    #   「ファイル None」だけが manifest に残る。
    ex.add_layer("OUTLINE", color=ColorIndex.GREEN)
    ex.add_layer("HOLE", color=ColorIndex.RED)

    # ⚠ **`ExportDXF` は平面でない形も「警告を出して」書いてしまう。**
    #   警告は標準出力へ流れるだけなので `FAB.md` には何も残らず、
    #   **平面でない DXF がそのまま加工屋に渡る**。実際 2 部品で出ていた
    #   （斜めの板を扱えるようにした `_plane_normals` の副作用で、法線が
    #   僅かにずれると輪郭が平面から浮く）。
    # ⚠ 捕まえ方を 2 回間違えた。記録として残す:
    #     1. 輪郭の**頂点**の Z 幅を見た → 円弧は端点しか頂点を持たないので
    #        途中が浮いていても 0 と出る。素通りした。
    #     2. `ex.write()` の標準出力を捕まえた → 警告が出るのは
    #        **`add_shape()` の中**（`exporters.py:1172`）。write では遅い。
    #   実際に数えているのは `_non_planar_point_count` なので、
    #   **`add_shape` のたびにそれを読む**。private だが公開 API が無い。
    def _add(shape, layer):
        ex.add_shape(shape, layer=layer)
        n = getattr(ex, "_non_planar_point_count", 0)
        if n:
            raise ValueError(f"輪郭が平面に載っていない（XY 平面の外に {n} 点）"
                             "。法線の取り方か、そもそも平板でない")

    _add(face.outer_wire(), "OUTLINE")
    for w in face.inner_wires():
        _add(w, "HOLE")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    ex.write(path)


def local_solid(sol):
    """部品をその bbox の中心（Z は最小）へ寄せた実体。STEP / STL 用。

    ⚠ 世界座標のまま書き出すと、部品 1 個の STEP が原点から 1.4m 離れた
      場所に浮く。加工屋の CAM も 3D プリンタのスライサも「原点の近く」を
      前提にしている。

    ⚠ **`Solid(sol.wrapped)` で裸のソリッドに作り直してから動かす。**
      組立から取り出したソリッドは `label` と木の親子（`topo_parent`）を
      Python 側の属性で持っている。`export_step` はラベルの付いた形を
      **アセンブリとして**書こうとし、子が無いので空の文書になって
      `RuntimeError: Failed to write STEP file` で落ちる。9 部品
      （mount_brk_* / press_post_* / odo_arm1,2 / odo_brk1,2 / hop_floor）が
      これで書き出せていなかった。切り分けの実測:

          mount_brk_fl  raw=NG  nolabel=NG  solidwrap=OK  compound=OK
          belt_clamp_L  raw=OK  nolabel=OK  solidwrap=OK  compound=OK
                        （落ちるのは label が付いていて loc が単位のもの）

      ラベルを消すだけでは直らない（属性ではなく木の側の問題）。
      `wrapped`（幾何の実体）から作り直すのがいちばん確実で、しかも
      TShape を共有するので複製の費用も掛からない。
    """
    try:
        bare = Solid(sol.wrapped)
    except Exception:
        bare = sol            # TopoDS_Solid でないもの（殻・複合）はそのまま
    bb = bare.bounding_box()
    return bare.moved(Location((-(bb.min.X + bb.max.X) / 2,
                                -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z)))


# ---------------------------------------------------------------------------
# 3D プリントの成立性
# ---------------------------------------------------------------------------
BED = (256.0, 256.0, 256.0)   # Bambu Lab P1S / X1C の造形範囲
NOZZLE = 0.4


def print_check(sol, name: str):
    """造形範囲に入るか・薄すぎる箇所が無いかを見る。指摘の並びを返す。

    ⚠ 「置ければ刷れる」ではない。壁がノズル 2 本ぶん（0.8mm）を切ると、
      スライサは黙ってその壁を**消す**（隙間として扱う）。図の上では
      存在するのに実物に無い、という食い違いになる。
    """
    bad = []
    e = sorted(_ext(sol.bounding_box()), reverse=True)
    bed = sorted(BED, reverse=True)
    if any(a > b for a, b in zip(e, bed)):
        bad.append(f"造形範囲を超える（{e[0]:.0f}×{e[1]:.0f}×{e[2]:.0f} vs "
                   f"{bed[0]:.0f}×{bed[1]:.0f}×{bed[2]:.0f}）→ 分割して印刷")
    if e[2] < 2 * NOZZLE:
        bad.append(f"いちばん薄い方向が {e[2]:.2f}mm（ノズル 2 本 {2 * NOZZLE}mm 未満）")
    return bad


# ---------------------------------------------------------------------------
def classify(name: str, mat: str, solids):
    """(工程, 詳細, 理由) を返す。工程は CUT / PRINT / BUY / NG。

    ⚠ **`NG` は「作れない」という意味**で、逃げ道ではない。自校でできるのは
      平板の 2D 切り抜き＋端面の横穴だけなので（上の `CAN_*`）、
      平板でもプリントでも購入でもないものは**そのままでは実物にならない**。
      前は `MILL`（削り出し）という受け皿があり、93 部品がそこへ落ちて
      「STEP を渡せば作れる」ように見えていた。作れないものは作れないと
      出さないと、設計が制約に寄っていかない。
    """
    for pre, why in BUY_PREFIX.items():
        if name.startswith(pre):
            return "BUY", None, why
    if name in BUY_NAME:
        return "BUY", None, BUY_NAME[name]
    if mat in BUY_MAT:
        return "BUY", None, BUY_MAT[mat]
    if len(solids) != 1:
        return "NG", None, f"ソリッドが {len(solids)} 個。平板 1 枚では表せない"
    sol = solids[0]
    if mat in PRINT_MAT:
        return "PRINT", None, "PETG 3Dプリント"
    s = slab_axis(sol, t_max=PRISM_T_MAX)
    if s is not None and mat in SHEET_MAT:
        t, k, area = s
        f = profile_face(sol, k)
        if f is None:
            return "NG", None, ("厚み方向の面が 1 枚にならない（段差・座ぐりが"
                                "ある）。**3D の削り出しはできない**")
        if area < AREA_MIN:
            # 小さくても平板なら切れる。切るのが不経済なだけで、作れないわけではない。
            return "CUT", (t, k, area, f), ""
        return "CUT", (t, k, area, f), ""
    if s is not None and mat not in SHEET_MAT:
        return "NG", None, f"{mat} は板材として買えない（板厚 {s[0]:.1f}mm）"
    if bend_hint(sol, mat, t_max=T_MAX):
        return "NG", None, ("**曲げ板。自校では板金ができない。** 平板 2 枚を"
                            "直交させ、端面タップで留める形に直すこと")
    return "NG", None, "平板でない。**3D の削り出しはできない**"


# 板厚をそのまま使う（＝面を削らない）上限 [mm]。
# ⚠ **これより厚いものに「買える板厚」を問うてはいけない。** t9 のピボット
#   ボスや t12 のウレタンタイヤは、そもそも板から切り出すのではなく
#   **厚い材から削り出す／旋盤で挽く**もの。規格厚に無いことは不良ではない。
#   ここを分けずに全部問うと、削り出し品が「買えない板厚」として毎回出て、
#   本当に買えない薄板（t0.9 のような打ち間違い）が埋もれる。
T_STOCK_MAX = 6.0


def nearest_stock(mat: str, t: float):
    """買える板厚のうち、いちばん近いものと差を返す。厚物は問わない。"""
    stock = SHEET_MAT.get(mat, ((), ()))[1]
    if not stock or t > T_STOCK_MAX:
        return None, 0.0
    s = min(stock, key=lambda v: abs(v - t))
    return s, t - s


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--no-write", action="store_true", help="ファイルを書かず一覧だけ")
    args = ap.parse_args()

    info, _F, _unknown, _claims = PA.build_parts("flat")

    rows, notes, bad, stale = [], [], [], []
    counts = {"CUT": 0, "PRINT": 0, "NG": 0, "BUY": 0}
    manifest = {}
    for name in sorted(info):
        if args.only and args.only not in name:
            continue
        d = info[name]
        mat, solids = d["mat"], d["solids"]
        kind, extra, why = classify(name, mat, solids)
        counts[kind] += 1
        if kind == "BUY":
            notes.append((name, mat, why))
            continue
        rec = {"part": name, "material": mat, "kind": kind,
               "mass_g": round(d["mass"] * 1000, 1)}
        # ⚠ **数値は書き出しより先に決める。** 先に書き出していたので、
        #   DXF の書き出しが 1 行で落ちた（`ColorIndex.WHITE` は存在しない）
        #   だけで、板厚も面積も rec に入らないまま次へ進み、あとの板取りが
        #   `KeyError: 't'` で落ちた。**分類の結果と、ファイルが書けたかは
        #   別のこと**。混ぜると、書き出しの些細な失敗が集計ごと壊す。
        write = None
        if kind == "CUT":
            t, k, area, face = extra
            bb = face.bounding_box()
            w, h = sorted((bb.size.X, bb.size.Y, bb.size.Z), reverse=True)[:2]
            rec.update(t=t, area=round(area, 1), w=round(w, 1), h=round(h, 1),
                       holes=len(face.inner_wires()), file=f"fab/dxf/{name}.dxf",
                       view=normal_str(k))
            stock, gap = nearest_stock(mat, t)
            if stock is not None and abs(gap) > 1e-6:
                bad.append((name, f"板厚 {t:.2f}mm は買えない"
                                  f"（いちばん近い規格 t{stock:g}）"))

            def write(_f=face, _k=k, _n=name):
                flat = face_to_xy(_f, _k)
                gap = assert_planar(flat, _n)
                if gap > PLANAR_TOL:
                    raise ValueError(
                        f"2D へ落とした輪郭が平面に載っていない（Z の幅 "
                        f"{gap:.3f}mm > {PLANAR_TOL}）。DXF は書かない")
                write_dxf(os.path.join(OUT, "dxf", f"{_n}.dxf"), flat)
        elif kind == "PRINT":
            e = _ext(solids[0].bounding_box())
            rec.update(size=[round(v, 1) for v in e], file=f"fab/stl/{name}.stl")
            for m in print_check(solids[0], name):
                bad.append((name, m))

            def write(_s=solids[0], _n=name):
                export_stl(local_solid(_s), os.path.join(OUT, "stl", f"{_n}.stl"),
                           tolerance=0.05, angular_tolerance=0.15)
        else:                                       # NG（作れない）
            e = _ext(solids[0].bounding_box())
            rec.update(size=[round(v, 1) for v in e], why=why,
                       file=f"fab/step/{name}.step")
            # ⚠ STEP は**直すための参考**として出す（形を見ないと直せない）。
            #   「STEP があるから発注できる」ではない。
            bad.append((name, f"作れない: {why}"))

            def write(_s=solids[0], _n=name):
                export_step(local_solid(_s), os.path.join(OUT, "step", f"{_n}.step"))

        if not args.no_write:
            try:
                os.makedirs(os.path.join(OUT, {"CUT": "dxf", "PRINT": "stl"}
                                         .get(kind, "step")), exist_ok=True)
                write()
            except Exception as exc:                # 書き出しの失敗は握りつぶさない
                bad.append((name, f"書き出しに失敗: {type(exc).__name__}: {exc}"))
                rec["file"] = None
        rows.append(rec)
        manifest[name] = rec

    if not args.no_write and not args.only:
        # ⚠ **今回作らなかったファイルは消す。** 分類が変われば出力先も変わる
        #   （`disp_panel` は t14 の「板」から購入品へ、`odo_wheel*` は
        #   ウレタンタイヤへ移した）。残しておくと、**もう作らない部品の
        #   DXF が加工屋に渡る**。`export_meshes` が書き出しの前に
        #   `*.stl` を消しているのと同じ理由。
        #   ⚠ `--only` のときは消さない（1 部品だけ見たいときに全部消える）。
        #   ⚠ 消したことは**不良ではない**ので `bad` に入れない（毎回
        #     `check_all` が落ちることになる）。数だけ報告する。
        keep = {r["file"] for r in rows if r.get("file")}
        for sub in ("dxf", "stl", "step"):
            dpath = os.path.join(OUT, sub)
            if not os.path.isdir(dpath):
                continue
            for f in os.listdir(dpath):
                if f"fab/{sub}/{f}" not in keep:
                    os.remove(os.path.join(dpath, f))
                    stale.append(f"fab/{sub}/{f}")
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8") as fp:
            json.dump({"parts": manifest}, fp, ensure_ascii=False, indent=1)

    # --- 板取り ------------------------------------------------------------
    sheets: dict[tuple, list] = {}
    for r in rows:
        if r["kind"] != "CUT":
            continue
        # ⚠ **板厚は丸めてから束ねる。** 板厚は bbox から出した浮動小数なので、
        #   同じ t4 でも 4.0 と 4.000000001 になる。丸めずに束ねると
        #   「A5052 t4」の行が 3 つに割れ、**1 枚で足りる板を 3 枚頼む**表になる。
        sheets.setdefault((r["material"], round(r["t"], 2)), []).append(r)

    if args.md:
        print("# 製作データ（`out/fab/`）\n")
        print(f"切り抜き {counts['CUT']} / 3Dプリント {counts['PRINT']}"
              f" / **作れない {counts['NG']}** / 購入・対象外 {counts['BUY']}\n")
        print("> 自校の加工能力: アルミ板は**2D 切り抜き + 端面の横穴**まで。"
              "**曲げ（板金）と 3D 削り出しはできない。** PETG は 3D プリント。\n")
        print("## 切り抜き（DXF）\n")
        print("レイヤ **OUTLINE**＝外形、**HOLE**＝穴。単位 mm、原点は板の重心。\n")
        print("⚠ 「向き」の列は**どちら側から見た図か**。左右対称の対"
              "（`*_L` / `*_R`）を裏返しに切ると鏡像の部品ができる。\n")
        print("| 部品 | 材質 | 板厚 | 外形 mm | 面積 mm² | 穴 | 質量 g | 向き | ファイル |")
        print("|---|---|---:|---|---:|---:|---:|---|---|")
        for r in sorted((r for r in rows if r["kind"] == "CUT"),
                        key=lambda r: (-r["area"],)):
            print(f"| {r['part']} | {r['material']} | t{r['t']:g} | "
                  f"{r['w']:.0f}×{r['h']:.0f} | {r['area']:,.0f} | {r['holes']} | "
                  f"{r['mass_g']:.1f} | {r.get('view', '')} | `{r['file']}` |")
        print("\n## 3Dプリント（STL）\n")
        print("PETG。⚠ 造形範囲は "
              f"{BED[0]:.0f}×{BED[1]:.0f}×{BED[2]:.0f}mm（Bambu Lab P1S / X1C）"
              "、壁はノズル 2 本ぶん 0.8mm を下限として見ている。\n")
        print("| 部品 | 外形 mm | 質量 g | ファイル |")
        print("|---|---|---:|---|")
        for r in sorted((r for r in rows if r["kind"] == "PRINT"), key=lambda r: r["part"]):
            s = r["size"]
            print(f"| {r['part']} | {s[0]:.0f}×{s[1]:.0f}×{s[2]:.0f} | "
                  f"{r['mass_g']:.1f} | `{r['file']}` |")
        print("\n## 作れない — 直すもの（STEP は参考）\n")
        print("| 部品 | 材質 | 外形 mm | 質量 g | なぜ切り抜きにできないか |")
        print("|---|---|---|---:|---|")
        for r in sorted((r for r in rows if r["kind"] == "NG"), key=lambda r: r["part"]):
            s = r["size"]
            print(f"| {r['part']} | {r['material']} | "
                  f"{s[0]:.0f}×{s[1]:.0f}×{s[2]:.0f} | {r['mass_g']:.1f} | "
                  f"{r.get('why', '')} |")
        print("\n## 板取り（材質 × 板厚）\n")
        print("| 材質 | 板厚 | 枚数 | 合計面積 mm² | 板 1 枚 | 必要枚数(60%歩留) |")
        print("|---|---:|---:|---:|---|---:|")
        for (mat, t), lst in sorted(sheets.items()):
            a = sum(r["area"] for r in lst)
            sw, sh = SHEET_SIZE.get(mat, (500.0, 1000.0))
            need = math.ceil(a / (sw * sh * 0.6))
            print(f"| {mat} | t{t:g} | {len(lst)} | {a:,.0f} | {sw:.0f}×{sh:.0f} | {need} |")
        if stale:
            print(f"\n> 前回の書き出しに残っていた {len(stale)} 個を消した"
                  f"（もう作らない部品）: {', '.join(sorted(stale)[:8])}"
                  + ("…" if len(stale) > 8 else "") + "\n")
        print("\n## 購入・対象外\n")
        print("| 部品 | 材質 | 理由 |")
        print("|---|---|---|")
        for nm, mat, why in notes:
            print(f"| {nm} | {mat} | {why} |")
        if bad:
            print(f"\n## 手が要るもの {len(bad)} 件\n")
            for nm, why in bad:
                print(f"- `{nm}` … {why}")
    else:
        for k, v in counts.items():
            print(f"{k:6s} {v:4d}")
        print(f"\n書き出し先 {OUT}")
        for nm, why in bad:
            print(f"  ⚠ {nm}: {why}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
