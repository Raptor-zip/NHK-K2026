"""TR の URDF ジェネレータ（gen_urdf）.

設計台帳（この URDF を読む人が最初に確認すべき前提）
=====================================================
* 単位      : URDF は m / kg / rad。CAD ソース(tr_params) は mm なので MM=0.001 を掛ける。
* ルート    : base_link。原点は「床面上・メカナム4輪の接地中心」。+X=前方(射出方向)、+Y=左、+Z=上。
* メッシュ  : meshes/*.stl（mm 単位）。URDF 側で scale=0.001 を掛けて m にする。
              visual は STL、collision は簡略プリミティブ（下記 COLLISION）。
* 関節     : tr_assembly.JOINTS が唯一の情報源。STEP のポーズ生成と同じ定義を使う。
              → CAD を直せば URDF も必ず追従する。
* 質量     : LINK_MASS_KG。tr_lib.LEDGER の合計と一致することを生成時に検算する。
* 慣性     : 各リンク形状の体積慣性テンソル（build123d matrix_of_inertia）に
              「そのリンクの平均密度」を掛けて算出。部品ごとの密度差は無視した近似。
              重心はリンク形状の体積重心（同上の近似）。
* 実機との差: 3段引きスライドレールの中間レール（1/2速で動く）は URDF に含めない。
              可動は grabber_slide の 1 自由度のみで表現する（表示・制御上の実害なし）。

モーター 11 軸と関節の対応（tr_params.MOTORS と 1:1）
  wheel_fl/fr/rl/rr  M3508 P19        メカナム駆動
  turret_yaw         M3508 P19 + 3:1  砲塔ヨー ±30°
  shooter_pitch      M2006 + ウォーム40:1 仰角 20〜70°（1条・セルフロック）
  roller_upper/lower M3508 GBレス     射出ローラー（回転数差でバックスピン制御）
  singulator         M2006            ピックローラー＋斜路送給
  grabber_slide      M2006 + ベルト   フォーク前後 316mm（レールの能力は 424.6）
  grabber_press      M2006 + ラック   上押さえ昇降 80mm
"""

from __future__ import annotations

import json
import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import tr_assembly as A  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402

MM = 0.001                      # mm -> m
MESH_SCALE = (0.001, 0.001, 0.001)
MESH_DIR = "meshes"

# --- リンク質量 [kg] ------------------------------------------------------
# tr_lib.LEDGER の系統別集計を、可動リンクへ配分した値。
# 可動リンクに属さないものはすべて base_link に入る。
LINK_MASS_KG = {
    "wheel_fl": 0.580,          # メカナム 0.50 + ハブアダプタ 0.08
    "wheel_fr": 0.580,
    "wheel_rl": 0.580,
    "wheel_rr": 0.580,
    "turret_yaw": 2.240,        # 旋回テーブル・サイドプレート・ヨーM3508・プーリ・漏斗
    "shooter_pitch": 1.730,     # 仰角側板・ウォーム減速機一式・入口ガイド・射出M3508×2
                                # （ウォーム軸受 6001×2・スラストカラー×2・
                                #   カップリングを実体で描いたぶん +0.02）
    "roller_upper": 0.495,      # ローラー5個 + 中空軸
    "roller_lower": 0.495,
    "singulator": 0.280,        # ピックローラー5個 + 軸 + M2006
    "grabber_slide": 2.110,     # キャリッジ・フォーク・押さえ駆動系
    "grabber_press": 0.350,     # 押さえプレート + パッド
    "fork_tilt": 0.680,         # 櫛歯5本 + 根元バー + カムフォロア
}

# --- 関節の出力上限（減速後） --------------------------------------------
# effort[N·m or N], velocity[rad/s or m/s]
JOINT_LIMITS = {
    "wheel_fl": (3.0, 50.5), "wheel_fr": (3.0, 50.5),
    "wheel_rl": (3.0, 50.5), "wheel_rr": (3.0, 50.5),
    "turret_yaw": (9.0, 16.8),        # M3508 3.0N·m × ベルト3:1
    # ⚠ 効率は 0.7 ではなく **0.42**。セルフロックのウォーム（進み角 3.576°、
    #   摩擦角 4.868°）の正転効率は η = tanγ/tan(γ+ρ') = 0.42 しか出ない。
    #   28N·m を上限として渡すと、制御側は出せないトルクを前提に設計してしまう。
    "shooter_pitch": (16.8, 1.31),    # M2006 1.0N·m × ウォーム40:1 × 効率0.42
    "roller_upper": (0.31, 942.0),    # GBレス素モーター 20A 時
    "roller_lower": (0.31, 942.0),
    "singulator": (1.0, 52.4),
    "grabber_slide": (50.0, 0.30),    # プーリ φ40 → 推力 50N / 実運用 0.3m/s
    "grabber_press": (50.0, 0.10),
    # 傾斜はカム駆動（モーターなし）。effort はカム反力の想定値
    "fork_tilt": (30.0, 2.0),
}

# --- collision プリミティブ（簡略化は意図的） ----------------------------
# 形式: ("box", (sx, sy, sz), (cx, cy, cz)) / ("cylinder", (r, len), (cx,cy,cz), rpy)
COLLISION = {
    "wheel": ("cylinder", (P.WHEEL_DIA / 2 * MM, P.WHEEL_WIDTH * MM),
              (0.0, 0.0, 0.0), (math.pi / 2, 0.0, 0.0)),
}

# base_link は bbox 1個だと「机の下や横をすり抜けられない巨大な箱」になり、
# 装填動作のシミュレーションが成立しない。実構造に沿った直方体の集合で近似する。
# 形式: (名前, (x0,x1), (y0,y1), (z0,z1))  単位 mm・base_link 座標系
BASE_COLLISION_BOXES = [
    ("chassis", (-421, 421), (-361, 361), (P.SKIRT_Z0, P.BASE_Z1)),
    ("tower_left", (-330, 400), (338, 362), (P.BASE_Z1, P.PEDESTAL_TOP_Z)),
    ("tower_right", (-330, 400), (-362, -338), (P.BASE_Z1, P.PEDESTAL_TOP_Z)),
    # ホッパーは中実ブロックではなく壁と底で表現する（傾斜した櫛歯が中へ入る）
    ("hopper_floor", (P.HOP_X0 - 4, P.HOP_X1 + 4), (-P.HOP_Y - 4, P.HOP_Y + 4),
     (P.HOP_TOP_Z - P.HOP_DEPTH_FRONT - 20, P.HOP_TOP_Z - P.HOP_DEPTH_FRONT)),
    ("hopper_wall_l", (P.HOP_X0 - 4, P.HOP_X1 + 4), (P.HOP_Y, P.HOP_Y + 4),
     (P.HOP_TOP_Z - P.HOP_DEPTH_FRONT, P.HOP_TOP_Z)),
    ("hopper_wall_r", (P.HOP_X0 - 4, P.HOP_X1 + 4), (-P.HOP_Y - 4, -P.HOP_Y),
     (P.HOP_TOP_Z - P.HOP_DEPTH_FRONT, P.HOP_TOP_Z)),
    ("hopper_wall_rear", (P.HOP_X0 - 4, P.HOP_X0), (-P.HOP_Y - 4, P.HOP_Y + 4),
     (P.HOP_TOP_Z - P.HOP_DEPTH_FRONT, P.HOP_TOP_Z)),
    ("hopper_wall_front", (P.HOP_X1, P.HOP_X1 + 4), (-P.HOP_Y - 4, P.HOP_Y + 4),
     (P.HOP_TOP_Z - P.HOP_DEPTH_FRONT, P.HOP_TOP_Z)),
    ("feed_ramp", (P.FEED_RAMP_X0, P.FEED_RAMP_X1), (-316, 316),
     (P.FEED_RAMP_Z0 - 10, P.FEED_RAMP_Z1 + 25)),
    ("turret_pedestal", (115, 355), (-352, 352), (P.PEDESTAL_TOP_Z - 20, P.PEDESTAL_TOP_Z)),
    ("mast_left", (-286, -264), (338, 362), (P.PEDESTAL_TOP_Z, P.MAST_TOP_Z)),
    ("mast_right", (-286, -264), (-362, -338), (P.PEDESTAL_TOP_Z, P.MAST_TOP_Z)),
    # ⚠ +X 端は 150 だった（片持ちビーム L410 の時代の値）。ビームは 300 に
    #   詰めて +35 まで、受け板の耳も +65 までしか無い。
    ("mast_head", (-285, 70), (-172, 172), (P.MAST_BEAM_Z - 12, P.BUCKET_SEAT_Z)),
    ("bucket", (-70 - 140, -70 + 140), (-140, 140), (P.BUCKET_SEAT_Z, P.BUCKET_TOP_Z)),
    ("chair_mascot", (85, 415), (5, 345), (P.CHAIR_SEAT_Z, P.CHAIR_SEAT_Z + 640)),
    ("stripper", (-70, -40), (-200, 200), (P.FORK_Z, 815)),
    # ⚠ 実体から取ること。手で書いた (405,466)/(85,155) は**古い値**で、
    #   実際は x 423..485 / z 77..147 だった（走査面 120 → 112 に下げた
    #   ときに追随していない）。18mm 外へ・8mm 上へずれた箱で当てていた。
    # ⚠ 箱は**胴だけでなくレベリング座と立板の脚まで**覆うこと。脚は
    #   |Y|<=72 まで外へ出ており、下は調整ねじの先端（M4×30）まで届く。
    #   UST-20LX の検出面は底面から 47.4mm なので、走査面を中心にすると
    #   **下側が 12.4mm 足りない**。胴だけの箱にすると、机や相手機に当たる
    #   部分がシムから抜ける。
    # ⚠ **LiDAR は下向きに吊ってある。** 検出面は本体の上寄り（取付面から
    #   47.4mm 下）なので、箱は走査面から**下へ 23mm・上へ 76mm**。
    #   上側にはレベリング座（可動板・すきま・固定板）と調整ねじの頭が載る。
    ("lidar_front", (P.LIDAR_LOW_X - 45, P.LIDAR_LOW_X + 32), (-42, 42),
     (P.LIDAR_LOW_Z - 23, P.LIDAR_LOW_Z + 76)),
    ("lidar_rear", (-P.LIDAR_LOW_X - 32, -P.LIDAR_LOW_X + 45), (-42, 42),
     (P.LIDAR_LOW_Z - 23, P.LIDAR_LOW_Z + 76)),
    ("lidar_high", (P.LIDAR_HIGH_X - 36, P.LIDAR_HIGH_X + 30),
     (P.LIDAR_HIGH_Y - 42, P.LIDAR_HIGH_Y + 42),
     (P.LIDAR_HIGH_Z - 88, P.LIDAR_HIGH_Z + 72)),
]

# センサーの取付フレーム（fixed 関節）。**可動部ではない**が、
# 「どこから・どの向きに測っているか」は URDF に無いと外から分からない。
# ビューアの LiDAR シムはここを読んで走査原点にする（`viewer/scripts/urdf-shape.ts`）。
# ⚠ 車体中心から飛ばすと**別の機械の点群**になる。前後の胴は 908mm 離れていて、
#   机の脚のような細い物は片方からしか見えない。
# 形式: (名前, (x, y, z) mm, yaw deg)
SENSOR_FRAMES = [
    ("lidar_low_front", (P.LIDAR_LOW_X + 2.0, 0.0, P.LIDAR_LOW_Z), 0.0),
    ("lidar_low_rear", (-P.LIDAR_LOW_X - 2.0, 0.0, P.LIDAR_LOW_Z), 180.0),
    ("lidar_high", (P.LIDAR_HIGH_X, P.LIDAR_HIGH_Y, P.LIDAR_HIGH_Z), 0.0),
]

# グラバー可動部も bbox 1個だと櫛歯が「板」になって山の下に入れない。
GRABBER_COLLISION_BOXES = [
    # 後端横梁のみ。櫛歯の上は開けておく（山が乗る面）
    ("cross_beam", (P.RAIL_X0 + P.FORK_LEN - 25, P.RAIL_X0 + P.FORK_LEN + 65), (-313, 313),
     (P.FORK_Z, P.FORK_Z + 30)),
    ("side_bracket_l", (P.RAIL_X0, P.RAIL_X0 + P.FORK_LEN), (309, 322), (P.FORK_Z, P.FORK_Z + 65)),
    ("side_bracket_r", (P.RAIL_X0, P.RAIL_X0 + P.FORK_LEN), (-322, -309), (P.FORK_Z, P.FORK_Z + 65)),
]

# 櫛歯は fork_tilt リンク（ヒンジ軸が原点、-X へ伸びる）
FORK_COLLISION_BOXES = [
    (f"tine_{i}", (-P.FORK_LEN, 0),
     ((i - (P.FORK_TINES - 1) / 2) * P.FORK_PITCH - P.FORK_TINE_W / 2,
      (i - (P.FORK_TINES - 1) / 2) * P.FORK_PITCH + P.FORK_TINE_W / 2),
     (-P.FORK_T, 0.0))
    for i in range(P.FORK_TINES)
] + [
    # 歯先も必ず入れる。ここを省いていたせいで、シムが机の天板との干渉を
    # 見落としていた（先端が天板より 1.1mm 下にあったのに接触0回だった）。
    # 上面テーパぶんは無視して直方体で置く＝**安全側に厚く**見る。
    (f"tip_{i}", (-P.FORK_LEN - P.FORK_TIP_LEN, -P.FORK_LEN),
     ((i - (P.FORK_TINES - 1) / 2) * P.FORK_PITCH - P.FORK_TINE_W / 2,
      (i - (P.FORK_TINES - 1) / 2) * P.FORK_PITCH + P.FORK_TINE_W / 2),
     (-P.FORK_T, 0.0))
    for i in range(P.FORK_TINES)
] + [("root_bar", (-30, 0), (-310, 310), (0, 24))]

LINK_MESH = {
    "base_link": "base_link",
    "wheel_fl": "wheel_left", "wheel_rl": "wheel_left",
    "wheel_fr": "wheel_right", "wheel_rr": "wheel_right",
    "turret_yaw": "turret_yaw",
    "shooter_pitch": "shooter_pitch",
    "roller_upper": "roller", "roller_lower": "roller",
    "singulator": "singulator",
    "grabber_slide": "grabber_slide",
    "grabber_press": "grabber_press",
    "fork_tilt": "fork",
}

# ⚠ ここはリンクごとに link_*() を直接呼ぶので、同じ名前の部品を 2 回
#   put することになる（左右の車輪・上下のローラー）。組立側の同名検査を
#   生成器だけ外す。外さないと **URDF が生成できず、CAD を変えても
#   tr.urdf が古いまま残る**（実際そうなっていた）。
A.PUT_ALLOW_DUP = True

LINK_SHAPE = {
    "base_link": lambda: A.link_base(),
    "wheel_fl": lambda: A.link_wheel(1), "wheel_rr": lambda: A.link_wheel(1),
    "wheel_fr": lambda: A.link_wheel(-1), "wheel_rl": lambda: A.link_wheel(-1),
    "turret_yaw": lambda: A.link_turret_yaw(),
    "shooter_pitch": lambda: A.link_pitch(),
    "roller_upper": lambda: A.link_roller(), "roller_lower": lambda: A.link_roller(),
    "singulator": lambda: A.link_singulator(),
    "grabber_slide": lambda: A.link_carriage(),
    "grabber_press": lambda: A.link_press(),
    "fork_tilt": lambda: A.link_fork(),
}


def _v3(v) -> str:
    return f"{v[0]:.6g} {v[1]:.6g} {v[2]:.6g}"


# 慣性計算から除外する形状（マスコットは規定上「重量に含まない」外装物。
# 300×300×600 の外形エンベロープをそのまま質量分布に入れると重心が大きく狂う）
INERTIA_SKIP = ("mascot",)


def _leaf_solids(shape, skip=()):
    """ラベルパスで除外しながら葉ソリッドを集める。"""
    out = []

    def walk(node, path):
        label = getattr(node, "label", "") or ""
        newpath = f"{path}/{label}" if label else path
        if any(s in newpath for s in skip):
            return
        children = list(getattr(node, "children", []) or [])
        if children:
            for c in children:
                walk(c, newpath)
        else:
            out.extend(node.solids())

    walk(shape, "")
    return out


def _mass_props(shape):
    """Compound を葉ソリッドまで展開し、体積・体積重心・重心まわり慣性(mm^5)を返す。

    Compound.volume は 0 を返すことがあるため、葉ソリッドを平行軸の定理で合成する。
    """
    solids = _leaf_solids(shape, skip=INERTIA_SKIP)
    vol = sum(s.volume for s in solids)
    if vol <= 0:
        raise ValueError("体積 0 の形状には慣性を定義できない")
    cx = sum(s.volume * s.center().X for s in solids) / vol
    cy = sum(s.volume * s.center().Y for s in solids) / vol
    cz = sum(s.volume * s.center().Z for s in solids) / vol
    inertia = [[0.0] * 3 for _ in range(3)]
    for s in solids:
        v = s.volume
        m = s.matrix_of_inertia
        c = s.center()
        d = (c.X - cx, c.Y - cy, c.Z - cz)
        d2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
        for i in range(3):
            for j in range(3):
                shift = v * (d2 * (1.0 if i == j else 0.0) - d[i] * d[j])
                inertia[i][j] += m[i][j] + shift
    return vol, (cx, cy, cz), inertia


def _inertial(elem, shape, mass_kg: float) -> None:
    """体積慣性テンソル × 平均密度 で inertial を書き出す（一様密度近似）。"""
    vol, com_t, m = _mass_props(shape)

    class _C:
        X, Y, Z = com_t

    com = _C()
    rho = mass_kg / vol  # kg/mm3
    scale = rho * 1e-6  # mm5 * kg/mm3 = kg*mm2 -> kg*m2
    ine = ET.SubElement(elem, "inertial")
    ET.SubElement(ine, "origin", {
        "xyz": _v3((com.X * MM, com.Y * MM, com.Z * MM)), "rpy": "0 0 0"})
    ET.SubElement(ine, "mass", {"value": f"{mass_kg:.6g}"})
    ET.SubElement(ine, "inertia", {
        "ixx": f"{m[0][0] * scale:.6g}", "ixy": f"{m[0][1] * scale:.6g}",
        "ixz": f"{m[0][2] * scale:.6g}", "iyy": f"{m[1][1] * scale:.6g}",
        "iyz": f"{m[1][2] * scale:.6g}", "izz": f"{m[2][2] * scale:.6g}"})


def _load_materials() -> dict:
    """`export_meshes.py` が残した「どの STL が何色か」。無ければ空。

    ⚠ **STL は色を持てない**ので、材質ごとに分けて書き出したファイルの
      割付をここで読む。無いときは 1 枚の灰色メッシュに落ちる（昔の挙動）。
    """
    path = os.path.join(HERE, MESH_DIR, "materials.json")
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as fp:
        return json.load(fp)


MATERIALS = _load_materials()


def _visual(elem, mesh_name: str) -> None:
    """リンクの見た目。材質ごとに <visual> を 1 つずつ出す。"""
    parts = MATERIALS.get(mesh_name)
    if not parts:
        parts = [{"file": f"{mesh_name}.stl", "material": "tr_alu",
                  "rgba": [0.72, 0.75, 0.78, 1.0]}]
    for p in parts:
        vis = ET.SubElement(elem, "visual")
        ET.SubElement(vis, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geo = ET.SubElement(vis, "geometry")
        ET.SubElement(geo, "mesh", {
            "filename": f"{MESH_DIR}/{p['file']}", "scale": _v3(MESH_SCALE)})
        mat = ET.SubElement(vis, "material", {"name": f"tr_{p['material']}"})
        ET.SubElement(mat, "color", {
            "rgba": " ".join(f"{v:.4g}" for v in p["rgba"])})


def _collision_box(elem, shape) -> None:
    """bbox ベースの直方体 collision（意図的な保守側簡略化）。"""
    bb = shape.bounding_box()
    col = ET.SubElement(elem, "collision")
    ET.SubElement(col, "origin", {
        "xyz": _v3(((bb.min.X + bb.max.X) / 2 * MM,
                    (bb.min.Y + bb.max.Y) / 2 * MM,
                    (bb.min.Z + bb.max.Z) / 2 * MM)), "rpy": "0 0 0"})
    geo = ET.SubElement(col, "geometry")
    ET.SubElement(geo, "box", {
        "size": _v3((bb.size.X * MM, bb.size.Y * MM, bb.size.Z * MM))})


def _collision_boxes(elem, boxes) -> None:
    """実構造に沿った直方体群を collision として並べる（mm 指定）。"""
    for _name, (x0, x1), (y0, y1), (z0, z1) in boxes:
        col = ET.SubElement(elem, "collision")
        ET.SubElement(col, "origin", {
            "xyz": _v3(((x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM)),
            "rpy": "0 0 0"})
        geo = ET.SubElement(col, "geometry")
        ET.SubElement(geo, "box", {
            "size": _v3(((x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM))})


def _collision_cyl(elem, radius_m: float, length_m: float, rpy) -> None:
    col = ET.SubElement(elem, "collision")
    ET.SubElement(col, "origin", {"xyz": "0 0 0", "rpy": _v3(rpy)})
    geo = ET.SubElement(col, "geometry")
    ET.SubElement(geo, "cylinder", {"radius": f"{radius_m:.6g}", "length": f"{length_m:.6g}"})


def gen_urdf():
    robot = ET.Element("robot", {"name": "tr"})

    ET.SubElement(robot, "material", {"name": "tr_alu"}).append(
        ET.Element("color", {"rgba": "0.72 0.75 0.78 1.0"}))

    shapes = {name: fn() for name, fn in LINK_SHAPE.items()}

    # 質量配分の検算（CAD 側の質量台帳と一致していること）
    A.build(P.POSE_MATCH)
    ledger_total = L.LEDGER.total_kg
    moving = sum(LINK_MASS_KG.values())
    base_mass = ledger_total - moving
    assert base_mass > 0, "可動リンクの質量配分が総質量を超えている"

    # --- base_link ---
    base = ET.SubElement(robot, "link", {"name": "base_link"})
    _inertial(base, shapes["base_link"], base_mass)
    _visual(base, LINK_MESH["base_link"])
    _collision_boxes(base, BASE_COLLISION_BOXES)

    for name, j in A.JOINTS.items():
        link = ET.SubElement(robot, "link", {"name": name})
        _inertial(link, shapes[name], LINK_MASS_KG[name])
        _visual(link, LINK_MESH[name])
        if name.startswith("wheel_"):
            kind, (r, ln), _, rpy = COLLISION["wheel"]
            _collision_cyl(link, r, ln, rpy)
        elif name == "grabber_slide":
            _collision_boxes(link, GRABBER_COLLISION_BOXES)
        elif name == "fork_tilt":
            _collision_boxes(link, FORK_COLLISION_BOXES)
        else:
            _collision_box(link, shapes[name])

        joint = ET.SubElement(robot, "joint", {"name": name, "type": j["type"]})
        ET.SubElement(joint, "parent", {"link": j["parent"]})
        ET.SubElement(joint, "child", {"link": name})
        ET.SubElement(joint, "origin", {
            "xyz": _v3(tuple(v * MM for v in j["origin"])), "rpy": "0 0 0"})
        ET.SubElement(joint, "axis", {"xyz": _v3(j["axis"])})
        eff, vel = JOINT_LIMITS[name]
        if j["type"] == "continuous":
            ET.SubElement(joint, "limit", {"effort": f"{eff:.6g}", "velocity": f"{vel:.6g}"})
        else:
            lo, hi = j["limits"]
            if j["type"] == "revolute":
                lo, hi = math.radians(lo), math.radians(hi)
            else:
                lo, hi = lo * MM, hi * MM
            ET.SubElement(joint, "limit", {
                "lower": f"{lo:.6g}", "upper": f"{hi:.6g}",
                "effort": f"{eff:.6g}", "velocity": f"{vel:.6g}"})
            ET.SubElement(joint, "dynamics", {"damping": "0.05", "friction": "0.02"})

    # --- センサーフレーム（fixed。実体は base_link 側の当たり判定にある） ---
    for name, (x, y, z), yaw in SENSOR_FRAMES:
        ET.SubElement(robot, "link", {"name": name})
        joint = ET.SubElement(robot, "joint", {"name": f"{name}_mount", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": "base_link"})
        ET.SubElement(joint, "child", {"link": name})
        ET.SubElement(joint, "origin", {
            "xyz": _v3((x * MM, y * MM, z * MM)),
            "rpy": f"0 0 {math.radians(yaw):.6g}"})

    return robot


def write(path: str | None = None) -> str:
    """URDF を書き出す（このファイルを直接実行すれば更新される）。"""
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tr.urdf")
    robot = gen_urdf()
    # PyBullet の URDF ローダーは、joint が参照する link が**先に**定義されている
    # ことを要求する（そうでないと "Cannot find parent link for a joint"）。
    # gen_urdf() は link と joint を交互に積むので、ここで並べ替える。
    order = {"material": 0, "link": 1, "joint": 2}
    children = sorted(list(robot), key=lambda e: order.get(e.tag, 3))
    for e in list(robot):
        robot.remove(e)
    robot.extend(children)
    xml = ET.tostring(robot, encoding="unicode")
    with open(path, "w", encoding="utf-8") as fp:
        fp.write('<?xml version="1.0"?>\n<!-- cadpy:sourcePath=tr_urdf.py -->\n')
        fp.write(xml)
        fp.write("\n")
    return path


if __name__ == "__main__":
    p = write()
    print(f"wrote {p}")
