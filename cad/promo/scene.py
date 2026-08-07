"""TR のプロモ用 Blender シーン。

URDF（cad/urdf/tr.urdf）のリンク階層をそのまま Blender の Empty 階層に写し、
材質ごとに分かれた STL を各リンクにぶら下げる。関節は 1 自由度ぶんの Empty を
1 個ずつ持つので、あとから角度・変位をキーフレームで動かせる。

    blender -b -P scene.py -- --out promo.blend

このファイルは単体で .blend を作るところまで。カメラワークとレンダリングは
shots.py が担当する。
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import bpy
import bmesh
from mathutils import Euler, Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.dirname(HERE)
URDF = os.path.join(CAD, "urdf", "tr.urdf")
HDRI = os.path.join(HERE, "hdri", "brown_photostudio_02.exr")


# ---------------------------------------------------------------- 材質
# CAD 側の rgba は「どの材質か」を見分けるための色でしかないので、ここでは
# 反射のふるまい（金属かどうか・どれくらい荒れているか）を材質ごとに与え直す。
# base は sRGB。Blender へは linear に直して渡す。
MATERIALS: dict[str, dict] = {
    # --- 金属 ---
    "STEEL": dict(base=(0.40, 0.43, 0.47), metallic=1.0, rough=0.34, noise=(90, 0.09), bump=0.05),
    "SUS304": dict(base=(0.60, 0.63, 0.68), metallic=1.0, rough=0.22, noise=(150, 0.07), bump=0.03),
    "A5052": dict(base=(0.72, 0.75, 0.79), metallic=1.0, rough=0.30, noise=(120, 0.08), bump=0.035),
    "A6005C": dict(base=(0.68, 0.71, 0.76), metallic=1.0, rough=0.26, noise=(180, 0.06), bump=0.025),
    "ADC12": dict(base=(0.48, 0.50, 0.54), metallic=1.0, rough=0.52, noise=(55, 0.11), bump=0.18),
    "MOTOR": dict(base=(0.055, 0.057, 0.065), metallic=0.85, rough=0.36, noise=(80, 0.07), bump=0.05),
    "MOTOR_SHAFT": dict(base=(0.72, 0.60, 0.28), metallic=1.0, rough=0.20, noise=(180, 0.05)),
    # --- 樹脂・ゴム ---
    "PETG": dict(base=(0.94, 0.24, 0.50), metallic=0.0, rough=0.38, coat=0.25, noise=(260, 0.09), bump=0.14),
    "PC": dict(base=(0.86, 0.93, 0.96), metallic=0.0, rough=0.045, transmission=0.92, ior=1.585),
    "POM": dict(base=(0.66, 0.66, 0.63), metallic=0.0, rough=0.42),
    "PP_DANPLA": dict(base=(0.50, 0.53, 0.49), metallic=0.0, rough=0.55, transmission=0.25, ior=1.49),
    "RUBBER": dict(base=(0.035, 0.035, 0.038), metallic=0.0, rough=0.72, noise=(240, 0.09), bump=0.13),
    "URETHANE": dict(base=(0.075, 0.065, 0.072), metallic=0.0, rough=0.58, noise=(200, 0.10), bump=0.11),
    "SILICONE": dict(base=(0.82, 0.26, 0.12), metallic=0.0, rough=0.48, sss=0.25, sss_r=(0.9, 0.35, 0.2)),
    "SPONGE": dict(base=(0.72, 0.69, 0.65), metallic=0.0, rough=0.92, noise=(320, 0.06), bump=0.26),
    "CABLE": dict(base=(0.022, 0.022, 0.026), metallic=0.0, rough=0.42, coat=0.20, noise=(300, 0.09), bump=0.16),
    # --- 電装・その他 ---
    # 表示器の画面。⚠ 地の色は**ほぼ黒**にする。画は Emission（`tex/screen.png`）
    #   で足すので、ベースまで明るいと消灯部分が灰色に浮いて液晶に見えない。
    "SCREEN": dict(base=(0.012, 0.014, 0.018), metallic=0.0, rough=0.055, coat=0.5,
                   coat_rough=0.02),
    "PCB": dict(base=(0.045, 0.22, 0.10), metallic=0.15, rough=0.34, coat=0.35),
    "SENSOR": dict(base=(0.10, 0.11, 0.13), metallic=0.35, rough=0.28, coat=0.25),
    "BATTERY": dict(base=(0.035, 0.045, 0.13), metallic=0.0, rough=0.36, coat=0.30),
    "ESTOP": dict(base=(0.72, 0.035, 0.035), metallic=0.0, rough=0.24, coat=0.9, coat_rough=0.08),
    "TEKCELL": dict(base=(0.86, 0.80, 0.62), metallic=0.0, rough=0.45),
    "PLYWOOD": dict(base=(0.62, 0.43, 0.22), metallic=0.0, rough=0.62, noise=(45, 0.13), bump=0.20),
    # --- マスコット ---
    "MASCOT": dict(base=(0.92, 0.76, 0.42), metallic=0.0, rough=0.62, sheen=0.3),
    "MASCOT_SUIT": dict(base=(0.055, 0.13, 0.34), metallic=0.0, rough=0.72, sheen=0.5),
    "MASCOT_TRIM": dict(base=(0.72, 0.075, 0.085), metallic=0.0, rough=0.70, sheen=0.45),
    "MASCOT_DARK": dict(base=(0.020, 0.020, 0.028), metallic=0.0, rough=0.55, sheen=0.3),
    "MASCOT_RAG": dict(base=(0.50, 0.54, 0.47), metallic=0.0, rough=0.88, sheen=0.6),
}

FALLBACK = dict(base=(0.5, 0.5, 0.5), metallic=0.0, rough=0.5)


def srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def set_input(node, name: str, value) -> None:
    if name in node.inputs:
        node.inputs[name].default_value = value


def build_material(name: str, mat: bpy.types.Material | None = None) -> bpy.types.Material:
    spec = MATERIALS.get(name, FALLBACK)
    if mat is None:
        mat = bpy.data.materials.new(f"TR_{name}")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()

    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.location = (400, 0)
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (60, 0)
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])

    base = tuple(srgb_to_linear(c) for c in spec["base"]) + (1.0,)
    set_input(bsdf, "Base Color", base)
    set_input(bsdf, "Metallic", spec.get("metallic", 0.0))
    set_input(bsdf, "Roughness", spec.get("rough", 0.5))
    set_input(bsdf, "IOR", spec.get("ior", 1.5))
    set_input(bsdf, "Coat Weight", spec.get("coat", 0.0))
    set_input(bsdf, "Coat Roughness", spec.get("coat_rough", 0.03))
    set_input(bsdf, "Transmission Weight", spec.get("transmission", 0.0))
    set_input(bsdf, "Sheen Weight", spec.get("sheen", 0.0))
    if spec.get("sss"):
        set_input(bsdf, "Subsurface Weight", spec["sss"])
        set_input(bsdf, "Subsurface Radius", spec.get("sss_r", (0.3, 0.15, 0.1)))
        set_input(bsdf, "Subsurface Scale", 0.01)

    # 荒れの揺らぎ。一様な roughness は CG くさく見えるので、実物の
    # 加工目・使用痕のかわりにノイズでわずかに散らす。
    if spec.get("noise"):
        scale, amount = spec["noise"]
        tex = nt.nodes.new("ShaderNodeTexNoise")
        tex.location = (-600, -220)
        tex.inputs["Scale"].default_value = scale
        tex.inputs["Detail"].default_value = 4.0
        tex.inputs["Roughness"].default_value = 0.55

        rng = nt.nodes.new("ShaderNodeMapRange")
        rng.location = (-360, -220)
        rng.inputs["From Min"].default_value = 0.25
        rng.inputs["From Max"].default_value = 0.75
        r = spec.get("rough", 0.5)
        rng.inputs["To Min"].default_value = max(0.01, r - amount)
        rng.inputs["To Max"].default_value = min(1.0, r + amount)
        nt.links.new(tex.outputs["Fac"], rng.inputs["Value"])
        nt.links.new(rng.outputs["Result"], bsdf.inputs["Roughness"])

        if spec.get("bump"):
            bump = nt.nodes.new("ShaderNodeBump")
            bump.location = (-160, -460)
            bump.inputs["Strength"].default_value = spec["bump"] * 0.06
            bump.inputs["Distance"].default_value = 0.0004
            nt.links.new(tex.outputs["Fac"], bump.inputs["Height"])
            nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])

    if name == "SCREEN":
        wire_screen(nt, bsdf)

    if spec.get("transmission", 0.0) > 0.5:
        mat.use_backface_culling = False
        mat.blend_method = "BLEND" if hasattr(mat, "blend_method") else mat.blend_method
    return mat


# ------------------------------------------------------------ 表示器の画面
SCREEN_TEX = os.path.join(HERE, "tex", "screen.png")
SCREEN_EMIT = 3.2       # 画の明るさ。1 だと暗いホリゾントの中で消灯に見える


def screen_image():
    """表示器に映す画。無ければ CAD の venv で `screen.py` を回して作る。

    ⚠ Blender の Python には Pillow が無いので、ここでは作れない。作るのは
      `cad/.venv` 側の `promo/screen.py`。
    """
    if not os.path.exists(SCREEN_TEX):
        py = os.path.join(CAD, ".venv", "bin", "python")
        if os.path.exists(py):
            subprocess.run([py, os.path.join(HERE, "screen.py")], check=False)
    if not os.path.exists(SCREEN_TEX):
        print(f"  ! 表示器の画が無い（画面は消灯で出る）: {SCREEN_TEX}")
        return None
    return bpy.data.images.load(SCREEN_TEX, check_existing=True)


def wire_screen(nt, bsdf) -> None:
    """画面に `tex/screen.png` を Emission として貼る。

    ⚠ **UV は無い。** メッシュは STL から来るので UV マップを持っていない。
      Generated（メッシュの bbox を 0..1 に正規化した座標）で貼る。有効表示
      領域の板は **X 薄・Y 幅・Z 高**、見るのは -X 側なので:
          u = 1 - y   （-X から見ると +Y が画面の左）
          v = z
      ⚠ `u = y` にすると鏡文字になる。板が薄いので裏返っても形は変わらず、
        文字だけが左右反転して出る（気づきにくい）。
    """
    img = screen_image()
    if img is None:
        return

    co = nt.nodes.new("ShaderNodeTexCoord")
    co.location = (-980, 320)
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep.location = (-800, 320)
    nt.links.new(co.outputs["Generated"], sep.inputs["Vector"])

    flip = nt.nodes.new("ShaderNodeMath")
    flip.location = (-640, 380)
    flip.operation = "SUBTRACT"
    flip.inputs[0].default_value = 1.0
    nt.links.new(sep.outputs["Y"], flip.inputs[1])

    uv = nt.nodes.new("ShaderNodeCombineXYZ")
    uv.location = (-470, 320)
    nt.links.new(flip.outputs["Value"], uv.inputs["X"])
    nt.links.new(sep.outputs["Z"], uv.inputs["Y"])

    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.location = (-300, 320)
    tex.image = img
    tex.extension = "EXTEND"
    tex.interpolation = "Cubic"
    nt.links.new(uv.outputs["Vector"], tex.inputs["Vector"])

    nt.links.new(tex.outputs["Color"], bsdf.inputs["Emission Color"])
    set_input(bsdf, "Emission Strength", SCREEN_EMIT)


# ---------------------------------------------------------------- URDF
def parse_urdf(path: str):
    root = ET.parse(path).getroot()
    links: dict[str, list] = {}
    for link in root.findall("link"):
        vis = []
        for v in link.findall("visual"):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            mat = v.find("material")
            mname = mat.get("name", "") if mat is not None else ""
            mname = mname[3:] if mname.startswith("tr_") else mname
            o = v.find("origin")
            xyz = [float(x) for x in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())]
            rpy = [float(x) for x in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())]
            scale = [float(x) for x in mesh.get("scale", "1 1 1").split()]
            vis.append(dict(file=mesh.get("filename"), material=mname, xyz=xyz, rpy=rpy, scale=scale))
        links[link.get("name")] = vis

    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        lim = j.find("limit")
        axis = j.find("axis")
        joints.append(
            dict(
                name=j.get("name"),
                type=j.get("type"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                xyz=[float(x) for x in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())],
                rpy=[float(x) for x in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())],
                axis=[float(x) for x in (axis.get("xyz", "0 0 1").split() if axis is not None else "0 0 1".split())],
                lower=float(lim.get("lower", 0.0)) if lim is not None else None,
                upper=float(lim.get("upper", 0.0)) if lim is not None else None,
            )
        )
    return links, joints


def align_z_to(axis) -> Matrix:
    """ローカル +Z が親空間の axis を向く回転を返す。"""
    a = Vector(axis).normalized()
    z = Vector((0.0, 0.0, 1.0))
    if (a - z).length < 1e-9:
        return Matrix.Identity(3)
    if (a + z).length < 1e-9:
        return Matrix.Rotation(math.pi, 3, "X")
    return z.rotation_difference(a).to_matrix()


def new_empty(name: str, parent=None, matrix: Matrix | None = None) -> bpy.types.Object:
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = 0.05
    bpy.context.scene.collection.objects.link(e)
    if parent is not None:
        e.parent = parent  # parent_inverse を単位のままにしたいので直接代入する
    if matrix is not None:
        e.matrix_basis = matrix
    return e


def import_stl(path: str, scale: float):
    before = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=path, global_scale=scale, forward_axis="Y", up_axis="Z")
    return [o for o in bpy.data.objects if o not in before]


def build_robot(links, joints):
    """URDF の階層を Empty で組み、メッシュをぶら下げる。

    1 関節につき joint_frame(原点) → axis_frame(軸を Z へ) → drive(ここを動かす)
    → link_root(軸合わせを戻す) の 4 段。こうしておくと軸が (0,-1,0) の
    ような向きでも、動かすのは drive の Z 回転 / Z 移動だけで済む。
    """
    mesh_dir = os.path.join(CAD, "urdf")
    mats: dict[str, bpy.types.Material] = {}
    link_roots: dict[str, bpy.types.Object] = {}
    drives: dict[str, dict] = {}

    root_names = set(links) - {j["child"] for j in joints}
    base = root_names.pop() if root_names else "base_link"
    link_roots[base] = new_empty(f"L_{base}")

    # 親が先に立っているとは限らないので、立てられるものから順に処理する
    todo = list(joints)
    while todo:
        progressed = False
        for j in list(todo):
            if j["parent"] not in link_roots:
                continue
            todo.remove(j)
            progressed = True
            R = align_z_to(j["axis"])
            jf = new_empty(
                f"J_{j['name']}",
                link_roots[j["parent"]],
                Matrix.LocRotScale(Vector(j["xyz"]), Euler(j["rpy"], "XYZ"), None),
            )
            af = new_empty(f"A_{j['name']}", jf, R.to_4x4())
            dv = new_empty(f"D_{j['name']}", af)
            lr = new_empty(f"L_{j['child']}", dv, R.inverted().to_4x4())
            link_roots[j["child"]] = lr
            drives[j["name"]] = dict(obj=dv, type=j["type"], lower=j["lower"], upper=j["upper"])
        if not progressed:
            raise RuntimeError(f"URDF の親が見つからない関節: {[j['name'] for j in todo]}")

    n_tri = 0
    for lname, visuals in links.items():
        if lname not in link_roots:
            continue
        for v in visuals:
            path = os.path.join(mesh_dir, v["file"])
            if not os.path.exists(path):
                print(f"  ! メッシュ無し: {path}")
                continue
            objs = import_stl(path, v["scale"][0])
            mname = v["material"] or "STEEL"
            if mname not in mats:
                mats[mname] = build_material(mname)
            for o in objs:
                o.name = f"{lname}__{mname}"
                o.data.materials.clear()
                o.data.materials.append(mats[mname])
                o.parent = link_roots[lname]
                n_tri += len(o.data.polygons)
                # CAD の平面テッセレーションをそのまま出すと円筒が角ばるので
                # 稜線角度でスムーズ／フラットを分ける
                bpy.context.view_layer.objects.active = o
                o.select_set(True)
                bpy.ops.object.shade_smooth_by_angle(angle=math.radians(31))
                o.select_set(False)
    print(f"  三角形: {n_tri:,}")
    return link_roots, drives


# ------------------------------------------------------- 環境（床・光・空気）
def build_world(strength: float = 0.34) -> None:
    world = bpy.data.worlds.new("TR_World")
    bpy.context.scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()

    out = nt.nodes.new("ShaderNodeOutputWorld")
    out.location = (600, 0)
    mix = nt.nodes.new("ShaderNodeMixShader")
    mix.location = (400, 0)
    lp = nt.nodes.new("ShaderNodeLightPath")
    lp.location = (200, 300)
    nt.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
    nt.links.new(mix.outputs["Shader"], out.inputs["Surface"])

    # 映り込み用：HDRI。カメラには見せない
    env = nt.nodes.new("ShaderNodeTexEnvironment")
    env.location = (-260, -160)
    if os.path.exists(HDRI):
        env.image = bpy.data.images.load(HDRI)
    mapping = nt.nodes.new("ShaderNodeMapping")
    mapping.location = (-460, -160)
    mapping.inputs["Rotation"].default_value = (0.0, 0.0, math.radians(-35))
    coord = nt.nodes.new("ShaderNodeTexCoord")
    coord.location = (-660, -160)
    nt.links.new(coord.outputs["Generated"], mapping.inputs["Vector"])
    nt.links.new(mapping.outputs["Vector"], env.inputs["Vector"])
    bg_refl = nt.nodes.new("ShaderNodeBackground")
    bg_refl.location = (100, -160)
    bg_refl.inputs["Strength"].default_value = strength
    nt.links.new(env.outputs["Color"], bg_refl.inputs["Color"])
    nt.links.new(bg_refl.outputs["Background"], mix.inputs[1])  # Fac=0 側＝カメラ以外

    # カメラに見せる背景：上が沈んだ紺、下がわずかに明るいグラデーション
    grad_coord = nt.nodes.new("ShaderNodeTexCoord")
    grad_coord.location = (-660, 220)
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep.location = (-460, 220)
    nt.links.new(grad_coord.outputs["Window"], sep.inputs["Vector"])
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.location = (-260, 220)
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = (0.0022, 0.0027, 0.0042, 1.0)
    ramp.color_ramp.elements[1].position = 1.0
    ramp.color_ramp.elements[1].color = (0.0004, 0.0005, 0.0009, 1.0)
    nt.links.new(sep.outputs["Y"], ramp.inputs["Fac"])
    bg_cam = nt.nodes.new("ShaderNodeBackground")
    bg_cam.location = (100, 220)
    bg_cam.inputs["Strength"].default_value = 1.0
    nt.links.new(ramp.outputs["Color"], bg_cam.inputs["Color"])
    nt.links.new(bg_cam.outputs["Background"], mix.inputs[2])  # Fac=1 側＝カメラに見える背景


def build_floor(radius: float = 9.0, wall: float = 6.2, fillet: float = 2.6, segs: int = 180) -> bpy.types.Object:
    """撮影スタジオのホリゾント。床から壁へ継ぎ目なく立ち上がる回転体。

    平面＋別の背景板だと必ず境目の線が出る。1 枚の面でつなぐと、
    背景が「どこまでも続く暗がり」になって機体だけが浮かぶ。
    """
    profile = [(0.0, 0.0, 0.0)]
    n_floor = 10
    for i in range(1, n_floor + 1):
        profile.append(((radius - fillet) * i / n_floor, 0.0, 0.0))
    n_fil = 28
    for i in range(1, n_fil + 1):
        a = (math.pi / 2) * i / n_fil
        profile.append((radius - fillet + fillet * math.sin(a), 0.0, fillet * (1.0 - math.cos(a))))
    n_wall = 14
    for i in range(1, n_wall + 1):
        profile.append((radius, 0.0, fillet + (wall - fillet) * i / n_wall))

    bm = bmesh.new()
    vs = [bm.verts.new(p) for p in profile]
    es = [bm.edges.new((vs[i], vs[i + 1])) for i in range(len(vs) - 1)]
    bmesh.ops.spin(bm, geom=es + vs, cent=(0, 0, 0), axis=(0, 0, 1),
                   angle=2 * math.pi, steps=segs, use_merge=False)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bmesh.ops.reverse_faces(bm, faces=bm.faces)  # 内側から見るので法線を内向きに

    me = bpy.data.meshes.new("Floor")
    bm.to_mesh(me)
    bm.free()
    floor = bpy.data.objects.new("Floor", me)
    bpy.context.scene.collection.objects.link(floor)
    bpy.context.view_layer.objects.active = floor
    floor.select_set(True)
    bpy.ops.object.shade_smooth()
    floor.select_set(False)

    mat = bpy.data.materials.new("TR_Floor")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.location = (400, 0)
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (100, 0)
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    set_input(bsdf, "Base Color", (0.006, 0.0065, 0.008, 1.0))
    set_input(bsdf, "Metallic", 0.0)
    set_input(bsdf, "Roughness", 0.20)
    set_input(bsdf, "IOR", 1.40)

    # 磨きムラ。均一な鏡面より、わずかに荒れているほうが実在感が出る
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.location = (-700, -200)
    tex.inputs["Scale"].default_value = 3.0
    tex.inputs["Detail"].default_value = 6.0
    rng = nt.nodes.new("ShaderNodeMapRange")
    rng.location = (-500, -200)
    rng.inputs["From Min"].default_value = 0.3
    rng.inputs["From Max"].default_value = 0.7
    rng.inputs["To Min"].default_value = 0.15
    rng.inputs["To Max"].default_value = 0.30
    nt.links.new(tex.outputs["Fac"], rng.inputs["Value"])

    # 立ち上がった壁は艶消しに。床だけが映り込む
    coord = nt.nodes.new("ShaderNodeTexCoord")
    coord.location = (-900, 180)
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    sep.location = (-700, 180)
    nt.links.new(coord.outputs["Object"], sep.inputs["Vector"])
    wallness = nt.nodes.new("ShaderNodeMapRange")
    wallness.location = (-500, 180)
    wallness.inputs["From Min"].default_value = 0.05
    wallness.inputs["From Max"].default_value = 1.6
    wallness.inputs["To Min"].default_value = 0.0
    wallness.inputs["To Max"].default_value = 1.0
    wallness.clamp = True
    nt.links.new(sep.outputs["Z"], wallness.inputs["Value"])

    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "FLOAT"
    mix.location = (-280, 0)
    mix.inputs["B"].default_value = 0.72
    nt.links.new(wallness.outputs["Result"], mix.inputs["Factor"])
    nt.links.new(rng.outputs["Result"], mix.inputs["A"])
    nt.links.new(mix.outputs["Result"], bsdf.inputs["Roughness"])

    # 壁の腰の高さだけを明るくして、背景に階調を作る。真っ暗な背景は
    # 被写体を切り抜いたように見せてしまう。機体の後ろに明→暗の帯があると
    # 輪郭が浮き、奥行きが出る。全周に入れるのはカメラが一周するため。
    hband = nt.nodes.new("ShaderNodeMapRange")
    hband.location = (-500, 400)
    hband.inputs["From Min"].default_value = 0.0
    hband.inputs["From Max"].default_value = 4.5
    hband.inputs["To Min"].default_value = 0.0
    hband.inputs["To Max"].default_value = 1.0
    hband.clamp = True
    nt.links.new(sep.outputs["Z"], hband.inputs["Value"])

    tint = nt.nodes.new("ShaderNodeValToRGB")
    tint.location = (-280, 400)
    cr = tint.color_ramp
    cr.elements[0].position, cr.elements[0].color = 0.0, (0.006, 0.0065, 0.008, 1.0)
    cr.elements[1].position, cr.elements[1].color = 1.0, (0.002, 0.0024, 0.0034, 1.0)
    for pos, col in ((0.17, (0.0075, 0.0082, 0.0105, 1.0)),
                     (0.27, (0.027, 0.030, 0.038, 1.0)),
                     (0.40, (0.0070, 0.0078, 0.0102, 1.0))):
        e = cr.elements.new(pos)
        e.color = col
    nt.links.new(tint.outputs["Color"], bsdf.inputs["Base Color"])

    floor.data.materials.append(mat)
    return floor


def _aim(obj, loc, target) -> None:
    obj.location = loc
    obj.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()


def build_softbox(name, size, loc, target, strength, color=(1.0, 1.0, 1.0),
                  kind="spot", fade=0.14) -> bpy.types.Object:
    """発光する板。エリアライトの代わりに使う。

    金属に写るハイライトの「形」を作れるのが要点。灯体そのものが反射像に
    なるので、板の縦横比とグラデーションがそのまま艶の質になる。細長い板
    （ストリップ）は金属の稜線に帯を走らせ、これが高級感の核になる。
    端をフェードさせないと反射像の縁が硬く、貼り付けたように見える。
    """
    me = bpy.data.meshes.new(f"LGT_{name}")
    bm = bmesh.new()
    hx, hy = size[0] * 0.5, size[1] * 0.5
    vs = [bm.verts.new(p) for p in ((-hx, -hy, 0), (hx, -hy, 0), (hx, hy, 0), (-hx, hy, 0))]
    bm.faces.new(vs)
    bm.to_mesh(me)
    bm.free()

    obj = bpy.data.objects.new(f"LGT_{name}", me)
    bpy.context.scene.collection.objects.link(obj)
    _aim(obj, loc, target)
    obj.visible_camera = False   # 灯体そのものは画面に入れない
    obj.visible_shadow = False   # 他の光を遮らせない

    mat = bpy.data.materials.new(f"LGT_{name}")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.location = (300, 0)
    emi = nt.nodes.new("ShaderNodeEmission")
    emi.location = (100, 0)
    emi.inputs["Strength"].default_value = strength
    nt.links.new(emi.outputs["Emission"], out.inputs["Surface"])

    if kind == "flat":
        emi.inputs["Color"].default_value = (*color, 1.0)
    else:
        coord = nt.nodes.new("ShaderNodeTexCoord")
        coord.location = (-620, 0)
        mapping = nt.nodes.new("ShaderNodeMapping")
        mapping.location = (-440, 0)
        grad = nt.nodes.new("ShaderNodeTexGradient")
        grad.location = (-260, 0)
        ramp = nt.nodes.new("ShaderNodeValToRGB")
        ramp.location = (-100, 0)
        nt.links.new(coord.outputs["Generated"], mapping.inputs["Vector"])
        nt.links.new(mapping.outputs["Vector"], grad.inputs["Vector"])
        nt.links.new(grad.outputs["Color"], ramp.inputs["Fac"])
        nt.links.new(ramp.outputs["Color"], emi.inputs["Color"])
        el = ramp.color_ramp.elements
        if kind == "strip":
            # 長手方向の両端だけ落とす台形
            grad.gradient_type = "LINEAR"
            el[0].position, el[0].color = 0.0, (0, 0, 0, 1)
            el[1].position, el[1].color = fade, (*color, 1)
            a = el.new(1.0 - fade)
            a.color = (*color, 1)
            b = el.new(1.0)
            b.color = (0, 0, 0, 1)
        else:
            # 中心にホットスポットのあるソフトボックス
            grad.gradient_type = "QUADRATIC_SPHERE"
            el[0].position, el[0].color = 0.0, (0, 0, 0, 1)
            el[1].position, el[1].color = 1.0, (*color, 1)
            m = el.new(0.55)
            m.color = tuple(c * 0.42 for c in color) + (1.0,)
    me.materials.append(mat)
    return obj


def build_negfill(name, size, loc, target, value=0.015) -> bpy.types.Object:
    """黒い板（ネガティブフィル）。当てるのではなく、映り込みを消して締める。

    明るい機体を暗い環境に置くと全面が同じ明度になって平板に見える。
    片側に黒を映し込むと、そこだけ反射が落ちて陰影の階調が戻る。
    """
    me = bpy.data.meshes.new(f"LGT_{name}")
    bm = bmesh.new()
    hx, hy = size[0] * 0.5, size[1] * 0.5
    vs = [bm.verts.new(p) for p in ((-hx, -hy, 0), (hx, -hy, 0), (hx, hy, 0), (-hx, hy, 0))]
    bm.faces.new(vs)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(f"LGT_{name}", me)
    bpy.context.scene.collection.objects.link(obj)
    _aim(obj, loc, target)
    obj.visible_camera = False
    obj.visible_shadow = False

    mat = bpy.data.materials.new(f"LGT_{name}")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    bsdf = nt.nodes.new("ShaderNodeBsdfDiffuse")
    bsdf.inputs["Color"].default_value = (value, value, value, 1.0)
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    me.materials.append(mat)
    return obj


def add_area(name, loc, target, size, power, color=(1, 1, 1), spread=None):
    lamp = bpy.data.lights.new(name, "AREA")
    lamp.shape = "RECTANGLE" if isinstance(size, tuple) else "SQUARE"
    if isinstance(size, tuple):
        lamp.size, lamp.size_y = size
    else:
        lamp.size = size
    lamp.energy = power
    lamp.color = color
    if spread is not None:
        lamp.spread = math.radians(spread)
    obj = bpy.data.objects.new(name, lamp)
    bpy.context.scene.collection.objects.link(obj)
    # 回り込むと光源そのものが画面に入るので、カメラからは隠す。
    # ⚠ visible_glossy は切らない。金属の艶は光源の映り込みそのもので、
    #   切ると機体が真っ黒に沈む。床に映る虚像は床側（粗さと IOR）で潰す。
    obj.visible_camera = False
    obj.location = loc
    direction = Vector(target) - Vector(loc)
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    return obj


def build_lights(center: Vector, top: float) -> None:
    """自動車 CG のスタジオに倣った灯体構成。

    要点は「点で照らす」のをやめて「形のあるものを映し込む」こと。
    細長いストリップの反射像が金属の稜線に帯を走らせ、これが艶の正体になる。
    エリアライトを増やしても帯は出ない（反射像が小さな四角にしかならない）。
    """
    cx, cy = center.x, center.y
    mid = (cx, cy, top * 0.52)
    high = (cx, cy, top * 0.85)

    # 主光源は真上寄りの大きなソフトボックス。カメラが機体の周りを一周する
    # ので、片側から当てると回り込んだ先で必ず陰になる。上からなら全方位で
    # 成立する。
    build_softbox("Key", (3.2, 2.4), (cx + 0.30, cy - 0.20, top + 1.85), mid,
                  150, (1.0, 0.97, 0.93), kind="spot")

    # ストリップ。細く・強く・斜めに置くほど帯が締まる
    build_softbox("StripFront", (0.13, 3.4), (cx + 1.32, cy - 0.90, top * 1.42), high,
                  620, (1.0, 0.98, 0.95), kind="strip", fade=0.18)
    build_softbox("StripRear", (0.11, 3.0), (cx - 1.24, cy - 1.06, top * 1.30), mid,
                  760, (0.72, 0.84, 1.0), kind="strip", fade=0.16)
    build_softbox("StripSide", (0.10, 2.6), (cx - 1.12, cy + 1.32, top * 1.22), mid,
                  500, (1.0, 0.86, 0.68), kind="strip", fade=0.16)
    build_softbox("StripCam", (0.10, 2.8), (cx + 1.16, cy + 1.45, top * 1.34), mid,
                  430, (0.92, 0.95, 1.0), kind="strip", fade=0.16)

    # 背面の大きな逆光。輪郭を起こして背景から引き剥がす
    build_softbox("RimBack", (2.2, 2.0), (cx - 2.05, cy - 0.35, top * 1.05), mid,
                  115, (0.62, 0.76, 1.0), kind="spot")

    # 影側の弱い起こし。潰さない程度に
    build_softbox("Fill", (2.4, 1.6), (cx + 1.5, cy + 2.3, top * 0.45), mid,
                  22, (0.82, 0.88, 1.0), kind="spot")

    # ⚠ 黒板（ネガティブフィル）は入れていない。ターンテーブルで回ると
    #   必ずカメラ側に来る角度があり、そこで機体の手前半分が死ぬ。
    #   止め絵なら効くが、一周する絵では使えない。


def build_haze(density: float, center: Vector, top: float) -> None:
    """薄い霧。光源からの筋が出て奥行きが立つ。"""
    if density <= 0:
        return
    bpy.ops.mesh.primitive_cube_add(size=1, location=(center.x, center.y, top * 0.55))
    cube = bpy.context.active_object
    cube.name = "Haze"
    cube.scale = (14.0, 14.0, max(4.0, top * 2.4))
    cube.display_type = "WIRE"
    cube.visible_shadow = False

    mat = bpy.data.materials.new("TR_Haze")
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    vol = nt.nodes.new("ShaderNodeVolumeScatter")
    vol.inputs["Density"].default_value = density
    vol.inputs["Anisotropy"].default_value = 0.35
    vol.inputs["Color"].default_value = (0.62, 0.72, 0.92, 1.0)
    nt.links.new(vol.outputs["Volume"], out.inputs["Volume"])
    cube.data.materials.append(mat)


def _node_set(node, name: str, value) -> bool:
    """プロパティでもソケットでも設定できるようにする。

    Blender 4.x で Glare 等の設定が順次ソケットへ移っており、版によって
    どちらに載っているかが違う。両方試して、入った方を使う。
    """
    if hasattr(node, name):
        setattr(node, name, value)
        return True
    for key in (name, name.replace("_", " ").title()):
        if key in node.inputs:
            node.inputs[key].default_value = value
            return True
    return False


def _socket(node, name: str, value, kind: str | None = None):
    """名前（と型）でソケットを引いて値を入れる。

    ⚠ Blender 4.5 でコンポジットの設定は大半がソケットへ移り、同名の旧
    プロパティ（glare.mix など）は残っていても効かない。プロパティを先に
    見ると黙って無視されるので、必ずソケットを先に探す。
    ColorBalance のように同名で型違いのソケットが並ぶものがあるため、
    kind を指定できるようにしてある。
    """
    for s in node.inputs:
        if s.name == name and (kind is None or s.type == kind):
            s.default_value = value
            return True
    if hasattr(node, name):
        setattr(node, name, value)
        return True
    return False


def build_compositor(bloom: float = 0.085, streak: float = 0.0, dispersion: float = 0.0045,
                     vignette: float = 0.26, sharpen: float = 0.06) -> None:
    """撮って出しに見えないようにする後処理。

    レンズを通していない絵は、どれだけ光を作り込んでも「CG のスクショ」に
    見える。滲み・わずかな色収差・周辺光量落ち・色被りを足すと、同じ
    レンダリング結果でも一段“写真”に寄る。処理はリニア空間で走り、その
    あとに AgX が乗るので、しきい値はリニア値で考える。
    """
    sc = bpy.context.scene
    sc.use_nodes = True
    nt = sc.node_tree
    nt.nodes.clear()

    rl = nt.nodes.new("CompositorNodeRLayers")
    rl.location = (-1000, 0)
    cur = rl.outputs["Image"]
    x = -780

    # ハイライトのにじみ。強い反射のまわりに空気感が出る
    fog = nt.nodes.new("CompositorNodeGlare")
    fog.location = (x, 0)
    fog.glare_type = "FOG_GLOW"
    fog.quality = "HIGH"
    _socket(fog, "Threshold", 1.60)
    _socket(fog, "Smoothness", 0.30)
    _socket(fog, "Size", 0.55)
    _socket(fog, "Strength", bloom)
    nt.links.new(cur, fog.inputs["Image"])
    cur = fog.outputs["Image"]
    x += 210

    # 金属のいちばん強い点にだけ走る筋。入れすぎると安っぽいので弱く
    st = nt.nodes.new("CompositorNodeGlare")
    st.location = (x, 0)
    st.glare_type = "STREAKS"
    st.quality = "HIGH"
    _socket(st, "Threshold", 6.0)
    _socket(st, "Streaks", 6)
    _socket(st, "Streaks Angle", math.radians(12))
    _socket(st, "Fade", 0.92)
    _socket(st, "Iterations", 3)
    _socket(st, "Strength", streak)
    nt.links.new(cur, st.inputs["Image"])
    cur = st.outputs["Image"]
    x += 210

    # 影を寒色、ハイライトを暖色へ。金属が単調な灰色に見えるのを避ける
    cb = nt.nodes.new("CompositorNodeColorBalance")
    cb.location = (x, 0)
    cb.correction_method = "LIFT_GAMMA_GAIN"
    _socket(cb, "Lift", (0.982, 0.994, 1.024, 1.0), kind="RGBA")
    _socket(cb, "Gamma", (1.0, 1.0, 1.0, 1.0), kind="RGBA")
    _socket(cb, "Gain", (1.030, 1.000, 0.966, 1.0), kind="RGBA")
    nt.links.new(cur, cb.inputs["Image"])
    cur = cb.outputs["Image"]
    x += 230

    # レンズの歪みと色収差。ごく僅かでよい
    ld = nt.nodes.new("CompositorNodeLensdist")
    ld.location = (x, 0)
    ld.use_fit = True
    _socket(ld, "Fit", True)
    _socket(ld, "Distortion", 0.0016)
    _socket(ld, "Dispersion", dispersion)
    nt.links.new(cur, ld.inputs["Image"])
    cur = ld.outputs["Image"]
    x += 210

    # 周辺光量落ち。中心に視線を集める
    mask = nt.nodes.new("CompositorNodeEllipseMask")
    mask.location = (x - 220, -440)
    _socket(mask, "Position", (0.5, 0.5))
    _socket(mask, "Size", (1.15, 1.28))
    blur = nt.nodes.new("CompositorNodeBlur")
    blur.location = (x, -440)
    blur.filter_type = "FAST_GAUSS"
    _socket(blur, "Size", (0.38, 0.38))
    nt.links.new(mask.outputs["Mask"], blur.inputs["Image"])

    vig = nt.nodes.new("CompositorNodeMixRGB")
    vig.location = (x + 220, 0)
    vig.blend_type = "MULTIPLY"
    vig.inputs["Fac"].default_value = vignette
    nt.links.new(cur, vig.inputs[1])
    nt.links.new(blur.outputs["Image"], vig.inputs[2])
    cur = vig.outputs["Image"]
    x += 440

    # 最後にごく軽く輪郭を立てる
    if sharpen > 0:
        sh = nt.nodes.new("CompositorNodeFilter")
        sh.location = (x, 0)
        sh.filter_type = "SHARPEN"
        sh.inputs["Fac"].default_value = sharpen
        nt.links.new(cur, sh.inputs["Image"])
        cur = sh.outputs["Image"]
        x += 210

    comp = nt.nodes.new("CompositorNodeComposite")
    comp.location = (x, 0)
    nt.links.new(cur, comp.inputs["Image"])


def world_bounds(objs) -> tuple[Vector, Vector]:
    lo = Vector((1e9, 1e9, 1e9))
    hi = Vector((-1e9, -1e9, -1e9))
    dg = bpy.context.evaluated_depsgraph_get()
    for o in objs:
        if o.type != "MESH":
            continue
        mw = o.matrix_world
        for c in o.bound_box:
            p = mw @ Vector(c)
            lo = Vector((min(lo[i], p[i]) for i in range(3)))
            hi = Vector((max(hi[i], p[i]) for i in range(3)))
    return lo, hi


def setup_render(res=(1920, 1080), samples=256, motion_blur=True) -> None:
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "GPU"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = d.type in ("OPTIX",)

    sc.cycles.samples = samples
    sc.cycles.use_adaptive_sampling = True
    # ⚠ 0.01 は「ノイズが 1% を切ったら打ち切る」で、低サンプルのまま止まる。
    #   そこへ denoiser が強く効いてディテールが溶ける。0.004 まで下げる。
    sc.cycles.adaptive_threshold = 0.004
    sc.cycles.use_denoising = True
    sc.cycles.denoiser = "OPTIX"
    sc.cycles.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    # 跳ね返り回数。暗いスタジオで 12 も要らない。落とすと素直に速くなる
    sc.cycles.max_bounces = 6
    sc.cycles.diffuse_bounces = 3
    sc.cycles.glossy_bounces = 4
    sc.cycles.transmission_bounces = 8
    sc.cycles.transparent_max_bounces = 8
    sc.cycles.volume_bounces = 0
    # 明るすぎる間接光の点（ファイアフライ）を抑えると少ないサンプルで収束する
    sc.cycles.sample_clamp_indirect = 3.0
    sc.cycles.light_sampling_threshold = 0.02
    # 透明バケツのコースティクスは絵に効かないわりに重く、ノイズも残る
    sc.cycles.caustics_reflective = False
    sc.cycles.caustics_refractive = False
    sc.cycles.blur_glossy = 1.5
    sc.cycles.use_persistent_data = True  # 連番では BVH を使い回す

    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False
    sc.render.use_motion_blur = motion_blur
    sc.render.motion_blur_shutter = 0.5
    sc.render.image_settings.file_format = "PNG"
    sc.render.image_settings.color_mode = "RGB"
    sc.render.image_settings.compression = 15
    sc.render.fps = 30

    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Punchy"
    sc.view_settings.exposure = -0.10
    sc.view_settings.gamma = 1.0


def build(out_path: str, haze: float = 0.0009) -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    links, joints = parse_urdf(URDF)
    print(f"URDF: リンク {len(links)} / 関節 {len(joints)}")
    link_roots, drives = build_robot(links, joints)

    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    lo, hi = world_bounds(meshes)
    center = (lo + hi) * 0.5
    print(f"外形: {(hi - lo).x:.3f} x {(hi - lo).y:.3f} x {(hi - lo).z:.3f} m")

    build_world()
    build_floor()
    build_lights(center, hi.z)
    build_haze(haze, center, hi.z)
    setup_render()
    build_compositor()

    # 関節をまとめて回すための目印。shots.py 側から名前で引く
    root = link_roots["base_link"]
    root.name = "TR_root"
    for jname, d in drives.items():
        d["obj"]["tr_joint"] = jname
        d["obj"]["tr_type"] = d["type"]

    bpy.ops.wm.save_as_mainfile(filepath=out_path)
    print(f"保存: {out_path}")


def relight(blend_path: str, haze: float) -> None:
    """機体はそのままに、環境（床・光・空気・レンダ設定）だけ作り直す。

    STL の読み直しに 2 分弱かかるので、絵づくりの試行はこちらを回す。
    """
    bpy.ops.wm.open_mainfile(filepath=blend_path)
    for o in list(bpy.data.objects):
        if o.type == "LIGHT" or o.name in ("Floor", "Haze") or o.name.startswith("LGT_"):
            bpy.data.objects.remove(o, do_unlink=True)
    for block in (bpy.data.worlds, bpy.data.lights):
        for b in list(block):
            if b.users == 0:
                block.remove(b)

    for m in bpy.data.materials:
        if m.name.startswith("TR_") and m.name != "TR_Floor":
            build_material(m.name[3:], mat=m)

    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    lo, hi = world_bounds(meshes)
    center = (lo + hi) * 0.5
    build_world()
    build_floor()
    build_lights(center, hi.z)
    build_haze(haze, center, hi.z)
    setup_render()
    build_compositor()
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    print(f"環境を作り直した: {blend_path}  中心={tuple(round(v, 3) for v in center)} 高さ={hi.z:.3f}")


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "tr_promo.blend"))
    ap.add_argument("--haze", type=float, default=0.0009)
    ap.add_argument("--relight", action="store_true", help="既存 .blend の環境だけ作り直す")
    args = ap.parse_args(argv)
    if args.relight:
        relight(args.out, args.haze)
    else:
        build(args.out, args.haze)


if __name__ == "__main__":
    main()
