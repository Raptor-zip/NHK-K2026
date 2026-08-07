"""scene.py が作った .blend を開いて、カメラを置いて撮る。

    # 構図あたり（低解像度・低サンプルで複数アングル）
    blender -b tr_promo.blend -P shots.py -- preview

    # 1 枚だけ高品質
    blender -b tr_promo.blend -P shots.py -- still --az 38 --el 12 --dist 3.2 --lens 85

    # 動画用の連番
    blender -b tr_promo.blend -P shots.py -- anim --shot hero
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

R = math.radians

# 関節の既定姿勢（rad / m）。URDF の limit の内側に収めてある
POSES = {
    "match": dict(turret_yaw=0.0, shooter_pitch=R(45), grabber_slide=0.0, grabber_press=0.0, fork_tilt=0.0),
    "aim_left": dict(turret_yaw=R(24), shooter_pitch=R(58), grabber_slide=0.0, grabber_press=0.0, fork_tilt=0.0),
    "load": dict(turret_yaw=0.0, shooter_pitch=R(22), grabber_slide=0.30, grabber_press=0.13, fork_tilt=R(20)),
    "stow": dict(turret_yaw=0.0, shooter_pitch=R(21), grabber_slide=0.0, grabber_press=0.0, fork_tilt=0.0),
}


def enable_gpu() -> None:
    """GPU の割り当ては .blend ではなくユーザー設定側に載るので、開くたびに指定し直す。"""
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.get_devices()
    used = []
    for d in prefs.devices:
        d.use = d.type == "OPTIX"
        if d.use:
            used.append(d.name)
    bpy.context.scene.cycles.device = "GPU"
    print(f"GPU: {used or 'なし（CPU にフォールバック）'}")


def drives() -> dict[str, bpy.types.Object]:
    return {o["tr_joint"]: o for o in bpy.data.objects if "tr_joint" in o}


def set_pose(values: dict[str, float], frame: int | None = None) -> None:
    """関節を動かす。drive Empty は Z 回転／Z 移動の 1 自由度だけを持つ。"""
    dv = drives()
    for name, v in values.items():
        o = dv.get(name)
        if o is None:
            continue
        if o["tr_type"] == "prismatic":
            o.location = (0.0, 0.0, v)
            if frame is not None:
                o.keyframe_insert("location", frame=frame)
        else:
            o.rotation_euler = (0.0, 0.0, v)
            if frame is not None:
                o.keyframe_insert("rotation_euler", frame=frame)


def make_camera() -> tuple[bpy.types.Object, bpy.types.Object]:
    target = bpy.data.objects.get("CamTarget")
    if target is None:
        target = bpy.data.objects.new("CamTarget", None)
        target.empty_display_size = 0.1
        bpy.context.scene.collection.objects.link(target)

    cam = bpy.data.objects.get("Camera")
    if cam is None:
        data = bpy.data.cameras.new("Camera")
        cam = bpy.data.objects.new("Camera", data)
        bpy.context.scene.collection.objects.link(cam)
        c = cam.constraints.new("TRACK_TO")
        c.target = target
        c.track_axis = "TRACK_NEGATIVE_Z"
        c.up_axis = "UP_Y"
    bpy.context.scene.camera = cam
    return cam, target


def robot_corners() -> list[Vector]:
    """床・霧・灯体を除いた機体の外形 8 隅（ワールド座標）。"""
    lo = Vector((1e9, 1e9, 1e9))
    hi = Vector((-1e9, -1e9, -1e9))
    for o in bpy.data.objects:
        if o.type != "MESH" or o.name in ("Floor", "Haze") or o.name.startswith("LGT_"):
            continue
        for c in o.bound_box:
            p = o.matrix_world @ Vector(c)
            lo = Vector((min(lo[i], p[i]) for i in range(3)))
            hi = Vector((max(hi[i], p[i]) for i in range(3)))
    return [Vector((x, y, z)) for x in (lo.x, hi.x) for y in (lo.y, hi.y) for z in (lo.z, hi.z)]


def fit_distance(lens: float, look, az: float, el: float,
                 margin: float = 1.04, sensor: float = 36.0) -> float:
    """外形 8 隅を実際に投影して、全部が画面に入る最小距離を出す。

    外接球で見積もると縦横比の分だけ引きすぎて機体が小さくなるので、
    隅ごとに「必要な距離」を解いて最大を取る。
    """
    a, e = R(az), R(el)
    u = Vector((math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)))  # look→カメラ
    right = u.cross(Vector((0.0, 0.0, 1.0)))
    right = right.normalized() if right.length > 1e-6 else Vector((0.0, 1.0, 0.0))
    up = right.cross(-u).normalized()

    sc = bpy.context.scene
    aspect = sc.render.resolution_x / sc.render.resolution_y
    th = math.tan(math.atan(sensor / (2 * lens)))
    tv = math.tan(math.atan((sensor / aspect) / (2 * lens)))

    need = 0.0
    for c in robot_corners():
        v = c - Vector(look)
        need = max(need, v.dot(u) + max(abs(v.dot(right)) / th, abs(v.dot(up)) / tv))
    return need * margin


def place_camera(cam, target, *, az: float, el: float, dist, lens: float,
                 look=(0.0, 0.0, 0.72), fstop: float = 9.0, shift=(0.0, 0.0)) -> None:
    """方位角・仰角・距離で置く。az=0 が機体前方（+X）。dist に "fit" で自動。"""
    if dist == "fit" or dist is None:
        dist = fit_distance(lens, look, az, el)
    a, e = R(az), R(el)
    cam.location = Vector(look) + Vector(
        (dist * math.cos(e) * math.cos(a), dist * math.cos(e) * math.sin(a), dist * math.sin(e))
    )
    target.location = Vector(look)
    cam.data.lens = lens
    cam.data.shift_x, cam.data.shift_y = shift
    cam.data.dof.use_dof = True
    cam.data.dof.focus_object = target
    cam.data.dof.aperture_fstop = fstop


def render_to(path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    bpy.context.scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print(f"  -> {path}")


# ------------------------------------------------------------------ preview
PREVIEW_ANGLES = [
    ("a_front34", dict(az=32, el=13, dist="fit", lens=80)),
    ("b_rear34", dict(az=148, el=15, dist="fit", lens=80)),
    ("c_low_hero", dict(az=52, el=4, dist="fit", lens=50, look=(0.0, 0.0, 0.62))),
    ("d_side", dict(az=90, el=10, dist="fit", lens=85)),
    ("e_high", dict(az=-38, el=34, dist="fit", lens=60)),
    ("f_turret", dict(az=20, el=8, dist=1.5, lens=85, look=(0.24, 0.0, 1.06))),
]


def cmd_preview(args) -> None:
    sc = bpy.context.scene
    sc.cycles.samples = args.samples
    sc.render.resolution_x, sc.render.resolution_y = 960, 540
    sc.render.use_motion_blur = False
    cam, target = make_camera()
    set_pose(POSES[args.pose])
    for name, kw in PREVIEW_ANGLES:
        place_camera(cam, target, **kw)
        render_to(os.path.join(OUT, "preview", f"{name}.png"))


def cmd_still(args) -> None:
    sc = bpy.context.scene
    sc.cycles.samples = args.samples
    sc.render.resolution_x, sc.render.resolution_y = args.res
    sc.render.use_motion_blur = False
    cam, target = make_camera()
    set_pose(POSES[args.pose])
    place_camera(cam, target, az=args.az, el=args.el, dist=args.dist, lens=args.lens,
                 look=args.look, fstop=args.fstop)
    render_to(args.out)


# ------------------------------------------------------------------- 動画
def ease(t: float, kind: str = "inout") -> float:
    t = min(1.0, max(0.0, t))
    if kind == "linear":
        return t
    if kind == "in":
        return t * t * t
    if kind == "out":
        return 1.0 - (1.0 - t) ** 3
    return t * t * (3.0 - 2.0 * t)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# 各ショットは 30fps。dist は "fit" なら自動距離への倍率、"abs" なら実距離[m]
SHOTS: dict[str, dict] = {
    "a_wake": dict(
        frames=110, ease="out", fstop=11.0, light_in=True,
        az=(66, 52), el=(2.5, 7.0), lens=(42, 42), dist=(1.30, 1.02), dist_mode="fit",
        look=((0.0, 0.0, 0.50), (0.0, 0.0, 0.68)),
        pose_from=POSES["stow"], pose_to=POSES["stow"],
    ),
    "b_orbit": dict(
        frames=200, ease="inout", fstop=13.0,
        # az の終わりを 160 までにしてあるのは、これ以上回すと床に映った
        # リムライトの虚像が画面左に板のように入るため。
        az=(20, 160), el=(10, 20), lens=(55, 55), dist=(1.05, 1.05), dist_mode="fit",
        look=((0.0, 0.0, 0.70), (0.0, 0.0, 0.72)),
        pose_from=POSES["stow"], pose_to=POSES["match"],
    ),
    "c_turret": dict(
        blur=True, frames=140, ease="inout", fstop=7.0,
        az=(35, 4), el=(5, 16), lens=(85, 85), dist=(1.50, 1.22), dist_mode="abs",
        look=((0.30, 0.0, 1.00), (0.14, 0.0, 1.05)),
        pose_from=dict(turret_yaw=R(-26), shooter_pitch=R(28)),
        pose_to=dict(turret_yaw=R(26), shooter_pitch=R(66)),
        spin=dict(roller_upper=6.0, roller_lower=-6.0),
    ),
    "d_grabber": dict(
        blur=True, frames=140, ease="inout", fstop=7.0,
        az=(160, 206), el=(14, 5), lens=(65, 65), dist=(1.95, 1.70), dist_mode="abs",
        look=((-0.20, 0.0, 0.84), (-0.28, 0.0, 0.78)),
        pose_from=dict(grabber_slide=0.0, grabber_press=0.0, fork_tilt=0.0, shooter_pitch=R(24)),
        pose_to=dict(grabber_slide=0.30, grabber_press=0.13, fork_tilt=R(20), shooter_pitch=R(24)),
        spin=dict(singulator=2.0),
    ),
    "e_drive": dict(
        blur=True, frames=110, ease="inout", fstop=9.0,
        az=(124, 62), el=(0.8, 2.4), lens=(24, 24), dist=(0.76, 0.70), dist_mode="fit",
        look=((0.10, 0.0, 0.30), (-0.04, 0.0, 0.36)),
        pose_from=POSES["match"], pose_to=POSES["match"],
        spin=dict(wheel_fl=2.2, wheel_fr=2.2, wheel_rl=2.2, wheel_rr=2.2),
    ),
    "f_finale": dict(
        frames=150, ease="inout", fstop=11.0,
        az=(42, 26), el=(7, 20), lens=(60, 60), dist=(1.00, 1.11), dist_mode="fit",
        look=((0.0, 0.0, 0.68), (0.0, 0.0, 0.74)),
        pose_from=POSES["match"], pose_to=POSES["aim_left"],
    ),
}


def animate_lights() -> None:
    """暗がりから順に灯る。リム→キー→補助の順に入れると立ち上がりが見える。

    灯体は発光メッシュなので、ライトの energy ではなく Emission ノードの
    Strength にキーを打つ。
    """
    order = ("LGT_RimBack", "LGT_StripRear", "LGT_Key",
             "LGT_StripFront", "LGT_StripSide", "LGT_StripCam", "LGT_Fill")
    for i, name in enumerate(order):
        o = bpy.data.objects.get(name)
        if o is None or not o.data.materials:
            continue
        nt = o.data.materials[0].node_tree
        emi = next((n for n in nt.nodes if n.type == "EMISSION"), None)
        if emi is None:
            continue
        sock = emi.inputs["Strength"]
        full = sock.default_value
        delay = 1 + i * 6
        sock.default_value = 0.0
        sock.keyframe_insert("default_value", frame=1)
        sock.keyframe_insert("default_value", frame=delay)
        sock.default_value = full
        sock.keyframe_insert("default_value", frame=delay + 26)


def apply_shot(shot: dict, f: int, n: int, cam, target, fps: int, key: bool = False) -> None:
    """ショットの f フレーム目の状態（カメラ・関節）を作る。"""
    t = ease((f - 1) / max(1, n - 1), shot.get("ease", "inout"))
    lens = lerp(*shot["lens"], t)
    look = tuple(lerp(a, b, t) for a, b in zip(*shot["look"]))
    az = lerp(*shot["az"], t)
    el = lerp(*shot["el"], t)
    d = lerp(*shot["dist"], t)
    if shot.get("dist_mode", "fit") == "fit":
        d *= fit_distance(lens, look, az, el)
    place_camera(cam, target, az=az, el=el, dist=d, lens=lens, look=look,
                 fstop=shot.get("fstop", 9.0))
    if key:
        cam.keyframe_insert("location", frame=f)
        cam.data.keyframe_insert("lens", frame=f)
        target.keyframe_insert("location", frame=f)

    pose = {k: lerp(shot["pose_from"][k], shot["pose_to"][k], t) for k in shot["pose_from"]}
    for j, rps in shot.get("spin", {}).items():
        pose[j] = 2 * math.pi * rps * (f - 1) / fps
    set_pose(pose, frame=f if key else None)


def cmd_contact(args) -> None:
    """各ショットの頭・中・尻だけを撮る。構図と動きの確認用。"""
    sc = bpy.context.scene
    sc.cycles.samples = args.samples
    sc.render.resolution_x, sc.render.resolution_y = args.res
    sc.render.use_motion_blur = False
    cam, target = make_camera()
    names = args.only.split(",") if args.only else list(SHOTS)
    for name in names:
        shot = SHOTS[name]
        n = shot["frames"]
        for lbl, f in (("a", 1), ("b", (n + 1) // 2), ("c", n)):
            apply_shot(shot, f, n, cam, target, sc.render.fps)
            render_to(os.path.join(OUT, "contact", f"{name}_{lbl}.png"))


def cmd_anim(args) -> None:
    shot = SHOTS[args.shot]
    sc = bpy.context.scene
    sc.cycles.samples = args.samples
    sc.cycles.adaptive_threshold = args.threshold
    sc.render.resolution_x, sc.render.resolution_y = args.res
    sc.render.use_motion_blur = shot.get("blur", False)
    n = shot["frames"]
    sc.frame_start, sc.frame_end = 1, n

    cam, target = make_camera()
    if shot.get("light_in"):
        animate_lights()
    for f in range(1, n + 1):
        apply_shot(shot, f, n, cam, target, sc.render.fps, key=True)

    # 回転は最短距離で補間されると巻き戻るので、全部リニアにしておく
    for act in bpy.data.actions:
        for fc in act.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = "LINEAR"

    out_dir = os.path.join(OUT, "anim", args.shot)
    os.makedirs(out_dir, exist_ok=True)
    sc.render.filepath = os.path.join(out_dir, "")
    print(f"[{args.shot}] {n} フレーム -> {out_dir}")
    bpy.ops.render.render(animation=True)


def parse_res(s: str) -> tuple[int, int]:
    w, h = s.lower().split("x")
    return int(w), int(h)


def parse_vec(s: str) -> tuple[float, float, float]:
    return tuple(float(x) for x in s.split(","))


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("preview")
    p.add_argument("--samples", type=int, default=48)
    p.add_argument("--pose", default="match")
    p.set_defaults(func=cmd_preview)

    p = sub.add_parser("still")
    p.add_argument("--samples", type=int, default=512)
    p.add_argument("--pose", default="match")
    p.add_argument("--az", type=float, default=32)
    p.add_argument("--el", type=float, default=13)
    p.add_argument("--dist", type=lambda s: s if s == "fit" else float(s), default="fit")
    p.add_argument("--lens", type=float, default=80)
    p.add_argument("--fstop", type=float, default=9.0)
    p.add_argument("--look", type=parse_vec, default=(0.0, 0.0, 0.72))
    p.add_argument("--res", type=parse_res, default=(2560, 1440))
    p.add_argument("--out", default=os.path.join(OUT, "still.png"))
    p.set_defaults(func=cmd_still)

    p = sub.add_parser("contact")
    p.add_argument("--samples", type=int, default=32)
    p.add_argument("--res", type=parse_res, default=(1280, 720))
    p.add_argument("--only", default="", help="カンマ区切りでショットを絞る")
    p.set_defaults(func=cmd_contact)

    p = sub.add_parser("anim")
    p.add_argument("--shot", required=True, choices=sorted(SHOTS))
    p.add_argument("--samples", type=int, default=64)
    p.add_argument("--res", type=parse_res, default=(1920, 1080))
    p.add_argument("--threshold", type=float, default=0.007,
                   help="適応サンプリングの打ち切り。静止画より緩めてよい")
    p.add_argument("--frames", type=int, default=0, help="0 以外でショットの尺を上書き")
    p.set_defaults(func=cmd_anim)

    args = ap.parse_args(argv)
    if getattr(args, "frames", 0):
        SHOTS[args.shot]["frames"] = args.frames
    enable_gpu()
    args.func(args)


if __name__ == "__main__":
    main()
