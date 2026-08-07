"""画質と所要時間のトレードオフを実測する。

    blender -b tr_promo.blend -P quality_test.py -- --res 1280x720

同じ構図を設定違いで撮り、1 枚あたりの秒数を出す。等倍で見比べて
`setup_render()` の既定値を決めるために使う。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shots  # noqa: E402

PRESETS: dict[str, dict] = {
    # 絞り。プロダクト写真は「製品全体にピントが合っている」のが基本で、
    # 開けすぎると被写体の大半がボケて「解像していない絵」になる
    "f28_now": dict(samples=256, threshold=0.005, fstop=2.8),
    "f56": dict(samples=256, threshold=0.005, fstop=5.6),
    "f110": dict(samples=256, threshold=0.005, fstop=11.0),
    "f110_nobump": dict(samples=256, threshold=0.005, fstop=11.0, nobump=True),
    "f110_sharp": dict(samples=256, threshold=0.005, fstop=11.0, filter_width=1.1),
}


def strip_bump() -> None:
    """材質の手続きノイズ（roughness 変調と bump）を外す。

    scale 600 のような細かいノイズは、画素より細かくなるとジリジリした
    粒に見える。これが「画質が悪い」印象の一因になっていないかを見る。
    """
    for mat in bpy.data.materials:
        if not mat.name.startswith("TR_") or not mat.use_nodes:
            continue
        nt = mat.node_tree
        bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is None:
            continue
        for sock in ("Roughness", "Normal"):
            for link in list(nt.links):
                if link.to_node is bsdf and link.to_socket.name == sock:
                    if sock == "Roughness":
                        src = link.from_node
                        if src.type == "MAP_RANGE":
                            mid = (src.inputs["To Min"].default_value
                                   + src.inputs["To Max"].default_value) * 0.5
                            nt.links.remove(link)
                            bsdf.inputs["Roughness"].default_value = mid
                    else:
                        nt.links.remove(link)


def apply(preset: dict) -> None:
    c = bpy.context.scene.cycles
    c.samples = preset["samples"]
    c.adaptive_threshold = preset["threshold"]
    c.use_denoising = preset.get("denoise", True)
    if c.use_denoising:
        c.denoiser = preset.get("denoiser", "OPTIX")
        c.denoising_input_passes = "RGB_ALBEDO_NORMAL"
        c.denoising_prefilter = "ACCURATE"
        if hasattr(c, "denoising_use_gpu"):
            c.denoising_use_gpu = preset.get("gpu_denoise", False)
    c.filter_width = preset.get("filter_width", 1.5)
    if preset.get("nobump"):
        strip_bump()


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--only", default="")
    args = ap.parse_args(argv)

    shots.enable_gpu()
    sc = bpy.context.scene
    w, h = (int(v) for v in args.res.lower().split("x"))
    sc.render.resolution_x, sc.render.resolution_y = w, h
    sc.render.use_motion_blur = False

    cam, target = shots.make_camera()
    shots.set_pose(shots.POSES["match"])
    # 細い部材とローラーが同時に入る、粗が出やすい画
    def frame(fstop):
        shots.place_camera(cam, target, az=26, el=9, dist=1.35, lens=85,
                           look=(0.18, 0.0, 1.04), fstop=fstop)

    names = args.only.split(",") if args.only else list(PRESETS)
    for name in names:
        apply(PRESETS[name])
        frame(PRESETS[name].get("fstop", 2.8))
        path = os.path.join(HERE, "out", "quality", f"{name}.png")
        t0 = time.time()
        shots.render_to(path)
        print(f"[計測] {name}: {time.time() - t0:.1f} 秒  "
              f"samples={PRESETS[name]['samples']} th={PRESETS[name]['threshold']} "
              f"f/{PRESETS[name].get('fstop', 2.8)} "
              f"filter={PRESETS[name].get('filter_width', 1.5)} "
              f"nobump={PRESETS[name].get('nobump', False)}")


if __name__ == "__main__":
    main()
