"""3D透視投影 + 厚みスラブ雑巾の描画 (viewer/scripts/rag-draw.ts の PIL 移植)。"""
from __future__ import annotations

import colorsys
from dataclasses import dataclass

import numpy as np
from PIL import ImageDraw, ImageFont

from . import mesh

FONT_PATH = "/usr/share/fonts/opentype/ipaexfont-gothic/ipaexg.ttf"
_font_cache: dict[int, ImageFont.FreeTypeFont] = {}


def font(size: int) -> ImageFont.FreeTypeFont:
    if size not in _font_cache:
        _font_cache[size] = ImageFont.truetype(FONT_PATH, size)
    return _font_cache[size]


UP = np.array([0.0, 1.0, 0.0])


def _norm(a: np.ndarray) -> np.ndarray:
    l = np.linalg.norm(a)
    return a / l if l > 1e-12 else a


@dataclass
class Cam:
    eye: np.ndarray
    right: np.ndarray
    up: np.ndarray
    fwd: np.ndarray
    focal: float
    vx: float
    vy: float
    vw: float
    vh: float


def make_cam(eye: np.ndarray, center: np.ndarray, fov_deg: float, vx: float, vy: float, vw: float, vh: float) -> Cam:
    fwd = _norm(eye - center)
    right = _norm(np.cross(UP, fwd))
    up = np.cross(fwd, right)
    focal = vh / 2 / np.tan(np.radians(fov_deg) / 2)
    return Cam(eye, right, up, fwd, focal, vx, vy, vw, vh)


def project(cam: Cam, p: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """p: [...,3] → (sx, sy, front)"""
    d = p - cam.eye
    cx = d @ cam.right
    cy = d @ cam.up
    cz = d @ cam.fwd
    front = cz < -0.02
    denom = np.where(front, -cz, 0.02)
    sx = cam.vx + cam.vw / 2 + cam.focal * cx / denom
    sy = cam.vy + cam.vh / 2 - cam.focal * cy / denom
    return sx, sy, front


def hsl(h: float, s: float, l: float) -> tuple[int, int, int]:
    r, g, b = colorsys.hls_to_rgb((h % 360) / 360, l / 100, s / 100)
    return int(r * 255), int(g * 255), int(b * 255)


_QT = mesh.surface_quads(0)
_QB = mesh.surface_quads(1)[:, ::-1]
_SIDES = np.array([[i, j, j + mesh.N_SURF, i + mesh.N_SURF] for i, j in mesh.boundary_edges()])
_LIGHT = _norm(np.array([-0.3, 0.9, 0.25]))


def draw_rag(draw: ImageDraw.ImageDraw, cam: Cam, nodes: np.ndarray, base_hue: float) -> None:
    """物理2層スラブの雑巾を厚みごと描く(ペインターズ法+法線陰影)。nodes: [442,3]"""
    quads = np.concatenate([_QT, _QB, _SIDES])  # [F,4]
    side_mul = np.concatenate([np.ones(len(_QT) + len(_QB)), np.full(len(_SIDES), 0.68)])
    ps = nodes[quads]  # [F,4,3]
    n = np.cross(ps[:, 2] - ps[:, 0], ps[:, 3] - ps[:, 1])
    n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-12
    centers = ps.mean(axis=1)
    depth = np.linalg.norm(centers - cam.eye, axis=-1)
    order = np.argsort(-depth)
    sx, sy, front = project(cam, ps.reshape(-1, 3))
    sx = sx.reshape(-1, 4)
    sy = sy.reshape(-1, 4)
    front = front.reshape(-1, 4)
    shade = (0.35 + 0.65 * np.abs(n @ _LIGHT)) * side_mul
    for f in order:
        if not front[f].all():
            continue
        l = int(round(30 + shade[f] * 58))
        sat = 35 if side_mul[f] < 1 else 52
        poly = list(zip(sx[f].tolist(), sy[f].tolist()))
        draw.polygon(poly, fill=hsl(base_hue, sat, l), outline=hsl(base_hue, 40, max(18, l - 20)))


def line3(draw: ImageDraw.ImageDraw, cam: Cam, a, b, color, w: float) -> None:
    p = np.asarray([a, b], dtype=np.float64)
    sx, sy, front = project(cam, p)
    if not front.all():
        return
    draw.line([(sx[0], sy[0]), (sx[1], sy[1])], fill=color, width=max(1, int(round(w))))


def dot3(draw: ImageDraw.ImageDraw, cam: Cam, p, r: float, color) -> None:
    sx, sy, front = project(cam, np.asarray(p, dtype=np.float64))
    if not front:
        return
    draw.ellipse([sx - r, sy - r, sx + r, sy + r], fill=color)


def path3(draw: ImageDraw.ImageDraw, cam: Cam, pts: np.ndarray, color, w: float, dash: int = 0) -> None:
    """3D 折れ線。dash>0 で点線(セグメント数間引き)"""
    if len(pts) < 2:
        return
    sx, sy, front = project(cam, pts)
    seg = [(float(x), float(y)) for x, y, fr in zip(sx, sy, front) if fr]
    if len(seg) < 2:
        return
    if dash:
        for k in range(0, len(seg) - 1, 2):
            draw.line([seg[k], seg[k + 1]], fill=color, width=max(1, int(w)))
    else:
        draw.line(seg, fill=color, width=max(1, int(w)), joint="curve")
