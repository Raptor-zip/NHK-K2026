"""雑巾メッシュのトポロジー(viewer/src/sim/rapier-rag.ts の移植)。

2層(上下面)スラブ: 12×16セル、13×17=221ノード/層 ×2層 = 442ノード。
ばね5種(structural/shear/bend/thickness/interShear)。ばね剛性は解像度較正済み
(bend/interShear のみ rBend でスケール)。数値は TS 版と同一。
"""
from __future__ import annotations

import numpy as np

NX, NY = 12, 16
BASE_NX, BASE_NY = 8, 11
RAG_W, RAG_L, THICK = 0.2, 0.3, 0.014
NODES_X, NODES_Y = NX + 1, NY + 1
N_SURF = NODES_X * NODES_Y
N_NODES = 2 * N_SURF
NODE_R = THICK / 2

RHO = 1.225
ADDED_MASS_C = 1.0

R_BEND = (NX / BASE_NX + NY / BASE_NY) / 2  # ≒1.48

# (ks, kd)
SPRING = {
    "structural": (130.0, 0.5),
    "shear": (3.0, 0.08),
    "bend": (0.04 * R_BEND, 0.012 * R_BEND),
    "thickness": (260.0, 1.0),
    "interShear": (0.8 * R_BEND, 0.04 * R_BEND),
}


def idx(ix: int, iy: int, layer: int = 0) -> int:
    return layer * N_SURF + iy * NODES_X + ix


def node_mass(rag_m: float) -> float:
    """付加質量(円板近似)込みの1ノード質量"""
    A = RAG_W * RAG_L
    mam = ADDED_MASS_C * (8 / 3) * RHO * (A / np.pi) ** 1.5
    return (rag_m + mam) / N_NODES


def build_springs() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """ばね接続 (i, j, ks, kd)。静止長は初期形状から別途計算する(TS版と同じ)。"""
    ii: list[int] = []
    jj: list[int] = []
    ks: list[float] = []
    kd: list[float] = []

    def link(a: int, b: int, kind: str) -> None:
        ii.append(a)
        jj.append(b)
        ks.append(SPRING[kind][0])
        kd.append(SPRING[kind][1])

    for layer in range(2):
        for iy in range(NODES_Y):
            for ix in range(NODES_X):
                a = idx(ix, iy, layer)
                if ix + 1 < NODES_X:
                    link(a, idx(ix + 1, iy, layer), "structural")
                if iy + 1 < NODES_Y:
                    link(a, idx(ix, iy + 1, layer), "structural")
                if ix + 1 < NODES_X and iy + 1 < NODES_Y:
                    link(a, idx(ix + 1, iy + 1, layer), "shear")
                    link(idx(ix + 1, iy, layer), idx(ix, iy + 1, layer), "shear")
                if ix + 2 < NODES_X:
                    link(a, idx(ix + 2, iy, layer), "bend")
                if iy + 2 < NODES_Y:
                    link(a, idx(ix, iy + 2, layer), "bend")
    for iy in range(NODES_Y):
        for ix in range(NODES_X):
            t = idx(ix, iy, 0)
            bt = idx(ix, iy, 1)
            link(t, bt, "thickness")
            if ix + 1 < NODES_X:
                link(t, idx(ix + 1, iy, 1), "interShear")
                link(idx(ix + 1, iy, 0), bt, "interShear")
            if iy + 1 < NODES_Y:
                link(t, idx(ix, iy + 1, 1), "interShear")
                link(idx(ix, iy + 1, 0), bt, "interShear")
    return (
        np.asarray(ii, dtype=np.int64),
        np.asarray(jj, dtype=np.int64),
        np.asarray(ks, dtype=np.float32),
        np.asarray(kd, dtype=np.float32),
    )


def surface_quads(layer: int) -> np.ndarray:
    """[NX*NY, 4] 片層パネルの頂点index(描画・空力)"""
    out = []
    for iy in range(NY):
        for ix in range(NX):
            out.append([idx(ix, iy, layer), idx(ix + 1, iy, layer), idx(ix + 1, iy + 1, layer), idx(ix, iy + 1, layer)])
    return np.asarray(out, dtype=np.int64)


def boundary_edges() -> np.ndarray:
    e = []
    for ix in range(NX):
        e.append([idx(ix, 0), idx(ix + 1, 0)])
        e.append([idx(ix + 1, NY), idx(ix, NY)])
    for iy in range(NY):
        e.append([idx(NX, iy), idx(NX, iy + 1)])
        e.append([idx(0, iy + 1), idx(0, iy)])
    return np.asarray(e, dtype=np.int64)


def place_hanging(grip: np.ndarray, hang: np.ndarray, lateral: np.ndarray, normal: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """把持点 grip から hang 方向へ伸びる静止雑巾 [442,3] (placeClothHanging 移植)。
    ベルト装填ではトラック後方(-trackDir)を hang に渡して寝かせる。"""
    pos = np.zeros((N_NODES, 3), dtype=np.float64)
    half = THICK / 2
    for layer in range(2):
        off = half if layer == 0 else -half
        for iy in range(NODES_Y):
            for ix in range(NODES_X):
                a = (iy / NY) * RAG_L
                b = (ix / NX - 0.5) * RAG_W
                ripple = 0.007 * np.sin(3.1 * (iy / NY) + 2.3 * (ix / NX)) + 0.003 * rng.standard_normal()
                r = hang * a + lateral * b + normal * (ripple + off)
                jitter = rng.standard_normal(3) * 0.002
                pos[idx(ix, iy, layer)] = grip + r + jitter
    return pos.astype(np.float32)
