"""板材のトポロジー最適化.

    core  … 共通の型（Region / TopoResult / Outline）とマスク作り
    simp  … SIMP + OC 法のソルバ（密度場を出す）
    shape … 密度場 → 切り抜き輪郭 → build123d の面（製造制約の検査つき）
    draw  … 荷重ヒートマップと平面図の PNG

使い方は `scripts/topo_opt.py` を見ること。
"""

from .core import Circle, Outline, Rect, Region, TopoResult  # noqa: F401

__all__ = ["Region", "TopoResult", "Outline", "Rect", "Circle"]
