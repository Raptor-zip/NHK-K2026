"""スタート時コンフィグ（1000×1000×1200 の検証用ポーズ）。"""

from __future__ import annotations

import os as _os
import sys as _sys

_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import tr_assembly as A
import tr_params as P


def gen_step():
    return A.build(P.POSE_STOWED)
