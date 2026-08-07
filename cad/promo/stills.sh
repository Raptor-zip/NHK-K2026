#!/usr/bin/env bash
# 宣材写真を焼く。既定は 2560x1440 / 512 サンプルで 1 枚 3 分ほど（RTX / OptiX）。
#
# ⚠ 絞りは f/8〜13 で渡している。プロダクト写真は製品全体にピントが合って
#   いるのが基本で、f/2.8 のような開放だと画素数を上げても解像しない。
#
#   ./stills.sh                             # 既定
#   RES=1920x1080 SAMPLES=128 ./stills.sh   # 下見
#   RES=3840x2160 SAMPLES=768 OUT=$PWD/out/stills4k ./stills.sh   # 4K
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
BLENDER=${BLENDER:-$HOME/opt/blender-4.5.12-linux-x64/blender}
BLEND=${BLEND:-$HERE/tr_promo.blend}
RES=${RES:-2560x1440}
SAMPLES=${SAMPLES:-512}
OUT=${OUT:-$HERE/out/stills}

shot() {  # 名前 姿勢 方位 仰角 画角 距離 注視点 絞り
  echo "=== $1 ==="
  # ⚠ 負の値は --opt=値 の形で渡す。--look -0.26,0,0.8 は argparse が
  #   オプション名と誤認する（カンマ入りなので負数として扱われない）。
  "$BLENDER" -b "$BLEND" -P "$HERE/shots.py" -- still \
    --pose "$2" --az="$3" --el="$4" --lens "$5" --dist "$6" --look="$7" --fstop "$8" \
    --samples "$SAMPLES" --res "$RES" --out "$OUT/$1.png"
}

shot hero        match 46  8   55 fit  0,0,0.66 11
shot hero_low    match 62  3   42 fit  0,0,0.60 9
shot rear34      load  158 12  62 fit  0,0,0.70 11
shot turret      match 26  9   85 1.35 0.18,0,1.04 8
shot grabber     load  198 8   65 1.75 -0.26,0,0.80 8
shot topdown     match -34 40  50 fit  0,0,0.72 13
shot display     match 176 4   100 1.05 -0.30,0,0.48 8
# 寄りの検査ショット。LiDAR の水平出し台（下段 φ 走査面 z=69.6）を正面から見る
shot lidar_front match 22  7   100 0.62 0.445,0,0.11 8

echo "できた: $OUT"
ls -la "$OUT"
