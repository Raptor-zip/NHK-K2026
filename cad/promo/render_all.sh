#!/usr/bin/env bash
# 全ショットを連番でレンダリングし、1 本の mp4 にまとめる。
#
#   ./render_all.sh                 # 本番（1920x1080 / 48 サンプル・GPU で 2〜3 時間）
#   SAMPLES=24 RES=960x540 ./render_all.sh   # 下見
#
# 途中で止めても、済んだショットの連番は残るので SHOTS を絞って再開できる。
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
BLENDER=${BLENDER:-$HOME/opt/blender-4.5.12-linux-x64/blender}
BLEND=${BLEND:-$HERE/tr_promo.blend}
SAMPLES=${SAMPLES:-48}
RES=${RES:-1920x1080}
FPS=30
OUT=$HERE/out
SHOTS=(${SHOTS:-a_wake b_orbit c_turret d_grabber e_drive f_finale})

for s in "${SHOTS[@]}"; do
  echo "=== $s ==="
  "$BLENDER" -b "$BLEND" -P "$HERE/shots.py" -- anim --shot "$s" --samples "$SAMPLES" --res "$RES"
done

# ショットごとに h264 へ。頭と尻だけ暗転から／暗転へ
mkdir -p "$OUT/clips"
for s in "${SHOTS[@]}"; do
  dir=$OUT/anim/$s
  n=$(ls "$dir"/*.png 2>/dev/null | wc -l | tr -d " ")
  [ "$n" -gt 0 ] || { echo "!! $s の連番が無い"; exit 1; }
  filt="scale=trunc(iw/2)*2:trunc(ih/2)*2"
  case "$s" in
    "${SHOTS[0]}") filt="$filt,fade=t=in:st=0:d=0.7" ;;
    "${SHOTS[${#SHOTS[@]}-1]}")
      dur=$(python3 -c "print(f'{$n/$FPS - 1.1:.3f}')")
      filt="$filt,fade=t=out:st=$dur:d=1.1" ;;
  esac
  ffmpeg -y -loglevel error -framerate $FPS -start_number 1 -i "$dir/%04d.png" \
    -vf "$filt" -c:v libx264 -crf 16 -preset slow -pix_fmt yuv420p \
    "$OUT/clips/$s.mp4"
  echo "  clip: $s ($n フレーム)"
done

: > "$OUT/clips/concat.txt"
for s in "${SHOTS[@]}"; do echo "file '$OUT/clips/$s.mp4'" >> "$OUT/clips/concat.txt"; done
ffmpeg -y -loglevel error -f concat -safe 0 -i "$OUT/clips/concat.txt" -c copy "$OUT/TR_promo.mp4"
echo "できた: $OUT/TR_promo.mp4"
ffprobe -v error -show_entries format=duration -of default=nk=1:nw=1 "$OUT/TR_promo.mp4"
