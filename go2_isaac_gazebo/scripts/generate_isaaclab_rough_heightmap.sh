#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${PROJECT_ROOT}/src/rl_sar/worlds/isaaclab_rough_heightmap.png"
BASE_MAP="$(mktemp /tmp/go2_rough_base.XXXXXX.png)"
HOLE_MASK="$(mktemp /tmp/go2_rough_holes.XXXXXX.png)"
trap 'rm -f "${BASE_MAP}" "${HOLE_MASK}"' EXIT

if ! command -v convert >/dev/null 2>&1; then
  echo "ImageMagick 'convert' is required to regenerate the terrain." >&2
  exit 1
fi

# Generate spatially correlated random ground instead of sharp pixel noise.
# Gazebo scales the final grayscale range to the height in the world file.
convert \
  -seed 20260818 \
  -size 257x257 xc:gray50 \
  +noise Random \
  -colorspace Gray \
  -blur 0x2.0 \
  -resize 513x513! \
  -normalize \
  -posterize 12 \
  "${BASE_MAP}"

# Sparse random seeds become shallow circular holes. At 12.5 cm per height
# sample, the blurred three-pixel cores are approximately 25-50 cm wide.
convert \
  -seed 20260819 \
  -size 513x513 xc:gray50 \
  +noise Random \
  -colorspace Gray \
  -threshold 99.98% \
  -morphology Dilate Disk:1 \
  -blur 0x0.45 \
  "${HOLE_MASK}"

convert \
  "${BASE_MAP}" \
  "${HOLE_MASK}" \
  -fx 'max(0,u-0.35*v)' \
  -fill black \
  -draw 'rectangle 0,0 512,4 rectangle 0,508 512,512 rectangle 0,0 4,512 rectangle 508,0 512,512' \
  -fill '#777777' \
  -draw 'rectangle 253,253 259,259' \
  -depth 8 \
  "${OUTPUT}"

echo "Generated ${OUTPUT}"
