#!/bin/sh
set -eu

if [ "$#" -lt 2 ]; then
  echo "usage: examples/run_visual_demos.sh <path-to-jolt> <path-to-libJolt.a> [scene] [frames] [screenshot]" >&2
  exit 2
fi

jolt_source=$1
jolt_library=$2
shift 2

build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
demo_binary=$build_dir/jolt_visual_demos

nim cpp --path:src --path:examples \
  --nimcache:"$build_dir/nimcache" \
  --out:"$demo_binary" \
  --passC:"-I$jolt_source" \
  --passC:-std=c++17 \
  --passC:-DNDEBUG \
  --passC:-DJPH_OBJECT_STREAM \
  --passC:-DJPH_USE_AVX \
  --passC:-DJPH_USE_AVX2 \
  --passC:-DJPH_USE_CPU_COMPUTE \
  --passC:-DJPH_USE_F16C \
  --passC:-DJPH_USE_FMADD \
  --passC:-DJPH_USE_LZCNT \
  --passC:-DJPH_USE_SSE4_1 \
  --passC:-DJPH_USE_SSE4_2 \
  --passC:-DJPH_USE_TZCNT \
  --passC:-mavx2 \
  --passC:-mbmi \
  --passC:-mpopcnt \
  --passC:-mlzcnt \
  --passC:-mf16c \
  --passC:-mfma \
  --passC:-mfpmath=sse \
  --passC:-pthread \
  --passL:"$jolt_library" \
  --passL:-lraylib \
  --passL:-pthread \
  --passL:-flto \
  examples/visual_demos.nim

"$demo_binary" "$@"
