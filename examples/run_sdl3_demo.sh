#!/bin/sh
set -eu

if [ "$#" -lt 2 ]; then
  echo "usage: examples/run_sdl3_demo.sh <path-to-jolt> <path-to-libJolt.a> [frames] [screenshot.bmp]" >&2
  exit 2
fi

jolt_source=$1
jolt_library=$2
shift 2
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

nim cpp --path:src \
  --nimcache:"$build_dir/nimcache" \
  --out:"$build_dir/jolt_sdl3_demo" \
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
  --passL:-pthread \
  --passL:-flto \
  examples/sdl3_demo.nim

"$build_dir/jolt_sdl3_demo" "$@"
