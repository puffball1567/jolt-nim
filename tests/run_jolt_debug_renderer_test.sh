#!/bin/sh
set -eu

if [ "$#" -ne 2 ]; then
  echo "usage: tests/run_jolt_debug_renderer_test.sh <path-to-jolt> <path-to-libJolt.a>" >&2
  exit 2
fi

jolt_source=$1
jolt_library=$2
nim_cache=$(mktemp -d)
trap 'rm -rf "$nim_cache"' EXIT HUP INT TERM

nim cpp -r --path:src \
  --nimcache:"$nim_cache" \
  --out:"$nim_cache/test_raw_debug_renderer" \
  --passC:"-I$jolt_source" \
  --passC:-std=c++17 \
  --passC:-fno-rtti \
  --passC:-DNDEBUG \
  --passC:-DJPH_DEBUG_RENDERER \
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
  tests/test_raw_debug_renderer.nim
