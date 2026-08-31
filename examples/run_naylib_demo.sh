#!/bin/sh
set -eu

if [ "$#" -lt 3 ]; then
  echo "usage: examples/run_naylib_demo.sh <path-to-jolt> <path-to-libJolt.a> <path-to-naylib> [frames] [screenshot]" >&2
  exit 2
fi

jolt_source=$1
jolt_library=$2
naylib_source=$3
shift 3
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

nim cpp --path:src --path:"$naylib_source" \
  --nimcache:"$build_dir/nimcache" \
  --out:"$build_dir/jolt_naylib_demo" \
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
  examples/naylib_demo.nim

"$build_dir/jolt_naylib_demo" "$@"
