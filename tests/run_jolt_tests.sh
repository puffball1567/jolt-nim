#!/bin/sh
set -eu

if [ "$#" -lt 2 ]; then
  echo "usage: tests/run_jolt_tests.sh <path-to-jolt> <path-to-libJolt.a> [test.nim ...]" >&2
  exit 2
fi

jolt_source=$1
jolt_library=$2
shift 2
nim_cache=$(mktemp -d)
trap 'rm -rf "$nim_cache"' EXIT HUP INT TERM

tests/check_raw_api_constructors.sh "$jolt_source"

run_test() {
  test_source=$1
  test_name=${test_source##*/}
  test_name=${test_name%.nim}
  test_binary="$nim_cache/$test_name"
  nim cpp --path:src \
    --nimcache:"$nim_cache" \
    --out:"$test_binary" \
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
    "$test_source"
  if [ -f "$test_binary.exe" ]; then
    test_binary="$test_binary.exe"
  fi
  "$test_binary"
}

if [ "$#" -eq 0 ]; then
  set -- \
    tests/test_all.nim \
    tests/test_raw_object_stream.nim \
    tests/test_utility_geometry.nim \
    tests/test_raw_namespace_helpers.nim \
    tests/test_raw_compute_cpu.nim
fi

for test_source in "$@"; do
  run_test "$test_source"
done
