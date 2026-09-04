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

run_test() {
  test_source=$1
  test_name=${test_source##*/}
  test_name=${test_name%.nim}
  test_binary="$nim_cache/$test_name"
  platform_link_flags=
  case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*)
      test_binary="$test_binary.exe"
      platform_link_flags="--passL:-static-libgcc --passL:-static-libstdc++"
      ;;
  esac
  nim cpp --path:src \
    -d:joltDebugRenderer \
    --nimcache:"$nim_cache" \
    --out:"$test_binary" \
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
    $platform_link_flags \
    "$test_source"
  "$test_binary"
}

run_test tests/test_raw_debug_renderer.nim
run_test tests/test_debug_draw.nim
