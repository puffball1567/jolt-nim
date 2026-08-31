#!/bin/sh
set -eu

if [ "$#" -ne 2 ]; then
  echo "usage: tests/check_raw_compute_vk.sh <path-to-jolt> <path-to-vulkan-headers>" >&2
  exit 2
fi

jolt_source=$1
vulkan_headers=$2
nim_cache=$(mktemp -d)
trap 'rm -rf "$nim_cache"' EXIT HUP INT TERM

nim cpp --noLinking:on --path:src \
  --nimcache:"$nim_cache" \
  --passC:"-I$jolt_source" \
  --passC:"-I$vulkan_headers" \
  --passC:-std=c++17 \
  --passC:-DNDEBUG \
  --passC:-DJPH_OBJECT_STREAM \
  --passC:-DJPH_USE_VK \
  tests/test_raw_compute_vk_compile.nim
