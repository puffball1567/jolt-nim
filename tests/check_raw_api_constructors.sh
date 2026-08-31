#!/bin/sh
set -eu

if [ "$#" -ne 1 ]; then
  echo "usage: tests/check_raw_api_constructors.sh <path-to-jolt>" >&2
  exit 2
fi

jolt_source=$1
work_dir=$(mktemp -d)
trap 'rm -rf "$work_dir"' EXIT HUP INT TERM

generate_release_smoke() {
  awk '
    BEGIN { print "import jolt/raw as api\n\nproc smoke() =" }
    /^[[:space:]]*proc construct[^*]*\*\(\):/ {
      line = $0
      sub(/^[[:space:]]*proc /, "", line)
      sub(/\*.*/, "", line)
      if (line != "constructAssertLastParam" && line != "constructBodyAccess" &&
          line != "constructDebugRenderer_Triangle" &&
          line != "constructDebugRendererRecorder_TextBlob" &&
          line != "constructBodyManager_DrawSettings" &&
          line != "constructHair_DrawSettings" &&
          line != "constructSkeletonPose_DrawSettings" && !seen[line]++)
        print "  discard api." line "()"
    }
    END { print "\nsmoke()" }
  ' src/jolt/raw_api.nim > "$work_dir/release_constructors.nim"
}

generate_assert_smoke() {
  awk '
    BEGIN { print "import jolt/raw as api\n\nproc smoke() =" }
    /^[[:space:]]*proc construct[^*]*\*\(\):/ {
      line = $0
      sub(/^[[:space:]]*proc /, "", line)
      sub(/\*.*/, "", line)
      if (line != "constructConstraintManager" &&
          line != "constructDebugRenderer_Triangle" &&
          line != "constructDebugRendererRecorder_TextBlob" &&
          line != "constructBodyManager_DrawSettings" &&
          line != "constructHair_DrawSettings" &&
          line != "constructSkeletonPose_DrawSettings" && !seen[line]++)
        print "  discard api." line "()"
    }
    END { print "\nsmoke()" }
  ' src/jolt/raw_api.nim > "$work_dir/assert_constructors.nim"
}

compile_smoke() {
  source_file=$1
  cache_dir=$2
  shift 2
  nim cpp --noLinking:on --path:src \
    --nimcache:"$cache_dir" \
    --passC:"-I$jolt_source" \
    --passC:-std=c++17 \
    --passC:-DJPH_OBJECT_STREAM \
    --passC:-DJPH_USE_CPU_COMPUTE \
    "$@" \
    "$source_file"
}

generate_release_smoke
generate_assert_smoke
compile_smoke "$work_dir/release_constructors.nim" "$work_dir/release-cache" \
  --passC:-DNDEBUG
compile_smoke "$work_dir/assert_constructors.nim" "$work_dir/assert-cache" \
  -d:joltEnableAsserts \
  --passC:-DJPH_ENABLE_ASSERTS
