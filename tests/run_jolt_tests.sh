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
  nim cpp -r --path:src \
    --nimcache:"$nim_cache" \
    --out:"$nim_cache/$test_name" \
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
    "$test_source"
}

if [ "$#" -eq 0 ]; then
  set -- \
    tests/test_raw_api.nim \
    tests/test_raw_object_stream.nim \
    tests/test_utility_geometry.nim \
    tests/test_raw_namespace_helpers.nim \
    tests/test_raw_compute_cpu.nim \
    tests/test_raw.nim \
    tests/test_simulation_settings.nim \
    tests/test_falling_box.nim \
    tests/test_shapes_and_motion.nim \
    tests/test_extended_shapes.nim \
    tests/test_additional_shapes.nim \
    tests/test_body_dynamics.nim \
    tests/test_body_snapshots.nim \
    tests/test_body_config.nim \
    tests/test_body_batches.nim \
    tests/test_body_shape_and_buoyancy.nim \
    tests/test_physics_materials.nim \
    tests/test_character.nim \
    tests/test_rigid_character.nim \
    tests/test_complex_shapes.nim \
    tests/test_triangle_mesh.nim \
    tests/test_height_field.nim \
    tests/test_compound_shapes.nim \
    tests/test_decorated_and_mutable_shapes.nim \
    tests/test_vehicle.nim \
    tests/test_tracked_vehicle.nim \
    tests/test_constraints_and_queries.nim \
    tests/test_advanced_constraints.nim \
    tests/test_specialized_constraints.nim \
    tests/test_spatial_queries.nim \
    tests/test_shape_casts.nim \
    tests/test_convex_queries.nim \
    tests/test_broad_phase_queries.nim \
    tests/test_collision_layers.nim \
    tests/test_collision_groups.nim \
    tests/test_sensors.nim \
    tests/test_events.nim \
    tests/test_contact_policies.nim \
    tests/test_state_restore.nim \
    tests/test_soft_body.nim \
    tests/test_ragdoll.nim \
    tests/test_skeleton_mapper.nim \
    tests/test_skeletal_animation.nim \
    tests/test_physics_scene.nim
fi

for test_source in "$@"; do
  run_test "$test_source"
done
