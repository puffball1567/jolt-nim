# Changelog

All notable changes to joltnim are documented in this file.

## 0.2.0 - Unreleased

### Added

- Detached world-wide body query snapshots covering rigid, soft, character,
  scene and ragdoll bodies.
- Declarative property filters for motion type, collision layer, activation,
  sensor, soft-body, broad-phase and user-data state.
- Caller-thread Nim predicates that resolve to reusable native query filters
  without invoking Nim from Jolt worker threads.

## 0.1.1 - 2026-09-01

### Fixed

- Corrected const-qualified C++ bridge declarations for Clang compatibility.
- Stabilized native test linking and execution with MinGW on Windows.
- Consolidated the native test runner while retaining raw API coverage.

## 0.1.0 - 2026-08-31

Initial public release targeting Jolt Physics 5.6.0.

### Added

- Direct `jolt/raw` bindings for every callable owner/name group inventoried in
  the supported CPU, DebugRenderer, Vulkan, DirectX 12, and Metal build
  configurations.
- Ownership-safe `World`, rigid and soft body, shape, constraint, query,
  character, vehicle, ragdoll, skeleton, animation, scene, and rollback APIs.
- Bounded native event queues that avoid calling Nim from Jolt worker threads.
- Interactive raylib scenes plus optional Naylib and SDL3 integration demos.
- Native tests on Linux, macOS, and Windows with Jolt Physics 5.6.0 pinned in
  continuous integration.

### Compatibility

- Nim 2.0 or newer.
- C++17.
- Jolt Physics 5.6.0. Jolt build flags used by the Nim translation unit must
  match those used to build the linked Jolt library.
