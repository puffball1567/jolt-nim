# jolt-nim examples

## Headless falling box

`falling_box.nim` is a minimal rigid-body simulation with no renderer. Build it
using the Jolt flags shown in the top-level README.

## Interactive visual demos

`visual_demos.nim` uses raylib as an optional renderer. Raylib is not a runtime
dependency of the binding itself. The demo maintains a fixed 60 Hz physics
step and renders each body's live Jolt position and quaternion.

| Key | Scene | Features exercised |
| --- | --- | --- |
| `1` | Box tower | Box contacts, stacking, sleeping, deterministic cleanup |
| `2` | Sphere rain | Native batch creation of 64 varied spheres, arena walls, friction and restitution |
| `3` | Domino wave | Initial rotations, angular velocity, impulse and contact propagation |
| `4` | Mixed playground | Boxes, spheres, capsules, rotated static ramp and moving kinematic platform |
| `5` | Constraint bridge | Point-constraint ownership, load response and closest-hit ray casting |
| `6` | Joint laboratory | Hinge/slider motors and soft limits, Cone, SwingTwist, SixDOF, and a motorized Hermite path |
| `7` | Cylinder cascade | Cylinder creation, rolling, tumbling and mixed contacts |
| `8` | Welded machine | Fixed constraints, retained body ownership, nested child transforms and static-compound motion |
| `9` | Query scanner | Narrow point tests; sorted narrow/broad ray casts; sphere/rotated-box casts; moving narrow/broad overlaps; exact collision-layer filtering |
| `0` | Torque arena | Torque, angular impulse, damping, rotated custom principal inertia and mixed-shape collisions |
| `C` | Character course | User-controlled CharacterVirtual, a 24-member mutually colliding autonomous crowd with query-visible inner bodies and live spatial-hash diagnostics, plus a body-backed Character that jumps and pushes dynamic crates |
| `V` | Vehicle course | Custom six-wheel/three-differential layout plus an autonomous two-wheel MotorcycleController, tuned powertrains and tire curves, cylinder-cast wheel contacts, live telemetry, steering/lean stabilization, per-axle anti-roll bars and height-field terrain |
| `M` | Shape workshop | Tapered capsule/cylinder, triangle, plane and empty shapes; decorators; per-child material colors; live mutable compounds; collision-group pairs |
| `F` | Buoyancy tank | Explicit mass/inertia, gyroscopic creation settings, per-body float/sink ratios, fluid drag and live shape replacement |
| `T` | Tracked vehicle | Eighteen wheels, independent left/right track ratios, pivot turns, sphere-cast contacts, RPM/gear/track-speed HUD and heavy-body obstacle response |
| `L` | Soft-body laboratory | Free cloth over rigid obstacles, fixed curtain and four-corner canopy, three bend/material configurations and live triangle rendering |
| `G` | Advanced soft constraints | Side-by-side no-LRA/Euclidean/geodesic cloth, surface-only/volume-constrained tetrahedra, a face-free Cosserat rod chain, and an animated two-joint skinned ribbon |
| `Y` | Ragdoll pose laboratory | Seven parent-joint kinds, a looping SkeletalAnimation driving motors, an additional distance link, self-collision filtering, falling/motor/kinematic ragdolls, Slider/SixDOF shuttles and a high-detail SkeletonMapper overlay |
| `P` | Scene serialization | Captures a separate World, writes checked binary scene bytes, restores them, instantiates the result, and simulates the restored bodies and constraint |
| `N` | Contact policy laboratory | Compares layer-pair behavior for rigid bodies and cloth, plus a two-child compound where the green body's reject rule is overridden by a conveyor rule only on the left child while gray remains an ordinary sibling control |

Requirements:

- Jolt Physics 5.6.0 headers and a matching static library;
- Nim 2.0 or newer and a C++17 compiler;
- raylib headers and a linkable `raylib` library.

Launch the interactive scene runner:

```sh
examples/run_visual_demos.sh <path-to-jolt> <path-to-libJolt.a>
```

The controls are `1`–`9`, `0`, `C`, `V`, `M`, `F`, `T`, `L`, `G`, `Y`, `P`, and `N` for scene selection, `R` to reset,
Space to pause, and Escape or the window close button to quit. In the character
scene use `WASD` and `J` for the blue virtual controller; the red rigid
character runs autonomously. The colored 24-character crowd exercises the
shared virtual-character spatial hash, whose cells and average candidate count
are shown in the HUD. In the vehicle scene use `WASD`, `B` for the brake,
and `H` for the handbrake.
The tracked-vehicle scene also uses `WASD`; pressing `A` or `D` while stopped
commands opposite track directions for a pivot turn, and `B` brakes both tracks.
The contact-policy scene shows its live Body and sub-shape rules in the HUD.
The green and gray boxes share ordinary layers and a two-child compound; only
the green Body has a reject rule overridden by a left-child conveyor rule.

The vehicle scene drives a custom six-wheel vehicle while an orange two-wheel
motorcycle exercises Jolt's lean controller autonomously. Both use cylinder-cast
wheel contacts. The six-wheeler has three independently configured
differentials, anti-roll bars, and per-wheel tire curves. Its chassis-following HUD reports
the chassis speed, engine RPM, current gear, clutch friction, clutch-side wheel
speed, number of grounded wheels, and live front/rear steering angles; the green
wheel-normal lines show Jolt's current contact results. A separate line reports
motorcycle speed, grounded wheels, wheelbase, and lean-controller state.

Contact-added events appear briefly as contact points and normal lines. A custom
physics material colors the marker; contacts without one remain magenta.
Soft-body vertex contacts use cyan markers and are consumed from the separate
worker-thread-safe soft contact queue.
The overlay also reports events from the latest step and cumulative contact
events, demonstrating that worker-thread callbacks are consumed safely by Nim.
In the query scanner, cyan bodies use a separate object layer; cyan wire markers
show the subset selected by an exact-layer ray query, green rotated boxes show
general convex casts at their reported hit transforms, blue markers show
broad-phase ray candidates, and the rotating orange box shows broad-phase
oriented-box candidates. The moving point changes color only on an exact
narrow-phase hit.

The joint laboratory draws the Hermite track in green while its motor drives a
body along the curved path.

The shape workshop renders the debug colors stored in Jolt physics materials.
Its mutable compound assigns different materials to individual children and
keeps the material assignment when a child shape is replaced at runtime.

The same executable supports deterministic screenshot capture. `all` runs all
twenty scenes in one process, which also exercises repeated World teardown and
process-wide Jolt lifecycle reuse:

```sh
examples/run_visual_demos.sh \
  <path-to-jolt> <path-to-libJolt.a> all 240 screenshots
```

The helper assumes the default x86-64 Jolt CMake feature flags documented in
the top-level README. If the supplied Jolt library was built differently,
adjust the compile definitions and CPU flags in the helper to match it.

## Naylib 3D demo

`naylib_demo.nim` uses Naylib rather than the minimal direct raylib declarations.
It renders per-cell material-colored height-field terrain, independently
material-colored static and mutable compound children, decorated,
tapered, triangle, empty, and cylinder shapes through Naylib and `rlgl`, builds
a fixed-constraint cluster and motorized SixDOF rig, drives an RWD vehicle with custom axle geometry and rear
steering, consumes contact events, and visualizes moving convex overlaps. The
mutable compound changes child transforms and geometry while the simulation is
running. Narrow-phase overlap markers use the material resolved from each hit's
sub-shape ID.
The vehicle uses a tuned automatic transmission and differential plus
sphere-cast wheel contacts, and the HUD reports its live RPM, gear, clutch, and
clutch-side wheel speed. Fixed LRA cloth, a volume-constrained tetrahedron, and
a face-free Cosserat rod and an animated two-joint skinned ribbon are rendered through Naylib from the same live
per-vertex and constraint APIs used by the standalone soft-body scenes.
The same scene also renders a seven-part ownership-safe ragdoll with hinge and
additional distance constraints, including its
live part transforms and parent links, while it collides with the shared terrain.
Every third cylinder uses a separate object layer; cyan sphere-overlap markers
and green rotated-box overlap markers show exact-layer filtered subsets while
preserving the same collision behavior. Blue markers and the rotating orange
box display broad-phase ray and oriented-box candidates.
The HUD separately counts rigid and per-vertex soft-body contact events.

Naylib compiles its vendored raylib, so no separately linkable system raylib is
needed for this example.

```sh
examples/run_naylib_demo.sh \
  <path-to-jolt> <path-to-libJolt.a> <path-to-naylib>
```

For a deterministic run and image capture, append a frame count and output
path:

```sh
examples/run_naylib_demo.sh \
  <path-to-jolt> <path-to-libJolt.a> <path-to-naylib> \
  240 screenshot.png
```

Naylib is optional and is not a dependency of the core `jolt` modules.

## SDL3 3D projection demo

`sdl3_demo.nim` is an independent renderer integration using the `sdl3_nim`
package and SDL3's accelerated renderer. It perspective-projects live Jolt 3D
positions and quaternions into wireframe boxes and spheres, while an autonomous
`CharacterVirtual` traverses the scene and drains its bounded contact-event
queue. This verifies that the binding is not coupled to raylib or Naylib.

Requirements are SDL3, `sdl3_nim`, and the same Jolt build used by the native
tests. Run interactively and press Escape to exit:

```sh
examples/run_sdl3_demo.sh <path-to-jolt> <path-to-libJolt.a>
```

For a deterministic run with a BMP capture:

```sh
examples/run_sdl3_demo.sh \
  <path-to-jolt> <path-to-libJolt.a> 240 screenshot.bmp
```

SDL3 and `sdl3_nim` are optional demo dependencies and are not required by the
core `jolt` modules.
