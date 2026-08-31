# joltnim

Native Nim bindings for Jolt Physics 5.6.0.

The package has three layers:

- `jolt/raw` provides 5,604 direct low-level procedures. The default CPU
  configuration reaches 318 of Jolt 5.6.0's 367 headers and represents all
  3,853 Clang-inventoried callable owner/name groups. The complete
  `Jolt/Physics` subset contributes 2,659 groups. With DebugRenderer enabled,
  CPU, Vulkan, DirectX 12 and Metal respectively cover 3,981, 4,010, 3,997 and
  3,987 groups, with zero missing groups and zero unresolved overload deficits
  in every configuration. Their dependency-graph union reaches 337/367
  headers; the remaining 30 are shader-source or macro include contexts rather
  than independently callable library headers;
- `jolt/bridge` contains the focused native helpers used by the safe layer;
- `jolt` owns the initial Jolt lifecycle and provides `World`, `Body`, `Shape`,
  `SoftBody`, `Character`, `RigidCharacter`, `Vehicle`, `TrackedVehicle`, and
  `Ragdoll` types with deterministic cleanup.

Jolt itself is not bundled. Build Jolt 5.6.0 separately and compile Nim with
the C++ backend. The preprocessor definitions and CPU instruction options used
for the Nim translation unit must match the Jolt library build.

## Requirements and installation

- Nim 2.0 or newer;
- a C++17 compiler;
- Jolt Physics 5.6.0 headers and a matching static library.

Clone the repository and register the package in development mode:

```sh
git clone https://github.com/puffball1567/jolt-nim.git
cd jolt-nim
nimble develop
```

Applications import the ownership-safe API with `import jolt`, or the direct
bindings with `import jolt/raw`. Jolt remains a separate native dependency;
the exact compiler and linker options are described in [Build](#build).

This is the initial `v0.1.0` release. See [CHANGELOG.md](CHANGELOG.md) for the
release contents and [CONTRIBUTING.md](CONTRIBUTING.md) before submitting a
change.

## Minimal example

```nim
import jolt

let world = newWorld()
defer:
  world.close()

let floor = world.addStaticBody(
  boxShape(vec3(100, 1, 100)),
  vec3(0, -1, 0)
)
let box = world.addDynamicBody(
  boxShape(vec3(0.5, 0.5, 0.5)),
  vec3(0, 2, 0)
)

for _ in 0 ..< 120:
  discard world.step(1.0'f32 / 60.0'f32)

echo box.position.y
```

Keep each returned `Body` handle alive for as long as its native body is
needed. Dropping or closing the handle removes and destroys that body; closing
the world invalidates every remaining handle.

## Body configuration and runtime changes

`BodyConfig` exposes Jolt's creation-time rigid-body settings without making
the common constructors verbose. A zero mass selects shape-derived mass;
positive values override mass while retaining shape-derived inertia geometry:

```nim
var bodyConfig = defaultBodyConfig()
bodyConfig.mass = 25
bodyConfig.inertiaMultiplier = 1.5
bodyConfig.allowedDOFs = plane2DAllowedDOFs()
bodyConfig.motionQuality = MotionQuality.LinearCast
bodyConfig.linearVelocity = vec3(4, 0, 0)
bodyConfig.allowSleeping = false
bodyConfig.numVelocityStepsOverride = 8

let actor = world.addDynamicBody(
  boxShape(vec3(0.5, 0.5, 0.5)),
  vec3(0, 3, 0),
  config = bodyConfig)
```

Initial friction, restitution, damping, velocity caps, gravity factor, user
data, manifold reduction, gyroscopic force, internal-edge handling, and solver
step overrides are configurable too. Mass, these behavior flags, and solver
overrides can also be changed and inspected at runtime.

For applications that already know the complete inertia tensor, principal
moments and their body-local orientation can replace shape-derived inertia.
This represents any positive-definite symmetric tensor without exposing a
layout-sensitive matrix type:

```nim
import std/[math, options]

var bodyConfig = defaultBodyConfig()
bodyConfig.massProperties = some(bodyMassProperties(
  mass = 20,
  inertiaDiagonal = vec3(1, 12, 18),
  inertiaRotation = quatFromAxisAngle(vec3(0, 0, 1), PI.float32 / 4)))
let rotor = world.addDynamicBody(shape, position, config = bodyConfig)

rotor.setMassProperties(bodyMassProperties(25, vec3(2, 14, 20)))
echo rotor.massProperties
```

Custom properties are exclusive with `BodyConfig.mass` and
`inertiaMultiplier`. Runtime reads return Jolt's canonical principal-axis
decomposition, whose equivalent axes may be reordered.

An unconstrained body can replace its live shape with `setShape`, optionally
preserving its current mass properties. `applyBuoyancyImpulse` calculates the
submerged volume against a fluid plane and applies Jolt's buoyancy and drag
impulse to dynamic bodies.

`snapshot` captures transform, velocity, activation, sensor, material-response
and motion-property state as one detached value. For several bodies,
`bodySnapshots` uses one native multi-body read lock and preserves input order,
so related transforms come from the same locked observation:

```nim
import std/options

let states = world.bodySnapshots([player, platform, crate])
for state in states:
  echo state.bodyId, " at ", state.position
  if state.motion.isSome:
    echo state.motion.get.allowedDOFs
```

Static bodies have no `motion` value. Dynamic mass and the complete principal
inertia decomposition are optional as well, because Jolt represents locked
translation or rotation axes using zero inverse mass or inertia.

Related live state can also be changed without exposing a locked native
pointer. `velocities`, `setVelocities` and `addVelocities` operate on the
linear/angular pair together. `setTransformAndVelocity` replaces the complete
motion state in one native operation, while `setTransformWhenChanged` avoids
waking a body or updating the broad phase when its transform is unchanged.

Large populations can be inserted through Jolt's native batch broad-phase
path. A batch may mix shapes, motion types, layers, sensors and body settings;
validation, shape cooking and native allocation are atomic from the caller's
perspective. `closeBodies` performs the matching checked batch removal:

```nim
var specs: seq[BodySpec]
for z in 0 ..< 20:
  for x in 0 ..< 20:
    specs.add(dynamicBodySpec(
      boxShape(vec3(0.45, 0.45, 0.45)),
      vec3(x, 2 + z, 0)))
let bodies = world.addBodies(specs)
# ... simulate ...
closeBodies(bodies)
```

## Simulation tuning

`SimulationSettings` covers Jolt's complete solver, contact-cache, sleeping,
determinism and debug-toggle setting set. Values are validated before the
native world is changed, and reads return detached snapshots:

```nim
var settings = defaultSimulationSettings()
settings.numVelocitySteps = 12
settings.numPositionSteps = 4
settings.deterministicSimulation = true

let tunedWorld = newWorld(defaultWorldConfig(), settings)
var live = tunedWorld.simulationSettings
live.timeBeforeSleep = 1.0
tunedWorld.setSimulationSettings(live)
```

## Constraints and events

Point, distance, hinge, slider, fixed, cone, swing-twist, SixDOF, gear, pulley,
rack-and-pinion, and Hermite-path constraints retain both body handles until
the constraint is closed. Gear and rack-and-pinion constraints also retain their optional
companion hinge/slider constraints. Hinge and slider constraints expose live
state, hard/soft limits, friction, and velocity or position motors:

```nim
let hinge = addHingeConstraint(
  anchor, door, vec3(0, 2, 0), vec3(0, 1, 0), -1.2, 1.2)
hinge.setFriction(0.15)
echo hinge.currentAngle
```

Motor and spring settings are shared across powered constraint types:

```nim
var motor = defaultMotorSettings()
motor.minTorque = -500
motor.maxTorque = 500
hinge.setMotor(MotorState.Position, 0, 0.75, motor)
hinge.setLimitSpring(springSettings(4, 0.8))
```

SixDOF axes can independently be free, fixed, or limited and can be updated or
motorized at runtime. Swing limits can use Jolt's cone or pyramid solver:

```nim
var six = defaultSixDOFConfig()
six.swingType = SixDOFSwingType.SwingCone
six.limits[SixDOFAxis.TranslationX] = limitedAxis(-2, 2)
six.limits[SixDOFAxis.RotationY] = limitedAxis(-0.5, 0.5)
six.limits[SixDOFAxis.RotationZ] = limitedAxis(-0.4, 0.4)
let joint = addSixDOFConstraint(anchor, body, pivot, six)
joint.configureAxisMotor(SixDOFAxis.RotationY, motor)
joint.setAxisMotorState(SixDOFAxis.RotationY, MotorState.Position)
```

Specialized mechanisms can share the live hinge/slider constraints that keep
their coupled bodies on the intended axes:

```nim
let gear = addGearConstraint(
  gearBody1, gearBody2, axis1, axis2, ratio = 2,
  hinge1 = hinge1, hinge2 = hinge2)
let pulley = addPulleyConstraint(
  load1, load2, load1.position, fixed1, load2.position, fixed2,
  ratio = 1.5, minLength = 10, maxLength = 10)
echo gear.totalLambda, " ", pulley.currentLength
```

Hermite paths validate each position/tangent/normal frame and expose friction,
rotation modes, current fraction, and velocity/position motor control:

```nim
let track = addPathConstraint(anchor, cart, [
  pathPoint(vec3(-4, 3, 0), vec3(4, 0, 0), vec3(0, 1, 0)),
  pathPoint(vec3(0, 5, 0), vec3(4, 0, 0), vec3(0, 1, 0)),
  pathPoint(vec3(4, 3, 0), vec3(4, 0, 0), vec3(0, 1, 0))
])
track.setPathMotor(MotorState.PositionAndVelocity, 1.5, 2)
```

Every constraint also exposes common runtime solver controls: enable/disable,
priority, velocity/position iteration overrides, 64-bit user data and warm-start
reset. `solverImpulse` normalizes Jolt's latest per-kind position, rotation,
limit and motor impulses for telemetry or application-defined break thresholds:

```nim
tether.setPriority(10)
tether.setSolverStepOverrides(8, 4)
let impulse = tether.solverImpulse
if abs(impulse.position.x) / dt > breakForce:
  tether.setEnabled(false)
```

The values are impulses from the most recently solved step; divide by the step
duration to estimate force or torque. Component axes follow the native
constraint kind.

Contact and activation notifications are copied into a bounded native queue.
Consume them from the Nim thread after stepping; no Nim procedure is called
from a Jolt worker thread:

```nim
discard world.step(1.0'f32 / 60.0'f32)
for event in world.drainEvents():
  if event.kind == PhysicsEventKind.ContactAdded:
    echo event.contactPoint, " ", event.material1(world), " / ",
      event.material2(world)
```

`WorldConfig.maxQueuedEvents` sets the queue capacity. When a consumer falls
behind, the oldest event is discarded and `world.droppedEventCount()` reports
the overflow.

Rigid and soft-body contact validation and native contact-setting modification
can be configured per collision-layer pair without calling Nim from a worker
thread. Policies are
validated and copied into the native listener when the world is created:

```nim
import std/options

var config = defaultWorldConfig()
config.contactPolicies = @[
  contactPolicy(
    nonMovingLayer, movingLayer,
    friction = some(1.0'f32),
    restitution = some(0.25'f32),
    linearSurfaceVelocity = vec3(3, 0, 0))]
let conveyorWorld = newWorld(config)
```

An exact pair can be changed at runtime without widening the rule to every
body on the same layers:

```nim
let floor = world.addStaticBody(boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
let crate = world.addDynamicBody(boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 1, 0))
world.setBodyPairContactPolicy(
  floor, crate,
  bodyPairContactPolicy(
    friction = some(1.0'f32),
    linearSurfaceVelocity = vec3(3, 0, 0)))

discard world.removeBodyPairContactPolicy(floor, crate)
```

Exact policies override layer policies, invalidate existing contact caches on
installation/removal, support rigid and soft-body handles, and are removed
automatically when either body is destroyed. Mutate them between simulation
steps; contact workers only read the synchronized native policy table.

Rigid compound/mesh contacts can be narrowed further to one exact pair of
sub-shapes. These rules override both Body-pair and layer-pair rules:

```nim
let platformHit = world.castRay(
  vec3(-2, 5, 0), vec3(0, -1, 0), 10).get
let crateHit = world.castRay(
  vec3(-2, 3, 0), vec3(0, -1, 0), 5).get
world.setSubShapePairContactPolicy(
  platform, platformHit.subShapeId,
  crate, crateHit.subShapeId,
  bodyPairContactPolicy(response = ContactPolicySensor))
```

Jolt may merge coplanar manifolds and retain only one child ID. Disable
manifold reduction on the involved bodies when friction, restitution, sensor,
mass-scale or surface-velocity settings must preserve exact coplanar child
identity. Sub-shape rejection itself is decided before manifold reduction.
Rules are cleared automatically when either Body is destroyed or replaces its
Shape, and when mutable-compound child indices can change.

A policy can reject a layer pair after broad/narrow-phase detection, turn its
contacts into sensors, or scale the two sides' inverse masses. For rigid-rigid
contacts it can additionally override combined friction/restitution and each
side's inverse inertia, or apply relative linear/angular surface velocity.
Jolt's soft-contact listener exposes only the soft inverse-mass scale and the
rigid inverse-mass/inertia scales, so the other fields are intentionally
rigid-only. Directional fields use `layer2 - layer1`; for a same-layer rigid
policy, Jolt's body-ID order defines side 1 and side 2. The soft body is the
matching side for its configured layer in a soft-rigid pair. Ordinary
collision-layer filtering remains preferable when a pair should never reach
collision detection.

## World state rollback

`saveState` captures the dynamic state of the world for deterministic replay,
including gravity, bodies, contact caches, constraints, virtual characters,
and wheeled or tracked vehicle controllers:

```nim
let checkpoint = world.saveState()
defer:
  checkpoint.close()

for _ in 0 ..< 120:
  discard world.step(1.0'f32 / 60.0'f32)

world.restoreState(checkpoint)
```

A checkpoint belongs to one `World`, and its body, constraint, character, and
vehicle topology must remain unchanged while it is used. Restoration preserves
the existing Nim handles and clears queued pre-restore contact/activation
events. Jolt state recording covers simulation state, not configuration:
shapes, materials, friction, restitution, collision layers, motor settings,
and similar configuration must not be changed if deterministic replay is
required. `checkpoint.byteSize` reports the native state payload size.

## Physics scenes

`PhysicsScene` captures the creation settings for rigid bodies, soft bodies,
their cooked shapes, physics materials, collision-group filters, and two-body
constraints. It can be serialized, restored independently, and instantiated as
an owned group in another compatible `World`:

```nim
let captured = sourceWorld.capturePhysicsScene()
defer: captured.close()

let bytes = captured.serialize()
let restored = restorePhysicsScene(bytes)
defer: restored.close()

let instance = restored.instantiate(targetWorld)
defer: instance.close()
echo instance.bodyCount, " bodies and ", instance.constraintCount, " constraints"
```

The instance exposes each restored body ID, motion type, collision layer,
transform, and velocity. Closing it removes only the bodies and constraints it
created; closing its world safely invalidates the instance.

Serialized data uses a binding-level length and checksum envelope so truncated
or accidentally corrupted payloads are rejected before Jolt reads them. The
embedded Jolt binary state is intended for the same Jolt version and compatible
build configuration; it is not a cross-version interchange format or a security
boundary for untrusted input. Runtime solver caches, event queues, virtual
characters, vehicles, and other helper controllers are not scene resources.
Use `saveState` instead when restoring simulation state in the same unchanged
world.

Jolt-authored `PhysicsScene` ObjectStreams can also be loaded from either Text
or Binary data; the format is detected automatically. Scenes loaded this way
retain their `ShapeSettings` and can be written back as inspectable text or
compact binary data:

```nim
let authored = restorePhysicsSceneObjectStream(readFile("scene.tof"))
defer: authored.close()

if authored.objectStreamSerializable:
  let text = authored.serializeObjectStreamText()
  let binary = authored.serializeObjectStream(PhysicsSceneStreamBinary)
```

An ObjectStream-ready scene can also be authored directly from `BodySpec`
values. All 17 built-in `ShapeKind` families are supported, including triangle
meshes, compressed height fields, recursively nested static/mutable compounds
and all three decorators, together with the complete rigid-body configuration.
Named RGBA materials and per-triangle/per-cell material mappings are retained
through ObjectStream round trips:

```nim
let scene = newPhysicsScene([
  staticBodySpec(boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0)),
  dynamicBodySpec(sphereShape(0.5), vec3(0, 4, 0))
])
defer: scene.close()

discard scene.addDistanceConstraint(
  0, 1,
  localPoint1 = vec3(0, 4.5, 0),
  localPoint2 = vec3(0, 0, 0),
  minDistance = 0,
  maxDistance = 0)
writeFile("scene.tof", scene.serializeObjectStreamText())
```

Authored constraints use stable scene-body indices rather than runtime
`BodyId` values. All twelve constraint families are supported: Point,
Distance, Fixed, Hinge, Slider, Cone, SwingTwist, SixDOF, Gear, Pulley,
RackAndPinion and Hermite Path. Most attachment points and axes are expressed
in each body's local center-of-mass space; Pulley uses world-space authoring
points, while Path geometry is relative to the path body's transform. Gear and
RackAndPinion require two dynamic bodies. Jolt does not include their optional
companion hinge/slider references in ObjectStream settings, so use the live
`World` APIs when those drift-correction references are required.

`configureConstraint` applies the common serializable settings—initial enabled
state, deterministic priority, velocity/position solver-step overrides, debug
draw size and 64-bit user data—by authored constraint index. After
instantiation, `PhysicsSceneInstance` exposes the same live state and setters,
plus warm-start reset, without transferring ownership out of the scene group.
Hinge and Slider friction/limit springs, SwingTwist friction, and per-axis
SixDOF friction/translation springs can also be configured before serialization.
SixDOF cone/pyramid swing selection survives ObjectStream restoration, and
scene instances expose checked per-axis limit readback and mutation.
Distance constraints support serializable soft-limit springs and live range/
spring updates. Fixed constraints can use either automatic relative-transform
detection or explicit, validated attachment frames.
Motor tuning is serializable for Hinge, Slider, SwingTwist swing/twist,
per-axis SixDOF and Hermite Path constraints. The instantiated group exposes
the restored native tuning and checked runtime motor state/target controls, so
imported assets can be validated and activated without escaping scene ownership.
Use `fixedWorldBodyIndex` as either endpoint to attach a supported authored
constraint directly to Jolt's fixed world without creating a dummy static body.
Instance diagnostics expose each constraint's kind, optional runtime body IDs
and normalized solver impulse. Gear and RackAndPinion intentionally reject a
fixed-world endpoint because their native solvers require two dynamic bodies.

Scenes captured from a running `World` contain cooked runtime shapes instead
of authoring-time `ShapeSettings`, so `objectStreamSerializable` returns false
for them; use the checked `serialize` format shown above. ObjectStreams use
Jolt RTTI and schema descriptions, but compatibility should still be tested
when changing Jolt versions. Treat imported scene data as trusted assets, not
as a sandboxed input format.

## Soft bodies

`SoftBody` is an ownership-safe world object built from particles and triangle
faces. The binding creates Jolt edge, shear, and optional distance/dihedral bend
constraints from that surface mesh:

```nim
import std/options
import jolt

let mesh = clothSoftBodyMesh(
  columns = 16,
  rows = 12,
  spacing = 0.35,
  fixedVertices = [0, 15])
var config = defaultSoftBodyConfig()
config.material = some(physicsMaterial(
  "fabric", materialColor(40, 170, 220)))
config.facesDoubleSided = true

let cloth = world.addSoftBody(mesh, vec3(0, 8, 0), config = config)
for vertex in cloth.vertices:
  echo vertex.position
```

Zero inverse mass fixes a vertex to the world; cloth helpers automatically
disable center-position updates when fixed particles are present. Runtime APIs
expose world-space vertex positions and velocities, inverse-mass pinning,
pressure, volume, solver iterations, vertex radius, double-sided faces,
activation, transforms, and state rollback. Soft-body construction accepts an
arbitrary validated triangular surface through `SoftBodyMesh`, in addition to
the cloth helper. Edge springs, dihedral bends and long-range attachments can
either be generated from faces or authored explicitly; native constraint-count
inspection verifies the cooked topology. Faces can select independent named
materials with debug colors, and ray/sub-shape queries resolve the exact native
face material. Optional per-vertex attributes independently control edge,
shear, bend and LRA behavior, allowing reinforced seams and unconstrained
regions without splitting a body. Tetrahedral volume constraints,
Euclidean/geodesic generated attachments, face-free Cosserat rod chains,
live rod rotation/angular-velocity state and local bounds are also exposed and
native-tested. `customUpdate` advances one soft body immediately on the calling
thread between world steps, while `settle` repeats that operation for
teleport/attachment settling without advancing the rest of the world. The
binding temporarily removes and safely reinserts the body while preserving its
ID; native tests cover rigid-floor collision and returning to ordinary world
stepping. Animated skin
constraints support up to four normalized joint weights per vertex, bind-pose
transforms, maximum-distance and back-stop limits, hard resets, live joint-pose
updates and runtime constraint controls. User data, sleeping and collision
groups can be configured and changed through the standard body controls. Jolt
may reorder rod constraints for parallel execution; authored rod indices remain
stable in `rodState`, and `rodOptimizationRemap` exposes independent copies of
the exact authored-to-native maps when raw interoperation needs them. Jolt
5.6 still
labels soft bodies as under development, so their upstream caveats also apply
to this binding. Jolt has no separate hair subsystem; hair and rope-like
simulation are modeled with these rod constraints.

## Skeletal animation

`SkeletalAnimation` owns a Jolt skeleton and name-based keyframe tracks. Jolt
performs linear translation interpolation and quaternion SLERP. Sampling can
return either parent-relative local transforms or fully composed model-space
transforms; joints without a track retain their neutral local transform:

```nim
let animation = newSkeletalAnimation(skeleton, [
  skeletalAnimationTrack("root", @[
    skeletalAnimationKeyframe(0, vec3(0, 0, 0)),
    skeletalAnimationKeyframe(2, vec3(2, 0, 0))]),
  skeletalAnimationTrack("arm", @[
    skeletalAnimationKeyframe(0, vec3(0, 1, 0)),
    skeletalAnimationKeyframe(
      2, vec3(0, 1, 0),
      quatFromAxisAngle(vec3(0, 0, 1), PI.float32))])])
defer: animation.close()

let localPose = animation.sampleLocalPose(0.5)
let modelPose = animation.sampleModelPose(0.5)
```

Animations can loop or hold their final keys, support uniform joint-translation
scaling, feed model poses directly into `SkeletonMapper`, and drive a matching
ragdoll through `driveMotors(animation, ...)`. Skeleton names and parent order
are checked before animation-to-ragdoll coupling. `animation.serialize()` uses
Jolt's native binary state inside a versioned, sized and checksummed envelope;
`restoreSkeletalAnimation(skeleton, data)` validates every restored track,
keyframe and joint name before exposing the new owned resource. The binary
payload requires a compatible Jolt build and the matching skeleton.

## Ragdolls

`Ragdoll` owns a validated skeleton, all of its rigid parts, and the
swing-twist, hinge, point, fixed, cone, slider, or SixDOF constraints that connect each child
to its parent. Parents must
precede their children, and Body IDs returned by `partId` remain valid only
while the ragdoll and its world are alive:

```nim
let limb = capsuleShape(0.35, 0.14)
let ragdoll = world.addRagdoll(ragdollConfig(@[
  ragdollPart(
    "root", limb, vec3(0, 4.8, 0), ragdollRootJoint()),
  ragdollPart(
    "child", limb, vec3(0, 4.0, 0),
    ragdollJoint(
      parent = 0,
      position = vec3(0, 4.4, 0),
      twistAxis = vec3(0, 1, 0),
      planeAxis = vec3(0, 0, 1)))
]))

ragdoll.addImpulse(vec3(2, 0, 0))
ragdoll.addPartImpulse(1, vec3(0, 0, 0.5))
discard world.step(1.0'f32 / 60.0'f32)
echo ragdoll.partName(1), " at ", ragdoll.partPosition(1)
```

The high-level API supports dynamic, kinematic, or static parts; per-part
shapes, layers, body settings,
swing-twist/hinge/point/fixed/cone/slider/SixDOF joints,
friction and motor tuning; non-parent distance, point, fixed, hinge, slider,
cone, swing-twist and SixDOF constraints;
automatic stabilization, constraint priorities and parent/child collision
filtering; world and local pose control; aggregate velocity and impulse
operations; root/part transforms; activation; group-ID changes; and world-state
rollback. Dynamic ragdolls can be driven toward local joint rotations with
`driveMotors`, including consecutive poses for position-and-velocity motor
targets; kinematic ragdolls use world-space transforms with
`driveKinematic`. Constraint diagnostics expose each native kind, connected
part indices, and SixDOF cone/pyramid and per-axis limit state without leaking
native constraint ownership. Owned constraints also expose checked enabled,
priority, solver-step, user-data, warm-start and impulse controls. SixDOF
ragdoll joints support live axis limits, friction, translation springs and
per-axis motors with checked velocity/position/orientation targets and native
state readback. Hinge and slider constraints expose scalar motor tuning,
velocity/position targets and state readback; swing-twist constraints expose
independent swing/twist tuning and states with shared angular-velocity and
orientation targets. Optional `RagdollScalarMotorPreset`,
`RagdollSwingTwistMotorPreset`, and `RagdollSixDOFMotorPreset` values apply
validated settings, initial states and targets while `addRagdoll` constructs
additional constraints, so no post-creation setup pass is required.

`SkeletonMapper` maps a low-detail physics skeleton to a higher-detail render
skeleton by joint name. It preserves intermediate animated chains, supports
reverse mapping for motor targets, and can lock target translations to remove
visible ragdoll stretch:

```nim
let mapper = newSkeletonMapper(physicsSkeleton, renderSkeleton)
defer: mapper.close()
mapper.lockAllTranslations()
let renderModelPose = mapper.mappedPose(
  physicsModelPose, renderLocalPose)
```

## Compound and decorated shapes

Shapes can be scaled, locally rotated/translated, or given a shifted center of
mass. Decorators compose and may be used both by bodies and convex queries:

```nim
let offsetBox = rotatedTranslatedShape(
  scaledShape(boxShape(vec3(0.5, 0.5, 0.5)), vec3(2, 1, 1)),
  vec3(1, 0, 0))
let stableBody = world.addDynamicBody(
  offsetCenterOfMassShape(offsetBox, vec3(0, -0.4, 0)),
  vec3(0, 5, 0))
```

Mutable compounds support live child insertion, removal, transform changes,
replacement, and batched transform changes. Each edit takes the body write lock,
clones the native compound, applies the change, and atomically replaces the
body's shape. Queries that already retained the previous shape can therefore
finish safely. The swap also recomputes center of mass and mass properties,
updates broad-phase bounds, invalidates contact caches, and wakes the body:

```nim
let assembly = world.addDynamicBody(
  mutableCompoundShape([
    compoundChild(boxShape(vec3(0.5, 0.5, 0.5)))
  ]),
  vec3(0, 4, 0))
let child = assembly.addMutableChild(
  compoundChild(sphereShape(0.4), vec3(1, 0, 0)))
assembly.setMutableChildTransform(child, vec3(1.5, 0, 0))
assembly.setMutableChildTransforms(0, [
  compoundChildTransform(vec3(-0.5, 0, 0)),
  compoundChildTransform(vec3(1.5, 0, 0))
])
```

Live mutable-compound changes are intentionally rejected while the body owns
constraints because a center-of-mass shift would also require updating every
attached constraint.

## Physics materials

Materials are immutable Nim value descriptors containing a debug name and
color. They can be assigned uniformly to primitives, decorators, compounds,
triangle meshes, or height fields. Mesh triangles and height-field cells can
also select independent materials:

```nim
let grass = physicsMaterial("grass", materialColor(45, 175, 70))
let stone = physicsMaterial("stone", materialColor(115, 125, 135))
let terrain = triangleMeshShape(vertices, triangleIndices)
  .withMaterials([grass, stone], triangleMaterialIndices)
let terrainBody = world.addStaticBody(terrain, vec3(0, 0, 0))
```

Ray hits, sphere/convex cast hits, overlaps, and contact events retain Jolt's
target sub-shape IDs. `hit.material(world)`, `body.materialAt(subShapeId)`, and
`event.material1(world)` / `event.material2(world)` resolve the exact child,
triangle, or terrain-cell material. The native material object remains owned by
the cooked shape, so temporary Nim descriptors do not create a lifetime hazard.

## Spatial queries

The high-level API provides closest-hit and sorted all-hit ray casts, sphere
casts/overlaps, and general convex casts/overlaps for boxes, spheres, capsules,
cylinders, convex hulls, and convex decorated shapes. Shape queries accept
rotation, exact object-layer filters, or reusable multi-layer sets and return
penetration or contact data:

```nim
for hit in world.castRayAll(vec3(-10, 2, 0), vec3(1, 0, 0), 20):
  echo hit.bodyId, " at ", hit.distance

for hit in world.overlapSphere(vec3(0, 2, 0), 1.5):
  echo hit.bodyId, " penetration ", hit.penetrationDepth

for hit in world.castSphereAll(0.5, vec3(-10, 2, 0), vec3(1, 0, 0), 20):
  echo hit.bodyId, " contact ", hit.contactPoint

for hit in world.castShapeAll(
    capsuleShape(0.6, 0.3),
    vec3(-10, 2, 0), vec3(1, 0, 0), 20):
  echo hit.bodyId, " contact ", hit.contactPoint

for hit in world.overlapShape(
    boxShape(vec3(2, 0.5, 1)), vec3(0, 2, 0)):
  echo hit.bodyId, " penetration ", hit.penetrationDepth

for bodyId in world.collidePoint(vec3(0, 2, 0)):
  echo "point inside body ", bodyId

for candidate in world.broadPhaseCastRay(
    vec3(-10, 2, 0), vec3(1, 0, 0), 20):
  echo "AABB candidate at ", candidate.distance
```

Worlds can define their own object layers, broad-phase mapping, and collision
matrix. Bodies, characters, and vehicle wheel queries can select those layers,
and body layers can be changed at runtime:

```nim
import std/options

const enemyLayer = CollisionLayer(2)
var config = defaultWorldConfig()
config.collisionLayers.add collisionLayerConfig(1)
config.collisionPairs.add collisionPair(movingLayer, enemyLayer)
let layeredWorld = newWorld(config)

let enemy = layeredWorld.addDynamicBody(
  sphereShape(0.5), vec3(0, 3, 0), layer = enemyLayer)
let enemyHit = layeredWorld.castRay(
  vec3(0, 10, 0), vec3(0, -1, 0), 20,
  layer = some(enemyLayer))

let gameplayLayers = queryLayerSet([movingLayer, enemyLayer])
for hit in layeredWorld.castRayAll(
    vec3(0, 10, 0), vec3(0, -1, 0), 20, gameplayLayers):
  echo hit.bodyId

let ignoreEnemy = excludeBodies([enemy.id])
discard layeredWorld.castRay(
  vec3(0, 10, 0), vec3(0, -1, 0), 20,
  bodyFilter = ignoreEnemy)

let exactPart = querySubShape(enemyHit.get)
discard layeredWorld.castRay(
  vec3(0, 10, 0), vec3(0, -1, 0), 20,
  subShapeFilter = includeSubShapes([exactPart]))
```

The same `QueryLayerSet` overload is available for every narrow-phase and
broad-phase query. Sets remove duplicate layers, validate against the queried
world, and execute as one native Jolt query through an object-layer-set filter.
Cast collectors remain globally sorted and `maxHits` applies to the complete
set rather than to each layer separately.

`includeBodies` and `excludeBodies` create reusable native body-ID filters for
all narrow-phase ray, sphere, convex-shape, overlap, and point queries, as well
as broad-phase box, sphere, point, oriented-box, ray, and box-cast queries. They
can be combined with either an exact layer or `QueryLayerSet`; stale IDs and IDs
outside the queried world are rejected before entering Jolt.

`includeSubShapes` and `excludeSubShapes` select exact `(BodyId, SubShapeId)`
pairs returned by earlier query or contact results. They cover compound children,
mesh triangles, and height-field cells and compose with layer and body filters.
Filtering happens in the native collector so excluding a nearer child can still
select a farther child of the same body. Sub-shape IDs should be reacquired after
changing a mutable compound's topology.

Bodies can additionally share a reference-counted collision-group table. This
supports per-subgroup collision rules independently of object layers:

```nim
let groupFilter = newCollisionGroupFilter(4)
groupFilter.setCollisionEnabled(0, 1, false)
left.setCollisionGroup(groupFilter.bodyCollisionGroup(7, 0))
right.setCollisionGroup(groupFilter.bodyCollisionGroup(7, 1))
```

Bodies can be created as non-blocking sensors while still producing contact
events, and sensor mode can be changed at runtime with `body.setSensor`.

## Characters and vehicles

`Character` wraps Jolt's virtual character controller. Its position is the
character's foot position, and `move` handles gravity, moving-platform velocity,
stairs, floor adhesion, and jumping:

```nim
var characterConfig = defaultCharacterConfig()
characterConfig.maxNumHits = 128
characterConfig.penetrationRecoverySpeed = 0.75
let player = world.newCharacter(
  capsuleShape(0.6, 0.35), vec3(0, 3, 0), characterConfig)
player.move(vec3(3, 0, 0), 1.0'f32 / 60.0'f32, jump = true)
discard world.step(1.0'f32 / 60.0'f32)
```

Predictive contact distance, contact capacity, hit-normal reduction, and
penetration recovery are configurable. The latter three also have live
getters/setters, and `maxHitsExceeded` diagnoses overly complex collision
geometry.

Virtual characters in the same world collide with each other automatically and
have stable `characterId` values. `contacts` exposes body/character identity,
sub-shape, normals, velocity, distance, motion type, sensor/back-face flags and
user data from Jolt's complete active-contact list. The lower-level
`canWalkStairs`, `walkStairs`, `stickToFloor`, and
`cancelVelocityTowardsSteepSlopes` operations are available when an application
needs to compose movement itself.

The world stores virtual characters in a shared native spatial hash instead of
checking every character pair. `characterBroadPhaseCellSize` in `WorldConfig`
controls its cell size. Teleports, rotation/shape changes, stair movement,
ordinary updates, removal, and world-state restoration update the index
automatically. Cumulative diagnostics make crowd tuning measurable:

```nim
world.resetCharacterBroadPhaseStats()
player.refreshContacts()
let broadPhase = world.characterBroadPhaseStats
echo broadPhase.registeredCharacters
echo broadPhase.candidateCount, " candidates for ",
  broadPhase.queryCount, " queries"
```

Each virtual character also owns a bounded native contact queue. It reports
body and virtual-character added, persisted, removed, and solver events without
calling Nim from inside Jolt. Queue capacity and overflow are observable, and
declarative response settings control whether contacts may push the character,
whether it may impulse rigid bodies, and whether an idle character should stop
sliding on walkable static ground:

```nim
characterConfig.maxQueuedContactEvents = 2048
characterConfig.canReceiveImpulses = false
let player = world.newCharacter(
  capsuleShape(0.6, 0.35), vec3(0, 3, 0), characterConfig)
for event in player.drainContactEvents():
  echo event.kind, " ", event.bodyId, " ", event.characterId
```

An optional kinematic inner body makes a virtual character visible to normal
Jolt queries, contact listeners and fast linear-cast bodies:

```nim
var characterConfig = defaultCharacterConfig()
characterConfig.innerBodyShape = some(capsuleShape(0.5, 0.28))
characterConfig.innerBodyLayer = movingLayer
let visiblePlayer = world.newCharacter(
  capsuleShape(0.6, 0.35), vec3(0, 0, 0), characterConfig)
echo world.castRay(
  vec3(0, 3, 0), vec3(0, -1, 0), 5,
  bodyFilter = includeBodies([visiblePlayer.innerBodyId.get]))
```

The inner body follows position/rotation changes, participates in rollback and
can change shape at runtime. Its ID is removed atomically with the character.
Advanced creation settings also cover back-face mode, collision/constraint
iteration limits, timing tolerance and character user data.

`RigidCharacter` separately wraps Jolt's body-backed `Character`. It is a
dynamic body controlled through velocity, so gravity and solver contacts are
handled by the normal rigid-body update and the character can push dynamic
bodies. `World.step` automatically refreshes its ground state after each Jolt
update:

```nim
var rigidConfig = defaultRigidCharacterConfig()
rigidConfig.supportingHeight = 0.35
rigidConfig.friction = 0.5
let npc = world.newRigidCharacter(
  capsuleShape(0.6, 0.35), vec3(0, 3, 0), rigidConfig)
npc.move(vec3(3, 0, 0), jump = npc.isSupported)
discard world.step(1.0'f32 / 60.0'f32)
echo npc.bodyId, " standing on ", npc.groundBodyId
```

Its shape uses normal rigid-body origin semantics. Position, rotation, center
of mass, velocity, impulse, collision layer, slope/up/support settings and
checked live shape replacement are available. Body-ID filters accept the
character body, and world snapshots preserve its body state while keeping the
same Nim handle alive.

`Vehicle` adds Jolt's wheeled-vehicle constraint to a dynamic box chassis or an
offset-center-of-mass box chassis.
It retains that chassis until closed and exposes the computed transform,
suspension, steering, angular velocity, and detailed ground contact for every
wheel:

```nim
let chassis = world.addDynamicBody(
  boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 1.2, 0))
var vehicleConfig = defaultVehicleConfig()
vehicleConfig.fourWheelDrive = false
vehicleConfig.frontWheelDrive = false # Rear-wheel drive.
vehicleConfig.wheelTrack = 1.55
vehicleConfig.frontAxleOffset = 1.45
vehicleConfig.rearAxleOffset = 1.3
vehicleConfig.rearMaxSteerAngle = 0.08
vehicleConfig.engineMaxTorque = 950
vehicleConfig.engineMinRPM = 900
vehicleConfig.engineMaxRPM = 6800
vehicleConfig.engineTorqueCurve = @[
  vehicleTorquePoint(0, 0.6),
  vehicleTorquePoint(0.55, 1.05),
  vehicleTorquePoint(1, 0.7)
]
vehicleConfig.gearRatios = @[3.1'f32, 2.05, 1.45, 1.05, 0.82]
vehicleConfig.shiftUpRPM = 5400
vehicleConfig.differentialRatio = 3.8
vehicleConfig.differentialLimitedSlipRatio = 1.8
vehicleConfig.wheelCollisionMode = VehicleWheelCollisionMode.SphereCast
vehicleConfig.wheelSphereCastRadius = 0.08
let car = chassis.newVehicle(vehicleConfig)
car.setInput(forward = 1, steering = 0.25)
discard world.step(1.0'f32 / 60.0'f32)
echo car.wheelState(0).hasContact
echo car.wheelState(0).contactSubShapeId
echo car.wheelState(0).material(world).get.name
echo car.powertrainState.engineRPM, " RPM in gear ",
  car.powertrainState.currentGear
```

`wheelTrack` is the full left-to-right wheel-center distance. A zero track or
axle offset keeps the chassis-derived automatic layout; nonzero front/rear axle
offsets are distances along local +Z/-Z. The suspension attachment height is a
ratio of chassis half-height. With `fourWheelDrive = false`, `frontWheelDrive`
selects FWD or RWD; for AWD, `frontTorqueRatio` controls the front/rear split.
Engine torque curves, RPM limits/inertia/damping, automatic or manual gearboxes,
forward/reverse ratios, shift and clutch timing, axle/center limited-slip ratios,
wheel inertia/damping, and longitudinal/lateral tire impulse scales are
configurable. `powertrainState`, `differentialState`, and `wheelState` expose
live RPM, gear/clutch state, native differential settings, tire slip, and
combined friction. A grounded wheel also reports its exact contact sub-shape
and material, contact-point velocity, longitudinal/lateral contact axes, hard
point, and suspension/longitudinal/lateral impulses. Manual gearboxes can use
`setTransmission`; engine RPM can be seeded or clamped with `setEngineRPM`.
Wheel-ground detection can use Jolt's ray, sphere-cast, or cylinder-cast tester.
Ray and sphere modes accept a world-space up vector and maximum slope angle;
sphere radius and cylinder convex-radius fraction are configurable separately.

Leaving `VehicleConfig.wheels` empty preserves the convenient automatic
four-wheel layout. Supplying wheel, differential, and anti-roll-bar sequences
enables arbitrary layouts while retaining the same `Vehicle` API:

```nim
var truckConfig = defaultVehicleConfig()
for axle, z in [2.0'f32, 0.0'f32, -2.0'f32]:
  var left = defaultVehicleWheelConfig(vec3(0.95, -0.3, z))
  var right = defaultVehicleWheelConfig(vec3(-0.95, -0.3, z))
  if axle == 0:
    left.maxSteerAngle = 0.5
    right.maxSteerAngle = 0.5
  truckConfig.wheels.add(left)
  truckConfig.wheels.add(right)
truckConfig.differentials = @[
  vehicleDifferential(0, 1, engineTorqueRatio = 0.25),
  vehicleDifferential(2, 3, engineTorqueRatio = 0.25),
  vehicleDifferential(4, 5, engineTorqueRatio = 0.5)
]
truckConfig.antiRollBars = @[
  vehicleAntiRollBar(0, 1),
  vehicleAntiRollBar(2, 3),
  vehicleAntiRollBar(4, 5)
]
for wheel in truckConfig.wheels.mitems:
  wheel.longitudinalFrictionCurve = @[
    vehicleTireFrictionPoint(0, 0),
    vehicleTireFrictionPoint(0.08, 1.25),
    vehicleTireFrictionPoint(0.3, 0.95)
  ]
  wheel.lateralFrictionCurve = @[
    vehicleTireFrictionPoint(0, 0),
    vehicleTireFrictionPoint(4, 1.15),
    vehicleTireFrictionPoint(22, 0.9)
  ]
let truck = truckChassis.newVehicle(truckConfig)
```

Each custom wheel controls its local frame, suspension force point and spring,
dimensions, inertia, steering, brakes, tire impulse scales, and longitudinal and
lateral friction curves independently. Longitudinal curve X values are slip
ratios; lateral curve X values are slip angles in degrees. Empty curves retain
Jolt's defaults. Nonempty curves require at least two finite, non-negative
points in strictly increasing slip order; values outside the supplied range use
the nearest endpoint.
Differential torque shares must sum to one. All wheel/differential/bar indices
and unit direction vectors are validated before entering Jolt.

Jolt's `MotorcycleController` uses the same ownership-safe `Vehicle` handle,
powertrain controls, wheel/contact telemetry, and world rollback support. The
motorcycle constructor requires exactly two custom wheels; the default uses a
low center of mass, a castor-style front suspension, rear-wheel drive, and
cylinder-cast contacts:

```nim
var bodyConfig = defaultBodyConfig()
bodyConfig.mass = 240
let motorcycleChassis = world.addDynamicBody(
  offsetCenterOfMassShape(
    boxShape(vec3(0.2, 0.3, 0.4)), vec3(0, -0.3, 0)),
  vec3(0, 1.5, 0), config = bodyConfig)
let motorcycle = motorcycleChassis.newMotorcycle()
motorcycle.setInput(forward = 1, steering = 0.15)
discard world.step(1.0'f32 / 60.0'f32)
echo motorcycle.motorcycleControllerState.wheelBase
```

`defaultMotorcycleConfig` exposes the maximum lean angle, lean spring,
damping, integration/decay, smoothing, and lean-controller/steering-limit
switches. `configureMotorcycleLean` changes the runtime controller safely;
the two switches also have individual setters. `isMotorcycle` guards code that
shares ordinary cars and motorcycles through the common `Vehicle` API.
Jolt 5.6 labels this upstream controller as still in development, so production
handling should be tuned and stress-tested for the intended chassis and track.

`TrackedVehicle` wraps Jolt's independent left/right track controller. The
default configuration creates eighteen road wheels; custom configurations can
replace every wheel and control track membership, driven wheels, inertia,
damping, brake torque, differential ratios, engine, gearbox, and collision
tester:

```nim
var bodyConfig = defaultBodyConfig()
bodyConfig.mass = 4_000
let trackedChassis = world.addDynamicBody(
  boxShape(vec3(1.7, 0.5, 3.2)), vec3(0, 1.4, 0),
  config = bodyConfig)
var trackedConfig = defaultTrackedVehicleConfig()
trackedConfig.engineMaxTorque = 2_000
trackedConfig.wheelCollisionMode = VehicleWheelCollisionMode.SphereCast
let tracked = trackedChassis.newTrackedVehicle(trackedConfig)
tracked.setInput(forward = 1, leftRatio = -1, rightRatio = 1)
discard world.step(1.0'f32 / 60.0'f32)
echo tracked.trackState(TrackedVehicleSide.LeftTrack).angularVelocity
echo tracked.powertrainState.engineRPM
```

Every wheel must belong to exactly one track and each driven wheel must belong
to that track. `wheelState` exposes the same transform, suspension, exact
contact sub-shape/material, axes, impulses, and combined-friction diagnostics as
a wheeled vehicle. Opposite ratio signs perform a pivot turn.

## Build

Supply the Jolt include directory, the built library, and its public build
flags. For a default x86-64 Jolt 5.6.0 CMake build, the command is equivalent
to:

```sh
nim cpp -r --path:src \
  --passC:-I<path-to-jolt> \
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
  --passL:<path-to-libJolt.a> \
  --passL:-pthread \
  --passL:-flto \
  examples/falling_box.nim
```

If Jolt was built with different options, use the definitions and compiler
options exported by that build instead. The high-level world constructor checks
Jolt's configuration/version ID before initialization and rejects mismatches.

When Jolt is built with `JPH_DEBUG_RENDERER`, add that same definition to the
Nim C++ translation unit. `jolt/raw` then exposes the complete renderer,
recorder/playback, body/shape/constraint/soft-body draw surface and callback
adapters. A matching configuration can be checked with:

```sh
tests/run_jolt_debug_renderer_test.sh \
  <path-to-jolt> <path-to-debug-renderer-libJolt.a>
```

The Vulkan Compute configuration is available from `jolt/raw` when Jolt and
the Nim translation unit both use `JPH_USE_VK`. Its complete backend surface,
including buffers, shaders, queues, allocator integration and the standalone
implementation, can be compile-checked against Vulkan-Headers without creating
a Vulkan device:

```sh
tests/check_raw_compute_vk.sh \
  <path-to-jolt> <path-to-vulkan-headers>
```

## Visual demos

The optional raylib demo provides twenty switchable 3D scenes covering stacks,
mixed and convex-hull shapes, constraints, spatial queries, a virtual
character, wheeled vehicles, a tracked vehicle, soft-body cloth, volume/LRA
comparisons, Cosserat rods, dynamic/motor-driven/kinematic ragdolls, and a
checked PhysicsScene save/restore/instantiate round trip.
The contact-policy laboratory compares rigid and cloth rejection/sensor
conversion with rigid fully elastic and conveyor contacts side by side, then
contrasts two ordinary same-layer boxes on a compound where the green Body's
reject rule is overridden by a live conveyor rule for only one child.
With raylib headers and library installed, run:

```sh
examples/run_visual_demos.sh <path-to-jolt> <path-to-libJolt.a>
```

Press `1` through `9`, `0`, `C`, `V`, `M`, `F`, `T`, `L`, `G`, `Y`, `P`, or `N` to change scenes, `R` to reset, and
Space to pause. To
run every scene deterministically for 240 frames and capture screenshots:

```sh
examples/run_visual_demos.sh \
  <path-to-jolt> <path-to-libJolt.a> all 240 screenshots
```

See `examples/README.md` for scene coverage and invocation details.

A separate Naylib example exercises the same Jolt API through Naylib's typed
raylib wrapper. Naylib remains an optional demo dependency and is not required
by the binding:

```sh
examples/run_naylib_demo.sh \
  <path-to-jolt> <path-to-libJolt.a> <path-to-naylib> \
  240 screenshot.png
```

An independent SDL3 example uses `sdl3_nim` and SDL3's accelerated 2D renderer
to draw a perspective wireframe view of live Jolt 3D transforms. It includes a
ramp, a box pyramid, falling spheres, an autonomous virtual character, and
consumes the character contact queue:

```sh
examples/run_sdl3_demo.sh \
  <path-to-jolt> <path-to-libJolt.a> 240 screenshot.bmp
```

## Tests

The native suite contains 293 high-level cases plus raw ABI/configuration checks.
It covers shape validation and contacts, body lifetime and dynamics, all twelve
constraint families, narrow- and broad-phase queries, collision groups, sensors,
triangle-mesh, height-field, decorated shapes and live mutable compounds,
native batch body insertion/removal with allocation rollback,
full custom principal inertia at creation and runtime,
per-sub-shape materials, virtual and body-backed character movement, wheeled and tracked
vehicle behavior,
soft-body cloth/collision/runtime control/rollback plus volume, Euclidean and
geodesic LRA, Cosserat rods and multi-joint skinning with back-stop and rollback
behavior, complete ragdoll construction, collision filtering, pose/motor/
kinematic control, skeletal animation interpolation/looping/model-pose mapping,
animation-driven motors, lifetime and rollback, event queue overflow, checked
PhysicsScene capture, checked binary serialization, Jolt Text/Binary
ObjectStream import/export, restoration, independent
instantiation and teardown including constraints, soft bodies, shared shapes,
sub-shape materials, authored constraint common, passive and motor tuning
settings, explicit fixed frames, live distance limits/springs, live instance
constraint controls, fixed-world endpoints, deterministic scene-body ordering,
solver diagnostics and rejected corrupt data, and
declarative rigid/soft contact rejection, sensor conversion and directional
inverse-mass scaling, plus rigid friction/restitution and conveyor surface
velocity, plus
repeated process-wide Jolt lifecycle use:

```sh
tests/run_jolt_tests.sh <path-to-jolt> <path-to-libJolt.a>
```

## Current scope

The current implementation covers:

- process-wide Jolt initialization shared safely by multiple worlds;
- user-configurable object layers, broad-phase mapping and collision pairs,
  including creation-time/runtime body layers, exact/multi-layer spatial
  queries, native include/exclude body-ID filters, and exact include/exclude
  `(BodyId, SubShapeId)` filters;
- box, sphere, capsule, cylinder, cooked convex-hull, static triangle-mesh, and
  compressed height-field shapes with terrain holes, plus tapered capsule,
  tapered cylinder, triangle, plane, and empty shapes;
- static compound shapes with translated/rotated and nested children, usable by
  static or dynamic bodies when every leaf shape permits it;
- mutable compounds with body-locked live child insertion, removal, transform
  changes and replacement, including mass and broad-phase updates;
- composable scaled, rotated-translated and center-of-mass-offset decorators;
- static, kinematic, and dynamic rigid bodies with quaternion transforms,
  coherent single/multi-body read-lock snapshots, paired velocity operations,
  and atomic complete transform/velocity replacement;
- world gravity, linear/angular velocity, force, impulse, torque, angular
  impulse, damping, activation, friction, restitution, gravity factor, and
  kinematic target control;
- continuous-collision motion quality, velocity caps, center-of-mass and point
  velocity inspection, off-center force/impulse application, user data,
  manifold reduction, sleep-timer reset and contact-cache invalidation;
- ownership-aware point, distance, hinge, slider, fixed, cone, swing-twist, and
  SixDOF constraints, plus gear, pulley, and rack-and-pinion mechanisms with
  retained companion constraints, common enable/priority/solver/user-data
  controls, live pulley lengths, and normalized per-kind solver impulses;
- validated looping/non-looping Hermite-path constraints with selectable
  rotation modes, friction, live path fractions, and velocity/position motors;
- normalized closest-hit and sorted all-hit ray casting with body ID, fraction,
  distance, and hit position;
- closest-hit and sorted all-hit sphere casting with body ID, sphere position,
  target contact point, normal, fraction, and distance;
- sphere overlaps with body ID, penetration depth, contact point, and normal;
- rotated closest/all-hit convex shape casts and overlaps for boxes, spheres,
  capsules, cylinders, tapered capsules/cylinders, convex hulls and convex
  decorators, with exact-layer filtering;
- exact narrow-phase point queries and broad-phase box, sphere, point, oriented
  box, ray, and box-cast queries, including world broad-phase bounds;
- reference-counted collision-group tables with runtime subgroup rules and
  per-body group assignment/removal;
- non-blocking sensor bodies with creation-time and runtime sensor control;
- bounded, thread-safe contact and body-activation event collection through
  `pollEvent` / `drainEvents`, with overflow accounting and no Nim callback on
  a Jolt worker thread, plus immutable per-layer rigid-contact policies for
  native validation, sensor conversion, material response, inverse mass/inertia
  scaling and relative surface velocities;
- worker-thread-safe soft-body contact queues with soft/other body IDs,
  colliding vertex, world-space contact point/normal and sensor-overlap events;
- per-shape physics materials with name/debug-color metadata, per-triangle and
  per-height-field-cell indices, and sub-shape resolution from query/contact hits;
- virtual capsule characters with stair/floor handling, jumping, moving-platform
  inheritance, ground/contact metadata, predictive contacts, hit reduction and
  live penetration-recovery/contact-capacity controls, automatic peer collision
  through a configurable shared spatial hash, and pruning diagnostics;
- wheeled vehicles with configurable suspension, front/rear axle placement,
  track and attachment height, front/rear steering and braking, FWD/RWD/AWD
  torque distribution, anti-roll bars, engine torque curves, automatic/manual
  transmissions, forward/reverse gearing, clutch and differential tuning, tire
  impulse scales and per-wheel slip/friction curves, ray/sphere/cylinder wheel
  collision testers, arbitrary
  wheel/differential/anti-roll layouts, and live
  powertrain, wheel-slip, contact-material and contact-impulse diagnostics;
- two-wheel motorcycles with a dedicated default layout, castor suspension,
  lean stabilization/steering limits, live lean-controller state and settings,
  common powertrain/wheel diagnostics, validation and rollback;
- tracked vehicles with validated left/right wheel membership, driven wheels,
  independent track inertia/damping/brakes/differential ratios, configurable
  engine/transmission and live track, powertrain, wheel-contact and friction
  diagnostics;
- soft-body triangular meshes and generated cloth with fixed/dynamic particles,
  generated or explicitly authored edge/LRA/dihedral constraints and
  per-vertex edge/shear/bend/LRA attributes,
  per-face named materials with ray/sub-shape lookup, distance or dihedral
  bending, compliance/material/collision-group tuning, user data, sleeping, live
  vertex state and pinning, pressure/volume/runtime solver controls, rigid-body
  collision, tetrahedral volume preservation, Euclidean/geodesic long-range
  attachments, face-free Cosserat rods with stretch/shear, bend/twist and live
  rotation/angular-velocity diagnostics, plus local bounds and native cooked
  constraint counts,
  four-weight animated skinning with bind poses, maximum-distance/back-stop
  limits, hard resets and deterministic state restoration,
  deterministic ownership and whole-world rollback;
- ownership-safe ragdolls with validated parent-first skeletons, per-part body
  settings, all seven practical parent-joint kinds, powered joint motors,
  additional distance, point, fixed, hinge, slider, cone, swing-twist and
  SixDOF links,
  low-detail/high-detail skeleton mapping with chain interpolation, reverse
  mapping and translation locking, plus owned skeletal-animation resources with
  keyframe interpolation, looping, local/model sampling and ragdoll motor drive,
  parent/child collision filtering,
  stabilization and priorities, pose inspection/teleportation, dynamic motor
  drive, kinematic drive, aggregate impulses and rollback;
- ownership-safe whole-world dynamic state checkpoints covering bodies,
  contacts, constraints, virtual characters and all vehicle controllers, with
  deterministic repeated restoration and unchanged-topology validation;
- ownership-safe PhysicsScene capture, checked binary serialization and
  restoration, Jolt Text/Binary ObjectStream import/export for authored
  ShapeSettings scenes, Nim-side authoring of all 17 built-in shape families
  with complete body configuration and recursive/per-sub-shape materials,
  plus all twelve authored constraint families, serializable passive and motor
  tuning for powered constraints, and independent group
  instantiation/removal for rigid and soft bodies, cooked shapes, materials,
  collision-group filters and two-body constraints;
- stepping, explicit cleanup, and cleanup on destruction;
- high-level validation before invalid shape, transform, material, or world
  settings reach Jolt;
- capacity error reporting from `PhysicsSystem::Update`.

High-level debug-renderer integration, additional ownership-safe wrappers for
direct user callbacks and property-based body/shape query filters,
broad cross-version ObjectStream compatibility guarantees,
arbitrary soft-body contact callbacks, soft-soft collision, a dedicated hair
abstraction, double
precision, and shared-library builds remain to be added.

## License

The Nim binding is distributed under the MIT License. Jolt Physics is also MIT
licensed and remains a separate dependency; see [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
