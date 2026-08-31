import std/[math, options, os, strformat, strutils]
import jolt
import raylib_minimal

type
  VisualBody = object
    body: Body
    tint: RlColor

  ContactMarker = object
    position: Vec3
    normal: Vec3
    tint: RlColor
    remaining: float32

  BuoyantBody = object
    body: Body
    buoyancy: float32

  SceneVisual = object
    shape: Shape
    tint: RlColor

  DemoState = object
    world: World
    bodies: seq[VisualBody]
    constraints: seq[Constraint]
    contactMarkers: seq[ContactMarker]
    movingPlatform: Body
    character: Character
    characterInput: Vec3
    characterJump: bool
    virtualNpcs: seq[Character]
    rigidCharacter: RigidCharacter
    rigidCharacterDirection: float32
    vehicle: Vehicle
    motorcycle: Vehicle
    vehicleForward: float32
    vehicleSteering: float32
    vehicleBrake: float32
    vehicleHandBrake: float32
    trackedVehicle: TrackedVehicle
    softBodies: seq[SoftBody]
    ragdolls: seq[Ragdoll]
    motorRagdoll: Ragdoll
    kinematicRagdoll: Ragdoll
    motorPose: seq[RagdollTransform]
    kinematicPose: seq[RagdollTransform]
    ragdollAnimation: SkeletalAnimation
    skeletonMapper: SkeletonMapper
    skeletonTargetParents: seq[int]
    skeletonTargetLocalPose: seq[SkeletonTransform]
    mappedSkeletonPose: seq[SkeletonTransform]
    trackedForward: float32
    trackedLeftRatio: float32
    trackedRightRatio: float32
    trackedBrake: float32
    mutableBody: Body
    mutablePhase: int
    buoyantBodies: seq[BuoyantBody]
    shapeSwapBody: Body
    shapeSwapPhase: int
    physicsScene: PhysicsScene
    sceneInstance: PhysicsSceneInstance
    sceneVisuals: seq[SceneVisual]
    serializedSceneBytes: int
    scene: int
    simulationTime: float32
    eventsLastStep: int
    totalContactEvents: int

const
  fixedDeltaTime = 1.0'f32 / 60.0'f32
  scannerLayer = CollisionLayer(2)
  contactRejectLayer = CollisionLayer(2)
  contactSensorLayer = CollisionLayer(3)
  contactBounceLayer = CollisionLayer(4)
  contactConveyorLayer = CollisionLayer(5)
  sceneNames = [
    "box tower",
    "sphere rain",
    "domino wave",
    "mixed playground",
    "constraint bridge",
    "joint laboratory",
    "cylinder cascade",
    "welded machine",
    "query scanner",
    "torque arena",
    "character course",
    "vehicle course",
    "shape workshop",
    "buoyancy tank",
    "tracked vehicle",
    "soft body laboratory",
    "advanced soft constraints",
    "ragdoll pose laboratory",
    "scene serialization",
    "contact policy laboratory"
  ]
  background = RlColor(r: 238, g: 242, b: 247, a: 255)
  outline = RlColor(r: 35, g: 45, b: 55, a: 255)
  floorColor = RlColor(r: 125, g: 139, b: 145, a: 255)
  palette = [
    RlColor(r: 231, g: 76, b: 60, a: 255),
    RlColor(r: 52, g: 152, b: 219, a: 255),
    RlColor(r: 46, g: 204, b: 113, a: 255),
    RlColor(r: 241, g: 196, b: 15, a: 255),
    RlColor(r: 155, g: 89, b: 182, a: 255),
    RlColor(r: 230, g: 126, b: 34, a: 255)
  ]

proc addVisual(state: var DemoState; body: Body; tint: RlColor;
               friction = 0.5'f32; restitution = 0.0'f32): Body =
  body.setFriction(friction)
  body.setRestitution(restitution)
  state.bodies.add(VisualBody(body: body, tint: tint))
  body

proc addFloor(state: var DemoState; halfSize = 14.0'f32) =
  discard state.addVisual(
    state.world.addStaticBody(
      boxShape(vec3(halfSize, 0.5, halfSize)),
      vec3(0, -0.5, 0)
    ),
    floorColor,
    friction = 0.8
  )

proc addArenaWalls(state: var DemoState; halfSize = 7.0'f32) =
  let wallShapeX = boxShape(vec3(0.25, 1.5, halfSize))
  let wallShapeZ = boxShape(vec3(halfSize, 1.5, 0.25))
  for x in [-halfSize, halfSize]:
    discard state.addVisual(
      state.world.addStaticBody(wallShapeX, vec3(x, 1.5, 0)),
      floorColor
    )
  for z in [-halfSize, halfSize]:
    discard state.addVisual(
      state.world.addStaticBody(wallShapeZ, vec3(0, 1.5, z)),
      floorColor
    )

proc raylibColor(color: MaterialColor): RlColor =
  RlColor(r: color.r, g: color.g, b: color.b, a: color.a)

proc buildTower(state: var DemoState) =
  state.addFloor()
  let blockShape = boxShape(vec3(0.48, 0.48, 0.48))
  for level in 0 ..< 9:
    let count = 9 - level
    for column in 0 ..< count:
      let x = (float32(column) - float32(count - 1) * 0.5'f32) * 1.02'f32
      let y = 0.5'f32 + float32(level) * 1.01'f32
      let z = if level mod 2 == 0: 0.15'f32 else: -0.15'f32
      discard state.addVisual(
        state.world.addDynamicBody(blockShape, vec3(x, y, z)),
        palette[(level + column) mod palette.len],
        friction = 0.65
      )

proc buildSphereRain(state: var DemoState) =
  state.addFloor(8)
  state.addArenaWalls(8)
  var specs: seq[BodySpec]
  for index in 0 ..< 64:
    let radius = 0.28'f32 + float32(index mod 4) * 0.055'f32
    let row = index div 8
    let column = index mod 8
    let x = (float32(column) - 3.5'f32) * 1.25'f32
    let z = (float32((index * 5) mod 9) - 4.0'f32) * 0.85'f32
    let y = 2.5'f32 + float32(row) * 1.05'f32 + float32(index mod 3) * 0.18'f32
    specs.add(dynamicBodySpec(sphereShape(radius), vec3(x, y, z)))
  let bodies = state.world.addBodies(specs)
  for index, body in bodies:
    discard state.addVisual(
      body,
      palette[index mod palette.len],
      friction = 0.25,
      restitution = 0.72
    )

proc buildDominoWave(state: var DemoState) =
  state.addFloor(12)
  let domino = boxShape(vec3(0.13, 0.9, 0.42), convexRadius = 0.025)
  var first: Body
  for index in 0 ..< 44:
    let t = float32(index)
    let x = -9.0'f32 + t * 0.43'f32
    let z = sin(t * 0.22'f32) * 2.2'f32
    let tangentZ = cos(t * 0.22'f32) * 0.22'f32 * 2.2'f32
    let yaw = arctan2(tangentZ, 1.0'f32)
    let body = state.world.addDynamicBody(
      domino,
      vec3(x, 0.92, z),
      quatFromAxisAngle(vec3(0, 1, 0), -yaw)
    )
    discard state.addVisual(body, palette[index mod palette.len], friction = 0.75)
    if index == 0:
      first = body

  first.setAngularVelocity(vec3(0, 0, -2.8))
  first.addImpulse(vec3(1.2, 0, 0))

proc buildMixedPlayground(state: var DemoState) =
  state.addFloor(14)

  discard state.addVisual(
    state.world.addStaticBody(
      boxShape(vec3(5.0, 0.25, 2.5)),
      vec3(-3.5, 2.0, 0),
      quatFromAxisAngle(vec3(0, 0, 1), -0.22)
    ),
    RlColor(r: 95, g: 106, b: 117, a: 255),
    friction = 0.55
  )

  state.movingPlatform = state.world.addKinematicBody(
    boxShape(vec3(2.0, 0.2, 2.0)),
    vec3(5.0, 1.2, 0)
  )
  discard state.addVisual(
    state.movingPlatform,
    RlColor(r: 26, g: 188, b: 156, a: 255),
    friction = 0.8
  )

  for index in 0 ..< 36:
    let x = -7.0'f32 + float32(index mod 6) * 1.05'f32
    let y = 5.0'f32 + float32(index div 6) * 1.25'f32
    let z = (float32((index * 7) mod 7) - 3.0'f32) * 0.75'f32
    let shape = case index mod 3
      of 0: boxShape(vec3(0.38, 0.38, 0.38))
      of 1: sphereShape(0.42)
      else: capsuleShape(0.42, 0.28)
    let rotation = quatFromAxisAngle(
      vec3(0, 1, 0),
      float32(index mod 8) * 0.24'f32
    )
    discard state.addVisual(
      state.world.addDynamicBody(shape, vec3(x, y, z), rotation),
      palette[index mod palette.len],
      friction = 0.45,
      restitution = if shape.kind == ShapeKind.Sphere: 0.45'f32 else: 0.1'f32
    )

proc buildConstraintBridge(state: var DemoState) =
  state.addFloor(14)
  let anchorShape = boxShape(vec3(0.35, 0.6, 1.4))
  let plankShape = boxShape(vec3(0.34, 0.12, 1.25), convexRadius = 0.025)
  let leftAnchor = state.world.addStaticBody(anchorShape, vec3(-7.0, 4.0, 0))
  let rightAnchor = state.world.addStaticBody(anchorShape, vec3(7.0, 4.0, 0))
  discard state.addVisual(leftAnchor, floorColor, friction = 0.8)
  discard state.addVisual(rightAnchor, floorColor, friction = 0.8)

  var previous = leftAnchor
  for index in 0 ..< 19:
    let x = -6.3'f32 + float32(index) * 0.7'f32
    let plank = state.world.addDynamicBody(plankShape, vec3(x, 4.0, 0))
    discard state.addVisual(
      plank,
      palette[index mod palette.len],
      friction = 0.85
    )
    let jointX = x - 0.35'f32
    state.constraints.add(addPointConstraint(previous, plank, vec3(jointX, 4, 0)))
    previous = plank
  state.constraints.add(addPointConstraint(previous, rightAnchor, vec3(6.65, 4, 0)))

  for index in 0 ..< 10:
    let x = -4.5'f32 + float32(index) * 1.0'f32
    let shape = if index mod 2 == 0: sphereShape(0.42) else: capsuleShape(0.4, 0.3)
    discard state.addVisual(
      state.world.addDynamicBody(shape, vec3(x, 7.0 + float32(index mod 3), 0)),
      palette[(index + 2) mod palette.len],
      friction = 0.6,
      restitution = 0.15
    )

proc buildJointLaboratory(state: var DemoState) =
  state.addFloor(12)

  let post = state.world.addStaticBody(
    boxShape(vec3(0.2, 2.0, 0.2)),
    vec3(-5.35, 2, 0)
  )
  let door = state.world.addDynamicBody(
    boxShape(vec3(1.25, 1.5, 0.14)),
    vec3(-3.75, 2, 0)
  )
  discard state.addVisual(post, floorColor, friction = 0.8)
  discard state.addVisual(door, palette[0], friction = 0.55)
  let doorHinge = addHingeConstraint(
    post,
    door,
    vec3(-5, 2, 0),
    vec3(0, 1, 0),
    -1.2,
    1.2
  )
  doorHinge.setFriction(0.15)
  var angularMotor = defaultMotorSettings()
  angularMotor.minTorque = -600
  angularMotor.maxTorque = 600
  doorHinge.setMotor(
    MotorState.Velocity, 0.8, 0, angularMotor)
  state.constraints.add(doorHinge)
  door.setAngularVelocity(vec3(0, 2.4, 0))

  let pendulumAnchor = state.world.addStaticBody(
    boxShape(vec3(0.35, 0.25, 0.35)),
    vec3(0, 7, 0)
  )
  let pendulum = state.world.addDynamicBody(
    boxShape(vec3(0.28, 1.7, 0.28)),
    vec3(0, 5.05, 0)
  )
  discard state.addVisual(pendulumAnchor, floorColor)
  discard state.addVisual(pendulum, palette[4], friction = 0.5)
  let pendulumHinge = addHingeConstraint(
    pendulumAnchor,
    pendulum,
    vec3(0, 6.75, 0),
    vec3(0, 0, 1),
    -1.35,
    1.35
  )
  state.constraints.add(pendulumHinge)
  pendulum.setAngularVelocity(vec3(0, 0, 1.8))

  let guide = state.world.addStaticBody(
    boxShape(vec3(3.0, 0.1, 0.1)),
    vec3(4, 3, 1.0)
  )
  let carriage = state.world.addDynamicBody(
    boxShape(vec3(0.65, 0.35, 0.55)),
    vec3(4, 3, 0)
  )
  discard state.addVisual(guide, floorColor, friction = 0.7)
  discard state.addVisual(carriage, palette[2], friction = 0.45)
  let slider = addSliderConstraint(
    guide,
    carriage,
    vec3(4, 3, 0),
    vec3(1, 0, 0),
    -2.5,
    2.5
  )
  slider.setFriction(0.2)
  var linearMotor = defaultMotorSettings()
  linearMotor.minForce = -800
  linearMotor.maxForce = 800
  slider.setMotor(
    MotorState.PositionAndVelocity, 0, -1.5, linearMotor)
  state.constraints.add(slider)
  carriage.setLinearVelocity(vec3(2.5, 0, 0))

  let advancedAnchorShape = boxShape(vec3(0.3, 0.3, 0.3))
  let advancedBodyShape = boxShape(vec3(0.28, 1.0, 0.28))

  let coneAnchor = state.world.addStaticBody(
    advancedAnchorShape, vec3(-5, 7, -4))
  let coneBody = state.world.addDynamicBody(
    advancedBodyShape, vec3(-5, 5.5, -4))
  discard state.addVisual(coneAnchor, floorColor)
  discard state.addVisual(coneBody, palette[1])
  let cone = addConeConstraint(
    coneAnchor, coneBody,
    vec3(-5, 6.7, -4), vec3(0, 1, 0), 0.4)
  state.constraints.add(cone)
  coneBody.setAngularVelocity(vec3(2.5, 0, 1.5))

  let swingAnchor = state.world.addStaticBody(
    advancedAnchorShape, vec3(0, 7, -4))
  let swingBody = state.world.addDynamicBody(
    advancedBodyShape, vec3(0, 5.5, -4))
  discard state.addVisual(swingAnchor, floorColor)
  discard state.addVisual(swingBody, palette[3])
  let swingTwist = addSwingTwistConstraint(
    swingAnchor, swingBody,
    vec3(0, 6.7, -4), vec3(0, 1, 0), vec3(1, 0, 0),
    0.45, 0.3, -0.35, 0.35)
  swingTwist.configureTwistMotor(angularMotor)
  swingTwist.setSwingTwistMotorTargets(vec3(0, 0, 0), quatIdentity())
  swingTwist.setTwistMotorState(MotorState.Position)
  state.constraints.add(swingTwist)
  swingBody.setAngularVelocity(vec3(0.5, 2.5, 1.2))

  let sixAnchor = state.world.addStaticBody(
    advancedAnchorShape, vec3(5, 7, -4))
  let sixBody = state.world.addDynamicBody(
    advancedBodyShape, vec3(5, 5.5, -4))
  discard state.addVisual(sixAnchor, floorColor)
  discard state.addVisual(sixBody, palette[5])
  var sixConfig = defaultSixDOFConfig()
  sixConfig.limits[SixDOFAxis.RotationY] = freeAxis()
  let sixDOF = addSixDOFConstraint(
    sixAnchor, sixBody, vec3(5, 6.7, -4), sixConfig)
  sixDOF.configureAxisMotor(SixDOFAxis.RotationY, angularMotor)
  sixDOF.setSixDOFMotorTargets(
    vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0),
    quatFromAxisAngle(vec3(0, 1, 0), 0.8))
  sixDOF.setAxisMotorState(SixDOFAxis.RotationY, MotorState.Position)
  state.constraints.add(sixDOF)

  let pathAnchor = state.world.addStaticBody(emptyShape(), vec3(0, 0, 0))
  let pathCart = state.world.addDynamicBody(sphereShape(0.5), vec3(-7, 3, 4))
  pathCart.setGravityFactor(0)
  discard state.addVisual(
    pathAnchor, RlColor(r: 245, g: 90, b: 120, a: 255))
  discard state.addVisual(pathCart, palette[2], friction = 0.2)
  let path = addPathConstraint(
    pathAnchor,
    pathCart,
    [
      pathPoint(vec3(-7, 3, 4), vec3(7, 0, 0), vec3(0, 1, 0)),
      pathPoint(vec3(0, 6, 4), vec3(7, 0, 0), vec3(0, 1, 0)),
      pathPoint(vec3(7, 3, 4), vec3(7, 0, 0), vec3(0, 1, 0))
    ],
    rotationConstraint = PathRotationConstraintType.PathRotationToPath)
  var pathMotor = defaultMotorSettings()
  pathMotor.minForce = -10_000
  pathMotor.maxForce = 10_000
  path.setPathMotor(MotorState.PositionAndVelocity, 1.5, 2, pathMotor)
  state.constraints.add(path)

  for index in 0 ..< 10:
    let x = -4.2'f32 + float32(index) * 0.9'f32
    let shape = if index mod 2 == 0:
        sphereShape(0.3)
      else:
        boxShape(vec3(0.3, 0.3, 0.3))
    discard state.addVisual(
      state.world.addDynamicBody(shape, vec3(x, 9 + float32(index mod 3), 0)),
      palette[(index + 1) mod palette.len],
      restitution = 0.25
    )

proc buildCylinderCascade(state: var DemoState) =
  state.addFloor(14)
  discard state.addVisual(
    state.world.addStaticBody(
      boxShape(vec3(6.5, 0.25, 2.8)),
      vec3(-1, 3.0, 0),
      quatFromAxisAngle(vec3(0, 0, 1), -0.22)
    ),
    floorColor,
    friction = 0.85
  )
  let roller = cylinderShape(0.42, 0.38, convexRadius = 0.025)
  for index in 0 ..< 42:
    let row = index div 7
    let column = index mod 7
    let body = state.world.addDynamicBody(
      roller,
      vec3(-6.0 + float32(column) * 1.15, 5.2 + float32(row) * 0.95, 0),
      quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)
    )
    discard state.addVisual(
      body,
      palette[index mod palette.len],
      friction = 0.9,
      restitution = 0.08
    )

proc buildWeldedMachine(state: var DemoState) =
  state.addFloor(14)
  let compoundShape = staticCompoundShape([
    compoundChild(boxShape(vec3(0.65, 0.25, 0.25))),
    compoundChild(sphereShape(0.42), vec3(-1.05, 0, 0)),
    compoundChild(sphereShape(0.42), vec3(1.05, 0, 0)),
    compoundChild(
      cylinderShape(0.65, 0.18),
      vec3(0, 0.75, 0),
      quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5))
  ])
  for index in 0 ..< 6:
    let compound = state.world.addDynamicBody(
      compoundShape,
      vec3(-6.5 + float32(index) * 2.6, 4.5 + float32(index mod 2), -3),
      quatFromAxisAngle(vec3(0, 1, 0), float32(index) * 0.4))
    compound.setAngularVelocity(vec3(0.5, 1.2 + float32(index) * 0.1, 0.3))
    discard state.addVisual(
      compound, palette[(index + 1) mod palette.len], friction = 0.75)

  let core = state.world.addDynamicBody(
    boxShape(vec3(0.55, 0.55, 0.55)),
    vec3(0, 8, 0)
  )
  discard state.addVisual(core, palette[1], friction = 0.7)
  let offsets = [
    vec3(1.4, 0, 0), vec3(-1.4, 0, 0),
    vec3(0, 1.4, 0), vec3(0, -1.4, 0),
    vec3(0, 0, 1.4), vec3(0, 0, -1.4)
  ]
  for index, offset in offsets:
    let partShape = if index mod 2 == 0:
        cylinderShape(0.6, 0.32)
      else:
        boxShape(vec3(0.65, 0.28, 0.28))
    let part = state.world.addDynamicBody(
      partShape,
      vec3(offset.x, 8 + offset.y, offset.z),
      if partShape.kind == ShapeKind.Cylinder:
        quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)
      else:
        quatIdentity()
    )
    discard state.addVisual(part, palette[(index + 2) mod palette.len], friction = 0.7)
    state.constraints.add(addFixedConstraint(core, part))
  core.setLinearVelocity(vec3(1.8, 0, 0.8))
  core.addAngularImpulse(vec3(700, 900, 500))

  for index in 0 ..< 18:
    let x = -6.0'f32 + float32(index mod 6) * 2.2'f32
    let z = -4.0'f32 + float32(index div 6) * 3.5'f32
    discard state.addVisual(
      state.world.addDynamicBody(cylinderShape(0.35, 0.45), vec3(x, 1, z)),
      palette[index mod palette.len],
      friction = 0.75
    )

proc buildQueryScanner(state: var DemoState) =
  state.addFloor(14)
  for index in 0 ..< 9:
    let x = -8.0'f32 + float32(index) * 2.0'f32
    let shape = case index mod 4
      of 0: boxShape(vec3(0.55, 0.8, 0.55))
      of 1: sphereShape(0.7)
      of 2: capsuleShape(0.65, 0.4)
      else: cylinderShape(0.7, 0.5)
    let layer = if index mod 2 == 0: scannerLayer else: nonMovingLayer
    discard state.addVisual(
      state.world.addStaticBody(shape, vec3(x, 3, 0), layer = layer),
      if layer == scannerLayer:
        RlColor(r: 40, g: 190, b: 210, a: 255)
      else:
        palette[index mod palette.len],
      friction = 0.6
    )
  for index in 0 ..< 16:
    let x = -7.5'f32 + float32(index mod 8) * 2.0'f32
    let z = if index < 8: -3.0'f32 else: 3.0'f32
    discard state.addVisual(
      state.world.addDynamicBody(sphereShape(0.35), vec3(x, 5, z)),
      palette[(index + 2) mod palette.len],
      restitution = 0.35
    )

proc buildTorqueArena(state: var DemoState) =
  state.addFloor(12)
  state.addArenaWalls(9)
  var fastAxisConfig = defaultBodyConfig()
  fastAxisConfig.gravityFactor = 0
  fastAxisConfig.angularDamping = 0
  fastAxisConfig.massProperties = some(bodyMassProperties(
    20, vec3(1, 16, 16)))
  var rotatedAxisConfig = fastAxisConfig
  rotatedAxisConfig.massProperties = some(bodyMassProperties(
    20,
    vec3(1, 16, 16),
    quatFromAxisAngle(vec3(0, 0, 1), PI.float32 / 2)))
  let comparisonShape = boxShape(vec3(1.5, 0.18, 0.35))
  let fastAxis = state.world.addDynamicBody(
    comparisonShape, vec3(-3.5, 7.5, 0), config = fastAxisConfig)
  let rotatedAxis = state.world.addDynamicBody(
    comparisonShape, vec3(3.5, 7.5, 0), config = rotatedAxisConfig)
  fastAxis.addAngularImpulse(vec3(8, 0, 0))
  rotatedAxis.addAngularImpulse(vec3(8, 0, 0))
  discard state.addVisual(fastAxis, palette[0])
  discard state.addVisual(rotatedAxis, palette[1])
  for index in 0 ..< 36:
    let x = -6.0'f32 + float32(index mod 6) * 2.2'f32
    let z = -5.0'f32 + float32(index div 6) * 2.0'f32
    let shape = if index mod 2 == 0:
        cylinderShape(0.5, 0.42)
      else:
        boxShape(vec3(0.48, 0.48, 0.48))
    let body = state.world.addDynamicBody(shape, vec3(x, 2.0 + float32(index mod 3), z))
    body.setDamping(0.03, 0.02 + float32(index mod 4) * 0.04)
    body.setAngularVelocity(vec3(
      1.0 + float32(index mod 3),
      1.5 + float32(index mod 5),
      0.8 + float32(index mod 4)
    ))
    discard state.addVisual(
      body,
      palette[index mod palette.len],
      friction = 0.45,
      restitution = 0.2
    )

proc buildCharacterCourse(state: var DemoState) =
  state.addFloor(14)
  state.addArenaWalls(10)

  for index in 0 ..< 4:
    let height = 0.12'f32 + float32(index) * 0.1'f32
    discard state.addVisual(
      state.world.addStaticBody(
        boxShape(vec3(0.7, height, 1.5)),
        vec3(-5.0'f32 + float32(index) * 1.35'f32, height, 0)),
      floorColor,
      friction = 0.9)

  let ramp = triangleMeshShape(
    [
      vec3(-2.5, 0, -1.5), vec3(2.5, 1.2, -1.5),
      vec3(2.5, 1.2, 1.5), vec3(-2.5, 0, 1.5)
    ],
    [0'u32, 3, 2, 0, 2, 1])
  discard state.addVisual(
    state.world.addStaticBody(ramp, vec3(2.3, 0, 0)),
    RlColor(r: 85, g: 105, b: 115, a: 255),
    friction = 0.9)
  discard state.addVisual(
    state.world.addStaticBody(
      boxShape(vec3(0.3, 1.4, 2.0)), vec3(7.5, 1.4, 0)),
    floorColor)

  let pyramid = convexHullShape([
    vec3(-0.7, -0.6, -0.7), vec3(0.7, -0.6, -0.7),
    vec3(0.7, -0.6, 0.7), vec3(-0.7, -0.6, 0.7),
    vec3(0, 0.9, 0)
  ])
  for index in 0 ..< 8:
    let x = -2.5'f32 + float32(index mod 4) * 1.6'f32
    let z = if index < 4: -3.5'f32 else: 3.5'f32
    discard state.addVisual(
      state.world.addDynamicBody(
        pyramid, vec3(x, 1.4 + float32(index mod 3) * 0.8, z),
        quatFromAxisAngle(vec3(0, 1, 0), float32(index) * 0.37)),
      palette[index mod palette.len],
      friction = 0.7)

  var characterConfig = defaultCharacterConfig()
  characterConfig.predictiveContactDistance = 0.15
  characterConfig.maxNumHits = 96
  characterConfig.hitReductionCosMaxAngle = 0.98
  characterConfig.penetrationRecoverySpeed = 0.75
  state.character = state.world.newCharacter(
    capsuleShape(0.6, 0.35), vec3(-8.5, 0, 0), characterConfig)
  state.characterInput = vec3(2.6, 0, 0)

  var rigidConfig = defaultRigidCharacterConfig()
  rigidConfig.friction = 0.5
  rigidConfig.supportingHeight = 0.35
  state.rigidCharacter = state.world.newRigidCharacter(
    capsuleShape(0.6, 0.35), vec3(-8.2, 0.95, 5.8), rigidConfig)
  state.rigidCharacterDirection = 1

  var npcConfig = defaultCharacterConfig()
  npcConfig.innerBodyShape = some(capsuleShape(0.5, 0.28))
  npcConfig.innerBodyLayer = movingLayer
  npcConfig.maxCollisionIterations = 8
  for row in 0 ..< 3:
    for column in 0 ..< 8:
      state.virtualNpcs.add(state.world.newCharacter(
        capsuleShape(0.5, 0.3),
        vec3(-7 + float32(column) * 2, 0,
          -7 + float32(row) * 1.5),
        npcConfig))
  for index in 0 ..< 7:
    discard state.addVisual(
      state.world.addDynamicBody(
        boxShape(vec3(0.35, 0.35, 0.35)),
        vec3(-4.5 + float32(index) * 1.25, 0.35, 5.8)),
      palette[(index + 1) mod palette.len],
      friction = 0.65)

proc buildVehicleCourse(state: var DemoState) =
  const terrainSamples = 16
  var heights = newSeq[float32](terrainSamples * terrainSamples)
  for z in 0 ..< terrainSamples:
    for x in 0 ..< terrainSamples:
      heights[z * terrainSamples + x] =
        sin(float32(x) * 0.65'f32) * 0.16'f32 +
        cos(float32(z) * 0.55'f32) * 0.12'f32
  discard state.addVisual(
    state.world.addStaticBody(
      heightFieldShape(
        heights, terrainSamples,
        offset = vec3(-15, 0, -15),
        scale = vec3(2, 1, 2)),
      vec3(0, 0, 0)),
    floorColor,
    friction = 1)
  state.addArenaWalls(14)

  for index in 0 ..< 5:
    let z = -2.0'f32 + float32(index) * 3.0'f32
    discard state.addVisual(
      state.world.addStaticBody(
        boxShape(vec3(1.5, 0.18, 1.0)),
        vec3(if index mod 2 == 0: -3.5'f32 else: 3.5'f32, 0.18, z),
        quatFromAxisAngle(
          vec3(0, 0, 1),
          if index mod 2 == 0: -0.16'f32 else: 0.16'f32)),
      floorColor,
      friction = 1)

  for index in 0 ..< 12:
    let x = if index mod 2 == 0: -2.8'f32 else: 2.8'f32
    let z = -8.0'f32 + float32(index div 2) * 3.2'f32
    discard state.addVisual(
      state.world.addDynamicBody(
        cylinderShape(0.45, 0.25), vec3(x, 0.5, z)),
      palette[index mod palette.len],
      friction = 0.9)

  let chassis = state.world.addDynamicBody(
    boxShape(vec3(1.1, 0.35, 2.8)), vec3(0, 1.25, -10))
  discard state.addVisual(
    chassis, RlColor(r: 40, g: 105, b: 210, a: 255), friction = 0.8)
  var vehicleConfig = defaultVehicleConfig()
  vehicleConfig.engineMaxTorque = 950
  vehicleConfig.engineMinRPM = 900
  vehicleConfig.engineMaxRPM = 6_800
  vehicleConfig.engineTorqueCurve = @[
    vehicleTorquePoint(0, 0.6),
    vehicleTorquePoint(0.55, 1.05),
    vehicleTorquePoint(1, 0.7)
  ]
  vehicleConfig.gearRatios = @[3.1'f32, 2.05, 1.45, 1.05, 0.82]
  vehicleConfig.shiftUpRPM = 5_400
  vehicleConfig.shiftDownRPM = 1_800
  vehicleConfig.differentialRatio = 3.8
  vehicleConfig.differentialLimitedSlipRatio = 1.8
  vehicleConfig.wheelCollisionMode = VehicleWheelCollisionMode.CylinderCast
  vehicleConfig.wheelCylinderConvexRadiusFraction = 0.2
  for axle, z in [2.0'f32, 0.0'f32, -2.0'f32]:
    var left = defaultVehicleWheelConfig(vec3(0.95, -0.3, z))
    var right = defaultVehicleWheelConfig(vec3(-0.95, -0.3, z))
    left.radius = 0.38
    right.radius = 0.38
    left.width = 0.24
    right.width = 0.24
    left.longitudinalFrictionCurve = @[
      vehicleTireFrictionPoint(0, 0),
      vehicleTireFrictionPoint(0.08, 1.25),
      vehicleTireFrictionPoint(0.3, 0.95)
    ]
    right.longitudinalFrictionCurve = left.longitudinalFrictionCurve
    left.lateralFrictionCurve = @[
      vehicleTireFrictionPoint(0, 0),
      vehicleTireFrictionPoint(4, 1.15),
      vehicleTireFrictionPoint(22, 0.9)
    ]
    right.lateralFrictionCurve = left.lateralFrictionCurve
    if axle == 0:
      left.maxSteerAngle = PI.float32 / 6
      right.maxSteerAngle = PI.float32 / 6
      left.maxBrakeTorque = 1_800
      right.maxBrakeTorque = 1_800
    elif axle == 2:
      left.maxSteerAngle = PI.float32 / 48
      right.maxSteerAngle = PI.float32 / 48
      left.maxHandBrakeTorque = 4_500
      right.maxHandBrakeTorque = 4_500
    vehicleConfig.wheels.add(left)
    vehicleConfig.wheels.add(right)
  vehicleConfig.differentials = @[
    vehicleDifferential(0, 1, engineTorqueRatio = 0.25,
      differentialRatio = 3.8, limitedSlipRatio = 1.8),
    vehicleDifferential(2, 3, engineTorqueRatio = 0.25,
      differentialRatio = 3.8, limitedSlipRatio = 1.8),
    vehicleDifferential(4, 5, engineTorqueRatio = 0.5,
      differentialRatio = 3.8, limitedSlipRatio = 1.8)
  ]
  vehicleConfig.antiRollBars = @[
    vehicleAntiRollBar(0, 1, 1_200),
    vehicleAntiRollBar(2, 3, 900),
    vehicleAntiRollBar(4, 5, 1_400)
  ]
  state.vehicle = chassis.newVehicle(vehicleConfig)
  state.vehicleForward = 0.8

  var motorcycleBodyConfig = defaultBodyConfig()
  motorcycleBodyConfig.mass = 240
  let motorcycleChassis = state.world.addDynamicBody(
    offsetCenterOfMassShape(
      boxShape(vec3(0.2, 0.3, 0.4)), vec3(0, -0.3, 0)),
    vec3(6, 1.5, -10), config = motorcycleBodyConfig)
  discard state.addVisual(
    motorcycleChassis, RlColor(r: 225, g: 115, b: 35, a: 255),
    friction = 0.8)
  state.motorcycle = motorcycleChassis.newMotorcycle()

proc buildTrackedVehicleCourse(state: var DemoState) =
  state.addFloor(24)
  state.addArenaWalls(22)
  for index in 0 ..< 8:
    let x = if index mod 2 == 0: -5.0'f32 else: 5.0'f32
    let z = -8.0'f32 + float32(index div 2) * 5.0'f32
    discard state.addVisual(
      state.world.addStaticBody(
        boxShape(vec3(2.2, 0.2, 1.4)),
        vec3(x, 0.35, z),
        quatFromAxisAngle(
          vec3(0, 0, 1),
          if index mod 2 == 0: -0.18'f32 else: 0.18'f32)),
      floorColor,
      friction = 1)
  for index in 0 ..< 16:
    let x = if index mod 2 == 0: -3.7'f32 else: 3.7'f32
    let z = -10.0'f32 + float32(index div 2) * 2.8'f32
    discard state.addVisual(
      state.world.addDynamicBody(
        boxShape(vec3(0.45, 0.45, 0.45)), vec3(x, 0.6, z)),
      palette[index mod palette.len],
      friction = 0.8)

  var chassisConfig = defaultBodyConfig()
  chassisConfig.mass = 4_000
  let chassis = state.world.addDynamicBody(
    boxShape(vec3(1.7, 0.5, 3.2)), vec3(0, 1.4, -13),
    config = chassisConfig)
  discard state.addVisual(
    chassis, RlColor(r: 65, g: 105, b: 75, a: 255), friction = 0.9)
  var config = defaultTrackedVehicleConfig()
  config.engineMaxTorque = 2_000
  config.wheelCollisionMode = VehicleWheelCollisionMode.SphereCast
  config.wheelSphereCastRadius = 0.06
  state.trackedVehicle = chassis.newTrackedVehicle(config)
  state.trackedForward = 0.75
  state.trackedLeftRatio = 1
  state.trackedRightRatio = 1

proc buildSoftBodyLaboratory(state: var DemoState) =
  state.addFloor(16)
  discard state.addVisual(
    state.world.addStaticBody(sphereShape(2.2), vec3(0, 2.1, 0)),
    RlColor(r: 75, g: 90, b: 105, a: 255), friction = 0.7)
  for x in [-7.0'f32, 7.0'f32]:
    discard state.addVisual(
      state.world.addStaticBody(
        boxShape(vec3(1.4, 0.3, 3.2)), vec3(x, 2.0, 0),
        quatFromAxisAngle(vec3(0, 0, 1), if x < 0: -0.35 else: 0.35)),
      floorColor, friction = 0.8)

  var freeConfig = defaultSoftBodyConfig()
  freeConfig.material = some(physicsMaterial(
    "free cloth", materialColor(45, 165, 225)))
  freeConfig.facesDoubleSided = true
  freeConfig.vertexRadius = 0.04
  state.softBodies.add(state.world.addSoftBody(
    clothSoftBodyMesh(14, 14, 0.42), vec3(0, 10, 0),
    rotation = quatFromAxisAngle(vec3(0, 1, 0), 0.35),
    config = freeConfig))

  let curtainColumns = 13
  var fixedTop: seq[int]
  for column in 0 ..< curtainColumns:
    fixedTop.add(column)
  var curtainConfig = defaultSoftBodyConfig()
  curtainConfig.material = some(physicsMaterial(
    "fixed curtain", materialColor(220, 75, 125)))
  curtainConfig.facesDoubleSided = true
  curtainConfig.bendType = SoftBodyBendType.DihedralBend
  state.softBodies.add(state.world.addSoftBody(
    clothSoftBodyMesh(curtainColumns, 12, 0.38, fixedTop),
    vec3(-5.5, 8.5, -5),
    rotation = quatFromAxisAngle(vec3(1, 0, 0), PI.float32 * 0.5),
    config = curtainConfig))

  let canopyColumns = 11
  let canopyRows = 11
  let canopyLast = canopyColumns * canopyRows - 1
  var canopyConfig = defaultSoftBodyConfig()
  canopyConfig.material = some(physicsMaterial(
    "fixed canopy", materialColor(85, 195, 105)))
  canopyConfig.facesDoubleSided = true
  state.softBodies.add(state.world.addSoftBody(
    clothSoftBodyMesh(canopyColumns, canopyRows, 0.48,
      [0, canopyColumns - 1, canopyLast - canopyColumns + 1, canopyLast]),
    vec3(6, 8, 3), config = canopyConfig))

proc tetrahedronSoftBodyMesh(withVolume: bool): SoftBodyMesh =
  result.vertices = @[
    softBodyVertex(vec3(0, 1.35, 0)),
    softBodyVertex(vec3(-1.15, -0.75, -0.85)),
    softBodyVertex(vec3(1.15, -0.75, -0.85)),
    softBodyVertex(vec3(0, -0.75, 1.25))
  ]
  result.faces = @[
    softBodyFace(0, 2, 1),
    softBodyFace(0, 3, 2),
    softBodyFace(0, 1, 3),
    softBodyFace(1, 2, 3)
  ]
  if withVolume:
    result.volumeConstraints.add(softBodyVolumeConstraint(0, 1, 2, 3))

proc skinnedRibbonMesh(columns, rows: int; spacing: float32): SoftBodyMesh =
  result = clothSoftBodyMesh(columns, rows, spacing)
  let minimumZ = -0.5'f32 * spacing * float32(rows - 1)
  let maximumZ = -minimumZ
  result.skinBindPose = @[
    softBodyJointTransform(vec3(0, 0, minimumZ)),
    softBodyJointTransform(vec3(0, 0, maximumZ))]
  for row in 0 ..< rows:
    let fraction = float32(row) / float32(rows - 1)
    for column in 0 ..< columns:
      var weights: seq[SoftBodySkinWeight]
      if fraction < 1.0'f32:
        weights.add(softBodySkinWeight(0, 1.0'f32 - fraction))
      if fraction > 0.0'f32:
        weights.add(softBodySkinWeight(1, fraction))
      result.skinConstraints.add(softBodySkinConstraint(
        column + row * columns, weights,
        maxDistance = 0.45, backStopDistance = 0.08))

proc buildAdvancedSoftConstraints(state: var DemoState) =
  state.addFloor(18)

  const clothColumns = 7
  const clothRows = 12
  var fixedTop: seq[int]
  for column in 0 ..< clothColumns:
    fixedTop.add(column)
  for index, lraType in [
      SoftBodyLRAType.NoLRA,
      SoftBodyLRAType.EuclideanLRA,
      SoftBodyLRAType.GeodesicLRA]:
    var config = defaultSoftBodyConfig()
    config.material = some(physicsMaterial(
      ["no LRA", "Euclidean LRA", "geodesic LRA"][index],
      [materialColor(225, 95, 85), materialColor(55, 155, 225),
       materialColor(65, 190, 110)][index]))
    config.facesDoubleSided = true
    config.edgeCompliance = 0.08
    config.shearCompliance = 0.08
    config.bendCompliance = 0.08
    config.lraType = lraType
    config.lraMaxDistanceMultiplier = 1.05
    config.numIterations = 8
    let cloth = state.world.addSoftBody(
      clothSoftBodyMesh(clothColumns, clothRows, 0.34, fixedTop),
      vec3(-7.5 + float32(index) * 5.0, 10.5, -4),
      rotation = quatFromAxisAngle(vec3(1, 0, 0), PI.float32 * 0.5),
      config = config)
    cloth.setVertexVelocity(
      clothColumns * clothRows - 1,
      vec3(0, -12, 5 + float32(index)))
    state.softBodies.add(cloth)

  for index, withVolume in [false, true]:
    var config = defaultSoftBodyConfig()
    config.material = some(physicsMaterial(
      if withVolume: "volume constrained" else: "surface only",
      if withVolume: materialColor(175, 85, 215) else: materialColor(235, 155, 45)))
    config.facesDoubleSided = true
    config.edgeCompliance = 0.06
    config.shearCompliance = 0.06
    config.bendCompliance = 0.06
    config.numIterations = 10
    let body = state.world.addSoftBody(
      tetrahedronSoftBodyMesh(withVolume),
      vec3(5.5 + float32(index) * 3.5, 8.5, -1), config = config)
    body.setVertexVelocity(0, vec3(4, -10, 2))
    body.setVertexVelocity(1, vec3(-3, 2, -2))
    state.softBodies.add(body)

  var rodPoints: seq[Vec3]
  for index in 0 ..< 20:
    let t = float32(index)
    rodPoints.add(vec3(t * 0.42, 0.35 * sin(t * 0.65),
      0.45 * cos(t * 0.45)))
  var rodConfig = defaultSoftBodyConfig()
  rodConfig.material = some(physicsMaterial(
    "Cosserat rod", materialColor(35, 185, 190)))
  rodConfig.numIterations = 12
  rodConfig.vertexRadius = 0.07
  state.softBodies.add(state.world.addSoftBody(
    rodSoftBodyMesh(
      rodPoints, rodCompliance = 1.0e-6,
      bendTwistCompliance = 2.0e-4),
    vec3(-4, 9, 5), config = rodConfig))

  var skinConfig = defaultSoftBodyConfig()
  skinConfig.material = some(physicsMaterial(
    "two-joint skin", materialColor(245, 205, 55)))
  skinConfig.facesDoubleSided = true
  skinConfig.updatePosition = false
  skinConfig.edgeCompliance = 0.015
  skinConfig.shearCompliance = 0.015
  skinConfig.numIterations = 10
  state.softBodies.add(state.world.addSoftBody(
    skinnedRibbonMesh(8, 12, 0.32), vec3(6, 10, 5),
    rotation = quatFromAxisAngle(vec3(1, 0, 0), PI.float32 * 0.5),
    config = skinConfig))

proc buildShapeWorkshop(state: var DemoState) =
  let ceramic = physicsMaterial(
    "ceramic", materialColor(235, 230, 210))
  let rubber = physicsMaterial(
    "rubber", materialColor(220, 55, 65))
  let steel = physicsMaterial(
    "steel", materialColor(135, 160, 185))
  discard state.addVisual(
    state.world.addStaticBody(
      planeShape(vec3(0, 1, 0), halfExtent = 14).withMaterial(ceramic),
      vec3(0, 0, 0)),
    floorColor,
    friction = 0.8)
  state.addArenaWalls(14)

  for index in 0 ..< 6:
    let shape = if index mod 2 == 0:
        taperedCapsuleShape(
          0.65 + float32(index mod 3) * 0.12,
          0.24 + float32(index mod 2) * 0.08,
          0.5 + float32(index mod 3) * 0.06)
      else:
        taperedCylinderShape(
          0.7 + float32(index mod 3) * 0.1,
          0.3 + float32(index mod 2) * 0.08,
          0.62)
    let body = state.world.addDynamicBody(
      shape, vec3(-6.25 + float32(index) * 2.5, 12, -7))
    body.setAngularVelocity(vec3(0.4, 0.8, 0.25))
    discard state.addVisual(body, palette[index mod palette.len], friction = 0.7)

  discard state.addVisual(
    state.world.addStaticBody(
      triangleShape(
        vec3(-2.5, 0, -2), vec3(-2.5, 0, 2), vec3(2.5, 2.2, -2),
        convexRadius = 0.025).withMaterial(steel),
      vec3(8.5, 0.02, 0)),
    RlColor(r: 70, g: 175, b: 130, a: 255),
    friction = 0.75)

  let emptyAnchor = state.world.addKinematicBody(
    emptyShape(vec3(0, 0.2, 0)), vec3(-10, 7, 7))
  let anchoredPayload = state.world.addDynamicBody(
    sphereShape(0.5), vec3(-10, 4.5, 7))
  discard state.addVisual(
    emptyAnchor, RlColor(r: 235, g: 80, b: 120, a: 255))
  discard state.addVisual(anchoredPayload, palette[1])
  state.constraints.add(addDistanceConstraint(
    emptyAnchor, anchoredPayload, vec3(-10, 7, 7), vec3(-10, 4.5, 7),
    2.5, 2.5))

  let groupFilter = newCollisionGroupFilter(2)
  groupFilter.setCollisionEnabled(0, 1, false)
  for index, z in [-10.0'f32, -12.0'f32]:
    let left = state.world.addDynamicBody(sphereShape(0.48), vec3(-5, 2.2, z))
    let right = state.world.addDynamicBody(sphereShape(0.48), vec3(5, 2.2, z))
    left.setGravityFactor(0)
    right.setGravityFactor(0)
    left.setLinearVelocity(vec3(3, 0, 0))
    right.setLinearVelocity(vec3(-3, 0, 0))
    let leftGroupId = uint32(20 + index)
    let rightGroupId = if index == 0: leftGroupId else: leftGroupId + 1
    left.setCollisionGroup(groupFilter.bodyCollisionGroup(leftGroupId, 0))
    right.setCollisionGroup(groupFilter.bodyCollisionGroup(
      rightGroupId, 1))
    discard state.addVisual(left, palette[0])
    discard state.addVisual(right, palette[1])

  for index in 0 ..< 8:
    let scale = vec3(
      0.7'f32 + float32(index mod 3) * 0.45'f32,
      0.65'f32 + float32((index + 1) mod 3) * 0.35'f32,
      0.7'f32 + float32((index + 2) mod 3) * 0.4'f32)
    let body = state.world.addDynamicBody(
      scaledShape(boxShape(vec3(0.55, 0.55, 0.55)), scale),
      vec3(-7 + float32(index) * 2, 7 + float32(index mod 2), -3),
      quatFromAxisAngle(vec3(0, 1, 0), float32(index) * 0.3))
    discard state.addVisual(body, palette[index mod palette.len], friction = 0.75)

  for index in 0 ..< 5:
    let shape = rotatedTranslatedShape(
      capsuleShape(0.7, 0.28),
      vec3(0.8, 0, 0),
      quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5))
    let body = state.world.addDynamicBody(
      shape, vec3(-4 + float32(index) * 2, 10, 2))
    discard state.addVisual(body, palette[(index + 2) mod palette.len])

  for index in 0 ..< 4:
    let body = state.world.addDynamicBody(
      offsetCenterOfMassShape(
        boxShape(vec3(0.7, 1.25, 0.7)), vec3(0, -0.85, 0)),
      vec3(-4.5 + float32(index) * 3, 4.5, 5),
      quatFromAxisAngle(vec3(0, 0, 1), 0.45))
    discard state.addVisual(body, palette[(index + 4) mod palette.len])

  state.mutableBody = state.world.addDynamicBody(
    mutableCompoundShape([
      compoundChild(boxShape(vec3(0.7, 0.3, 0.3)).withMaterial(steel)),
      compoundChild(
        sphereShape(0.48).withMaterial(rubber), vec3(-1.35, 0, 0)),
      compoundChild(
        sphereShape(0.48).withMaterial(ceramic), vec3(1.35, 0, 0))
    ]),
    vec3(0, 9, -0.5))
  state.mutableBody.setAngularVelocity(vec3(0.4, 1.2, 0.25))
  discard state.addVisual(
    state.mutableBody, RlColor(r: 245, g: 155, b: 35, a: 255), friction = 0.8)

proc buildBuoyancyTank(state: var DemoState) =
  discard state.addVisual(
    state.world.addStaticBody(
      boxShape(vec3(10, 0.4, 8)), vec3(0, -5.4, 0)),
    floorColor,
    friction = 0.8)
  let wallX = boxShape(vec3(0.3, 4.2, 8))
  let wallZ = boxShape(vec3(10, 4.2, 0.3))
  for x in [-10.0'f32, 10.0'f32]:
    discard state.addVisual(
      state.world.addStaticBody(wallX, vec3(x, -1.2, 0)), floorColor)
  for z in [-8.0'f32, 8.0'f32]:
    discard state.addVisual(
      state.world.addStaticBody(wallZ, vec3(0, -1.2, z)), floorColor)

  for index in 0 ..< 15:
    let shape = case index mod 3
      of 0: boxShape(vec3(0.55, 0.55, 0.55))
      of 1: sphereShape(0.62)
      else: capsuleShape(0.55, 0.34)
    var config = defaultBodyConfig()
    config.mass = 3.0'f32 + float32(index mod 5) * 2.0'f32
    config.inertiaMultiplier = 1.0'f32 + float32(index mod 3) * 0.5'f32
    config.applyGyroscopicForce = index mod 2 == 0
    config.linearDamping = 0.01
    config.angularDamping = 0.01
    let body = state.world.addDynamicBody(
      shape,
      vec3(
        -7 + float32(index mod 5) * 3.5,
        4.5 + float32(index div 5) * 1.8,
        -3 + float32(index div 5) * 3),
      quatFromAxisAngle(vec3(0, 1, 0), float32(index) * 0.31),
      config = config)
    body.setAngularVelocity(vec3(
      0.4 + float32(index mod 3) * 0.25,
      0.6 + float32(index mod 4) * 0.2,
      0.25))
    let buoyancy = [0.65'f32, 1.0'f32, 1.45'f32][index mod 3]
    state.buoyantBodies.add(BuoyantBody(body: body, buoyancy: buoyancy))
    discard state.addVisual(
      body,
      if buoyancy < 0.9: palette[0]
      elif buoyancy > 1.1: palette[2]
      else: palette[1],
      friction = 0.4)

  var swapConfig = defaultBodyConfig()
  swapConfig.mass = 8
  swapConfig.motionQuality = MotionQuality.LinearCast
  state.shapeSwapBody = state.world.addDynamicBody(
    sphereShape(0.75), vec3(0, 1.5, 4.5), config = swapConfig)
  state.shapeSwapBody.setAngularVelocity(vec3(0.4, 1.0, 0.25))
  state.buoyantBodies.add(BuoyantBody(
    body: state.shapeSwapBody, buoyancy: 1.3))
  discard state.addVisual(
    state.shapeSwapBody, RlColor(r: 245, g: 155, b: 35, a: 255))

proc buildSerializedScene(state: var DemoState) =
  ## Build in a separate world, cross the binary boundary, then instantiate
  ## only the restored resource in the visible world.
  let source = newWorld()
  defer: source.close()

  var sourceBodies: seq[Body]
  let floorShape = boxShape(vec3(12, 0.5, 9))
  sourceBodies.add(source.addStaticBody(floorShape, vec3(0, -0.5, 0)))
  state.sceneVisuals.add(SceneVisual(shape: floorShape, tint: floorColor))

  for index in 0 ..< 18:
    let shape = case index mod 3
      of 0: boxShape(vec3(0.48, 0.48, 0.48))
      of 1: sphereShape(0.53)
      else: capsuleShape(0.35, 0.33)
    let column = index mod 6
    let row = index div 6
    let body = source.addDynamicBody(
      shape,
      vec3(-5.0'f32 + float32(column) * 2.0'f32,
        1.2'f32 + float32(row) * 1.45'f32,
        -1.4'f32 + float32(row) * 1.4'f32),
      quatFromAxisAngle(vec3(0, 1, 0), float32(index) * 0.17'f32))
    body.setAngularVelocity(vec3(
      0.12'f32 * float32(index mod 2),
      0.18'f32 * float32((index + 1) mod 3), 0.08))
    sourceBodies.add(body)
    state.sceneVisuals.add(SceneVisual(
      shape: shape, tint: palette[index mod palette.len]))

  let link = sourceBodies[^2].addDistanceConstraint(
    sourceBodies[^1], sourceBodies[^2].position,
    sourceBodies[^1].position, 1.8, 2.4)

  let captured = source.capturePhysicsScene()
  defer: captured.close()
  let bytes = captured.serialize()
  state.serializedSceneBytes = bytes.len
  state.physicsScene = restorePhysicsScene(bytes)
  state.sceneInstance = state.physicsScene.instantiate(state.world)
  discard link

proc buildContactPolicyLaboratory(state: var DemoState) =
  let platformShape = boxShape(vec3(1.5, 0.25, 1.5))
  let layers = [
    contactRejectLayer, contactSensorLayer,
    contactBounceLayer, contactConveyorLayer]
  let colors = [palette[0], palette[1], palette[4], palette[2]]
  for index, layer in layers:
    let x = -6.0'f32 + float32(index) * 4.0'f32
    discard state.addVisual(
      state.world.addStaticBody(platformShape, vec3(x, 0, 0)),
      RlColor(r: 105, g: 115, b: 125, a: 255), friction = 1)
    let shape = if index == 2: sphereShape(0.55)
      else: boxShape(vec3(0.5, 0.5, 0.5))
    let body = state.world.addDynamicBody(
      shape, vec3(x, if index == 3: 0.7 else: 3.2, 0), layer = layer)
    discard state.addVisual(body, colors[index], friction = 1)
  for x in [-6.0'f32, -2.0'f32]:
    discard state.addVisual(
      state.world.addStaticBody(
        platformShape, vec3(x, -3.5, 0), layer = movingLayer),
      RlColor(r: 65, g: 75, b: 85, a: 255), friction = 0.8)
  for index, layer in [contactRejectLayer, contactSensorLayer]:
    var clothConfig = defaultSoftBodyConfig()
    clothConfig.facesDoubleSided = true
    clothConfig.vertexRadius = 0.04
    clothConfig.material = some(physicsMaterial(
      if index == 0: "soft reject" else: "soft sensor",
      if index == 0:
        materialColor(225, 70, 65)
      else:
        materialColor(55, 155, 230)))
    state.softBodies.add(state.world.addSoftBody(
      clothSoftBodyMesh(5, 5, 0.35),
      vec3(-6.0'f32 + float32(index) * 4.0'f32, 5.2, 0),
      layer = layer, config = clothConfig))

  # Both boxes and this two-child platform use ordinary layers. The green box
  # has a rejecting body rule, overridden by a conveyor rule only on the left
  # child. The gray box is the unmodified control on the sibling child.
  var exactPlatformConfig = defaultBodyConfig()
  exactPlatformConfig.friction = 1
  let exactPlatformShape = staticCompoundShape([
    compoundChild(boxShape(vec3(1.5, 0.25, 1.25)), vec3(-1.7, 0, 0)),
    compoundChild(boxShape(vec3(1.5, 0.25, 1.25)), vec3(1.7, 0, 0))
  ])
  let exactPlatform = state.world.addStaticBody(
    exactPlatformShape, vec3(0, 0, -4),
    config = exactPlatformConfig)
  exactPlatform.setUseManifoldReduction(false)
  discard state.addVisual(
    exactPlatform, RlColor(r: 90, g: 100, b: 110, a: 255), friction = 1)
  let exactLeftChild = state.world.castRay(
    vec3(-1.4, 3, -4), vec3(0, -1, 0), 6).get
  let exactDriven = state.world.addDynamicBody(
    boxShape(vec3(0.45, 0.45, 0.45)), vec3(-1.4, 0.7, -4))
  exactDriven.setUseManifoldReduction(false)
  discard state.addVisual(
    exactDriven, RlColor(r: 45, g: 205, b: 115, a: 255), friction = 1)
  discard state.addVisual(
    state.world.addDynamicBody(
      boxShape(vec3(0.45, 0.45, 0.45)), vec3(1.4, 0.7, -4)),
    RlColor(r: 185, g: 190, b: 195, a: 255), friction = 1)
  let exactDrivenHit = state.world.castRay(
    vec3(-1.4, 3, -4), vec3(0, -1, 0), 6).get
  state.world.setBodyPairContactPolicy(
    exactPlatform, exactDriven,
    bodyPairContactPolicy(response = ContactPolicyReject))
  state.world.setSubShapePairContactPolicy(
    exactPlatform, exactLeftChild.subShapeId,
    exactDriven, exactDrivenHit.subShapeId,
    bodyPairContactPolicy(
      friction = some(1.0'f32),
      linearSurfaceVelocity = vec3(0.3, 0, 0)))

proc demoRagdollConfig(origin: Vec3; motionType: MotionType;
                       groupId: uint32): RagdollConfig =
  let sideways = quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)
  template p(dx, dy, dz: float32): Vec3 =
    vec3(origin.x + dx, origin.y + dy, origin.z + dz)
  ragdollConfig(@[
    ragdollPart(
      "pelvis", capsuleShape(0.24, 0.28), p(0, 1.25, 0),
      ragdollRootJoint(), rotation = sideways, motionType = motionType),
    ragdollPart(
      "torso", capsuleShape(0.42, 0.3), p(0, 1.9, 0),
      ragdollJoint(
        0, p(0, 1.55, 0), twistAxis = vec3(0, 1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.28,
        planeHalfConeAngle = 0.28, twistMinAngle = -0.2,
        twistMaxAngle = 0.2, maxFrictionTorque = 1.5,
        maxMotorTorque = 500),
      motionType = motionType),
    ragdollPart(
      "head", capsuleShape(0.12, 0.24), p(0, 2.62, 0),
      ragdollJoint(
        1, p(0, 2.35, 0), twistAxis = vec3(0, 1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.4,
        planeHalfConeAngle = 0.4, twistMinAngle = -0.45,
        twistMaxAngle = 0.45, maxFrictionTorque = 0.5,
        maxMotorTorque = 200),
      motionType = motionType),
    ragdollPart(
      "left arm", capsuleShape(0.4, 0.14), p(-0.72, 2.02, 0),
      ragdollHingeJoint(
        1, p(-0.34, 2.05, 0), hingeAxis = vec3(0, 0, 1),
        normalAxis = vec3(1, 0, 0), minAngle = -1.8,
        maxAngle = 1.8, maxMotorTorque = 350),
      rotation = sideways, motionType = motionType),
    ragdollPart(
      "right arm", capsuleShape(0.4, 0.14), p(0.72, 2.02, 0),
      ragdollHingeJoint(
        1, p(0.34, 2.05, 0), hingeAxis = vec3(0, 0, 1),
        normalAxis = vec3(1, 0, 0), minAngle = -1.8,
        maxAngle = 1.8, maxMotorTorque = 350),
      rotation = sideways, motionType = motionType),
    ragdollPart(
      "left leg", capsuleShape(0.52, 0.18), p(-0.22, 0.35, 0),
      ragdollJoint(
        0, p(-0.22, 0.9, 0), twistAxis = vec3(0, -1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.65,
        planeHalfConeAngle = 0.45, twistMinAngle = -0.35,
        twistMaxAngle = 0.35, maxMotorTorque = 600),
      motionType = motionType),
    ragdollPart(
      "right leg", capsuleShape(0.52, 0.18), p(0.22, 0.35, 0),
      ragdollJoint(
        0, p(0.22, 0.9, 0), twistAxis = vec3(0, -1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.65,
        planeHalfConeAngle = 0.45, twistMinAngle = -0.35,
        twistMaxAngle = 0.35, maxMotorTorque = 600),
      motionType = motionType)
  ], groupId = groupId, distanceConstraints = @[
    ragdollDistanceConstraint(
      3, 4, p(-0.72, 2.02, 0), p(0.72, 2.02, 0), 1.2, 1.8)
  ])

proc buildRagdollLaboratory(state: var DemoState) =
  state.addFloor(12)
  let sideways = quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)
  for x in [-4.0'f32, 0.0'f32, 4.0'f32]:
    discard state.addVisual(
      state.world.addStaticBody(
        boxShape(vec3(1.3, 0.18, 1.3)), vec3(x, 1.1, 0),
        quatFromAxisAngle(vec3(0, 0, 1), x * 0.025)),
      floorColor, friction = 0.85)

  let falling = state.world.addRagdoll(
    demoRagdollConfig(vec3(-4, 4.8, 0), MotionType.Dynamic, 101))
  falling.addImpulse(vec3(2.5, 0, 1.2))
  state.ragdolls.add(falling)

  state.motorRagdoll = state.world.addRagdoll(
    demoRagdollConfig(vec3(0, 5.0, 0), MotionType.Dynamic, 102))
  state.ragdolls.add(state.motorRagdoll)
  state.motorPose = @[
    RagdollTransform(position: vec3(0, 0, 0), rotation: sideways),
    RagdollTransform(position: vec3(0, 0.65, 0), rotation: sideways),
    RagdollTransform(position: vec3(0, 0.72, 0), rotation: quatIdentity()),
    RagdollTransform(position: vec3(-0.72, 0.12, 0), rotation: quatIdentity()),
    RagdollTransform(position: vec3(0.72, 0.12, 0), rotation: quatIdentity()),
    RagdollTransform(position: vec3(-0.22, -0.9, 0), rotation: sideways),
    RagdollTransform(position: vec3(0.22, -0.9, 0), rotation: sideways)]
  var animationJoints = newSeq[SkeletonJoint](state.motorPose.len)
  for index, transform in state.motorPose:
    animationJoints[index] = skeletonJoint(
      state.motorRagdoll.partName(index),
      if index == 0: -1 else: state.motorRagdoll.partParent(index).get,
      skeletonTransform(transform.position, transform.rotation))
  let armUp = PI.float32 * 0.5 + 0.75'f32
  let armDown = PI.float32 * 0.5 - 0.75'f32
  state.ragdollAnimation = newSkeletalAnimation(
    skeletonDefinition(animationJoints), [
      skeletalAnimationTrack("pelvis", @[
        skeletalAnimationKeyframe(0, state.motorPose[0].position, sideways),
        skeletalAnimationKeyframe(4, state.motorPose[0].position, sideways)]),
      skeletalAnimationTrack("left arm", @[
        skeletalAnimationKeyframe(0, state.motorPose[3].position,
          quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)),
        skeletalAnimationKeyframe(1, state.motorPose[3].position,
          quatFromAxisAngle(vec3(0, 0, 1), armUp)),
        skeletalAnimationKeyframe(2, state.motorPose[3].position,
          quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)),
        skeletalAnimationKeyframe(3, state.motorPose[3].position,
          quatFromAxisAngle(vec3(0, 0, 1), armDown)),
        skeletalAnimationKeyframe(4, state.motorPose[3].position,
          quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5))]),
      skeletalAnimationTrack("right arm", @[
        skeletalAnimationKeyframe(0, state.motorPose[4].position,
          quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)),
        skeletalAnimationKeyframe(1, state.motorPose[4].position,
          quatFromAxisAngle(vec3(0, 0, 1), armDown)),
        skeletalAnimationKeyframe(2, state.motorPose[4].position,
          quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)),
        skeletalAnimationKeyframe(3, state.motorPose[4].position,
          quatFromAxisAngle(vec3(0, 0, 1), armUp)),
        skeletalAnimationKeyframe(4, state.motorPose[4].position,
          quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5))]),
      skeletalAnimationTrack("left leg", @[
        skeletalAnimationKeyframe(0, state.motorPose[5].position, sideways),
        skeletalAnimationKeyframe(1, state.motorPose[5].position,
          quatFromAxisAngle(vec3(1, 0, 0), 0.28)),
        skeletalAnimationKeyframe(2, state.motorPose[5].position, sideways),
        skeletalAnimationKeyframe(3, state.motorPose[5].position,
          quatFromAxisAngle(vec3(1, 0, 0), -0.28)),
        skeletalAnimationKeyframe(4, state.motorPose[5].position, sideways)]),
      skeletalAnimationTrack("right leg", @[
        skeletalAnimationKeyframe(0, state.motorPose[6].position, sideways),
        skeletalAnimationKeyframe(1, state.motorPose[6].position,
          quatFromAxisAngle(vec3(1, 0, 0), -0.28)),
        skeletalAnimationKeyframe(2, state.motorPose[6].position, sideways),
        skeletalAnimationKeyframe(3, state.motorPose[6].position,
          quatFromAxisAngle(vec3(1, 0, 0), 0.28)),
        skeletalAnimationKeyframe(4, state.motorPose[6].position, sideways)])])
  state.motorPose = state.ragdollAnimation.sampleRagdollLocalPose(
    state.motorRagdoll, 0)

  state.kinematicRagdoll = state.world.addRagdoll(
    demoRagdollConfig(vec3(4, 3.0, 0), MotionType.Kinematic, 103))
  state.ragdolls.add(state.kinematicRagdoll)
  state.kinematicPose = state.kinematicRagdoll.pose

  let slider = state.world.addRagdoll(ragdollConfig(@[
    ragdollPart(
      "slider anchor", boxShape(vec3(0.22, 0.22, 0.22)),
      vec3(-2, 3.2, 3), ragdollRootJoint(),
      motionType = MotionType.Static),
    ragdollPart(
      "slider shuttle", boxShape(vec3(0.42, 0.28, 0.28)),
      vec3(-2, 3.2, 3),
      ragdollSliderJoint(
        0, vec3(-2, 3.2, 3), minPosition = -1.2,
        maxPosition = 1.2, maxFrictionForce = 0.08))
  ], groupId = 104))
  slider.addPartImpulse(1, vec3(12, 2, 0))
  state.ragdolls.add(slider)

  var sixConfig = defaultSixDOFConfig()
  sixConfig.limits[SixDOFAxis.TranslationX] = freeAxis()
  sixConfig.limits[SixDOFAxis.TranslationY] = limitedAxis(-0.45, 0.45)
  sixConfig.limits[SixDOFAxis.RotationZ] = limitedAxis(-0.5, 0.5)
  let sixDOF = state.world.addRagdoll(ragdollConfig(@[
    ragdollPart(
      "SixDOF anchor", boxShape(vec3(0.22, 0.22, 0.22)),
      vec3(2, 3.2, 3), ragdollRootJoint(),
      motionType = MotionType.Static),
    ragdollPart(
      "SixDOF shuttle", boxShape(vec3(0.42, 0.28, 0.28)),
      vec3(2, 3.2, 3),
      ragdollSixDOFJoint(
        0, vec3(2, 3.2, 3), config = sixConfig,
        linearFriction = vec3(0.04, 0, 0)))
  ], groupId = 105))
  sixDOF.addPartImpulse(1, vec3(12, 5, 2))
  state.ragdolls.add(sixDOF)

  let sourcePose = state.motorRagdoll.pose
  var sourceJoints = newSeq[SkeletonJoint](state.motorRagdoll.partCount)
  for index in 0 ..< sourceJoints.len:
    sourceJoints[index] = skeletonJoint(
      state.motorRagdoll.partName(index),
      if index == 0: -1 else: state.motorRagdoll.partParent(index).get,
      skeletonTransform(
        sourcePose[index].position, sourcePose[index].rotation))
  let chestPosition = vec3(
    sourcePose[1].position.x,
    (sourcePose[1].position.y + sourcePose[2].position.y) * 0.5,
    sourcePose[1].position.z)
  let targetJoints = @[
    skeletonJoint("pelvis", -1,
      skeletonTransform(sourcePose[0].position, sourcePose[0].rotation)),
    skeletonJoint("torso", 0,
      skeletonTransform(sourcePose[1].position, sourcePose[1].rotation)),
    skeletonJoint("chest detail", 1,
      skeletonTransform(chestPosition, sourcePose[1].rotation)),
    skeletonJoint("head", 2,
      skeletonTransform(sourcePose[2].position, sourcePose[2].rotation)),
    skeletonJoint("left arm", 1,
      skeletonTransform(sourcePose[3].position, sourcePose[3].rotation)),
    skeletonJoint("right arm", 1,
      skeletonTransform(sourcePose[4].position, sourcePose[4].rotation)),
    skeletonJoint("left leg", 0,
      skeletonTransform(sourcePose[5].position, sourcePose[5].rotation)),
    skeletonJoint("right leg", 0,
      skeletonTransform(sourcePose[6].position, sourcePose[6].rotation))]
  state.skeletonMapper = newSkeletonMapper(
    skeletonDefinition(sourceJoints), skeletonDefinition(targetJoints))
  state.skeletonMapper.lockAllTranslations()
  state.skeletonTargetParents = @[-1, 0, 1, 2, 1, 1, 0, 0]
  state.skeletonTargetLocalPose = @[
    skeletonTransform(sourcePose[0].position),
    skeletonTransform(vec3(0, 0.65, 0)),
    skeletonTransform(vec3(0, 0.36, 0)),
    skeletonTransform(vec3(0, 0.36, 0)),
    skeletonTransform(vec3(-0.72, 0.12, 0)),
    skeletonTransform(vec3(0.72, 0.12, 0)),
    skeletonTransform(vec3(-0.22, -0.9, 0)),
    skeletonTransform(vec3(0.22, -0.9, 0))]

proc reset(state: var DemoState; scene: int) =
  if not state.ragdollAnimation.isNil:
    state.ragdollAnimation.close()
  if not state.skeletonMapper.isNil:
    state.skeletonMapper.close()
  if not state.sceneInstance.isNil:
    state.sceneInstance.close()
  if not state.physicsScene.isNil:
    state.physicsScene.close()
  if not state.world.isNil:
    state.world.close()
  state.constraints.setLen(0)
  state.bodies.setLen(0)
  state.contactMarkers.setLen(0)
  state.movingPlatform = nil
  state.character = nil
  state.characterInput = vec3(0, 0, 0)
  state.characterJump = false
  state.virtualNpcs.setLen(0)
  state.rigidCharacter = nil
  state.rigidCharacterDirection = 1
  state.vehicle = nil
  state.motorcycle = nil
  state.vehicleForward = 0
  state.vehicleSteering = 0
  state.vehicleBrake = 0
  state.vehicleHandBrake = 0
  state.trackedVehicle = nil
  state.softBodies.setLen(0)
  state.ragdolls.setLen(0)
  state.motorRagdoll = nil
  state.kinematicRagdoll = nil
  state.ragdollAnimation = nil
  state.motorPose.setLen(0)
  state.kinematicPose.setLen(0)
  state.skeletonMapper = nil
  state.skeletonTargetParents.setLen(0)
  state.skeletonTargetLocalPose.setLen(0)
  state.mappedSkeletonPose.setLen(0)
  state.trackedForward = 0
  state.trackedLeftRatio = 1
  state.trackedRightRatio = 1
  state.trackedBrake = 0
  state.mutableBody = nil
  state.mutablePhase = -1
  state.buoyantBodies.setLen(0)
  state.shapeSwapBody = nil
  state.shapeSwapPhase = -1
  state.physicsScene = nil
  state.sceneInstance = nil
  state.sceneVisuals.setLen(0)
  state.serializedSceneBytes = 0
  state.scene = scene
  state.simulationTime = 0
  state.eventsLastStep = 0
  state.totalContactEvents = 0

  var config = defaultWorldConfig()
  config.maxBodies = 2_048
  config.maxBodyPairs = 8_192
  config.maxContactConstraints = 4_096
  if scene == 8:
    config.collisionLayers.add collisionLayerConfig(1)
    config.collisionPairs.add collisionPair(movingLayer, scannerLayer)
  elif scene == 19:
    for layer in [
        contactRejectLayer, contactSensorLayer,
        contactBounceLayer, contactConveyorLayer]:
      config.collisionLayers.add collisionLayerConfig(1)
      config.collisionPairs.add collisionPair(nonMovingLayer, layer)
    config.collisionPairs.add collisionPair(movingLayer, contactRejectLayer)
    config.collisionPairs.add collisionPair(movingLayer, contactSensorLayer)
    config.contactPolicies = @[
      contactPolicy(
        nonMovingLayer, contactRejectLayer,
        response = ContactPolicyReject),
      contactPolicy(
        nonMovingLayer, contactSensorLayer,
        response = ContactPolicySensor),
      contactPolicy(
        nonMovingLayer, contactBounceLayer,
        friction = some(0.0'f32), restitution = some(1.0'f32)),
      contactPolicy(
        nonMovingLayer, contactConveyorLayer,
        friction = some(1.0'f32),
        linearSurfaceVelocity = vec3(3, 0, 0))]
  state.world = newWorld(config)

  case scene
  of 0: state.buildTower()
  of 1: state.buildSphereRain()
  of 2: state.buildDominoWave()
  of 3: state.buildMixedPlayground()
  of 4: state.buildConstraintBridge()
  of 5: state.buildJointLaboratory()
  of 6: state.buildCylinderCascade()
  of 7: state.buildWeldedMachine()
  of 8: state.buildQueryScanner()
  of 9: state.buildTorqueArena()
  of 10: state.buildCharacterCourse()
  of 11: state.buildVehicleCourse()
  of 12: state.buildShapeWorkshop()
  of 13: state.buildBuoyancyTank()
  of 14: state.buildTrackedVehicleCourse()
  of 15: state.buildSoftBodyLaboratory()
  of 16: state.buildAdvancedSoftConstraints()
  of 17: state.buildRagdollLaboratory()
  of 18: state.buildSerializedScene()
  of 19: state.buildContactPolicyLaboratory()
  else: raise newException(ValueError, "unknown demo scene")
  state.world.optimizeBroadPhase()

proc step(state: var DemoState) =
  state.simulationTime += fixedDeltaTime
  if state.scene == 13:
    let swapPhase = int(state.simulationTime * 0.5'f32) mod 3
    if not state.shapeSwapBody.isNil and state.shapeSwapBody.isAlive and
        swapPhase != state.shapeSwapPhase:
      let replacement = case swapPhase
        of 0: sphereShape(0.75)
        of 1: boxShape(vec3(0.75, 0.55, 0.75))
        else: capsuleShape(0.65, 0.42)
      state.shapeSwapBody.setShape(
        replacement, updateMassProperties = false)
      state.shapeSwapPhase = swapPhase
    let gravity = state.world.gravity
    for item in state.buoyantBodies:
      if item.body.isAlive:
        discard item.body.applyBuoyancyImpulse(
          vec3(0, 3, 0), vec3(0, 1, 0), item.buoyancy,
          0.45, 0.08, vec3(0.35, 0, 0), gravity, fixedDeltaTime)
  if not state.mutableBody.isNil and state.mutableBody.isAlive:
    let phase = int(state.simulationTime * 1.25'f32) mod 4
    if phase != state.mutablePhase:
      let spread = if phase mod 2 == 0: 1.35'f32 else: 2.15'f32
      let height = if phase < 2: 0.0'f32 else: 0.85'f32
      state.mutableBody.setMutableChildTransform(1, vec3(-spread, height, 0))
      state.mutableBody.replaceMutableChild(
        2,
        compoundChild(
          (if phase mod 2 == 0:
              sphereShape(0.48)
            else:
              boxShape(vec3(0.5, 0.5, 0.5))).withMaterial(
            physicsMaterial("ceramic", materialColor(235, 230, 210))),
          vec3(spread, -height, 0)))
      state.mutablePhase = phase
  if not state.character.isNil and state.character.isAlive:
    state.character.move(
      state.characterInput,
      fixedDeltaTime,
      jump = state.characterJump)
    state.characterJump = false
  if not state.rigidCharacter.isNil and state.rigidCharacter.isAlive:
    if state.rigidCharacter.position.x > 8:
      state.rigidCharacterDirection = -1
    elif state.rigidCharacter.position.x < -8:
      state.rigidCharacterDirection = 1
    let autoJump = state.rigidCharacter.isSupported and
      int(state.simulationTime * 2.0'f32) mod 12 == 0
    state.rigidCharacter.move(
      vec3(3.0'f32 * state.rigidCharacterDirection, 0, 0),
      jump = autoJump,
      jumpSpeed = 5.5)
  let crowdDirection =
    if int(state.simulationTime / 4.0'f32) mod 2 == 0: 1.0'f32 else: -1.0'f32
  for index, npc in state.virtualNpcs:
    if npc.isAlive:
      let direction = if index mod 2 == 0: crowdDirection else: -crowdDirection
      npc.move(vec3(2.2'f32 * direction, 0, 0), fixedDeltaTime)
  if not state.vehicle.isNil and state.vehicle.isAlive:
    state.vehicle.setInput(
      state.vehicleForward,
      state.vehicleSteering,
      state.vehicleBrake,
      state.vehicleHandBrake)
  if not state.motorcycle.isNil and state.motorcycle.isAlive:
    let steering = sin(state.simulationTime * 0.55'f32) * 0.16'f32
    state.motorcycle.setInput(0.65, steering)
  if not state.trackedVehicle.isNil and state.trackedVehicle.isAlive:
    state.trackedVehicle.setInput(
      state.trackedForward,
      state.trackedLeftRatio,
      state.trackedRightRatio,
      state.trackedBrake)
  if not state.movingPlatform.isNil and state.movingPlatform.isAlive:
    let y = 1.2'f32 + sin(state.simulationTime * 1.4'f32) * 0.65'f32
    let z = sin(state.simulationTime * 0.7'f32) * 2.0'f32
    state.movingPlatform.moveKinematic(
      vec3(5, y, z),
      quatIdentity(),
      fixedDeltaTime
    )
  if state.scene == 16 and state.softBodies.len > 0:
    let skin = state.softBodies[^1]
    if skin.isAlive and skin.skinJointCount == 2:
      let extent = 0.5'f32 * 0.32'f32 * 11.0'f32
      let angle = 0.45'f32 * sin(state.simulationTime * 1.7'f32)
      skin.skinVertices([
        softBodyJointTransform(
          vec3(0, 0, -extent),
          quatFromAxisAngle(vec3(1, 0, 0), angle)),
        softBodyJointTransform(
          vec3(0, 0, extent),
          quatFromAxisAngle(vec3(1, 0, 0), -angle))])
  if state.scene == 17:
    if not state.motorRagdoll.isNil and state.motorRagdoll.isAlive and
        not state.ragdollAnimation.isNil and state.ragdollAnimation.isAlive:
      state.motorRagdoll.driveMotors(
        state.ragdollAnimation,
        state.simulationTime - fixedDeltaTime,
        state.simulationTime,
        fixedDeltaTime)
      state.motorPose = state.ragdollAnimation.sampleRagdollLocalPose(
        state.motorRagdoll, state.simulationTime)
    if not state.kinematicRagdoll.isNil and state.kinematicRagdoll.isAlive:
      var target = state.kinematicPose
      let offsetX = 1.2'f32 * sin(state.simulationTime * 0.8'f32)
      let offsetY = 0.25'f32 * sin(state.simulationTime * 1.6'f32)
      for index in 0 ..< target.len:
        target[index].position.x += offsetX
        target[index].position.y += offsetY
      target[3].rotation = quatFromAxisAngle(
        vec3(0, 0, 1), PI.float32 * 0.5 +
          0.45'f32 * sin(state.simulationTime * 2.0'f32))
      target[4].rotation = quatFromAxisAngle(
        vec3(0, 0, 1), PI.float32 * 0.5 -
          0.45'f32 * sin(state.simulationTime * 2.0'f32))
      state.kinematicRagdoll.driveKinematic(target, fixedDeltaTime)
  let errors = state.world.step(fixedDeltaTime)
  if errors != {}:
    raise newException(JoltError, "physics update capacity exceeded: " & $errors)
  if state.scene == 17 and not state.skeletonMapper.isNil and
      state.skeletonMapper.isAlive and state.motorRagdoll.isAlive:
    let ragdollPose = state.motorRagdoll.pose
    var sourceModelPose = newSeq[SkeletonTransform](ragdollPose.len)
    for index, transform in ragdollPose:
      sourceModelPose[index] = skeletonTransform(
        transform.position, transform.rotation)
    state.mappedSkeletonPose = state.skeletonMapper.mappedPose(
      sourceModelPose, state.skeletonTargetLocalPose)

  for index in countdown(state.contactMarkers.high, 0):
    state.contactMarkers[index].remaining -= fixedDeltaTime
    if state.contactMarkers[index].remaining <= 0:
      state.contactMarkers.delete(index)

  let events = state.world.drainEvents()
  let softEvents = state.world.drainSoftBodyContactEvents()
  state.eventsLastStep = events.len + softEvents.len
  for event in events:
    if event.kind in {
        PhysicsEventKind.ContactAdded,
        PhysicsEventKind.ContactPersisted,
        PhysicsEventKind.ContactRemoved
      }:
      inc state.totalContactEvents
    if event.kind == PhysicsEventKind.ContactAdded and event.hasManifold:
      if state.contactMarkers.len == 128:
        state.contactMarkers.delete(0)
      let firstMaterial = event.material1(state.world)
      let secondMaterial = event.material2(state.world)
      let markerTint = if firstMaterial.isSome:
          firstMaterial.get.debugColor.raylibColor
        elif secondMaterial.isSome:
          secondMaterial.get.debugColor.raylibColor
        else:
          RlColor(r: 255, g: 45, b: 85, a: 255)
      state.contactMarkers.add(ContactMarker(
        position: event.contactPoint,
        normal: event.contactNormal,
        tint: markerTint,
        remaining: 0.45
      ))
  for event in softEvents:
    inc state.totalContactEvents
    if event.vertex.isSome:
      if state.contactMarkers.len == 128:
        state.contactMarkers.delete(0)
      state.contactMarkers.add(ContactMarker(
        position: event.contactPoint,
        normal: event.contactNormal,
        tint: RlColor(r: 30, g: 205, b: 225, a: 255),
        remaining: 0.35))

proc axisAngle(rotation: Quat): tuple[axis: Vec3, degrees: float32] =
  let normalizedRotation = rotation.normalized
  let w = clamp(normalizedRotation.w, -1.0'f32, 1.0'f32)
  let angle = 2.0'f32 * arccos(w)
  let denominator = sqrt(max(0.0'f32, 1.0'f32 - w * w))
  if denominator < 1.0e-5'f32:
    result.axis = vec3(0, 1, 0)
  else:
    result.axis = vec3(
      normalizedRotation.x / denominator,
      normalizedRotation.y / denominator,
      normalizedRotation.z / denominator
    )
  result.degrees = angle * 180.0'f32 / PI.float32

proc forwardDirection(rotation: Quat): Vec3 =
  let q = rotation.normalized
  result = vec3(
    2.0'f32 * (q.x * q.z + q.w * q.y),
    0,
    1.0'f32 - 2.0'f32 * (q.x * q.x + q.y * q.y))
  let horizontalLength = sqrt(result.x * result.x + result.z * result.z)
  if horizontalLength < 1.0e-5'f32:
    result = vec3(0, 0, 1)
  else:
    result.x /= horizontalLength
    result.z /= horizontalLength

proc follow(camera: var RlCamera3D; vehicle: Vehicle; snap: bool) =
  if vehicle.isNil or not vehicle.isAlive:
    return
  let chassis = vehicle.chassis
  let position = chassis.position
  let forward = chassis.rotation.forwardDirection
  let target = rlVector3(position.x, position.y + 0.65, position.z)
  let desired = rlVector3(
    position.x - forward.x * 8.0'f32,
    position.y + 4.2'f32,
    position.z - forward.z * 8.0'f32)
  let blend = if snap: 1.0'f32 else: 0.12'f32
  camera.position.x += (desired.x - camera.position.x) * blend
  camera.position.y += (desired.y - camera.position.y) * blend
  camera.position.z += (desired.z - camera.position.z) * blend
  camera.target.x += (target.x - camera.target.x) * blend
  camera.target.y += (target.y - camera.target.y) * blend
  camera.target.z += (target.z - camera.target.z) * blend
  camera.up = rlVector3(0, 1, 0)

proc follow(camera: var RlCamera3D; vehicle: TrackedVehicle; snap: bool) =
  if vehicle.isNil or not vehicle.isAlive:
    return
  let chassis = vehicle.chassis
  let position = chassis.position
  let forward = chassis.rotation.forwardDirection
  let target = rlVector3(position.x, position.y + 0.8, position.z)
  let desired = rlVector3(
    position.x - forward.x * 11.0'f32,
    position.y + 6.0'f32,
    position.z - forward.z * 11.0'f32)
  let blend = if snap: 1.0'f32 else: 0.12'f32
  camera.position.x += (desired.x - camera.position.x) * blend
  camera.position.y += (desired.y - camera.position.y) * blend
  camera.position.z += (desired.z - camera.position.z) * blend
  camera.target.x += (target.x - camera.target.x) * blend
  camera.target.y += (target.y - camera.target.y) * blend
  camera.target.z += (target.z - camera.target.z) * blend
  camera.up = rlVector3(0, 1, 0)

proc drawShape(shape: Shape; fallbackTint: RlColor) =
  let tint = if shape.material.isSome:
      shape.material.get.debugColor.raylibColor
    else:
      fallbackTint
  case shape.kind
  of ShapeKind.Box:
    let size = rlVector3(
      shape.halfExtent.x * 2,
      shape.halfExtent.y * 2,
      shape.halfExtent.z * 2
    )
    drawCubeV(rlVector3(0, 0, 0), size, tint)
    drawCubeWiresV(rlVector3(0, 0, 0), size, outline)
  of ShapeKind.Sphere:
    drawSphereEx(rlVector3(0, 0, 0), shape.radius, 12, 16, tint)
    drawSphereWires(rlVector3(0, 0, 0), shape.radius, 8, 12, outline)
  of ShapeKind.Capsule:
    let startPosition = rlVector3(0, -shape.halfHeight, 0)
    let endPosition = rlVector3(0, shape.halfHeight, 0)
    drawCapsule(startPosition, endPosition, shape.radius, 12, 6, tint)
    drawCapsuleWires(startPosition, endPosition, shape.radius, 8, 4, outline)
  of ShapeKind.Cylinder:
    drawCylinder(
      rlVector3(0, 0, 0), shape.radius, shape.radius,
      shape.halfHeight * 2, 18, tint)
    drawCylinderWires(
      rlVector3(0, 0, 0), shape.radius, shape.radius,
      shape.halfHeight * 2, 18, outline)
  of ShapeKind.TaperedCapsule:
    drawCylinder(
      rlVector3(0, 0, 0), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, tint)
    drawCylinderWires(
      rlVector3(0, 0, 0), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, outline)
    drawSphereEx(
      rlVector3(0, -shape.halfHeight, 0), shape.bottomRadius,
      10, 14, tint)
    drawSphereWires(
      rlVector3(0, -shape.halfHeight, 0), shape.bottomRadius,
      8, 12, outline)
    drawSphereEx(
      rlVector3(0, shape.halfHeight, 0), shape.topRadius,
      10, 14, tint)
    drawSphereWires(
      rlVector3(0, shape.halfHeight, 0), shape.topRadius,
      8, 12, outline)
  of ShapeKind.TaperedCylinder:
    drawCylinder(
      rlVector3(0, 0, 0), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, tint)
    drawCylinderWires(
      rlVector3(0, 0, 0), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, outline)
  of ShapeKind.Triangle:
    let a = shape.points[0]
    let b = shape.points[1]
    let c = shape.points[2]
    drawTriangle3D(
      rlVector3(a.x, a.y, a.z),
      rlVector3(b.x, b.y, b.z),
      rlVector3(c.x, c.y, c.z), tint)
    drawLine3D(rlVector3(a.x, a.y, a.z), rlVector3(b.x, b.y, b.z), outline)
    drawLine3D(rlVector3(b.x, b.y, b.z), rlVector3(c.x, c.y, c.z), outline)
    drawLine3D(rlVector3(c.x, c.y, c.z), rlVector3(a.x, a.y, a.z), outline)
  of ShapeKind.Plane:
    let extent = min(shape.planeHalfExtent, 20.0'f32)
    drawCubeV(
      rlVector3(0, -0.015, 0), rlVector3(extent * 2, 0.03, extent * 2),
      tint)
  of ShapeKind.Empty:
    drawSphereWires(
      rlVector3(shape.centerOfMass.x, shape.centerOfMass.y, shape.centerOfMass.z),
      0.16, 6, 8, tint)
  of ShapeKind.ConvexHull:
    for point in shape.points:
      drawSphereEx(
        rlVector3(point.x, point.y, point.z),
        0.055, 4, 6, tint)
    for first in 0 ..< shape.points.len:
      for second in first + 1 ..< shape.points.len:
        let a = shape.points[first]
        let b = shape.points[second]
        drawLine3D(
          rlVector3(a.x, a.y, a.z),
          rlVector3(b.x, b.y, b.z), outline)
  of ShapeKind.TriangleMesh:
    for triangle in 0 ..< shape.triangleIndices.len div 3:
      let triangleTint = if triangle < shape.materialIndices.len and
          int(shape.materialIndices[triangle]) < shape.materials.len:
          shape.materials[int(shape.materialIndices[triangle])].debugColor.raylibColor
        else:
          tint
      let offset = triangle * 3
      let a = shape.vertices[int(shape.triangleIndices[offset])]
      let b = shape.vertices[int(shape.triangleIndices[offset + 1])]
      let c = shape.vertices[int(shape.triangleIndices[offset + 2])]
      drawTriangle3D(
        rlVector3(a.x, a.y, a.z),
        rlVector3(b.x, b.y, b.z),
        rlVector3(c.x, c.y, c.z), triangleTint)
      drawLine3D(rlVector3(a.x, a.y, a.z), rlVector3(b.x, b.y, b.z), outline)
      drawLine3D(rlVector3(b.x, b.y, b.z), rlVector3(c.x, c.y, c.z), outline)
      drawLine3D(rlVector3(c.x, c.y, c.z), rlVector3(a.x, a.y, a.z), outline)
  of ShapeKind.HeightField:
    let sampleCount = int(shape.sampleCount)
    for z in 0 ..< sampleCount - 1:
      for x in 0 ..< sampleCount - 1:
        let cell = x + z * (sampleCount - 1)
        let cellTint = if cell < shape.materialIndices.len and
            int(shape.materialIndices[cell]) < shape.materials.len:
            shape.materials[int(shape.materialIndices[cell])].debugColor.raylibColor
          else:
            tint
        let a = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[z * sampleCount + x],
          shape.heightOffset.z + shape.heightScale.z * float32(z))
        let b = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x + 1),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[z * sampleCount + x + 1],
          shape.heightOffset.z + shape.heightScale.z * float32(z))
        let c = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x + 1),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[(z + 1) * sampleCount + x + 1],
          shape.heightOffset.z + shape.heightScale.z * float32(z + 1))
        let d = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[(z + 1) * sampleCount + x],
          shape.heightOffset.z + shape.heightScale.z * float32(z + 1))
        drawTriangle3D(
          rlVector3(a.x, a.y, a.z), rlVector3(d.x, d.y, d.z),
          rlVector3(c.x, c.y, c.z), cellTint)
        drawTriangle3D(
          rlVector3(a.x, a.y, a.z), rlVector3(c.x, c.y, c.z),
          rlVector3(b.x, b.y, b.z), cellTint)
        drawLine3D(
          rlVector3(a.x, a.y, a.z), rlVector3(b.x, b.y, b.z), outline)
        drawLine3D(
          rlVector3(a.x, a.y, a.z), rlVector3(d.x, d.y, d.z), outline)
  of ShapeKind.StaticCompound, ShapeKind.MutableCompound:
    for child in shape.children:
      let rotation = child.rotation.axisAngle
      rlPushMatrix()
      rlTranslatef(child.position.x, child.position.y, child.position.z)
      rlRotatef(
        rotation.degrees,
        rotation.axis.x,
        rotation.axis.y,
        rotation.axis.z)
      drawShape(child.shape, tint)
      rlPopMatrix()
  of ShapeKind.Scaled:
    rlPushMatrix()
    rlScalef(shape.shapeScale.x, shape.shapeScale.y, shape.shapeScale.z)
    drawShape(shape.innerShape, tint)
    rlPopMatrix()
  of ShapeKind.RotatedTranslated:
    let rotation = shape.shapeRotation.axisAngle
    rlPushMatrix()
    rlTranslatef(
      shape.shapePosition.x, shape.shapePosition.y, shape.shapePosition.z)
    rlRotatef(
      rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawShape(shape.innerShape, tint)
    rlPopMatrix()
  of ShapeKind.OffsetCenterOfMass:
    drawShape(shape.innerShape, tint)

proc draw(body: VisualBody) =
  if not body.body.isAlive:
    return
  let position = body.body.position
  let rotation = body.body.rotation.axisAngle
  rlPushMatrix()
  rlTranslatef(position.x, position.y, position.z)
  rlRotatef(
    rotation.degrees,
    rotation.axis.x,
    rotation.axis.y,
    rotation.axis.z)
  drawShape(body.body.shape, body.tint)
  rlPopMatrix()

proc draw(instance: PhysicsSceneInstance; visuals: openArray[SceneVisual]) =
  if instance.isNil or not instance.isAlive:
    return
  let count = min(instance.bodyCount, visuals.len)
  for index in 0 ..< count:
    let position = instance.bodyPosition(index)
    let rotation = instance.bodyRotation(index).axisAngle
    rlPushMatrix()
    rlTranslatef(position.x, position.y, position.z)
    rlRotatef(
      rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawShape(visuals[index].shape, visuals[index].tint)
    rlPopMatrix()
  if count >= 2 and instance.constraintCount > 0:
    let first = instance.bodyPosition(count - 2)
    let second = instance.bodyPosition(count - 1)
    drawLine3D(
      rlVector3(first.x, first.y, first.z),
      rlVector3(second.x, second.y, second.z),
      RlColor(r: 35, g: 210, b: 130, a: 255))

proc draw(ragdoll: Ragdoll; colorOffset: int) =
  if ragdoll.isNil or not ragdoll.isAlive:
    return
  var positions = newSeq[Vec3](ragdoll.partCount)
  for index in 0 ..< ragdoll.partCount:
    positions[index] = ragdoll.partPosition(index)
    let rotation = ragdoll.partRotation(index).axisAngle
    rlPushMatrix()
    rlTranslatef(positions[index].x, positions[index].y, positions[index].z)
    rlRotatef(
      rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawShape(
      ragdoll.partShape(index), palette[(index + colorOffset) mod palette.len])
    rlPopMatrix()
  for index in 1 ..< ragdoll.partCount:
    let parent = ragdoll.partParent(index).get
    drawLine3D(
      rlVector3(positions[parent].x, positions[parent].y,
        positions[parent].z),
      rlVector3(positions[index].x, positions[index].y, positions[index].z),
      RlColor(r: 35, g: 45, b: 55, a: 150))

proc draw(character: Character; tint: RlColor) =
  if character.isNil or not character.isAlive:
    return
  let foot = character.position
  let shape = character.shape
  let centerY = foot.y + shape.halfHeight + shape.radius
  let startPosition = rlVector3(foot.x, centerY - shape.halfHeight, foot.z)
  let endPosition = rlVector3(foot.x, centerY + shape.halfHeight, foot.z)
  drawCapsule(startPosition, endPosition, shape.radius, 12, 6, tint)
  drawCapsuleWires(startPosition, endPosition, shape.radius, 8, 4, outline)

proc draw(character: Character) =
  character.draw(RlColor(r: 40, g: 190, b: 225, a: 255))

proc draw(character: RigidCharacter) =
  if character.isNil or not character.isAlive:
    return
  let center = character.position
  let shape = character.shape
  if shape.kind == ShapeKind.Capsule:
    let startPosition = rlVector3(
      center.x, center.y - shape.halfHeight, center.z)
    let endPosition = rlVector3(
      center.x, center.y + shape.halfHeight, center.z)
    let tint = RlColor(r: 235, g: 95, b: 75, a: 255)
    drawCapsule(startPosition, endPosition, shape.radius, 12, 6, tint)
    drawCapsuleWires(startPosition, endPosition, shape.radius, 8, 4, outline)
  else:
    let rotation = character.rotation.axisAngle
    rlPushMatrix()
    rlTranslatef(center.x, center.y, center.z)
    rlRotatef(
      rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawShape(shape, RlColor(r: 235, g: 95, b: 75, a: 255))
    rlPopMatrix()

proc draw(vehicle: Vehicle) =
  if vehicle.isNil or not vehicle.isAlive:
    return
  let config = vehicle.configuration
  for wheel in 0 ..< vehicle.wheelCount:
    let state = vehicle.wheelState(wheel)
    let radius = if config.wheels.len > 0:
        config.wheels[wheel].radius
      else:
        config.wheelRadius
    let width = if config.wheels.len > 0:
        config.wheels[wheel].width
      else:
        config.wheelWidth
    let rotation = state.rotation.axisAngle
    rlPushMatrix()
    rlTranslatef(state.position.x, state.position.y, state.position.z)
    rlRotatef(
      rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawCylinder(
      rlVector3(0, 0, 0), radius, radius,
      width, 16, RlColor(r: 45, g: 50, b: 55, a: 255))
    drawCylinderWires(
      rlVector3(0, 0, 0), radius, radius,
      width, 16, RlColor(r: 230, g: 235, b: 240, a: 255))
    rlPopMatrix()
    if state.hasContact:
      drawLine3D(
        rlVector3(
          state.contactPosition.x, state.contactPosition.y,
          state.contactPosition.z),
        rlVector3(
          state.contactPosition.x + state.contactNormal.x * 0.4,
          state.contactPosition.y + state.contactNormal.y * 0.4,
          state.contactPosition.z + state.contactNormal.z * 0.4),
        RlColor(r: 40, g: 210, b: 110, a: 255))

proc draw(vehicle: TrackedVehicle) =
  if vehicle.isNil or not vehicle.isAlive:
    return
  let config = vehicle.configuration
  for wheel in 0 ..< vehicle.wheelCount:
    let state = vehicle.wheelState(wheel)
    let radius = config.wheels[wheel].radius
    let width = config.wheels[wheel].width
    let rotation = state.rotation.axisAngle
    rlPushMatrix()
    rlTranslatef(state.position.x, state.position.y, state.position.z)
    rlRotatef(
      rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawCylinder(
      rlVector3(0, 0, 0), radius, radius, width, 14,
      RlColor(r: 40, g: 48, b: 42, a: 255))
    drawCylinderWires(
      rlVector3(0, 0, 0), radius, radius, width, 14,
      RlColor(r: 190, g: 205, b: 185, a: 255))
    rlPopMatrix()
    if state.hasContact:
      drawLine3D(
        rlVector3(
          state.contactPosition.x, state.contactPosition.y,
          state.contactPosition.z),
        rlVector3(
          state.contactPosition.x + state.contactNormal.x * 0.35,
          state.contactPosition.y + state.contactNormal.y * 0.35,
          state.contactPosition.z + state.contactNormal.z * 0.35),
        RlColor(r: 60, g: 225, b: 120, a: 255))

proc draw(body: SoftBody) =
  if body.isNil or not body.isAlive:
    return
  let vertices = body.vertices
  let material = body.configuration.material
  let tint = if material.isSome:
      material.get.debugColor.raylibColor
    else:
      RlColor(r: 120, g: 170, b: 220, a: 255)
  for faceIndex in 0 ..< body.faceCount:
    let face = body.face(faceIndex)
    let a = vertices[int(face.vertices[0])].position
    let b = vertices[int(face.vertices[1])].position
    let c = vertices[int(face.vertices[2])].position
    let ra = rlVector3(a.x, a.y, a.z)
    let rb = rlVector3(b.x, b.y, b.z)
    let rc = rlVector3(c.x, c.y, c.z)
    drawTriangle3D(ra, rb, rc, tint)
    drawTriangle3D(rc, rb, ra, tint)
    drawLine3D(ra, rb, outline)
    drawLine3D(rb, rc, outline)
    drawLine3D(rc, ra, outline)
  for rodIndex in 0 ..< body.rodCount:
    let rod = body.rod(rodIndex)
    let a = vertices[int(rod.vertices[0])].position
    let b = vertices[int(rod.vertices[1])].position
    let ra = rlVector3(a.x, a.y, a.z)
    let rb = rlVector3(b.x, b.y, b.z)
    drawLine3D(ra, rb, tint)
    drawSphereEx(ra, 0.085, 6, 8, tint)
    if rodIndex == body.rodCount - 1:
      drawSphereEx(rb, 0.085, 6, 8, tint)

proc parseScene(value: string): int =
  case value.toLowerAscii
  of "1", "tower", "stack": 0
  of "2", "rain", "spheres": 1
  of "3", "domino", "dominoes": 2
  of "4", "mixed", "playground": 3
  of "5", "bridge", "constraints": 4
  of "6", "joints", "hinge", "slider": 5
  of "7", "cylinders", "cascade": 6
  of "8", "welded", "fixed", "machine": 7
  of "9", "queries", "scanner": 8
  of "0", "10", "torque", "arena": 9
  of "11", "c", "character", "player", "course": 10
  of "12", "v", "vehicle", "car", "drive": 11
  of "13", "m", "mutable", "decorated", "shapes": 12
  of "14", "f", "fluid", "water", "buoyancy": 13
  of "15", "t", "tracked", "tank", "tracks": 14
  of "16", "l", "soft", "cloth", "fabric": 15
  of "17", "g", "advanced", "lra", "volume", "rod", "rods": 16
  of "18", "y", "ragdoll", "ragdolls", "pose": 17
  of "19", "p", "scene", "serialization", "restore": 18
  of "20", "n", "contact", "policy", "conveyor": 19
  else: raise newException(ValueError, "unknown scene; use 1-20 or all")

proc screenshotPath(allScenes: bool; output: string; scene: int): string =
  if allScenes:
    output / (sceneNames[scene].replace(" ", "_") & ".png")
  else:
    output

proc captureScreenshot(path: string) =
  let directory = path.parentDir
  if directory.len > 0:
    createDir(directory)
  let filename = path.extractFilename
  takeScreenshot(filename.cstring)
  if path != filename:
    moveFile(filename, path)

proc main() =
  let sceneArgument = if paramCount() >= 1: paramStr(1) else: "tower"
  let allScenes = sceneArgument.toLowerAscii == "all"
  let framesPerScene = if paramCount() >= 2: parseInt(paramStr(2)) else: 0
  let screenshotOutput = if paramCount() >= 3: paramStr(3) else: ""
  if framesPerScene < 0:
    raise newException(ValueError, "frame count must be non-negative")
  if allScenes and (framesPerScene == 0 or screenshotOutput.len == 0):
    raise newException(ValueError, "all mode requires a frame count and output directory")
  if allScenes:
    createDir(screenshotOutput)

  let initialScene = if allScenes: 0 else: parseScene(sceneArgument)
  let windowFlags = if screenshotOutput.len > 0:
      FlagWindowHidden or FlagMsaa4xHint
    else:
      FlagWindowResizable or FlagMsaa4xHint
  setConfigFlags(windowFlags)
  setTraceLogLevel(LogWarning)
  initWindow(1280, 720, "jolt-nim visual demos")
  defer:
    closeWindow()
  if framesPerScene == 0:
    setTargetFPS(60)

  var camera = RlCamera3D(
    position: rlVector3(18, 13, 18),
    target: rlVector3(0, 3, 0),
    up: rlVector3(0, 1, 0),
    fovy: 45,
    projection: CameraPerspective
  )
  var state: DemoState
  state.reset(initialScene)
  defer:
    if not state.sceneInstance.isNil:
      state.sceneInstance.close()
    if not state.physicsScene.isNil:
      state.physicsScene.close()
    state.world.close()

  var paused = false
  var sceneFrame = 0
  while not windowShouldClose():
    if not allScenes:
      var requestedScene = state.scene
      if isKeyPressed(KeyOne): requestedScene = 0
      if isKeyPressed(KeyTwo): requestedScene = 1
      if isKeyPressed(KeyThree): requestedScene = 2
      if isKeyPressed(KeyFour): requestedScene = 3
      if isKeyPressed(KeyFive): requestedScene = 4
      if isKeyPressed(KeySix): requestedScene = 5
      if isKeyPressed(KeySeven): requestedScene = 6
      if isKeyPressed(KeyEight): requestedScene = 7
      if isKeyPressed(KeyNine): requestedScene = 8
      if isKeyPressed(KeyZero): requestedScene = 9
      if isKeyPressed(KeyC): requestedScene = 10
      if isKeyPressed(KeyV): requestedScene = 11
      if isKeyPressed(KeyM): requestedScene = 12
      if isKeyPressed(KeyF): requestedScene = 13
      if isKeyPressed(KeyT): requestedScene = 14
      if isKeyPressed(KeyL): requestedScene = 15
      if isKeyPressed(KeyG): requestedScene = 16
      if isKeyPressed(KeyY): requestedScene = 17
      if isKeyPressed(KeyP): requestedScene = 18
      if isKeyPressed(KeyN): requestedScene = 19
      if requestedScene != state.scene or isKeyPressed(KeyR):
        state.reset(requestedScene)
        sceneFrame = 0
      if isKeyPressed(KeySpace):
        paused = not paused

      if state.scene == 10:
        var movement = vec3(0, 0, 0)
        if isKeyDown(KeyW): movement.z -= 3.5
        if isKeyDown(KeyS): movement.z += 3.5
        if isKeyDown(KeyA): movement.x -= 3.5
        if isKeyDown(KeyD): movement.x += 3.5
        state.characterInput = movement
        state.characterJump = isKeyPressed(KeyJ)
      elif state.scene == 11:
        state.vehicleForward = 0
        state.vehicleSteering = 0
        state.vehicleBrake = 0
        state.vehicleHandBrake = 0
        if isKeyDown(KeyW): state.vehicleForward = 1
        if isKeyDown(KeyS): state.vehicleForward = -1
        if isKeyDown(KeyA): state.vehicleSteering = -1
        if isKeyDown(KeyD): state.vehicleSteering = 1
        if isKeyDown(KeyB): state.vehicleBrake = 1
        if isKeyDown(KeyH): state.vehicleHandBrake = 1
      elif state.scene == 14:
        state.trackedForward = 0
        state.trackedLeftRatio = 1
        state.trackedRightRatio = 1
        state.trackedBrake = 0
        if isKeyDown(KeyW): state.trackedForward = 1
        if isKeyDown(KeyS): state.trackedForward = -1
        if isKeyDown(KeyA):
          if state.trackedForward == 0:
            state.trackedForward = 1
            state.trackedLeftRatio = -1
          else:
            state.trackedLeftRatio = 0.45
        if isKeyDown(KeyD):
          if state.trackedForward == 0:
            state.trackedForward = 1
            state.trackedRightRatio = -1
          else:
            state.trackedRightRatio = 0.45
        if isKeyDown(KeyB): state.trackedBrake = 1

    if not paused:
      state.step()
    if state.scene == 11:
      camera.follow(state.vehicle, snap = sceneFrame == 0)
    elif state.scene == 14:
      camera.follow(state.trackedVehicle, snap = sceneFrame == 0)
    elif framesPerScene == 0:
      updateCamera(addr camera, CameraOrbital)

    beginDrawing()
    clearBackground(background)
    beginMode3D(camera)
    drawGrid(32, 1.0)
    for body in state.bodies:
      body.draw()
    state.sceneInstance.draw(state.sceneVisuals)
    state.character.draw()
    for index, npc in state.virtualNpcs:
      npc.draw(palette[(index + 3) mod palette.len])
    state.rigidCharacter.draw()
    state.vehicle.draw()
    state.motorcycle.draw()
    state.trackedVehicle.draw()
    for softBody in state.softBodies:
      softBody.draw()
    for index, ragdoll in state.ragdolls:
      ragdoll.draw(index * 2)
    if state.scene == 17:
      for index, transform in state.mappedSkeletonPose:
        drawSphereEx(
          rlVector3(
            transform.position.x, transform.position.y,
            transform.position.z),
          0.075, 6, 8, RlColor(r: 30, g: 225, b: 245, a: 255))
        let parent = state.skeletonTargetParents[index]
        if parent >= 0:
          let parentTransform = state.mappedSkeletonPose[parent]
          drawLine3D(
            rlVector3(
              parentTransform.position.x, parentTransform.position.y,
              parentTransform.position.z),
            rlVector3(
              transform.position.x, transform.position.y,
              transform.position.z),
            RlColor(r: 15, g: 175, b: 235, a: 255))
    for marker in state.contactMarkers:
      let endPosition = vec3(
        marker.position.x + marker.normal.x * 0.55'f32,
        marker.position.y + marker.normal.y * 0.55'f32,
        marker.position.z + marker.normal.z * 0.55'f32
      )
      drawSphereEx(
        rlVector3(marker.position.x, marker.position.y, marker.position.z),
        0.09,
        5,
        7,
        marker.tint
      )
      drawLine3D(
        rlVector3(marker.position.x, marker.position.y, marker.position.z),
        rlVector3(endPosition.x, endPosition.y, endPosition.z),
        marker.tint
      )
    if state.scene == 4:
      let rayOrigin = vec3(0, 12, 0)
      let hit = state.world.castRay(rayOrigin, vec3(0, -1, 0), 20)
      if hit.isSome:
        let hitPosition = hit.get.position
        drawLine3D(
          rlVector3(rayOrigin.x, rayOrigin.y, rayOrigin.z),
          rlVector3(hitPosition.x, hitPosition.y, hitPosition.z),
          RlColor(r: 231, g: 76, b: 60, a: 255)
        )
        drawSphereEx(
          rlVector3(hitPosition.x, hitPosition.y, hitPosition.z),
          0.12,
          6,
          8,
          RlColor(r: 231, g: 76, b: 60, a: 255)
        )
    if state.scene == 5:
      let points = [vec3(-7, 3, 4), vec3(0, 6, 4), vec3(7, 3, 4)]
      let tangents = [vec3(7, 0, 0), vec3(7, 0, 0), vec3(7, 0, 0)]
      for segment in 0 ..< 2:
        var previous = points[segment]
        for sample in 1 .. 24:
          let t = float32(sample) / 24.0'f32
          let t2 = t * t
          let t3 = t2 * t
          let h00 = 2 * t3 - 3 * t2 + 1
          let h10 = t3 - 2 * t2 + t
          let h01 = -2 * t3 + 3 * t2
          let h11 = t3 - t2
          let current = vec3(
            h00 * points[segment].x + h10 * tangents[segment].x +
              h01 * points[segment + 1].x + h11 * tangents[segment + 1].x,
            h00 * points[segment].y + h10 * tangents[segment].y +
              h01 * points[segment + 1].y + h11 * tangents[segment + 1].y,
            h00 * points[segment].z + h10 * tangents[segment].z +
              h01 * points[segment + 1].z + h11 * tangents[segment + 1].z)
          drawLine3D(
            rlVector3(previous.x, previous.y, previous.z),
            rlVector3(current.x, current.y, current.z),
            RlColor(r: 35, g: 185, b: 105, a: 255))
          previous = current
    if state.scene == 8:
      let rayOrigin = vec3(-11, 3, 0)
      let rayEnd = vec3(11, 3, 0)
      drawLine3D(
        rlVector3(rayOrigin.x, rayOrigin.y, rayOrigin.z),
        rlVector3(rayEnd.x, rayEnd.y, rayEnd.z),
        RlColor(r: 245, g: 180, b: 30, a: 255)
      )
      for hit in state.world.castRayAll(rayOrigin, vec3(1, 0, 0), 22):
        drawSphereEx(
          rlVector3(hit.position.x, hit.position.y, hit.position.z),
          0.13, 6, 8, RlColor(r: 245, g: 180, b: 30, a: 255))
      for hit in state.world.castRayAll(
          rayOrigin, vec3(1, 0, 0), 22,
          layer = some(scannerLayer)):
        drawSphereWires(
          rlVector3(hit.position.x, hit.position.y, hit.position.z),
          0.24, 8, 10, RlColor(r: 30, g: 190, b: 210, a: 255))
      let overlapCenter = vec3(sin(state.simulationTime * 0.8) * 7, 3, 0)
      drawSphereWires(
        rlVector3(overlapCenter.x, overlapCenter.y, overlapCenter.z),
        1.5, 12, 16, RlColor(r: 190, g: 60, b: 230, a: 255))
      for hit in state.world.overlapSphere(overlapCenter, 1.5):
        drawSphereEx(
          rlVector3(hit.contactPoint.x, hit.contactPoint.y, hit.contactPoint.z),
          0.11, 6, 8, RlColor(r: 190, g: 60, b: 230, a: 255))
      let sphereCastOrigin = vec3(-11, 4, 0)
      let sphereCastRadius = 0.35'f32
      let sphereCastEnd = vec3(11, 4, 0)
      drawLine3D(
        rlVector3(sphereCastOrigin.x, sphereCastOrigin.y, sphereCastOrigin.z),
        rlVector3(sphereCastEnd.x, sphereCastEnd.y, sphereCastEnd.z),
        RlColor(r: 30, g: 190, b: 210, a: 255))
      drawSphereWires(
        rlVector3(sphereCastOrigin.x, sphereCastOrigin.y, sphereCastOrigin.z),
        sphereCastRadius, 8, 12, RlColor(r: 30, g: 190, b: 210, a: 255))
      for hit in state.world.castSphereAll(
          sphereCastRadius, sphereCastOrigin, vec3(1, 0, 0), 22):
        drawSphereWires(
          rlVector3(hit.position.x, hit.position.y, hit.position.z),
          sphereCastRadius, 8, 12, RlColor(r: 30, g: 190, b: 210, a: 255))
        drawSphereEx(
          rlVector3(hit.contactPoint.x, hit.contactPoint.y, hit.contactPoint.z),
          0.11, 6, 8, RlColor(r: 30, g: 190, b: 210, a: 255))
        let normalEnd = vec3(
          hit.contactPoint.x + hit.normal.x * 0.5'f32,
          hit.contactPoint.y + hit.normal.y * 0.5'f32,
          hit.contactPoint.z + hit.normal.z * 0.5'f32)
        drawLine3D(
          rlVector3(hit.contactPoint.x, hit.contactPoint.y, hit.contactPoint.z),
          rlVector3(normalEnd.x, normalEnd.y, normalEnd.z),
          RlColor(r: 30, g: 190, b: 210, a: 255))
      let boxCastShape = boxShape(vec3(0.25, 0.9, 0.25))
      let boxCastRotation = quatFromAxisAngle(
        vec3(0, 0, 1), PI.float32 * 0.25)
      for hit in state.world.castShapeAll(
          boxCastShape,
          rayOrigin,
          vec3(1, 0, 0),
          22,
          rotation = boxCastRotation,
          layer = some(scannerLayer)):
        let rotation = boxCastRotation.axisAngle
        rlPushMatrix()
        rlTranslatef(hit.position.x, hit.position.y, hit.position.z)
        rlRotatef(
          rotation.degrees,
          rotation.axis.x,
          rotation.axis.y,
          rotation.axis.z)
        drawCubeWiresV(
          rlVector3(0, 0, 0),
          rlVector3(0.5, 1.8, 0.5),
          RlColor(r: 45, g: 190, b: 95, a: 255))
        rlPopMatrix()
      let broadOrigin = vec3(-11, 2, 1.5)
      let broadDirection = vec3(1, 0, 0)
      drawLine3D(
        rlVector3(broadOrigin.x, broadOrigin.y, broadOrigin.z),
        rlVector3(11, broadOrigin.y, broadOrigin.z),
        RlColor(r: 80, g: 95, b: 225, a: 255))
      for hit in state.world.broadPhaseCastRay(
          broadOrigin, broadDirection, 22):
        let position = vec3(
          broadOrigin.x + hit.distance,
          broadOrigin.y,
          broadOrigin.z)
        drawCubeWiresV(
          rlVector3(position.x, position.y, position.z),
          rlVector3(0.22, 0.22, 0.22),
          RlColor(r: 80, g: 95, b: 225, a: 255))
      let broadBoxCenter = vec3(
        sin(state.simulationTime * 0.55) * 6, 3, 0)
      let broadBoxRotation = quatFromAxisAngle(
        vec3(0, 1, 0), state.simulationTime * 0.35)
      let broadBoxHits = state.world.broadPhaseQueryOrientedBox(
        broadBoxCenter, vec3(1.8, 1.0, 0.65), broadBoxRotation)
      let broadRotation = broadBoxRotation.axisAngle
      rlPushMatrix()
      rlTranslatef(broadBoxCenter.x, broadBoxCenter.y, broadBoxCenter.z)
      rlRotatef(
        broadRotation.degrees,
        broadRotation.axis.x,
        broadRotation.axis.y,
        broadRotation.axis.z)
      drawCubeWiresV(
        rlVector3(0, 0, 0), rlVector3(3.6, 2.0, 1.3),
        if broadBoxHits.len > 0:
          RlColor(r: 255, g: 105, b: 45, a: 255)
        else:
          RlColor(r: 125, g: 135, b: 145, a: 255))
      rlPopMatrix()
      let point = vec3(
        sin(state.simulationTime * 1.15) * 9, 3, 0)
      let pointHits = state.world.collidePoint(point)
      drawSphereEx(
        rlVector3(point.x, point.y, point.z), 0.11, 6, 8,
        if pointHits.len > 0:
          RlColor(r: 255, g: 40, b: 80, a: 255)
        else:
          RlColor(r: 30, g: 210, b: 115, a: 255))
    if state.scene == 13:
      drawCubeV(
        rlVector3(0, 3, 0), rlVector3(20, 0.06, 16),
        RlColor(r: 45, g: 155, b: 230, a: 90))
      drawCubeWiresV(
        rlVector3(0, -1.2, 0), rlVector3(20, 8.4, 16),
        RlColor(r: 45, g: 155, b: 230, a: 255))
    endMode3D()

    let overlayHeight = if state.scene in [11, 14]: 140.cint
      elif state.scene in [10, 18, 19]: 108.cint
      else: 82.cint
    drawRectangle(
      12, 12, 980, overlayHeight,
      RlColor(r: 255, g: 255, b: 255, a: 225))
    var ragdollBodies = 0
    for ragdoll in state.ragdolls:
      if ragdoll.isAlive:
        ragdollBodies += ragdoll.partCount
    let sceneBodies = if state.sceneInstance.isNil or
        not state.sceneInstance.isAlive: 0 else: state.sceneInstance.bodyCount
    var characterBodies =
      if state.rigidCharacter.isNil or not state.rigidCharacter.isAlive: 0 else: 1
    for npc in state.virtualNpcs:
      if npc.isAlive and npc.innerBodyId.isSome:
        inc characterBodies
    let title = &"jolt-nim  |  {state.scene + 1}: {sceneNames[state.scene]}  |  {state.bodies.len + ragdollBodies + sceneBodies + characterBodies} bodies  |  {state.eventsLastStep} events  |  {state.totalContactEvents} contacts"
    drawText(title.cstring, 26, 22, 24, outline)
    let controls = case state.scene
      of 10: "blue: player   24 colored Virtual + inner body crowd   red: rigid AI"
      of 11: "WASD: drive six-wheeler   orange: autonomous motorcycle   B/H: brakes   R: reset"
      of 14: "WASD: tracks/pivot   B: brake   R: reset   SPACE: pause"
      of 17: "7 joints / animation + mapper / dynamic + motor + kinematic   Y: scene"
      of 18: "separate World -> checked bytes -> restored scene -> live World   P: scene"
      of 19: "layers: red reject | blue sensor | purple bounce | green conveyor   N: scene"
      else: "1-9 / 0 / C / V / M / F / T / L / G / Y / P / N: scene   R: reset   SPACE: pause"
    if state.scene == 10 and state.character.isAlive:
      let broadPhase = state.world.characterBroadPhaseStats
      let candidatesPerQuery =
        if broadPhase.queryCount == 0: 0.0
        else: float64(broadPhase.candidateCount) /
          float64(broadPhase.queryCount)
      let broadPhaseTelemetry = &"virtual broad phase: {broadPhase.registeredCharacters} characters   {broadPhase.occupiedCells} cells   {candidatesPerQuery:.2f} candidates/query   {broadPhase.narrowPhaseTestCount} narrow tests"
      drawText(broadPhaseTelemetry.cstring, 26, 54, 18, outline)
      drawText(controls.cstring, 26, 80, 18, outline)
    elif state.scene == 19:
      let bodyPolicies = state.world.bodyPairContactPolicyCount
      let subShapePolicies = state.world.subShapePairContactPolicyCount
      let telemetry = &"green: body reject -> left child conveyor   gray: sibling control   {bodyPolicies} body / {subShapePolicies} child rule"
      drawText(telemetry.cstring, 26, 54, 18, outline)
      drawText(controls.cstring, 26, 80, 18, outline)
    elif state.scene == 11 and state.vehicle.isAlive:
      let velocity = state.vehicle.chassis.linearVelocity
      let powertrain = state.vehicle.powertrainState
      let speedKmh = sqrt(
        velocity.x * velocity.x + velocity.y * velocity.y +
        velocity.z * velocity.z) * 3.6'f32
      var groundedWheels = 0
      var frontSteerAngle = 0.0'f32
      var rearSteerAngle = 0.0'f32
      for wheelIndex in 0 ..< state.vehicle.wheelCount:
        let wheel = state.vehicle.wheelState(wheelIndex)
        if wheel.hasContact:
          inc groundedWheels
        if wheelIndex == 0:
          frontSteerAngle = wheel.steerAngle
        elif wheelIndex == state.vehicle.wheelCount - 2:
          rearSteerAngle = wheel.steerAngle
      let telemetry = &"speed: {speedKmh:.1f} km/h   engine: {powertrain.engineRPM:.0f} rpm   gear: {powertrain.currentGear}   clutch: {powertrain.clutchFriction:.2f}"
      let wheelTelemetry = &"grounded: {groundedWheels}/{state.vehicle.wheelCount}   steer F/R: {frontSteerAngle * 180.0'f32 / PI.float32:.1f}/{rearSteerAngle * 180.0'f32 / PI.float32:.1f} deg   clutch wheel speed: {powertrain.wheelSpeedAtClutch:.1f} rad/s"
      var motorcycleTelemetry = "motorcycle: unavailable"
      if not state.motorcycle.isNil and state.motorcycle.isAlive:
        let bikeVelocity = state.motorcycle.chassis.linearVelocity
        let bikeSpeedKmh = sqrt(
          bikeVelocity.x * bikeVelocity.x +
          bikeVelocity.y * bikeVelocity.y +
          bikeVelocity.z * bikeVelocity.z) * 3.6'f32
        var bikeGrounded = 0
        for wheelIndex in 0 ..< state.motorcycle.wheelCount:
          if state.motorcycle.wheelState(wheelIndex).hasContact:
            inc bikeGrounded
        let lean = state.motorcycle.motorcycleControllerState
        motorcycleTelemetry = &"motorcycle: {bikeSpeedKmh:.1f} km/h   grounded: {bikeGrounded}/2   wheelbase: {lean.wheelBase:.2f} m   lean controller: {lean.leanControllerEnabled}"
      drawText(telemetry.cstring, 26, 54, 20, outline)
      drawText(wheelTelemetry.cstring, 26, 80, 18, outline)
      drawText(motorcycleTelemetry.cstring, 26, 106, 18, outline)
      drawText(controls.cstring, 26, 132, 18, outline)
    elif state.scene == 14 and state.trackedVehicle.isAlive:
      let velocity = state.trackedVehicle.chassis.linearVelocity
      let powertrain = state.trackedVehicle.powertrainState
      let left = state.trackedVehicle.trackState(TrackedVehicleSide.LeftTrack)
      let right = state.trackedVehicle.trackState(
        TrackedVehicleSide.RightTrack)
      let speedKmh = sqrt(
        velocity.x * velocity.x + velocity.y * velocity.y +
        velocity.z * velocity.z) * 3.6'f32
      var groundedWheels = 0
      for wheelIndex in 0 ..< state.trackedVehicle.wheelCount:
        if state.trackedVehicle.wheelState(wheelIndex).hasContact:
          inc groundedWheels
      let telemetry = &"speed: {speedKmh:.1f} km/h   engine: {powertrain.engineRPM:.0f} rpm   gear: {powertrain.currentGear}   clutch: {powertrain.clutchFriction:.2f}"
      let trackTelemetry = &"grounded: {groundedWheels}/{state.trackedVehicle.wheelCount}   left/right track: {left.angularVelocity:.1f}/{right.angularVelocity:.1f} rad/s"
      drawText(telemetry.cstring, 26, 54, 20, outline)
      drawText(trackTelemetry.cstring, 26, 80, 18, outline)
      drawText(controls.cstring, 26, 108, 18, outline)
    elif state.scene == 18 and state.sceneInstance.isAlive:
      let telemetry = &"{state.serializedSceneBytes} bytes   {state.sceneInstance.bodyCount} restored bodies   {state.sceneInstance.constraintCount} restored constraint"
      drawText(telemetry.cstring, 26, 54, 20, outline)
      drawText(controls.cstring, 26, 80, 18, outline)
    else:
      drawText(controls.cstring, 26, 56, 20, outline)
    if framesPerScene == 0:
      drawFPS(1160, 20)
    endDrawing()

    inc sceneFrame
    if framesPerScene > 0 and sceneFrame >= framesPerScene:
      if screenshotOutput.len > 0:
        let path = screenshotPath(allScenes, screenshotOutput, state.scene)
        captureScreenshot(path)
        echo "captured ", path
      if allScenes and state.scene < sceneNames.high:
        state.reset(state.scene + 1)
        sceneFrame = 0
      else:
        break

main()
