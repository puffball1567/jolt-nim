import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc testWorld(): World =
  var config = defaultWorldConfig()
  config.numThreads = 1
  config.maxBodies = 256
  config.maxBodyPairs = 2_048
  config.maxContactConstraints = 2_048
  newWorld(config)

proc chainConfig(motionType = MotionType.Dynamic;
                 groupId = 7'u32): RagdollConfig =
  let shape = capsuleShape(0.35, 0.14)
  ragdollConfig(@[
    ragdollPart(
      "root", shape, vec3(0, 4.8, 0), ragdollRootJoint(),
      motionType = motionType),
    ragdollPart(
      "middle", shape, vec3(0, 4.0, 0),
      ragdollJoint(
        0, vec3(0, 4.4, 0), twistAxis = vec3(0, 1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.45,
        planeHalfConeAngle = 0.45, twistMinAngle = -0.3,
        twistMaxAngle = 0.3, maxFrictionTorque = 0.2),
      motionType = motionType),
    ragdollPart(
      "tip", shape, vec3(0, 3.2, 0),
      ragdollJoint(
        1, vec3(0, 3.6, 0), twistAxis = vec3(0, 1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.45,
        planeHalfConeAngle = 0.45, twistMinAngle = -0.3,
        twistMaxAngle = 0.3, maxFrictionTorque = 0.2),
      motionType = motionType)
  ], groupId = groupId)

proc hingeChainConfig(): RagdollConfig =
  let shape = capsuleShape(0.35, 0.14)
  ragdollConfig(@[
    ragdollPart(
      "root", shape, vec3(0, 4.8, 0), ragdollRootJoint()),
    ragdollPart(
      "hinged", shape, vec3(0, 4.0, 0),
      ragdollHingeJoint(
        0, vec3(0, 4.4, 0), hingeAxis = vec3(0, 0, 1),
        normalAxis = vec3(1, 0, 0), minAngle = -0.6, maxAngle = 0.6,
        maxFrictionTorque = 0.1))
  ])

proc mixedJointConfig(): RagdollConfig =
  let shape = sphereShape(0.24)
  ragdollConfig(@[
    ragdollPart(
      "root", shape, vec3(0, 5.4, 0), ragdollRootJoint()),
    ragdollPart(
      "point", shape, vec3(0, 4.7, 0),
      ragdollPointJoint(0, vec3(0, 5.05, 0))),
    ragdollPart(
      "fixed", shape, vec3(0, 4.0, 0),
      ragdollFixedJoint(1, vec3(0, 4.35, 0))),
    ragdollPart(
      "cone", shape, vec3(0, 3.3, 0),
      ragdollConeJoint(
        2, vec3(0, 3.65, 0), twistAxis = vec3(0, 1, 0),
        halfConeAngle = 0.5))
  ])

proc sliderRagdollConfig(): RagdollConfig =
  let shape = sphereShape(0.22)
  ragdollConfig(@[
    ragdollPart(
      "anchor", shape, vec3(-2, 3, 0), ragdollRootJoint(),
      motionType = MotionType.Static),
    ragdollPart(
      "slider", shape, vec3(-2, 3, 0),
      ragdollSliderJoint(
        0, vec3(-2, 3, 0), sliderAxis = vec3(1, 0, 0),
        normalAxis = vec3(0, 1, 0), minPosition = -0.5,
        maxPosition = 0.5, maxFrictionForce = 0.1))
  ])

proc sixDOFRagdollConfig(): RagdollConfig =
  let shape = sphereShape(0.22)
  var jointConfig = defaultSixDOFConfig()
  jointConfig.swingType = SixDOFSwingType.SwingCone
  jointConfig.limits[SixDOFAxis.TranslationX] = freeAxis()
  jointConfig.limits[SixDOFAxis.TranslationY] = limitedAxis(-0.2, 0.2)
  jointConfig.limits[SixDOFAxis.RotationY] = limitedAxis(-0.4, 0.4)
  jointConfig.limits[SixDOFAxis.RotationZ] = limitedAxis(-0.3, 0.3)
  ragdollConfig(@[
    ragdollPart(
      "anchor", shape, vec3(2, 3, 0), ragdollRootJoint(),
      motionType = MotionType.Static),
    ragdollPart(
      "six dof", shape, vec3(2, 3, 0),
      ragdollSixDOFJoint(
        0, vec3(2, 3, 0), config = jointConfig,
        linearFriction = vec3(0.05, 0, 0)))
  ])

proc distance(a, b: Vec3): float32 =
  let d = vec3(a.x - b.x, a.y - b.y, a.z - b.z)
  sqrt(d.x * d.x + d.y * d.y + d.z * d.z)

proc checkNear(actual, expected: float32; epsilon = 1.0e-4'f32) =
  check abs(actual - expected) <= epsilon

suite "Jolt ragdoll":
  test "dynamic hierarchy falls, collides and stays constrained":
    let world = testWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(20, 0.5, 20)), vec3(0, -0.5, 0))
    let ragdoll = world.addRagdoll(chainConfig())

    check ragdoll.partCount == 3
    check ragdoll.constraintCount == 2
    check ragdoll.partName(1) == "middle"
    check ragdoll.partId(0) != ragdoll.partId(1)
    let filtered = world.overlapSphere(
      ragdoll.partPosition(0), 0.4,
      bodyFilter = includeBodies([ragdoll.partId(0)]))
    check filtered.len > 0
    for hit in filtered:
      check hit.bodyId == ragdoll.partId(0)
    let initialRoot = ragdoll.rootTransform.position
    for _ in 0 ..< 300:
      check world.step(dt) == {}

    check ragdoll.rootTransform.position.y < initialRoot.y - 1
    check ragdoll.partPosition(2).y > 0.1
    check distance(ragdoll.partPosition(0), ragdoll.partPosition(1)) < 1.15
    check distance(ragdoll.partPosition(1), ragdoll.partPosition(2)) < 1.15

  test "pose round trip and aggregate velocity operations work":
    let world = testWorld()
    defer: world.close()
    let ragdoll = world.addRagdoll(chainConfig())
    var target = ragdoll.pose
    for index in 0 ..< target.len:
      target[index].position.x += 3
      target[index].position.y += 1
    ragdoll.setPose(target)
    let actual = ragdoll.pose
    for index in 0 ..< actual.len:
      checkNear(actual[index].position.x, target[index].position.x)
      checkNear(actual[index].position.y, target[index].position.y)

    ragdoll.setVelocity(vec3(1, 0, 0), vec3(0, 0, 0))
    checkNear(ragdoll.partLinearVelocity(0).x, 1)
    ragdoll.addLinearVelocity(vec3(0, 2, 0))
    checkNear(ragdoll.partLinearVelocity(1).y, 2)
    ragdoll.addImpulse(vec3(0, 0, 0.5))
    check ragdoll.partLinearVelocity(2).z > 0

  test "kinematic pose drive reaches its target after one step":
    let world = testWorld()
    defer: world.close()
    let ragdoll = world.addRagdoll(chainConfig(MotionType.Kinematic))
    var target = ragdoll.pose
    for index in 0 ..< target.len:
      target[index].position.x += 0.75
    ragdoll.driveKinematic(target, dt)
    discard world.step(dt)
    for index in 0 ..< target.len:
      checkNear(ragdoll.partPosition(index).x, target[index].position.x, 2.0e-3)

  test "swing twist motors rotate a dynamic child toward a local pose":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(chainConfig())
    let before = ragdoll.partRotation(1)
    var target = @[
      RagdollTransform(position: vec3(0, 0, 0), rotation: quatIdentity()),
      RagdollTransform(
        position: vec3(0, -0.8, 0),
        rotation: quatFromAxisAngle(vec3(0, 0, 1), 0.35)),
      RagdollTransform(
        position: vec3(0, -0.8, 0), rotation: quatIdentity())]
    for _ in 0 ..< 90:
      ragdoll.driveMotors(target)
      discard world.step(dt)
    let after = ragdoll.partRotation(1)
    check abs(after.z - before.z) > 0.02

  test "hinge motor consumes consecutive poses and bends on one axis":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(hingeChainConfig())
    let before = ragdoll.partRotation(1)
    let previous = @[
      RagdollTransform(position: vec3(0, 0, 0), rotation: quatIdentity()),
      RagdollTransform(position: vec3(0, -0.8, 0), rotation: quatIdentity())]
    var target = previous
    target[1].rotation = quatFromAxisAngle(vec3(0, 0, 1), 0.45)
    for _ in 0 ..< 60:
      ragdoll.driveMotors(previous, target, dt)
      discard world.step(dt)
    let after = ragdoll.partRotation(1)
    check ragdoll.constraintCount == 1
    check abs(after.z - before.z) > 0.03

  test "additional distance constraints link non-parent parts":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var config = chainConfig()
    config.distanceConstraints.add(ragdollDistanceConstraint(
      0, 2, config.parts[0].position, config.parts[2].position, 0.8, 0.9))
    let ragdoll = world.addRagdoll(config)
    check ragdoll.constraintCount == 3
    for _ in 0 ..< 120:
      discard world.step(dt)
    check distance(ragdoll.partPosition(0), ragdoll.partPosition(2)) < 1.25

  test "point fixed and cone parent joints form a mixed hierarchy":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(mixedJointConfig())
    check ragdoll.partCount == 4
    check ragdoll.constraintCount == 3
    ragdoll.addPartImpulse(3, vec3(1.5, 0, 0))
    for _ in 0 ..< 120:
      discard world.step(dt)
    for index in 1 ..< ragdoll.partCount:
      check distance(
        ragdoll.partPosition(index - 1), ragdoll.partPosition(index)) < 0.95
    expect JoltError:
      ragdoll.driveMotors(ragdoll.pose)

  test "slider parent joint permits only bounded axial movement":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(sliderRagdollConfig())
    let initial = ragdoll.partPosition(1)
    ragdoll.addPartImpulse(1, vec3(20, 2, 0))
    var maxX = initial.x
    for _ in 0 ..< 120:
      discard world.step(dt)
      maxX = max(maxX, ragdoll.partPosition(1).x)
    let moved = ragdoll.partPosition(1)
    check maxX > initial.x + 0.2
    check moved.x <= initial.x + 0.55
    check abs(moved.y - initial.y) < 0.08
    expect JoltError:
      ragdoll.driveMotors(ragdoll.pose)

  test "SixDOF parent joint applies free limited and fixed axes":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(sixDOFRagdollConfig())
    check ragdoll.constraintCount == 1
    check ragdoll.constraintKind(0) == ConstraintKind.SixDOF
    check ragdoll.constraintBodyParts(0) == (0, 1)
    check ragdoll.sixDOFSwingType(0) == SixDOFSwingType.SwingCone
    check ragdoll.axisLimit(0, SixDOFAxis.TranslationX).mode ==
      SixDOFAxisMode.AxisFree
    check ragdoll.axisLimit(0, SixDOFAxis.TranslationY) ==
      limitedAxis(-0.2, 0.2)
    check ragdoll.axisLimit(0, SixDOFAxis.RotationZ) ==
      limitedAxis(-0.3, 0.3)
    let initial = ragdoll.partPosition(1)
    ragdoll.addPartImpulse(1, vec3(20, 2, 1))
    var maxX = initial.x
    for _ in 0 ..< 120:
      discard world.step(dt)
      maxX = max(maxX, ragdoll.partPosition(1).x)
    let moved = ragdoll.partPosition(1)
    check maxX > initial.x + 0.2
    check abs(moved.y - initial.y) <= 0.25
    check abs(moved.z - initial.z) < 0.08

  test "ragdoll constraint controls update native SixDOF state":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(sixDOFRagdollConfig())
    check ragdoll.constraintEnabled(0)
    ragdoll.setConstraintEnabled(0, false)
    check not ragdoll.constraintEnabled(0)
    ragdoll.setConstraintEnabled(0, true)
    ragdoll.setConstraintPriority(0, 17)
    check ragdoll.constraintPriority(0) == 17
    ragdoll.setConstraintSolverStepOverrides(0, 3, 2)
    check ragdoll.constraintSolverStepOverrides(0) == (3'u32, 2'u32)
    ragdoll.setConstraintUserData(0, 0x1234'u64)
    check ragdoll.constraintUserData(0) == 0x1234'u64

    checkNear(ragdoll.sixDOFConstraintFriction(
      0, SixDOFAxis.TranslationX), 0.05)
    ragdoll.setAxisFriction(0, SixDOFAxis.TranslationX, 1.25)
    checkNear(ragdoll.sixDOFConstraintFriction(
      0, SixDOFAxis.TranslationX), 1.25)
    ragdoll.setAxisLimit(0, SixDOFAxis.TranslationY, freeAxis())
    check ragdoll.axisLimit(0, SixDOFAxis.TranslationY).mode ==
      SixDOFAxisMode.AxisFree
    ragdoll.setAxisLimit(
      0, SixDOFAxis.TranslationY, limitedAxis(-0.2, 0.2))

    let spring = springSettings(
      1250, 40, SpringMode.StiffnessAndDamping)
    ragdoll.setAxisLimitSpring(0, SixDOFAxis.TranslationY, spring)
    let restoredSpring = ragdoll.sixDOFConstraintLimitSpring(
      0, SixDOFAxis.TranslationY)
    check restoredSpring.mode == spring.mode
    checkNear(restoredSpring.value, spring.value)
    checkNear(restoredSpring.damping, spring.damping)
    ragdoll.resetConstraintWarmStart(0)
    ragdoll.addPartImpulse(1, vec3(1, 0, 0))
    discard world.step(dt)
    let impulse = ragdoll.constraintSolverImpulse(0)
    check classify(impulse.position.x) notin {fcNan, fcInf, fcNegInf}

    expect ValueError:
      ragdoll.setConstraintSolverStepOverrides(0, 256, 0)
    expect ValueError:
      ragdoll.setAxisLimitSpring(
        0, SixDOFAxis.RotationY, springSettings(2, 1))
    expect IndexDefect:
      discard ragdoll.constraintEnabled(99)

  test "ragdoll SixDOF position motor drives a free translation axis":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(sixDOFRagdollConfig())
    let initial = ragdoll.partPosition(1)
    var motor = defaultMotorSettings()
    motor.spring = springSettings(5, 1)
    motor.minForce = -1_000
    motor.maxForce = 1_000
    ragdoll.configureAxisMotor(0, SixDOFAxis.TranslationX, motor)
    let configured = ragdoll.sixDOFAxisMotorSettings(
      0, SixDOFAxis.TranslationX)
    check configured.spring.mode == motor.spring.mode
    checkNear(configured.spring.value, motor.spring.value)
    checkNear(configured.spring.damping, motor.spring.damping)
    checkNear(configured.minForce, motor.minForce)
    checkNear(configured.maxForce, motor.maxForce)

    ragdoll.setSixDOFMotorTargets(
      0, vec3(0, 0, 0), vec3(0, 0, 0),
      vec3(0.75, 0, 0), quatIdentity())
    ragdoll.setAxisMotorState(
      0, SixDOFAxis.TranslationX, MotorState.Position)
    let state = ragdoll.sixDOFMotor(0, SixDOFAxis.TranslationX)
    check state.state == MotorState.Position
    check state.targetVelocity == vec3(0, 0, 0)
    check state.targetAngularVelocity == vec3(0, 0, 0)
    check state.targetPosition == vec3(0.75, 0, 0)
    check state.targetOrientation == quatIdentity()
    for _ in 0 ..< 240:
      discard world.step(dt)
    check abs(ragdoll.partPosition(1).x - (initial.x + 0.75)) < 0.2
    check classify(ragdoll.constraintSolverImpulse(
      0).motorTranslation.x) notin {fcNan, fcInf, fcNegInf}
    ragdoll.setAxisMotorState(
      0, SixDOFAxis.TranslationX, MotorState.Disabled)

    var invalid = motor
    invalid.minForce = 1
    invalid.maxForce = -1
    expect ValueError:
      ragdoll.configureAxisMotor(0, SixDOFAxis.TranslationX, invalid)

  test "part velocity and impulse operations target one body":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(chainConfig())
    ragdoll.setPartLinearVelocity(1, vec3(2, 0, 0))
    ragdoll.setPartAngularVelocity(1, vec3(0, 0, 1))
    checkNear(ragdoll.partLinearVelocity(1).x, 2)
    checkNear(ragdoll.partAngularVelocity(1).z, 1)
    ragdoll.addPartImpulse(2, vec3(0, 0, 0.4))
    check ragdoll.partLinearVelocity(2).z > 0
    let rootPosition = ragdoll.partPosition(0)
    ragdoll.addPartImpulseAtPosition(
      0, vec3(0.2, 0, 0),
      vec3(rootPosition.x, rootPosition.y + 0.2, rootPosition.z))
    ragdoll.addPartAngularImpulse(0, vec3(0, 0.1, 0))
    check abs(ragdoll.partAngularVelocity(0).y) > 0

  test "parent-child collision filtering suppresses internal contacts":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var config = chainConfig()
    config.parts[1].position = vec3(0, 4.55, 0)
    let ragdoll = world.addRagdoll(config)
    for _ in 0 ..< 20:
      discard world.step(dt)
    for event in world.drainEvents():
      let a = event.body1
      let b = if event.body2.isSome: event.body2.get else: a
      check not ((a == ragdoll.partId(0) and b == ragdoll.partId(1)) or
                 (a == ragdoll.partId(1) and b == ragdoll.partId(0)))

  test "world snapshots restore ragdoll body and constraint state":
    let world = testWorld()
    defer: world.close()
    let ragdoll = world.addRagdoll(chainConfig())
    for _ in 0 ..< 20:
      discard world.step(dt)
    let saved = ragdoll.pose
    let state = world.saveState()
    defer: state.close()
    ragdoll.addImpulse(vec3(5, 2, 1))
    for _ in 0 ..< 90:
      discard world.step(dt)
    world.restoreState(state)
    let restored = ragdoll.pose
    for index in 0 ..< saved.len:
      checkNear(restored[index].position.x, saved[index].position.x)
      checkNear(restored[index].position.y, saved[index].position.y)
      checkNear(restored[index].position.z, saved[index].position.z)

  test "point fixed and cone constraints link non-parent parts":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))

    var pointConfig = chainConfig()
    pointConfig.stabilize = false
    pointConfig.pointConstraints.add(
      ragdollPointConstraint(0, 2, vec3(0, 4, 0)))
    let pointRagdoll = world.addRagdoll(pointConfig)
    check pointRagdoll.constraintCount == 3
    check pointRagdoll.constraintKind(2) == ConstraintKind.Point
    check pointRagdoll.constraintBodyParts(2) == (0, 2)
    pointRagdoll.addPartImpulse(2, vec3(2, 0, 0))
    for _ in 0 ..< 60:
      discard world.step(dt)
    check classify(pointRagdoll.constraintSolverImpulse(
      2).position.x) notin {fcNan, fcInf, fcNegInf}
    pointRagdoll.close()

    var fixedConfig = chainConfig(groupId = 8)
    fixedConfig.stabilize = false
    fixedConfig.fixedConstraints.add(ragdollFixedConstraint(0, 2))
    let fixedRagdoll = world.addRagdoll(fixedConfig)
    check fixedRagdoll.constraintCount == 3
    check fixedRagdoll.constraintKind(2) == ConstraintKind.Fixed
    check fixedRagdoll.constraintBodyParts(2) == (0, 2)
    fixedRagdoll.addPartImpulse(2, vec3(2, 0, 0))
    for _ in 0 ..< 60:
      discard world.step(dt)
    check abs(fixedRagdoll.partPosition(2).x -
      fixedRagdoll.partPosition(0).x) < 0.1
    fixedRagdoll.close()

    var coneConfig = chainConfig(groupId = 9)
    coneConfig.stabilize = false
    coneConfig.coneConstraints.add(ragdollConeConstraint(
      0, 2, vec3(0, 4, 0), vec3(0, 1, 0), vec3(0, 1, 0), 0.4))
    let coneRagdoll = world.addRagdoll(coneConfig)
    check coneRagdoll.constraintCount == 3
    check coneRagdoll.constraintKind(2) == ConstraintKind.Cone
    check coneRagdoll.constraintBodyParts(2) == (0, 2)
    coneRagdoll.addPartImpulse(2, vec3(1, 0, 0))
    for _ in 0 ..< 60:
      discard world.step(dt)
    let coneImpulse = coneRagdoll.constraintSolverImpulse(2)
    check classify(coneImpulse.position.x) notin {fcNan, fcInf, fcNegInf}
    check classify(coneImpulse.limit) notin {fcNan, fcInf, fcNegInf}

  test "hinge and slider constraints link non-parent parts":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))

    var hingeConfig = chainConfig(groupId = 10)
    hingeConfig.stabilize = false
    hingeConfig.hingeConstraints.add(ragdollHingeConstraint(
      0, 2, vec3(0, 4, 0), vec3(0, 0, 1), vec3(1, 0, 0),
      minAngle = -0.3, maxAngle = 0.3, maxFrictionTorque = 0.1))
    let hingeRagdoll = world.addRagdoll(hingeConfig)
    check hingeRagdoll.constraintCount == 3
    check hingeRagdoll.constraintKind(2) == ConstraintKind.Hinge
    check hingeRagdoll.constraintBodyParts(2) == (0, 2)
    hingeRagdoll.addPartAngularImpulse(2, vec3(0, 0, 2))
    for _ in 0 ..< 60:
      discard world.step(dt)
    let hingeImpulse = hingeRagdoll.constraintSolverImpulse(2)
    check classify(hingeImpulse.rotation.x) notin {fcNan, fcInf, fcNegInf}
    check classify(hingeImpulse.limit) notin {fcNan, fcInf, fcNegInf}
    hingeRagdoll.close()

    var sliderConfig = chainConfig(groupId = 11)
    sliderConfig.stabilize = false
    sliderConfig.sliderConstraints.add(ragdollSliderConstraint(
      0, 2, vec3(0, 4, 0), vec3(1, 0, 0), vec3(0, 1, 0),
      minPosition = -0.25, maxPosition = 0.25,
      maxFrictionForce = 0.1))
    let sliderRagdoll = world.addRagdoll(sliderConfig)
    check sliderRagdoll.constraintCount == 3
    check sliderRagdoll.constraintKind(2) == ConstraintKind.Slider
    check sliderRagdoll.constraintBodyParts(2) == (0, 2)
    let initialRelativeX = sliderRagdoll.partPosition(2).x -
      sliderRagdoll.partPosition(0).x
    sliderRagdoll.addPartImpulse(2, vec3(10, 0, 0))
    for _ in 0 ..< 120:
      discard world.step(dt)
    let finalRelativeX = sliderRagdoll.partPosition(2).x -
      sliderRagdoll.partPosition(0).x
    check abs(finalRelativeX - initialRelativeX) <= 0.3
    let sliderImpulse = sliderRagdoll.constraintSolverImpulse(2)
    check classify(sliderImpulse.position.x) notin {fcNan, fcInf, fcNegInf}
    check classify(sliderImpulse.limit) notin {fcNan, fcInf, fcNegInf}

  test "swing twist and SixDOF constraints link non-parent parts":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))

    var swingConfig = chainConfig(groupId = 12)
    swingConfig.stabilize = false
    swingConfig.swingTwistConstraints.add(ragdollSwingTwistConstraint(
      0, 2, vec3(0, 4, 0), twistAxis = vec3(0, 1, 0),
      planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.35,
      planeHalfConeAngle = 0.45, twistMinAngle = -0.25,
      twistMaxAngle = 0.25, maxFrictionTorque = 0.1))
    let swingRagdoll = world.addRagdoll(swingConfig)
    check swingRagdoll.constraintCount == 3
    check swingRagdoll.constraintKind(2) == ConstraintKind.SwingTwist
    check swingRagdoll.constraintBodyParts(2) == (0, 2)
    swingRagdoll.addPartAngularImpulse(2, vec3(0, 1, 1))
    for _ in 0 ..< 60:
      discard world.step(dt)
    let swingImpulse = swingRagdoll.constraintSolverImpulse(2)
    check classify(swingImpulse.rotation.x) notin {fcNan, fcInf, fcNegInf}
    check classify(swingImpulse.rotation.y) notin {fcNan, fcInf, fcNegInf}
    swingRagdoll.close()

    var axes = defaultSixDOFConfig()
    axes.swingType = SixDOFSwingType.SwingCone
    axes.limits[SixDOFAxis.TranslationX] = limitedAxis(-0.2, 0.2)
    axes.limits[SixDOFAxis.RotationY] = limitedAxis(-0.3, 0.3)
    axes.limits[SixDOFAxis.RotationZ] = limitedAxis(-0.25, 0.25)
    var sixConfig = chainConfig(groupId = 13)
    sixConfig.stabilize = false
    sixConfig.sixDOFConstraints.add(ragdollSixDOFConstraint(
      0, 2, vec3(0, 4, 0), config = axes,
      linearFriction = vec3(0.1, 0, 0),
      angularFriction = vec3(0, 0.1, 0.1)))
    let sixRagdoll = world.addRagdoll(sixConfig)
    check sixRagdoll.constraintCount == 3
    check sixRagdoll.constraintKind(2) == ConstraintKind.SixDOF
    check sixRagdoll.constraintBodyParts(2) == (0, 2)
    check sixRagdoll.sixDOFSwingType(2) == SixDOFSwingType.SwingCone
    check sixRagdoll.axisLimit(2, SixDOFAxis.TranslationX) ==
      limitedAxis(-0.2, 0.2)
    checkNear(sixRagdoll.sixDOFConstraintFriction(
      2, SixDOFAxis.TranslationX), 0.1)
    sixRagdoll.setConstraintSolverStepOverrides(2, 8, 8)
    let initialRelativeX = sixRagdoll.partPosition(2).x -
      sixRagdoll.partPosition(0).x
    sixRagdoll.addPartImpulse(2, vec3(10, 0, 0))
    for _ in 0 ..< 120:
      discard world.step(dt)
    let finalRelativeX = sixRagdoll.partPosition(2).x -
      sixRagdoll.partPosition(0).x
    check abs(finalRelativeX - initialRelativeX) <= 0.35
    let sixImpulse = sixRagdoll.constraintSolverImpulse(2)
    check classify(sixImpulse.position.x) notin {fcNan, fcInf, fcNegInf}
    check classify(sixImpulse.rotation.y) notin {fcNan, fcInf, fcNegInf}

  test "ragdoll scalar and swing twist motors expose native state":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var motorSettings = defaultMotorSettings()
    motorSettings.spring = springSettings(6, 1)
    motorSettings.minForce = -30
    motorSettings.maxForce = 30
    motorSettings.minTorque = -20
    motorSettings.maxTorque = 20

    var hingeConfig = chainConfig(groupId = 14)
    hingeConfig.stabilize = false
    hingeConfig.hingeConstraints.add(ragdollHingeConstraint(
      0, 2, vec3(0, 4, 0), vec3(0, 0, 1), vec3(1, 0, 0),
      minAngle = -0.4, maxAngle = 0.4))
    let hingeRagdoll = world.addRagdoll(hingeConfig)
    hingeRagdoll.setMotor(
      2, MotorState.Position, 0, 0.15, motorSettings)
    let hingeSettings = hingeRagdoll.motorSettings(2)
    checkNear(hingeSettings.spring.value, motorSettings.spring.value)
    checkNear(hingeSettings.minTorque, motorSettings.minTorque)
    checkNear(hingeSettings.maxTorque, motorSettings.maxTorque)
    let hingeMotor = hingeRagdoll.motor(2)
    check hingeMotor.state == MotorState.Position
    checkNear(hingeMotor.targetVelocity, 0)
    checkNear(hingeMotor.targetPosition, 0.15)
    for _ in 0 ..< 30:
      discard world.step(dt)
    check classify(hingeRagdoll.constraintSolverImpulse(
      2).motorRotation.x) notin {fcNan, fcInf, fcNegInf}
    hingeRagdoll.close()

    var sliderConfig = chainConfig(groupId = 15)
    sliderConfig.stabilize = false
    sliderConfig.sliderConstraints.add(ragdollSliderConstraint(
      0, 2, vec3(0, 4, 0), vec3(1, 0, 0), vec3(0, 1, 0),
      minPosition = -0.3, maxPosition = 0.3))
    let sliderRagdoll = world.addRagdoll(sliderConfig)
    sliderRagdoll.setMotor(
      2, MotorState.Position, 0, 0.15, motorSettings)
    let sliderSettings = sliderRagdoll.motorSettings(2)
    checkNear(sliderSettings.minForce, motorSettings.minForce)
    checkNear(sliderSettings.maxForce, motorSettings.maxForce)
    let sliderMotor = sliderRagdoll.motor(2)
    check sliderMotor.state == MotorState.Position
    checkNear(sliderMotor.targetPosition, 0.15)
    for _ in 0 ..< 30:
      discard world.step(dt)
    check classify(sliderRagdoll.constraintSolverImpulse(
      2).motorTranslation.x) notin {fcNan, fcInf, fcNegInf}
    sliderRagdoll.close()

    var swingConfig = chainConfig(groupId = 16)
    swingConfig.stabilize = false
    swingConfig.swingTwistConstraints.add(ragdollSwingTwistConstraint(
      0, 2, vec3(0, 4, 0), twistAxis = vec3(0, 1, 0),
      planeAxis = vec3(0, 0, 1)))
    let swingRagdoll = world.addRagdoll(swingConfig)
    swingRagdoll.configureSwingMotor(2, motorSettings)
    swingRagdoll.configureTwistMotor(2, motorSettings)
    let targetOrientation = quatFromAxisAngle(vec3(0, 0, 1), 0.15)
    swingRagdoll.setSwingTwistMotorTargets(
      2, vec3(0, 0.1, 0), targetOrientation)
    swingRagdoll.setSwingMotorState(2, MotorState.Position)
    swingRagdoll.setTwistMotorState(2, MotorState.Velocity)
    checkNear(swingRagdoll.swingMotorSettings(2).maxTorque, 20)
    checkNear(swingRagdoll.twistMotorSettings(2).minTorque, -20)
    let swingMotor = swingRagdoll.swingTwistMotor(2)
    check swingMotor.swingState == MotorState.Position
    check swingMotor.twistState == MotorState.Velocity
    checkNear(swingMotor.targetAngularVelocity.y, 0.1)
    checkNear(swingMotor.targetOrientation.z, targetOrientation.z)
    for _ in 0 ..< 30:
      discard world.step(dt)
    check classify(swingRagdoll.constraintSolverImpulse(
      2).motorRotation.z) notin {fcNan, fcInf, fcNegInf}

    var invalid = motorSettings
    invalid.minTorque = 1
    invalid.maxTorque = -1
    expect ValueError:
      swingRagdoll.configureSwingMotor(2, invalid)
    expect ValueError:
      discard swingRagdoll.motor(0)

  test "ragdoll config applies additional constraint motor presets":
    let world = testWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var settings = defaultMotorSettings()
    settings.spring = springSettings(7, 0.8)
    settings.minForce = -40
    settings.maxForce = 40
    settings.minTorque = -25
    settings.maxTorque = 25

    let scalarPreset = ragdollScalarMotorPreset(
      MotorState.PositionAndVelocity, targetVelocity = 0.2,
      targetPosition = 0.1, settings = settings)
    let swingPreset = ragdollSwingTwistMotorPreset(
      MotorState.Position, MotorState.Velocity,
      targetAngularVelocity = vec3(0, 0.15, 0),
      targetOrientation = quatFromAxisAngle(vec3(0, 0, 1), 0.12),
      swingSettings = settings, twistSettings = settings)
    var sixPreset = defaultRagdollSixDOFMotorPreset()
    sixPreset.settings[SixDOFAxis.TranslationX] = settings
    sixPreset.states[SixDOFAxis.TranslationX] = MotorState.Position
    sixPreset.targetPosition = vec3(0.16, 0, 0)
    var axes = defaultSixDOFConfig()
    axes.limits[SixDOFAxis.TranslationX] = limitedAxis(-0.25, 0.25)

    var config = chainConfig(groupId = 17)
    config.stabilize = false
    config.hingeConstraints.add(ragdollHingeConstraint(
      0, 2, vec3(0, 4, 0), vec3(0, 0, 1), vec3(1, 0, 0),
      minAngle = -0.4, maxAngle = 0.4, motor = some(scalarPreset)))
    config.sliderConstraints.add(ragdollSliderConstraint(
      0, 2, vec3(0, 4, 0), vec3(1, 0, 0), vec3(0, 1, 0),
      minPosition = -0.3, maxPosition = 0.3,
      motor = some(scalarPreset)))
    config.swingTwistConstraints.add(ragdollSwingTwistConstraint(
      0, 2, vec3(0, 4, 0), twistAxis = vec3(0, 1, 0),
      planeAxis = vec3(0, 0, 1), motor = some(swingPreset)))
    config.sixDOFConstraints.add(ragdollSixDOFConstraint(
      0, 2, vec3(0, 4, 0), config = axes,
      motor = some(sixPreset)))
    let ragdoll = world.addRagdoll(config)
    check ragdoll.constraintCount == 6

    let hinge = ragdoll.motor(2)
    check hinge.state == MotorState.PositionAndVelocity
    checkNear(hinge.targetVelocity, 0.2)
    checkNear(hinge.targetPosition, 0.1)
    checkNear(ragdoll.motorSettings(2).maxTorque, 25)
    let slider = ragdoll.motor(3)
    check slider.state == MotorState.PositionAndVelocity
    checkNear(ragdoll.motorSettings(3).maxForce, 40)

    let swing = ragdoll.swingTwistMotor(4)
    check swing.swingState == MotorState.Position
    check swing.twistState == MotorState.Velocity
    checkNear(swing.targetAngularVelocity.y, 0.15)
    checkNear(ragdoll.swingMotorSettings(4).spring.value, 7)
    let six = ragdoll.sixDOFMotor(5, SixDOFAxis.TranslationX)
    check six.state == MotorState.Position
    checkNear(six.targetPosition.x, 0.16)
    checkNear(ragdoll.sixDOFAxisMotorSettings(
      5, SixDOFAxis.TranslationX).maxForce, 40)

    var invalid = settings
    invalid.minForce = 2
    invalid.maxForce = -2
    var bad = chainConfig(groupId = 18)
    bad.hingeConstraints.add(ragdollHingeConstraint(
      0, 2, vec3(0, 4, 0), vec3(0, 0, 1), vec3(1, 0, 0),
      motor = some(ragdollScalarMotorPreset(
        MotorState.Position, settings = invalid))))
    expect ValueError:
      discard world.addRagdoll(bad)

  test "lifetime is idempotent and world ownership invalidates the handle":
    let world = testWorld()
    let ragdoll = world.addRagdoll(chainConfig())
    check ragdoll.isAlive
    ragdoll.close()
    ragdoll.close()
    check not ragdoll.isAlive
    expect JoltError:
      discard ragdoll.partCount

    let second = world.addRagdoll(chainConfig(groupId = 8))
    world.close()
    world.close()
    check not second.isAlive
    second.close()

  test "invalid hierarchy, axes, limits and poses are rejected":
    let world = testWorld()
    defer: world.close()
    var bad = chainConfig()
    bad.parts[1].joint.parent = 2
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.parts[1].joint.planeAxis = bad.parts[1].joint.twistAxis
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.parts[1].joint.twistMinAngle = 1
    bad.parts[1].joint.twistMaxAngle = -1
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.distanceConstraints.add(ragdollDistanceConstraint(
      0, 3, vec3(0, 0, 0), vec3(0, 0, 0), 0, 1))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = mixedJointConfig()
    bad.parts[3].joint.twistAxis = vec3(0, 0, 0)
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = sliderRagdollConfig()
    bad.parts[1].joint.twistMinAngle = 0.1
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = sixDOFRagdollConfig()
    bad.parts[1].joint.linearFriction.x = -1
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.distanceConstraints.add(ragdollDistanceConstraint(
      0, 2, vec3(0, 0, 0), vec3(0, 0, 0), 2, 1))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.pointConstraints.add(ragdollPointConstraint(0, 0, vec3(0, 4, 0)))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.fixedConstraints.add(ragdollFixedConstraint(0, 3))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.coneConstraints.add(ragdollConeConstraint(
      0, 2, vec3(0, 4, 0), vec3(0, 0, 0), vec3(0, 1, 0), 0.4))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.hingeConstraints.add(ragdollHingeConstraint(
      0, 2, vec3(0, 4, 0), vec3(0, 0, 1), vec3(0, 0, 1),
      minAngle = -0.3, maxAngle = 0.3))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.sliderConstraints.add(ragdollSliderConstraint(
      0, 2, vec3(0, 4, 0), vec3(1, 0, 0), vec3(0, 1, 0),
      minPosition = -0.2, maxPosition = -0.1))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    bad.swingTwistConstraints.add(ragdollSwingTwistConstraint(
      0, 2, vec3(0, 4, 0), normalHalfConeAngle = PI.float32 + 0.1))
    expect ValueError:
      discard world.addRagdoll(bad)
    bad = chainConfig()
    var badAxes = defaultSixDOFConfig()
    badAxes.axisY = badAxes.axisX
    bad.sixDOFConstraints.add(ragdollSixDOFConstraint(
      0, 2, vec3(0, 4, 0), config = badAxes))
    expect ValueError:
      discard world.addRagdoll(bad)

    let ragdoll = world.addRagdoll(chainConfig())
    expect ValueError:
      ragdoll.setPose(@[RagdollTransform(
        position: vec3(0, 0, 0), rotation: quatIdentity())])
    expect JoltError:
      ragdoll.driveKinematic(ragdoll.pose, dt)
    expect ValueError:
      ragdoll.driveMotors(ragdoll.pose, ragdoll.pose, 0)
    expect IndexDefect:
      ragdoll.addPartImpulse(10, vec3(1, 0, 0))
