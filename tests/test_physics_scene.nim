import std/[math, options, strutils, unittest]
import jolt

const
  dt = 1.0'f32 / 60.0'f32
  authoredBoxScene = """TOS 1.00

declare PhysicsScene 1
  mBodies array instance BodyCreationSettings

declare BodyCreationSettings 5
  mPosition vec3
  mRotation quat
  mShape instance ShapeSettings
  mObjectLayer uint32
  mMotionType uint32

declare ShapeSettings 0

declare BoxShapeSettings 2
  mHalfExtent vec3
  mConvexRadius float

object PhysicsScene 00000001
  1
      0 0 0
      0 0 0 1
      00000002
      0
      0

object BoxShapeSettings 00000002
  3 0.5 3
  0.05
"""

proc checkNear(actual, expected: float32; epsilon = 1.0e-3'f32) =
  check abs(actual - expected) <= epsilon

proc checkMotorSettings(actual, expected: MotorSettings) =
  check actual.spring.mode == expected.spring.mode
  checkNear(actual.spring.value, expected.spring.value)
  checkNear(actual.spring.damping, expected.spring.damping)
  checkNear(actual.minForce, expected.minForce)
  checkNear(actual.maxForce, expected.maxForce)
  checkNear(actual.minTorque, expected.minTorque)
  checkNear(actual.maxTorque, expected.maxTorque)

proc captureRigidScene(): PhysicsScene =
  let world = newWorld()
  let floor = world.addStaticBody(
    boxShape(vec3(5, 0.5, 5)), vec3(0, -0.5, 0))
  let first = world.addDynamicBody(
    sphereShape(0.3), vec3(-0.6, 3, 0))
  let second = world.addDynamicBody(
    sphereShape(0.3), vec3(0.6, 3, 0))
  first.setLinearVelocity(vec3(0.2, 0, 0))
  let link = first.addDistanceConstraint(
    second, first.position, second.position, 1.1, 1.3)
  result = world.capturePhysicsScene()
  link.close()
  first.close()
  second.close()
  floor.close()
  world.close()

suite "Jolt physics scene serialization":
  test "binary scenes round trip bodies and constraints":
    let scene = captureRigidScene()
    defer: scene.close()
    check scene.rigidBodyCount == 3
    check scene.softBodyCount == 0
    check scene.totalBodyCount == 3
    check scene.constraintCount == 1
    check scene.fixInvalidScales()

    let data = scene.serialize()
    check data.len > 16
    let restored = restorePhysicsScene(data)
    check restored.rigidBodyCount == 3
    check restored.softBodyCount == 0
    check restored.constraintCount == 1
    restored.close()

  test "text and binary ObjectStreams round trip and auto-detect":
    let fromText = restorePhysicsSceneObjectStream(authoredBoxScene)
    check fromText.rigidBodyCount == 1
    check fromText.constraintCount == 0
    check fromText.objectStreamSerializable

    let target = newWorld()
    defer: target.close()
    let instance = fromText.instantiate(target)
    check instance.bodyCount == 1
    check instance.bodyMotionType(0) == MotionType.Static
    let floorHit = target.castRay(
      vec3(0, 3, 0), vec3(0, -1, 0), 6).get
    check floorHit.bodyId == instance.bodyId(0)
    instance.close()

    let text = fromText.serializeObjectStreamText()
    check text.len > 100
    check text.contains("PhysicsScene")
    let textAgain = restorePhysicsSceneObjectStream(text)
    check textAgain.rigidBodyCount == 1
    textAgain.close()

    let binary = fromText.serializeObjectStream(PhysicsSceneStreamBinary)
    check binary.len > 32
    let fromBinary = restorePhysicsSceneObjectStream(binary)
    check fromBinary.rigidBodyCount == 1
    check fromBinary.constraintCount == 0
    fromBinary.close()
    fromText.close()

  test "all Nim-authored ShapeSettings families survive ObjectStream round trips":
    var movingConfig = defaultBodyConfig()
    movingConfig.linearVelocity = vec3(0.25, 0, 0)
    movingConfig.friction = 0.7
    movingConfig.userData = 0x1234'u64
    let specs = [
      staticBodySpec(boxShape(vec3(5, 0.5, 5)), vec3(0, -0.5, 0)),
      dynamicBodySpec(sphereShape(0.35), vec3(-2.5, 2, 0),
                      config = movingConfig),
      dynamicBodySpec(capsuleShape(0.4, 0.25), vec3(-1.5, 2.2, 0)),
      dynamicBodySpec(cylinderShape(0.4, 0.3), vec3(-0.5, 2.4, 0)),
      dynamicBodySpec(
        taperedCapsuleShape(0.35, 0.22, 0.3), vec3(0.7, 2.6, 0)),
      dynamicBodySpec(
        taperedCylinderShape(0.4, 0.24, 0.34), vec3(1.9, 2.8, 0)),
      dynamicBodySpec(
        triangleShape(vec3(-0.4, 0, -0.3), vec3(0.4, 0, -0.3),
                      vec3(0, 0.7, 0.3)), vec3(3.0, 3.0, 0)),
      staticBodySpec(planeShape(vec3(0, 1, 0)), vec3(0, -5, 0)),
      staticBodySpec(emptyShape(), vec3(0, -8, 0)),
      dynamicBodySpec(
        convexHullShape([
          vec3(-0.4, -0.4, -0.4), vec3(0.4, -0.4, -0.4),
          vec3(0, 0.5, -0.2), vec3(0, 0, 0.5)]),
        vec3(4.2, 3.2, 0)),
      staticBodySpec(
        triangleMeshShape(
          [vec3(-1, 0, -1), vec3(1, 0, -1),
           vec3(1, 0, 1), vec3(-1, 0, 1)],
          [0'u32, 2'u32, 1'u32, 0'u32, 3'u32, 2'u32]),
        vec3(8, 0.2, 0)),
      staticBodySpec(
        heightFieldShape(
          [0'f32, 0.1, 0.2, 0.1,
           0.1, 0.25, 0.35, 0.2,
           0.05, 0.2, 0.3, 0.15,
           0, 0.05, 0.1, 0], 4,
          scale = vec3(0.6, 1, 0.6)),
        vec3(10, 0, 0)),
      staticBodySpec(
        staticCompoundShape([
          compoundChild(boxShape(vec3(0.6, 0.2, 0.4)), vec3(-0.5, 0, 0)),
          compoundChild(sphereShape(0.35), vec3(0.55, 0.2, 0))]),
        vec3(6.5, 0.2, 0)),
      staticBodySpec(
        mutableCompoundShape([
          compoundChild(boxShape(vec3(0.25, 0.25, 0.25)), vec3(-0.3, 0, 0)),
          compoundChild(boxShape(vec3(0.25, 0.25, 0.25)), vec3(0.3, 0, 0))]),
        vec3(-4, 4, 1.5)),
      staticBodySpec(
        scaledShape(boxShape(vec3(0.3, 0.3, 0.3)), vec3(1.4, 0.8, 1)),
        vec3(-2.5, 4.2, 1.5)),
      staticBodySpec(
        rotatedTranslatedShape(
          boxShape(vec3(0.25, 0.45, 0.25)), vec3(0.2, 0, 0),
          quatFromAxisAngle(vec3(0, 0, 1), 0.35)),
        vec3(-1, 4.4, 1.5)),
      staticBodySpec(
        offsetCenterOfMassShape(
          boxShape(vec3(0.35, 0.35, 0.35)), vec3(0, -0.15, 0)),
        vec3(0.5, 4.6, 1.5))
    ]
    let authored = newPhysicsScene(specs)
    defer: authored.close()
    check authored.rigidBodyCount == specs.len
    check authored.objectStreamSerializable

    let text = authored.serializeObjectStreamText()
    check text.contains("TaperedCapsuleShapeSettings")
    check text.contains("TaperedCylinderShapeSettings")
    check text.contains("TriangleShapeSettings")
    check text.contains("PlaneShapeSettings")
    check text.contains("EmptyShapeSettings")
    check text.contains("ConvexHullShapeSettings")
    check text.contains("MeshShapeSettings")
    check text.contains("HeightFieldShapeSettings")
    check text.contains("StaticCompoundShapeSettings")
    check text.contains("MutableCompoundShapeSettings")
    check text.contains("ScaledShapeSettings")
    check text.contains("RotatedTranslatedShapeSettings")
    check text.contains("OffsetCenterOfMassShapeSettings")
    let restored = restorePhysicsSceneObjectStream(text)
    defer: restored.close()
    check restored.rigidBodyCount == specs.len
    check restored.objectStreamSerializable

    let world = newWorld()
    defer: world.close()
    let instance = restored.instantiate(world)
    defer: instance.close()
    check instance.bodyCount == specs.len
    check instance.bodyMotionType(0) == MotionType.Static
    for index in [1, 2, 3, 4, 5, 6, 9]:
      check instance.bodyMotionType(index) == MotionType.Dynamic
    for index in [0, 7, 8, 10, 11, 12, 13, 14, 15, 16]:
      check instance.bodyMotionType(index) == MotionType.Static
    checkNear(instance.bodyLinearVelocity(1).x, 0.25)
    for _ in 0 ..< 30:
      discard world.step(dt)
    check instance.bodyPosition(1).y < 2

  test "authored body batches validate before changing the scene":
    let scene = newPhysicsScene()
    defer: scene.close()
    let valid = staticBodySpec(boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))
    let unsupported = dynamicBodySpec(
      planeShape(vec3(0, 1, 0)), vec3(0, 0, 0))
    expect ValueError:
      discard scene.addBodies([valid, unsupported])
    check scene.rigidBodyCount == 0
    check scene.addBody(valid) == 0
    check scene.rigidBodyCount == 1
    check scene.objectStreamSerializable

  test "authored compound and decorated shapes simulate dynamically":
    let shapes = [
      mutableCompoundShape([
        compoundChild(boxShape(vec3(0.25, 0.25, 0.25)), vec3(-0.3, 0, 0)),
        compoundChild(boxShape(vec3(0.25, 0.25, 0.25)), vec3(0.3, 0, 0))]),
      scaledShape(boxShape(vec3(0.3, 0.3, 0.3)), vec3(1.4, 0.8, 1)),
      rotatedTranslatedShape(
        boxShape(vec3(0.25, 0.45, 0.25)), vec3(0.2, 0, 0),
        quatFromAxisAngle(vec3(0, 0, 1), 0.35)),
      offsetCenterOfMassShape(
        boxShape(vec3(0.35, 0.35, 0.35)), vec3(0, -0.15, 0))
    ]
    let names = [
      "mutable compound", "scaled", "rotated-translated", "offset COM"]
    for index, shape in shapes:
      checkpoint(names[index])
      let authored = newPhysicsScene([
        staticBodySpec(boxShape(vec3(3, 0.5, 3)), vec3(0, -0.5, 0)),
        dynamicBodySpec(shape, vec3(0, 2, 0))])
      let data = authored.serializeObjectStream(PhysicsSceneStreamBinary)
      let restored = restorePhysicsSceneObjectStream(data)
      let world = newWorld()
      let instance = restored.instantiate(world)
      for _ in 0 ..< 30:
        discard world.step(dt)
      let position = instance.bodyPosition(1)
      check not position.x.isNaN
      check not position.y.isNaN
      check not position.z.isNaN
      check position.y < 2
      instance.close()
      world.close()
      restored.close()
      authored.close()

  test "authored materials retain names colors and sub-shape mappings":
    let wood = physicsMaterial(
      "authored wood", materialColor(155, 95, 45, 230))
    let metal = physicsMaterial(
      "authored metal", materialColor(165, 180, 195, 240))
    let compound = staticCompoundShape([
      compoundChild(
        boxShape(vec3(0.7, 0.5, 0.7)).withMaterial(wood), vec3(-1, 0, 0)),
      compoundChild(
        sphereShape(0.7).withMaterial(metal), vec3(1, 0, 0))])
    let mesh = triangleMeshShape(
      [
        vec3(-4, 0, -2), vec3(0, 0, -2),
        vec3(0, 0, 2), vec3(-4, 0, 2),
        vec3(0, 0, -2), vec3(4, 0, -2),
        vec3(4, 0, 2), vec3(0, 0, 2)
      ],
      [0'u32, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6]
    ).withMaterials([wood, metal], [0'u32, 0, 1, 1])
    var terrainIndices = newSeq[uint32](9)
    for z in 0 ..< 3:
      for x in 0 ..< 3:
        terrainIndices[x + z * 3] = if x == 0: 0 else: 1
    let terrain = heightFieldShape(
      newSeq[float32](16), 4,
      offset = vec3(-1.5, 0, -1.5)).withMaterials(
        [wood, metal], terrainIndices)
    let authored = newPhysicsScene([
      staticBodySpec(compound, vec3(0, 0, 0)),
      staticBodySpec(mesh, vec3(10, 0, 0)),
      staticBodySpec(terrain, vec3(20, 0, 0))])
    defer: authored.close()
    let restored = restorePhysicsSceneObjectStream(
      authored.serializeObjectStream(PhysicsSceneStreamBinary))
    defer: restored.close()
    let world = newWorld()
    defer: world.close()
    let instance = restored.instantiate(world)
    defer: instance.close()

    for (x, expected) in [
        (-1'f32, wood), (1'f32, metal),
        (8'f32, wood), (12'f32, metal),
        (19'f32, wood), (21'f32, metal)]:
      let hit = world.castRay(
        vec3(x, 4, 0), vec3(0, -1, 0), 8).get
      check hit.material(world).get == expected

  test "authored common constraints round trip and solve":
    var specs = @[staticBodySpec(
      boxShape(vec3(0.3, 0.3, 0.3)), vec3(0, 5, 0))]
    for x in [-7'f32, -5, -3, -1, 1, 3, 5, 7]:
      specs.add dynamicBodySpec(sphereShape(0.25), vec3(x, 3, 0))
    let scene = newPhysicsScene(specs)
    defer: scene.close()
    check scene.addPointConstraint(
      0, 1, vec3(-7, -2, 0), vec3(0, 0, 0)) == 0
    check scene.addDistanceConstraint(
      0, 2, vec3(-5, -2, 0), vec3(0, 0, 0), 0, 0) == 1
    check scene.addFixedConstraint(
      0, 3, vec3(-3, -2, 0), vec3(0, 0, 0),
      vec3(1, 0, 0), vec3(0, 1, 0),
      vec3(1, 0, 0), vec3(0, 1, 0)) == 2
    check scene.addHingeConstraint(
      0, 4, vec3(-1, -2, 0), vec3(0, 0, 0),
      vec3(0, 0, 1), vec3(0, 0, 1), -0.5, 0.5) == 3
    check scene.addSliderConstraint(
      0, 5, vec3(1, -2, 0), vec3(0, 0, 0),
      vec3(0, 1, 0), -0.5, 0.5) == 4
    check scene.addConeConstraint(
      0, 6, vec3(3, -2, 0), vec3(0, 0, 0),
      vec3(0, 1, 0), vec3(0, 1, 0), 0.5) == 5
    check scene.addSwingTwistConstraint(
      0, 7, vec3(5, -2, 0), vec3(0, 0, 0),
      vec3(0, 1, 0), vec3(1, 0, 0),
      0.5, 0.5, -0.4, 0.4) == 6
    var sixDOFConfig = defaultSixDOFConfig()
    sixDOFConfig.swingType = SixDOFSwingType.SwingCone
    sixDOFConfig.limits[SixDOFAxis.RotationY] = limitedAxis(-0.4, 0.4)
    sixDOFConfig.limits[SixDOFAxis.RotationZ] = limitedAxis(-0.3, 0.3)
    check scene.addSixDOFConstraint(
      0, 8, vec3(7, -2, 0), vec3(0, 0, 0), sixDOFConfig) == 7
    check scene.constraintCount == 8
    let hingeSpring = springSettings(3.25, 0.65)
    let sliderSpring = springSettings(
      5000, 100, SpringMode.StiffnessAndDamping)
    let sixDOFSpring = springSettings(
      6, 0.7, SpringMode.MassNormalizedStiffnessAndDamping)
    let distanceSpring = springSettings(
      10000, 200, SpringMode.StiffnessAndDamping)
    var hingeMotor = defaultMotorSettings()
    hingeMotor.spring = springSettings(4, 0.8)
    hingeMotor.minTorque = -12
    hingeMotor.maxTorque = 15
    var sliderMotor = defaultMotorSettings()
    sliderMotor.spring = springSettings(
      4000, 80, SpringMode.StiffnessAndDamping)
    sliderMotor.minForce = -30
    sliderMotor.maxForce = 45
    var swingMotor = defaultMotorSettings()
    swingMotor.spring = springSettings(5, 0.9)
    swingMotor.minTorque = -9
    swingMotor.maxTorque = 11
    var twistMotor = defaultMotorSettings()
    twistMotor.spring = springSettings(6, 1)
    twistMotor.minTorque = -7
    twistMotor.maxTorque = 8
    var sixDOFMotor = defaultMotorSettings()
    sixDOFMotor.spring = springSettings(
      15, 2, SpringMode.MassNormalizedStiffnessAndDamping)
    sixDOFMotor.minForce = -20
    sixDOFMotor.maxForce = 25
    scene.configureAuthoredHingeTuning(3, 4.5, hingeSpring)
    scene.configureAuthoredDistanceSpring(1, distanceSpring)
    scene.configureAuthoredSliderTuning(4, 8, sliderSpring)
    scene.setAuthoredSwingTwistFriction(6, 3)
    scene.setAuthoredSixDOFFriction(
      7, SixDOFAxis.TranslationX, 6)
    scene.setAuthoredSixDOFFriction(
      7, SixDOFAxis.RotationZ, 2.5)
    scene.setAuthoredSixDOFTranslationSpring(
      7, SixDOFAxis.TranslationX, sixDOFSpring)
    scene.configureAuthoredHingeMotor(3, hingeMotor)
    scene.configureAuthoredSliderMotor(4, sliderMotor)
    scene.configureAuthoredSwingMotor(6, swingMotor)
    scene.configureAuthoredTwistMotor(6, twistMotor)
    scene.configureAuthoredSixDOFMotor(
      7, SixDOFAxis.TranslationX, sixDOFMotor)
    expect ValueError:
      discard scene.addFixedConstraint(0, 0)
    expect ValueError:
      discard scene.addFixedConstraint(0, 99)
    check scene.constraintCount == 8

    let text = scene.serializeObjectStreamText()
    for settingsName in [
        "PointConstraintSettings", "DistanceConstraintSettings",
        "FixedConstraintSettings", "HingeConstraintSettings",
        "SliderConstraintSettings", "ConeConstraintSettings",
        "SwingTwistConstraintSettings", "SixDOFConstraintSettings"]:
      check text.contains(settingsName)
    check text.contains("mMaxFrictionTorque")
    check text.contains("mMaxFrictionForce")
    check text.contains("mLimitsSpringSettings")
    check text.contains("mMaxFriction")
    check text.contains("mSwingType")
    check text.contains("mMotorSettings")
    check text.contains("mSwingMotorSettings")
    check text.contains("mTwistMotorSettings")
    let restored = restorePhysicsSceneObjectStream(text)
    defer: restored.close()
    check restored.constraintCount == 8
    let world = newWorld()
    defer: world.close()
    let instance = restored.instantiate(world)
    defer: instance.close()
    check instance.constraintCount == 8
    check instance.sixDOFSwingType(7) == SixDOFSwingType.SwingCone
    check instance.axisLimit(7, SixDOFAxis.RotationY) == limitedAxis(-0.4, 0.4)
    check instance.axisLimit(7, SixDOFAxis.TranslationY).mode ==
      SixDOFAxisMode.AxisFixed
    instance.setAxisLimit(7, SixDOFAxis.TranslationY, freeAxis())
    check instance.axisLimit(7, SixDOFAxis.TranslationY).mode ==
      SixDOFAxisMode.AxisFree
    instance.setAxisLimit(
      7, SixDOFAxis.TranslationY, limitedAxis(-0.25, 0.5))
    check instance.axisLimit(7, SixDOFAxis.TranslationY) ==
      limitedAxis(-0.25, 0.5)
    instance.setAxisLimit(7, SixDOFAxis.TranslationY, fixedAxis())
    expect ValueError:
      instance.setAxisLimit(
        7, SixDOFAxis.RotationY, limitedAxis(-0.2, 0.3))
    checkNear(instance.constraintFriction(3), 4.5)
    checkNear(instance.constraintFriction(4), 8)
    checkNear(instance.constraintFriction(6), 3)
    checkNear(instance.sixDOFConstraintFriction(
      7, SixDOFAxis.TranslationX), 6)
    checkNear(instance.sixDOFConstraintFriction(
      7, SixDOFAxis.RotationZ), 2.5)
    instance.setAxisFriction(7, SixDOFAxis.RotationZ, 4.5)
    checkNear(instance.sixDOFConstraintFriction(
      7, SixDOFAxis.RotationZ), 4.5)
    let restoredHingeSpring = instance.constraintLimitSpring(3)
    check restoredHingeSpring.mode == hingeSpring.mode
    checkNear(restoredHingeSpring.value, hingeSpring.value)
    checkNear(restoredHingeSpring.damping, hingeSpring.damping)
    let restoredDistanceSpring = instance.constraintLimitSpring(1)
    check restoredDistanceSpring.mode == distanceSpring.mode
    checkNear(restoredDistanceSpring.value, distanceSpring.value)
    checkNear(restoredDistanceSpring.damping, distanceSpring.damping)
    check instance.distanceLimits(1) == (0'f32, 0'f32)
    instance.setDistanceLimits(1, 0.1, 0.2)
    check instance.distanceLimits(1) == (0.1'f32, 0.2'f32)
    instance.setDistanceLimitSpring(1, springSettings(9, 0.9))
    check instance.constraintLimitSpring(1).mode ==
      SpringMode.FrequencyAndDamping
    checkNear(instance.constraintLimitSpring(1).value, 9)
    instance.setDistanceLimits(1, 0, 0)
    instance.setDistanceLimitSpring(1, distanceSpring)
    let restoredSliderSpring = instance.constraintLimitSpring(4)
    check restoredSliderSpring.mode == sliderSpring.mode
    checkNear(restoredSliderSpring.value, sliderSpring.value)
    checkNear(restoredSliderSpring.damping, sliderSpring.damping)
    let restoredSixDOFSpring = instance.sixDOFConstraintLimitSpring(
      7, SixDOFAxis.TranslationX)
    check restoredSixDOFSpring.mode == sixDOFSpring.mode
    checkNear(restoredSixDOFSpring.value, sixDOFSpring.value)
    checkNear(restoredSixDOFSpring.damping, sixDOFSpring.damping)
    let runtimeSixDOFSpring = springSettings(
      1750, 55, SpringMode.StiffnessAndDamping)
    instance.setAxisLimitSpring(
      7, SixDOFAxis.TranslationX, runtimeSixDOFSpring)
    let changedSixDOFSpring = instance.sixDOFConstraintLimitSpring(
      7, SixDOFAxis.TranslationX)
    check changedSixDOFSpring.mode == runtimeSixDOFSpring.mode
    checkNear(changedSixDOFSpring.value, runtimeSixDOFSpring.value)
    checkNear(changedSixDOFSpring.damping, runtimeSixDOFSpring.damping)
    checkMotorSettings(instance.motorSettings(3), hingeMotor)
    checkMotorSettings(instance.motorSettings(4), sliderMotor)
    checkMotorSettings(instance.swingMotorSettings(6), swingMotor)
    checkMotorSettings(instance.twistMotorSettings(6), twistMotor)
    checkMotorSettings(instance.sixDOFAxisMotorSettings(
      7, SixDOFAxis.TranslationX), sixDOFMotor)

    instance.setMotor(3, MotorState.Velocity, 1.25, 0.1)
    check instance.motor(3) == (MotorState.Velocity, 1.25'f32, 0.1'f32)
    instance.setMotor(4, MotorState.Position, 0.5, 0.2)
    check instance.motor(4) == (MotorState.Position, 0.5'f32, 0.2'f32)
    instance.setSwingTwistMotorTargets(
      6, vec3(0.1, 0.2, 0.3), quatIdentity())
    instance.setSwingMotorState(6, MotorState.Velocity)
    instance.setTwistMotorState(6, MotorState.Position)
    let restoredSwingTwistMotor = instance.swingTwistMotor(6)
    check restoredSwingTwistMotor.swingState == MotorState.Velocity
    check restoredSwingTwistMotor.twistState == MotorState.Position
    check restoredSwingTwistMotor.targetAngularVelocity == vec3(0.1, 0.2, 0.3)
    check restoredSwingTwistMotor.targetOrientation == quatIdentity()
    instance.setSixDOFMotorTargets(
      7, vec3(0.4, 0, 0), vec3(0, 0.3, 0),
      vec3(0.2, 0, 0), quatIdentity())
    instance.setAxisMotorState(
      7, SixDOFAxis.TranslationX, MotorState.PositionAndVelocity)
    let restoredSixDOFMotor = instance.sixDOFMotor(
      7, SixDOFAxis.TranslationX)
    check restoredSixDOFMotor.state == MotorState.PositionAndVelocity
    check restoredSixDOFMotor.targetVelocity == vec3(0.4, 0, 0)
    check restoredSixDOFMotor.targetAngularVelocity == vec3(0, 0.3, 0)
    check restoredSixDOFMotor.targetPosition == vec3(0.2, 0, 0)
    check restoredSixDOFMotor.targetOrientation == quatIdentity()
    instance.setMotor(3, MotorState.Disabled, 0, 0)
    instance.setMotor(4, MotorState.Disabled, 0, 0)
    instance.setSwingMotorState(6, MotorState.Disabled)
    instance.setTwistMotorState(6, MotorState.Disabled)
    instance.setAxisMotorState(
      7, SixDOFAxis.TranslationX, MotorState.Disabled)
    for _ in 0 ..< 60:
      check world.step(dt) == {}
    for bodyIndex in 1 ..< instance.bodyCount:
      let position = instance.bodyPosition(bodyIndex)
      check not position.x.isNaN
      check not position.y.isNaN
      check not position.z.isNaN
      check position.y > 1

  test "authored constraint tuning rejects mismatched families and values":
    let scene = newPhysicsScene([
      staticBodySpec(sphereShape(0.25), vec3(0, 4, 0)),
      dynamicBodySpec(sphereShape(0.25), vec3(-1, 3, 0)),
      dynamicBodySpec(sphereShape(0.25), vec3(1, 3, 0))])
    defer: scene.close()
    check scene.addHingeConstraint(
      0, 1, vec3(-1, -1, 0), vec3(0, 0, 0),
      vec3(0, 0, 1), vec3(0, 0, 1), -0.5, 0.5) == 0
    check scene.addSixDOFConstraint(
      0, 2, vec3(1, -1, 0), vec3(0, 0, 0)) == 1

    expect ValueError:
      scene.configureAuthoredSliderTuning(0, 1, springSettings(2, 1))
    expect ValueError:
      scene.configureAuthoredDistanceSpring(0, springSettings(2, 1))
    expect ValueError:
      scene.setAuthoredSwingTwistFriction(0, 1)
    expect ValueError:
      scene.setAuthoredSixDOFFriction(0, SixDOFAxis.TranslationX, 1)
    expect ValueError:
      scene.configureAuthoredHingeTuning(0, -1, springSettings(2, 1))
    expect ValueError:
      scene.setAuthoredSixDOFTranslationSpring(
        1, SixDOFAxis.RotationX, springSettings(2, 1))
    expect IndexDefect:
      scene.setAuthoredSwingTwistFriction(99, 1)
    expect ValueError:
      discard scene.addFixedConstraint(
        0, 1, vec3(0, 0, 0), vec3(0, 0, 0),
        vec3(1, 0, 0), vec3(1, 0, 0),
        vec3(1, 0, 0), vec3(0, 1, 0))

  test "authored distance spring produces a softer native response":
    let scene = newPhysicsScene([
      staticBodySpec(sphereShape(0.1), vec3(0, 2, 0)),
      dynamicBodySpec(sphereShape(0.1), vec3(1, 2, 0)),
      dynamicBodySpec(sphereShape(0.1), vec3(-1, 2, 0))])
    defer: scene.close()
    check scene.addDistanceConstraint(
      0, 1, vec3(0, 0, 0), vec3(0, 0, 0), 1, 1) == 0
    check scene.addDistanceConstraint(
      0, 2, vec3(0, 0, 0), vec3(0, 0, 0), 1, 1) == 1
    scene.configureAuthoredDistanceSpring(1, springSettings(1, 0.2))

    let restored = restorePhysicsSceneObjectStream(
      scene.serializeObjectStream(PhysicsSceneStreamBinary))
    defer: restored.close()
    let world = newWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let instance = restored.instantiate(world)
    defer: instance.close()
    instance.setBodyLinearVelocity(1, vec3(4, 0, 0))
    instance.setBodyLinearVelocity(2, vec3(-4, 0, 0))
    for _ in 0 ..< 15:
      check world.step(dt) == {}
    let hardDistance = abs(instance.bodyPosition(1).x)
    let softDistance = abs(instance.bodyPosition(2).x)
    check hardDistance < 1.05
    check softDistance > hardDistance + 0.1

  test "authored and instance motors validate families and limits":
    let scene = newPhysicsScene([
      staticBodySpec(sphereShape(0.25), vec3(0, 4, 0)),
      dynamicBodySpec(sphereShape(0.25), vec3(-1, 3, 0)),
      dynamicBodySpec(sphereShape(0.25), vec3(1, 3, 0))])
    defer: scene.close()
    check scene.addHingeConstraint(
      0, 1, vec3(-1, -1, 0), vec3(0, 0, 0),
      vec3(0, 0, 1), vec3(0, 0, 1), -0.5, 0.5) == 0
    check scene.addSixDOFConstraint(
      0, 2, vec3(1, -1, 0), vec3(0, 0, 0)) == 1

    var invalidMotor = defaultMotorSettings()
    invalidMotor.minTorque = 2
    invalidMotor.maxTorque = -2
    expect ValueError:
      scene.configureAuthoredHingeMotor(0, invalidMotor)
    expect ValueError:
      scene.configureAuthoredSliderMotor(0, defaultMotorSettings())
    expect ValueError:
      scene.configureAuthoredSwingMotor(1, defaultMotorSettings())
    expect ValueError:
      scene.configureAuthoredPathMotor(1, defaultMotorSettings())
    expect IndexDefect:
      scene.configureAuthoredHingeMotor(99, defaultMotorSettings())

    scene.configureAuthoredHingeMotor(0, defaultMotorSettings())
    scene.configureAuthoredSixDOFMotor(
      1, SixDOFAxis.RotationY, defaultMotorSettings())
    let restored = restorePhysicsSceneObjectStream(
      scene.serializeObjectStream(PhysicsSceneStreamBinary))
    defer: restored.close()
    let world = newWorld()
    defer: world.close()
    let instance = restored.instantiate(world)
    defer: instance.close()
    expect ValueError:
      instance.setPathMotor(0, MotorState.Velocity, 1, 0)
    expect ValueError:
      instance.setMotor(1, MotorState.Velocity, 1, 0)
    expect ValueError:
      instance.setSwingMotorState(1, MotorState.Velocity)
    expect ValueError:
      instance.setSixDOFMotorTargets(
        0, vec3(0, 0, 0), vec3(0, 0, 0),
        vec3(0, 0, 0), quatIdentity())
    expect IndexDefect:
      discard instance.motorSettings(99)

  test "authored specialized constraints round trip and solve":
    let scene = newPhysicsScene([
      dynamicBodySpec(sphereShape(0.2), vec3(-9, 3, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(-7, 3, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(-3, 3, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(-1, 3, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(3, 3, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(5, 3, 0)),
      staticBodySpec(sphereShape(0.2), vec3(9, 0, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(9, 2, 0))])
    defer: scene.close()

    check scene.addGearConstraint(
      0, 1, vec3(0, 0, 1), vec3(0, 0, 1), 2) == 0
    check scene.addPulleyConstraint(
      2, 3,
      vec3(-3, 3, 0), vec3(-3, 5, 0),
      vec3(-1, 3, 0), vec3(-1, 5, 0),
      ratio = 1, minLength = 0, maxLength = 4) == 1
    check scene.addRackAndPinionConstraint(
      4, 5, vec3(0, 0, 1), vec3(1, 0, 0), 1.5) == 2
    check scene.addPathConstraint(
      6, 7,
      [PathPoint(position: vec3(0, 2, 0),
                 tangent: vec3(2, 0, 0), normal: vec3(0, 1, 0)),
       PathPoint(position: vec3(2, 2.5, 0),
                 tangent: vec3(2, 0, 0), normal: vec3(0, 1, 0)),
       PathPoint(position: vec3(4, 2, 0),
                 tangent: vec3(2, 0, 0), normal: vec3(0, 1, 0))],
      maxFrictionForce = 2) == 3
    check scene.constraintCount == 4

    var pathMotor = defaultMotorSettings()
    pathMotor.spring = springSettings(7, 0.75)
    pathMotor.minForce = -18
    pathMotor.maxForce = 22
    scene.configureAuthoredPathMotor(3, pathMotor)

    var commonConfig = defaultAuthoredConstraintConfig()
    commonConfig.enabled = false
    commonConfig.priority = 73
    commonConfig.velocityStepsOverride = 7
    commonConfig.positionStepsOverride = 5
    commonConfig.drawSize = 2.5
    commonConfig.userData = 0x1122334455667788'u64
    scene.configureConstraint(1, commonConfig)

    expect ValueError:
      discard scene.addGearConstraint(
        0, 1, vec3(0, 0, 1), vec3(0, 0, 1), 0)
    expect ValueError:
      discard scene.addPulleyConstraint(
        2, 3, vec3(-3, 3, 0), vec3(-3, 5, 0),
        vec3(-1, 3, 0), vec3(-1, 5, 0), maxLength = -2)
    expect ValueError:
      discard scene.addPathConstraint(
        6, 7,
        [PathPoint(position: vec3(0, 0, 0),
                   tangent: vec3(1, 0, 0), normal: vec3(0, 1, 0))])
    expect IndexDefect:
      scene.configureConstraint(99, commonConfig)
    commonConfig.velocityStepsOverride = 256
    expect ValueError:
      scene.configureConstraint(1, commonConfig)
    check scene.constraintCount == 4

    let text = scene.serializeObjectStreamText()
    for settingsName in [
        "GearConstraintSettings", "PulleyConstraintSettings",
        "RackAndPinionConstraintSettings", "PathConstraintSettings",
        "PathConstraintPathHermite"]:
      check text.contains(settingsName)
    check text.contains("mConstraintPriority")
    check text.contains("mNumVelocityStepsOverride")
    check text.contains("mNumPositionStepsOverride")
    check text.contains("mDrawConstraintSize")
    check text.contains("mUserData")
    let restored = restorePhysicsSceneObjectStream(text)
    defer: restored.close()
    check restored.constraintCount == 4
    let world = newWorld()
    defer: world.close()
    let instance = restored.instantiate(world)
    defer: instance.close()
    check instance.constraintCount == 4
    check not instance.constraintEnabled(1)
    check instance.constraintPriority(1) == 73
    check instance.constraintSolverStepOverrides(1) == (7'u32, 5'u32)
    check instance.constraintUserData(1) == 0x1122334455667788'u64
    checkMotorSettings(instance.motorSettings(3), pathMotor)
    instance.setPathMotor(3, MotorState.Velocity, 0.25, 1)
    check instance.pathMotor(3) == (MotorState.Velocity, 0.25'f32, 1'f32)
    instance.setPathMotor(3, MotorState.Disabled, 0, 1)
    instance.setConstraintEnabled(1, true)
    instance.setConstraintPriority(1, 91)
    instance.setConstraintSolverStepOverrides(1, 4, 3)
    instance.setConstraintUserData(1, 0xaabbccdd'u64)
    instance.resetConstraintWarmStart(1)
    check instance.constraintEnabled(1)
    check instance.constraintPriority(1) == 91
    check instance.constraintSolverStepOverrides(1) == (4'u32, 3'u32)
    check instance.constraintUserData(1) == 0xaabbccdd'u64
    expect IndexDefect:
      discard instance.constraintPriority(99)
    for _ in 0 ..< 60:
      check world.step(dt) == {}
    for bodyIndex in 0 ..< instance.bodyCount:
      let position = instance.bodyPosition(bodyIndex)
      check not position.x.isNaN
      check not position.y.isNaN
      check not position.z.isNaN

    let unsafeGearScene = newPhysicsScene([
      staticBodySpec(sphereShape(0.2), vec3(0, 0, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(1, 0, 0))])
    defer: unsafeGearScene.close()
    expect JoltError:
      discard unsafeGearScene.addGearConstraint(
        0, 1, vec3(0, 0, 1), vec3(0, 0, 1), 1)

  test "authored constraints attach directly to fixed world":
    let scene = newPhysicsScene([
      dynamicBodySpec(sphereShape(0.25), vec3(0, 3, 0))])
    defer: scene.close()
    check scene.addPointConstraint(
      fixedWorldBodyIndex, 0, vec3(0, 3, 0), vec3(0, 0, 0)) == 0
    expect ValueError:
      discard scene.addPointConstraint(
        fixedWorldBodyIndex, fixedWorldBodyIndex,
        vec3(0, 3, 0), vec3(0, 0, 0))
    expect ValueError:
      discard scene.addGearConstraint(
        fixedWorldBodyIndex, 0,
        vec3(0, 0, 1), vec3(0, 0, 1), 1)

    let restored = restorePhysicsSceneObjectStream(
      scene.serializeObjectStream(PhysicsSceneStreamBinary))
    defer: restored.close()
    let world = newWorld()
    defer: world.close()
    let instance = restored.instantiate(world)
    defer: instance.close()
    check instance.bodyCount == 1
    check instance.constraintCount == 1
    check instance.constraintKind(0) == ConstraintKind.Point
    let endpoints = instance.constraintBodyIds(0)
    check endpoints.body1.isNone
    check endpoints.body2.get == instance.bodyId(0)
    for _ in 0 ..< 60:
      check world.step(dt) == {}
    checkNear(instance.bodyPosition(0).y, 3, 0.03)
    let impulse = instance.constraintSolverImpulse(0)
    check classify(impulse.position.y) notin {fcNan, fcInf, fcNegInf}

  test "captured runtime shapes reject ObjectStream output safely":
    let captured = captureRigidScene()
    defer: captured.close()
    check not captured.objectStreamSerializable
    expect JoltError:
      discard captured.serializeObjectStream()

  test "instantiated groups simulate expose state and close independently":
    let scene = captureRigidScene()
    defer: scene.close()
    let world = newWorld()
    defer: world.close()
    let existing = world.addStaticBody(
      boxShape(vec3(1, 0.2, 1)), vec3(8, 0, 0))
    let instance = scene.instantiate(world)
    check instance.bodyCount == 3
    check instance.constraintCount == 1

    var dynamicIndex = -1
    for index in 0 ..< instance.bodyCount:
      check not instance.isSoftBody(index)
      if instance.bodyMotionType(index) == MotionType.Dynamic:
        dynamicIndex = index
    check dynamicIndex >= 0
    check instance.bodyCollisionLayer(dynamicIndex) == movingLayer
    instance.setBodyTransform(dynamicIndex, vec3(0, 4, 0))
    instance.setBodyLinearVelocity(dynamicIndex, vec3(0.5, 0, 0))
    instance.setBodyAngularVelocity(dynamicIndex, vec3(0, 0, 0.25))
    checkNear(instance.bodyPosition(dynamicIndex).y, 4)
    checkNear(instance.bodyLinearVelocity(dynamicIndex).x, 0.5)
    checkNear(instance.bodyAngularVelocity(dynamicIndex).z, 0.25)

    let state = world.saveState()
    let savedPosition = instance.bodyPosition(dynamicIndex)
    for step in 0 ..< 10:
      check world.step(dt) == {}
    check instance.bodyPosition(dynamicIndex).y < savedPosition.y
    world.restoreState(state)
    checkNear(instance.bodyPosition(dynamicIndex).y, savedPosition.y)
    state.close()

    let restoredId = instance.bodyId(dynamicIndex)
    check restoredId != existing.id
    instance.close()
    instance.close()
    check not instance.isAlive
    check existing.isAlive
    check world.step(dt) == {}
    expect JoltError:
      discard instance.bodyCount

  test "soft body settings survive binary restoration":
    let source = newWorld()
    let cloth = source.addSoftBody(
      clothSoftBodyMesh(4, 4, 0.35, [0, 3]), vec3(0, 4, 0))
    let captured = source.capturePhysicsScene()
    let data = captured.serialize()
    cloth.close()
    source.close()
    captured.close()

    let restored = restorePhysicsScene(data)
    defer: restored.close()
    check restored.rigidBodyCount == 0
    check restored.softBodyCount == 1
    let target = newWorld()
    defer: target.close()
    let instance = restored.instantiate(target)
    check instance.bodyCount == 1
    check instance.isSoftBody(0)
    check instance.bodyMotionType(0) == MotionType.Dynamic
    for step in 0 ..< 5:
      check target.step(dt) == {}

  test "shared shapes and sub-shape materials survive restoration":
    let source = newWorld()
    let wood = physicsMaterial("scene wood", materialColor(150, 85, 35))
    let metal = physicsMaterial("scene metal", materialColor(165, 180, 200))
    let body = source.addStaticBody(staticCompoundShape([
      compoundChild(
        boxShape(vec3(0.7, 0.5, 0.7)).withMaterial(wood), vec3(-1, 0, 0)),
      compoundChild(
        sphereShape(0.7).withMaterial(metal), vec3(1, 0, 0))]),
      vec3(0, 0, 0))
    let scene = source.capturePhysicsScene()
    let data = scene.serialize()
    body.close()
    source.close()
    scene.close()

    let restored = restorePhysicsScene(data)
    defer: restored.close()
    let target = newWorld()
    defer: target.close()
    let instance = restored.instantiate(target)
    check instance.bodyCount == 1
    let leftHit = target.castRay(
      vec3(-1, 3, 0), vec3(0, -1, 0), 6).get
    let rightHit = target.castRay(
      vec3(1, 3, 0), vec3(0, -1, 0), 6).get
    check leftHit.bodyId == instance.bodyId(0)
    check rightHit.bodyId == instance.bodyId(0)
    check leftHit.subShapeId != rightHit.subShapeId
    check leftHit.material(target).get == wood
    check rightHit.material(target).get == metal

  test "collision group filters retain disabled subgroup behavior":
    let source = newWorld()
    source.setGravity(vec3(0, 0, 0))
    let filter = newCollisionGroupFilter(2)
    filter.setCollisionEnabled(0, 1, false)
    let left = source.addDynamicBody(sphereShape(0.5), vec3(-0.7, 2, 0))
    let right = source.addDynamicBody(sphereShape(0.5), vec3(0.7, 2, 0))
    left.setCollisionGroup(filter.bodyCollisionGroup(55, 0))
    right.setCollisionGroup(filter.bodyCollisionGroup(55, 1))
    left.setLinearVelocity(vec3(2, 0, 0))
    right.setLinearVelocity(vec3(-2, 0, 0))
    let scene = source.capturePhysicsScene()
    let data = scene.serialize()
    left.close()
    right.close()
    source.close()
    scene.close()

    let restored = restorePhysicsScene(data)
    defer: restored.close()
    let target = newWorld()
    defer: target.close()
    target.setGravity(vec3(0, 0, 0))
    let instance = restored.instantiate(target)
    for step in 0 ..< 30:
      check target.step(dt) == {}
    check instance.bodyPosition(0).x > 0.1
    check instance.bodyPosition(1).x < -0.1

  test "scene instances validate target layers and world lifetime":
    var sourceConfig = defaultWorldConfig()
    sourceConfig.collisionLayers.add(collisionLayerConfig(1))
    sourceConfig.collisionPairs.add(collisionPair(movingLayer, CollisionLayer(2)))
    let source = newWorld(sourceConfig)
    let body = source.addDynamicBody(
      sphereShape(0.2), vec3(0, 2, 0), layer = CollisionLayer(2))
    let scene = source.capturePhysicsScene()
    body.close()
    source.close()
    defer: scene.close()

    let incompatible = newWorld()
    expect JoltError:
      discard scene.instantiate(incompatible)
    incompatible.close()

    let compatible = newWorld(sourceConfig)
    let instance = scene.instantiate(compatible)
    check instance.isAlive
    compatible.close()
    check not instance.isAlive
    instance.close()
    instance.close()

  test "invalid payloads and closed resources are rejected":
    expect ValueError:
      discard restorePhysicsScene([])
    expect ValueError:
      discard restorePhysicsScene([byte 1, 2, 3, 4])

    let scene = captureRigidScene()
    let data = scene.serialize()
    var truncated = data[0 ..< data.len div 2]
    expect ValueError:
      discard restorePhysicsScene(truncated)
    var corrupted = data
    corrupted[^1] = corrupted[^1] xor 0xff'u8
    expect ValueError:
      discard restorePhysicsScene(corrupted)

    expect ValueError:
      discard restorePhysicsSceneObjectStream(newSeq[byte]())
    expect ValueError:
      discard restorePhysicsSceneObjectStream("")
    expect ValueError:
      discard restorePhysicsSceneObjectStream([byte 1, 2, 3, 4])
    let truncatedObjectText =
      authoredBoxScene[0 ..< authoredBoxScene.len div 2]
    expect ValueError:
      discard restorePhysicsSceneObjectStream(truncatedObjectText)
    scene.close()
    scene.close()
    check not scene.isAlive
    expect JoltError:
      discard scene.rigidBodyCount
    expect JoltError:
      discard scene.serializeObjectStream()
