import std/[math, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc constraintWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  result = newWorld(config)
  result.setGravity(vec3(0, 0, 0))

proc addRemoteAnchor(world: World): Body =
  world.addStaticBody(
    boxShape(vec3(0.15, 0.15, 0.15)), vec3(0, 10, 0))

proc step(world: World; count: int) =
  for _ in 0 ..< count:
    check world.step(dt) == {}

proc yAxisTilt(rotation: Quat): float32 =
  let q = rotation.normalized
  let rotatedY = 1.0'f32 - 2.0'f32 * (q.x * q.x + q.z * q.z)
  arccos(clamp(rotatedY, -1.0'f32, 1.0'f32))

suite "Jolt advanced constraints and motors":
  test "hinge velocity and position motors drive a body":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let rotor = world.addDynamicBody(
      boxShape(vec3(0.8, 0.2, 0.2)), vec3(0.8, 0, 0))
    let hinge = addHingeConstraint(
      anchor, rotor, vec3(0, 0, 0), vec3(0, 1, 0))
    var motor = defaultMotorSettings()
    motor.spring = springSettings(5, 1)
    motor.minTorque = -1_000
    motor.maxTorque = 1_000
    hinge.setMotor(
      MotorState.Velocity, targetVelocity = 2,
      targetPosition = 0, settings = motor)
    world.step(90)
    check abs(hinge.currentAngle) > 0.5

    let positionRotor = world.addDynamicBody(
      boxShape(vec3(0.8, 0.2, 0.2)), vec3(0.8, 0, 2))
    let positionHinge = addHingeConstraint(
      anchor, positionRotor, vec3(0, 0, 2), vec3(0, 1, 0))
    positionHinge.setMotor(
      MotorState.PositionAndVelocity, targetVelocity = 0,
      targetPosition = 0.8, settings = motor)
    check positionHinge.motor.state == MotorState.PositionAndVelocity
    check abs(positionHinge.motor.targetPosition - 0.8) < 0.001
    world.step(120)
    check positionHinge.currentAngle > 0.3
    check abs(positionHinge.currentAngle - 0.8) < 0.5

    positionHinge.setLimits(-0.3, 0.3)
    positionHinge.setLimitSpring(springSettings(4, 0.8))
    positionHinge.setMotorState(MotorState.Disabled)
    positionRotor.setAngularVelocity(vec3(0, 8, 0))
    world.step(120)
    check abs(positionHinge.currentAngle) < 0.5
    check classify(positionHinge.solverImpulse.motorRotation.x) notin
      {fcNan, fcInf, fcNegInf}

  test "slider velocity and position motors drive a carriage":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let carriage = world.addDynamicBody(
      boxShape(vec3(0.4, 0.25, 0.4)), vec3(0, 0, 0))
    let slider = addSliderConstraint(
      anchor, carriage, vec3(0, 0, 0), vec3(1, 0, 0), -4, 4)
    var motor = defaultMotorSettings()
    motor.spring = springSettings(5, 1)
    motor.minForce = -1_000
    motor.maxForce = 1_000
    slider.setMotor(
      MotorState.Velocity, targetVelocity = 2,
      targetPosition = 0, settings = motor)
    world.step(90)
    check slider.currentPosition > 0.6

    let positionCarriage = world.addDynamicBody(
      boxShape(vec3(0.4, 0.25, 0.4)), vec3(0, 0, 2))
    let positionSlider = addSliderConstraint(
      anchor, positionCarriage,
      vec3(0, 0, 2), vec3(1, 0, 0), -4, 4)
    positionSlider.setMotor(
      MotorState.PositionAndVelocity, targetVelocity = 0,
      targetPosition = -1, settings = motor)
    check positionSlider.motor.state == MotorState.PositionAndVelocity
    check abs(positionSlider.motor.targetPosition + 1) < 0.001
    world.step(150)
    check positionSlider.currentPosition < -0.5
    check abs(positionSlider.currentPosition + 1) < 0.5
    positionSlider.setLimitSpring(springSettings(
      50, 8, SpringMode.StiffnessAndDamping))
    check classify(positionSlider.solverImpulse.motorTranslation.x) notin
      {fcNan, fcInf, fcNegInf}

  test "cone constraint limits swing and supports runtime limits":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let body = world.addDynamicBody(
      boxShape(vec3(0.3, 0.8, 0.3)), vec3(0, 0, 0))
    let cone = addConeConstraint(
      anchor, body, vec3(0, 0, 0), vec3(0, 1, 0), 0.35)
    check cone.kind == ConstraintKind.Cone
    check abs(cone.halfConeAngle - 0.35) < 0.001
    body.setAngularVelocity(vec3(8, 0, 0))
    world.step(120)
    check body.rotation.yAxisTilt < 0.42

    cone.setHalfConeAngle(0.18)
    check abs(cone.halfConeAngle - 0.18) < 0.001
    body.setAngularVelocity(vec3(8, 0, 0))
    world.step(120)
    check body.rotation.yAxisTilt < 0.27
    check classify(cone.solverImpulse.rotation.x) notin {fcNan, fcInf, fcNegInf}

  test "swing-twist limits and twist motor control orientation":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let body = world.addDynamicBody(
      boxShape(vec3(0.7, 0.25, 0.25)), vec3(0, 0, 0))
    let joint = addSwingTwistConstraint(
      anchor, body,
      vec3(0, 0, 0), vec3(1, 0, 0), vec3(0, 1, 0),
      0.3, 0.35, -0.25, 0.25)
    check joint.kind == ConstraintKind.SwingTwist
    joint.setFriction(0.1)
    body.setAngularVelocity(vec3(8, 0, 0))
    world.step(120)
    var rotation = joint.rotationInConstraintSpace.normalized
    var twistAngle = 2.0'f32 * arctan2(abs(rotation.x), abs(rotation.w))
    check twistAngle < 0.32

    var motor = defaultMotorSettings()
    motor.minTorque = -100
    motor.maxTorque = 100
    joint.configureTwistMotor(motor)
    joint.setSwingTwistMotorTargets(vec3(0, 0, 0), quatIdentity())
    joint.setTwistMotorState(MotorState.Position)
    world.step(120)
    rotation = joint.rotationInConstraintSpace.normalized
    twistAngle = 2.0'f32 * arctan2(abs(rotation.x), abs(rotation.w))
    check twistAngle < 0.08

    joint.setSwingTwistLimits(0.2, 0.25, -0.15, 0.15)
    check classify(joint.solverImpulse.motorRotation.x) notin
      {fcNan, fcInf, fcNegInf}

  test "SixDOF axis modes constrain selected motion":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let body = world.addDynamicBody(sphereShape(0.3), vec3(0, 0, 0))
    var config = defaultSixDOFConfig()
    config.swingType = SixDOFSwingType.SwingCone
    config.limits[SixDOFAxis.TranslationX] = limitedAxis(-1, 1)
    config.limits[SixDOFAxis.RotationY] = freeAxis()
    let joint = addSixDOFConstraint(
      anchor, body, vec3(0, 0, 0), config)
    check joint.kind == ConstraintKind.SixDOF
    check joint.swingType == SixDOFSwingType.SwingCone
    check joint.axisLimit(SixDOFAxis.TranslationX).mode ==
      SixDOFAxisMode.AxisLimited
    check joint.axisLimit(SixDOFAxis.TranslationY).mode ==
      SixDOFAxisMode.AxisFixed
    check joint.axisLimit(SixDOFAxis.RotationY).mode ==
      SixDOFAxisMode.AxisFree
    joint.setAxisFriction(SixDOFAxis.RotationY, 0.05)
    body.setLinearVelocity(vec3(6, 3, 2))
    body.setAngularVelocity(vec3(0, 3, 0))
    world.step(120)
    check abs(body.position.x) < 1.05
    check abs(body.position.y) < 0.05
    check abs(body.position.z) < 0.05

    joint.setAxisLimit(SixDOFAxis.TranslationX, freeAxis())
    body.setLinearVelocity(vec3(3, 0, 0))
    world.step(60)
    check abs(body.position.x) > 1.5
    check classify(joint.solverImpulse.position.y) notin {fcNan, fcInf, fcNegInf}

  test "SixDOF cone swing validates symmetric live limits":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let body = world.addDynamicBody(sphereShape(0.3), vec3(0, 0, 0))
    var config = defaultSixDOFConfig()
    config.swingType = SixDOFSwingType.SwingCone
    config.limits[SixDOFAxis.RotationY] = limitedAxis(-0.4, 0.4)
    config.limits[SixDOFAxis.RotationZ] = limitedAxis(-0.3, 0.3)
    let joint = addSixDOFConstraint(
      anchor, body, vec3(0, 0, 0), config)
    check joint.swingType == SixDOFSwingType.SwingCone
    check joint.axisLimit(SixDOFAxis.RotationZ) == limitedAxis(-0.3, 0.3)
    joint.setAxisLimit(SixDOFAxis.RotationZ, limitedAxis(-0.2, 0.2))
    check joint.axisLimit(SixDOFAxis.RotationZ) == limitedAxis(-0.2, 0.2)
    let spring = springSettings(
      900, 30, SpringMode.StiffnessAndDamping)
    joint.setAxisLimitSpring(SixDOFAxis.TranslationX, spring)
    check joint.axisLimitSpring(SixDOFAxis.TranslationX).mode == spring.mode
    check abs(joint.axisLimitSpring(SixDOFAxis.TranslationX).value -
      spring.value) < 0.001
    expect ValueError:
      joint.setAxisLimit(SixDOFAxis.RotationY, limitedAxis(-0.2, 0.3))
    expect ValueError:
      joint.setAxisLimitSpring(
        SixDOFAxis.RotationY, springSettings(2, 1))

    config.limits[SixDOFAxis.RotationY] = limitedAxis(-0.2, 0.3)
    expect ValueError:
      discard addSixDOFConstraint(
        anchor, body, vec3(0, 0, 0), config)

  test "SixDOF translation and rotation motors share target controls":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let body = world.addDynamicBody(
      boxShape(vec3(0.35, 0.25, 0.2)), vec3(0, 0, 0))
    var config = defaultSixDOFConfig()
    config.limits[SixDOFAxis.TranslationX] = freeAxis()
    config.limits[SixDOFAxis.RotationY] = freeAxis()
    let joint = addSixDOFConstraint(
      anchor, body, vec3(0, 0, 0), config)
    var motor = defaultMotorSettings()
    motor.spring = springSettings(5, 1)
    motor.minForce = -1_000
    motor.maxForce = 1_000
    motor.minTorque = -500
    motor.maxTorque = 500
    joint.configureAxisMotor(SixDOFAxis.TranslationX, motor)
    joint.configureAxisMotor(SixDOFAxis.RotationY, motor)
    let translationMotor = joint.sixDOFAxisMotorSettings(
      SixDOFAxis.TranslationX)
    check translationMotor.spring.mode == motor.spring.mode
    check abs(translationMotor.spring.value - motor.spring.value) < 0.001
    check abs(translationMotor.minForce - motor.minForce) < 0.001
    check abs(translationMotor.maxForce - motor.maxForce) < 0.001
    joint.setSixDOFMotorTargets(
      vec3(0, 0, 0),
      vec3(0, 0, 0),
      vec3(2, 0, 0),
      quatFromAxisAngle(vec3(0, 1, 0), 0.5))
    joint.setAxisMotorState(SixDOFAxis.TranslationX, MotorState.Position)
    joint.setAxisMotorState(SixDOFAxis.RotationY, MotorState.Position)
    let translationState = joint.sixDOFMotor(SixDOFAxis.TranslationX)
    check translationState.state == MotorState.Position
    check translationState.targetPosition == vec3(2, 0, 0)
    check translationState.targetOrientation ==
      quatFromAxisAngle(vec3(0, 1, 0), 0.5)
    world.step(240)
    check abs(body.position.x - 2) < 0.2
    check abs(body.rotation.y) > 0.15
    let impulses = joint.solverImpulse
    check classify(impulses.motorTranslation.x) notin {fcNan, fcInf, fcNegInf}
    check classify(impulses.motorRotation.y) notin {fcNan, fcInf, fcNegInf}

  test "advanced constraint inputs and ownership are validated":
    let world = constraintWorld()
    defer: world.close()
    let anchor = world.addRemoteAnchor()
    let body = world.addDynamicBody(sphereShape(0.3), vec3(0, 0, 0))
    expect ValueError:
      discard addConeConstraint(
        anchor, body, vec3(0, 0, 0), vec3(0, 1, 0), PI.float32 + 0.1)
    expect ValueError:
      discard addSwingTwistConstraint(
        anchor, body,
        vec3(0, 0, 0), vec3(1, 0, 0), vec3(1, 0, 0),
        0.3, 0.3, -0.2, 0.2)

    var badSixDOF = defaultSixDOFConfig()
    badSixDOF.axisY = vec3(1, 0, 0)
    expect ValueError:
      discard addSixDOFConstraint(
        anchor, body, vec3(0, 0, 0), badSixDOF)

    let cone = addConeConstraint(
      anchor, body, vec3(0, 0, 0), vec3(0, 1, 0), 0.4)
    expect JoltError:
      body.close()
    expect ValueError:
      cone.configureMotor(defaultMotorSettings())
    expect ValueError:
      cone.setHalfConeAngle(-0.1)
    cone.close()
