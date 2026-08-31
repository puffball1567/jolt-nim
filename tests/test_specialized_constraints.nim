import std/[math, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

suite "Jolt specialized constraints":
  test "gear couples angular velocities through companion hinges":
    let world = newWorld()
    defer: world.close()
    let axis = vec3(0, 0, 1)
    let anchor1 = world.addStaticBody(emptyShape(), vec3(-1.5, 4, 0))
    let anchor2 = world.addStaticBody(emptyShape(), vec3(1.5, 4, 0))
    let gear1 = world.addDynamicBody(cylinderShape(0.35, 0.8), vec3(-1.5, 4, 0))
    let gear2 = world.addDynamicBody(cylinderShape(0.35, 0.5), vec3(1.5, 4, 0))
    gear1.setGravityFactor(0)
    gear2.setGravityFactor(0)
    let hinge1 = addHingeConstraint(anchor1, gear1, gear1.position, axis)
    let hinge2 = addHingeConstraint(anchor2, gear2, gear2.position, axis)
    let gear = addGearConstraint(
      gear1, gear2, axis, axis, 2.0, hinge1, hinge2)
    gear1.setAngularVelocity(vec3(0, 0, 4))

    for _ in 0 ..< 90:
      gear1.addTorque(vec3(0, 0, 180))
      discard world.step(dt)

    let omega1 = gear1.angularVelocity.z
    let omega2 = gear2.angularVelocity.z
    check gear.kind == ConstraintKind.Gear
    check abs(omega1) > 0.05
    check abs(omega2) > 0.05
    check abs(omega1 + 2.0'f32 * omega2) < 0.12
    check abs(gear.totalLambda) > 0
    check abs(gear.solverImpulse.rotation.x - gear.totalLambda) < 1.0e-5

  test "pulley preserves its weighted segment length and updates limits":
    let world = newWorld()
    defer: world.close()
    let body1 = world.addDynamicBody(sphereShape(0.45), vec3(-2, 4, 0))
    let body2 = world.addDynamicBody(sphereShape(0.45), vec3(2, 4, 0))
    body1.setGravityFactor(0)
    body2.setGravityFactor(0)
    let pulley = addPulleyConstraint(
      body1, body2,
      body1.position, vec3(-2, 8, 0),
      body2.position, vec3(2, 8, 0),
      ratio = 1.5, minLength = 10, maxLength = 10)
    body1.setLinearVelocity(vec3(0, -2, 0))

    for _ in 0 ..< 120:
      discard world.step(dt)

    check pulley.kind == ConstraintKind.Pulley
    check abs(pulley.currentLength - 10) < 0.08
    check body1.position.y < 3.8
    check body2.position.y > 4.05
    check abs(pulley.lengthLimits.minimum - 10) < 1.0e-5
    check abs(pulley.lengthLimits.maximum - 10) < 1.0e-5
    check classify(pulley.solverImpulse.position.x) notin {fcNan, fcInf, fcNegInf}

    pulley.setLengthLimits(9.5, 10.5)
    check abs(pulley.lengthLimits.minimum - 9.5) < 1.0e-5
    check abs(pulley.lengthLimits.maximum - 10.5) < 1.0e-5

  test "rack and pinion couples hinge rotation to slider motion":
    let world = newWorld()
    defer: world.close()
    let hingeAxis = vec3(0, 0, 1)
    let sliderAxis = vec3(1, 0, 0)
    let pinionAnchor = world.addStaticBody(emptyShape(), vec3(-2, 4, 0))
    let rackAnchor = world.addStaticBody(emptyShape(), vec3(1, 2, 0))
    let pinion = world.addDynamicBody(
      cylinderShape(0.35, 0.75), vec3(-2, 4, 0))
    let rack = world.addDynamicBody(
      boxShape(vec3(1.6, 0.25, 0.3)), vec3(1, 2, 0))
    pinion.setGravityFactor(0)
    rack.setGravityFactor(0)
    let hinge = addHingeConstraint(
      pinionAnchor, pinion, pinion.position, hingeAxis)
    let slider = addSliderConstraint(
      rackAnchor, rack, rack.position, sliderAxis, -4, 4)
    let coupling = addRackAndPinionConstraint(
      pinion, rack, hingeAxis, sliderAxis, 2.0, hinge, slider)
    pinion.setAngularVelocity(vec3(0, 0, 2))

    for _ in 0 ..< 90:
      pinion.addTorque(vec3(0, 0, 120))
      discard world.step(dt)

    check coupling.kind == ConstraintKind.RackAndPinion
    check abs(hinge.currentAngle) > 0.03
    check abs(slider.currentPosition) > 0.015
    check abs(hinge.currentAngle - 2.0'f32 * slider.currentPosition) < 0.12
    check abs(coupling.totalLambda) > 0
    check abs(coupling.solverImpulse.position.x - coupling.totalLambda) < 1.0e-5

  test "Hermite path motor drives a body while preserving the curve":
    let world = newWorld()
    defer: world.close()
    let pathBody = world.addStaticBody(emptyShape(), vec3(0, 0, 0))
    let cart = world.addDynamicBody(sphereShape(0.45), vec3(-4, 3, 0))
    cart.setGravityFactor(0)
    let path = addPathConstraint(
      pathBody,
      cart,
      [
        pathPoint(vec3(-4, 3, 0), vec3(4, 0, 0), vec3(0, 1, 0)),
        pathPoint(vec3(0, 5, 0), vec3(4, 0, 0), vec3(0, 1, 0)),
        pathPoint(vec3(4, 3, 0), vec3(4, 0, 0), vec3(0, 1, 0))
      ],
      rotationConstraint = PathRotationConstraintType.PathRotationToPath)
    var motor = defaultMotorSettings()
    motor.minForce = -10_000
    motor.maxForce = 10_000
    path.setPathMotor(MotorState.PositionAndVelocity, 1.5, 2, motor)

    for _ in 0 ..< 240:
      discard world.step(dt)

    check path.kind == ConstraintKind.Path
    check abs(path.pathMaxFraction - 2) < 1.0e-5
    check path.pathFraction > 1.5
    check cart.position.x > 1.5
    check abs(cart.position.z) < 0.05
    check path.pathMotor.state == MotorState.PositionAndVelocity
    check abs(path.pathMotor.targetVelocity - 1.5) < 1.0e-5
    check abs(path.pathMotor.targetFraction - 2) < 1.0e-5
    check classify(path.solverImpulse.motorTranslation.x) notin
      {fcNan, fcInf, fcNegInf}

    path.setPathFriction(25)
    path.setPathMotorState(MotorState.Disabled)

  test "specialized constraints validate ratios dependencies and limits":
    let world = newWorld()
    defer: world.close()
    let anchor = world.addStaticBody(emptyShape(), vec3(0, 4, 0))
    let first = world.addDynamicBody(sphereShape(0.5), vec3(-1, 3, 0))
    let second = world.addDynamicBody(sphereShape(0.5), vec3(1, 3, 0))
    let point = addPointConstraint(anchor, first, vec3(0, 4, 0))

    expect(ValueError):
      discard addGearConstraint(first, second, vec3(0, 0, 0), vec3(0, 1, 0), 1)
    expect(ValueError):
      discard addGearConstraint(
        first, second, vec3(0, 1, 0), vec3(0, 1, 0), 0)
    expect(ValueError):
      discard addGearConstraint(
        first, second, vec3(0, 1, 0), vec3(0, 1, 0), 1, point)
    expect(ValueError):
      discard addPulleyConstraint(
        first, second, first.position, vec3(-1, 6, 0),
        second.position, vec3(1, 6, 0), maxLength = -2)
    expect(ValueError):
      discard addRackAndPinionConstraint(
        first, second, vec3(0, 0, 1), vec3(1, 0, 0), -1)

    let pulley = addPulleyConstraint(
      first, second, first.position, vec3(-1, 6, 0),
      second.position, vec3(1, 6, 0))
    expect(ValueError): pulley.setLengthLimits(5, 4)
    expect(ValueError): discard point.currentLength

  test "Hermite paths validate frames fractions and motor targets":
    let world = newWorld()
    defer: world.close()
    let anchor = world.addStaticBody(emptyShape(), vec3(0, 0, 0))
    let body = world.addDynamicBody(sphereShape(0.4), vec3(0, 2, 0))
    let validPoints = [
      pathPoint(vec3(0, 2, 0), vec3(2, 0, 0), vec3(0, 1, 0)),
      pathPoint(vec3(2, 2, 0), vec3(2, 0, 0), vec3(0, 1, 0))
    ]

    expect(ValueError):
      discard addPathConstraint(anchor, body, [validPoints[0]])
    expect(ValueError):
      discard addPathConstraint(anchor, body, [
        pathPoint(vec3(0, 2, 0), vec3(0, 0, 0), vec3(0, 1, 0)),
        validPoints[1]
      ])
    expect(ValueError):
      discard addPathConstraint(anchor, body, [
        pathPoint(vec3(0, 2, 0), vec3(2, 0, 0), vec3(1, 0, 0)),
        validPoints[1]
      ])
    expect(ValueError):
      discard addPathConstraint(anchor, body, validPoints, pathFraction = 2)

    let path = addPathConstraint(anchor, body, validPoints)
    expect(ValueError): path.setPathMotorTargets(1, 2)
    expect(ValueError): path.setPathFriction(-1)
    let pointConstraint = addPointConstraint(anchor, body, vec3(0, 2, 0))
    expect(ValueError): discard pointConstraint.pathFraction
