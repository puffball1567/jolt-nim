import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc speed(body: Body): float32 =
  let velocity = body.linearVelocity
  sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
    velocity.z * velocity.z)

proc sixWheelConfig(): VehicleConfig =
  result = defaultVehicleConfig()
  result.engineMaxTorque = 1_100
  let axlePositions = [2.0'f32, 0.0'f32, -2.0'f32]
  for axle, z in axlePositions:
    var left = defaultVehicleWheelConfig(vec3(0.95, -0.3, z))
    var right = defaultVehicleWheelConfig(vec3(-0.95, -0.3, z))
    left.radius = 0.38
    right.radius = 0.38
    left.width = 0.24
    right.width = 0.24
    if axle == 0:
      left.maxSteerAngle = PI.float32 / 6
      right.maxSteerAngle = PI.float32 / 6
    if axle == 2:
      left.maxHandBrakeTorque = 5_000
      right.maxHandBrakeTorque = 5_000
    result.wheels.add(left)
    result.wheels.add(right)
  result.differentials = @[
    vehicleDifferential(0, 1, engineTorqueRatio = 0.25),
    vehicleDifferential(2, 3, engineTorqueRatio = 0.25),
    vehicleDifferential(4, 5, engineTorqueRatio = 0.5)
  ]
  result.antiRollBars = @[
    vehicleAntiRollBar(0, 1, 1_200),
    vehicleAntiRollBar(2, 3, 900),
    vehicleAntiRollBar(4, 5, 1_400)
  ]

suite "Jolt wheeled vehicle":
  test "four wheels settle on the floor and expose contact state":
    let world = newWorld()
    defer: world.close()
    let asphalt = physicsMaterial("asphalt", materialColor(70, 75, 80))
    let floor = world.addStaticBody(
      boxShape(vec3(40, 0.5, 40)).withMaterial(asphalt),
      vec3(0, -0.5, 0))
    floor.setFriction(1)
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 1.2, 0))
    let vehicle = chassis.newVehicle()

    check vehicle.wheelCount == 4
    for _ in 0 ..< 180:
      vehicle.setInput(0, 0)
      discard world.step(dt)

    var contacts = 0
    for wheel in 0 ..< vehicle.wheelCount:
      let state = vehicle.wheelState(wheel)
      check state.suspensionLength >= 0
      check state.position.y < chassis.position.y
      if state.hasContact:
        inc contacts
        check state.contactBodyId.isSome
        check state.contactBodyId.get == floor.id
        check state.contactSubShapeId.isSome
        check state.material(world).get == asphalt
        check state.contactNormal.y > 0.9
        check state.contactLongitudinal.x * state.contactLongitudinal.x +
          state.contactLongitudinal.y * state.contactLongitudinal.y +
          state.contactLongitudinal.z * state.contactLongitudinal.z > 0.9
        check state.contactLateral.x * state.contactLateral.x +
          state.contactLateral.y * state.contactLateral.y +
          state.contactLateral.z * state.contactLateral.z > 0.9
        check abs(state.suspensionImpulse) > 0
    check contacts >= 3

  test "engine input drives and steering turns the chassis":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(80, 0.5, 80)), vec3(0, -0.5, 0))
    floor.setFriction(1)
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 1.2, 0))
    let vehicle = chassis.newVehicle()

    for _ in 0 ..< 120:
      vehicle.setInput(0, 0)
      discard world.step(dt)
    let start = chassis.position
    for _ in 0 ..< 300:
      vehicle.setInput(1, 0)
      discard world.step(dt)
    let straight = chassis.position
    check abs(straight.z - start.z) > 4
    check chassis.speed > 1

    for _ in 0 ..< 180:
      vehicle.setInput(0.7, 0.65)
      discard world.step(dt)
    let turned = chassis.position
    check abs(turned.x - straight.x) > 0.5
    check abs(vehicle.wheelState(0).steerAngle) > 0.05

    for _ in 0 ..< 180:
      vehicle.setInput(0, 0, brake = 1, handBrake = 1)
      discard world.step(dt)
    check chassis.speed < 1.5

  test "wheel layout, rear steering and rear-wheel drive are configurable":
    let world = newWorld()
    defer: world.close()
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 4, 0))
    chassis.setGravityFactor(0)
    var config = defaultVehicleConfig()
    config.fourWheelDrive = false
    config.frontWheelDrive = false
    config.wheelTrack = 1.2
    config.frontAxleOffset = 1.1
    config.rearAxleOffset = 1.45
    config.suspensionAttachmentHeightRatio = -0.4
    config.rearMaxSteerAngle = PI.float32 / 12
    config.frontBrakeTorque = 900
    config.rearBrakeTorque = 1_800
    config.rearHandBrakeTorque = 5_000
    let vehicle = chassis.newVehicle(config)

    let frontLeft = vehicle.wheelState(0)
    let frontRight = vehicle.wheelState(1)
    let rearLeft = vehicle.wheelState(2)
    check abs((frontLeft.position.x - frontRight.position.x) - 1.2) < 0.02
    check abs((frontLeft.position.z - chassis.position.z) - 1.1) < 0.02
    check abs((rearLeft.position.z - chassis.position.z) + 1.45) < 0.02
    check vehicle.configuration.frontBrakeTorque == 900

    for _ in 0 ..< 120:
      vehicle.setInput(1, 0.8)
      discard world.step(dt)
    check abs(vehicle.wheelState(2).steerAngle) > 0.05
    check abs(vehicle.wheelState(3).angularVelocity) >
      abs(vehicle.wheelState(1).angularVelocity) + 0.1

  test "custom six-wheel layout drives through native wheel and axle settings":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(100, 0.5, 100)), vec3(0, -0.5, 0))
    floor.setFriction(1)
    let chassis = world.addDynamicBody(
      boxShape(vec3(1.1, 0.35, 2.8)), vec3(0, 1.1, 0))
    var config = sixWheelConfig()
    let vehicle = chassis.newVehicle(config)
    config.wheels[0].radius = 9
    var returnedConfig = vehicle.configuration
    returnedConfig.wheels[0].radius = 8

    check vehicle.wheelCount == 6
    check abs(vehicle.configuration.wheels[0].radius - 0.38) < 0.001
    check vehicle.differentialCount == 3
    check vehicle.antiRollBarCount == 3
    for axle in 0 ..< 3:
      let differential = vehicle.differentialState(axle)
      let antiRollBar = vehicle.antiRollBarState(axle)
      check differential.leftWheel == axle * 2
      check differential.rightWheel == axle * 2 + 1
      check antiRollBar.leftWheel == axle * 2
      check antiRollBar.rightWheel == axle * 2 + 1
    check abs(vehicle.antiRollBarState(0).stiffness - 1_200) < 0.1
    check abs(vehicle.antiRollBarState(2).stiffness - 1_400) < 0.1

    let frontLeft = vehicle.wheelState(0)
    let frontRight = vehicle.wheelState(1)
    let middleLeft = vehicle.wheelState(2)
    let rearLeft = vehicle.wheelState(4)
    check abs((frontLeft.position.x - frontRight.position.x) - 1.9) < 0.03
    check frontLeft.position.z > middleLeft.position.z
    check middleLeft.position.z > rearLeft.position.z

    for _ in 0 ..< 150:
      vehicle.setInput(0, 0)
      discard world.step(dt)
    var grounded = 0
    for wheel in 0 ..< vehicle.wheelCount:
      if vehicle.wheelState(wheel).hasContact:
        inc grounded
    check grounded >= 5
    let start = chassis.position
    for _ in 0 ..< 360:
      vehicle.setInput(1, 0.55)
      discard world.step(dt)
    check abs(chassis.position.z - start.z) > 4
    check abs(chassis.position.x - start.x) > 0.5
    check abs(vehicle.wheelState(0).steerAngle) > 0.05
    check abs(vehicle.wheelState(2).steerAngle) < 0.001
    expect(IndexDefect): discard vehicle.antiRollBarState(3)

  test "custom tire curves control native longitudinal and lateral grip":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(100, 0.5, 100)), vec3(0, -0.5, 0))
    floor.setFriction(1)
    let chassis = world.addDynamicBody(
      boxShape(vec3(1.1, 0.35, 2.8)), vec3(0, 1.1, 0))
    var config = sixWheelConfig()
    let zeroLongitudinal = @[
      vehicleTireFrictionPoint(0, 0),
      vehicleTireFrictionPoint(1, 0)
    ]
    let zeroLateral = @[
      vehicleTireFrictionPoint(0, 0),
      vehicleTireFrictionPoint(20, 0)
    ]
    for wheel in config.wheels.mitems:
      wheel.longitudinalFrictionCurve = zeroLongitudinal
      wheel.lateralFrictionCurve = zeroLateral
    let vehicle = chassis.newVehicle(config)
    config.wheels[0].longitudinalFrictionCurve[0].friction = 9
    var returnedConfig = vehicle.configuration
    returnedConfig.wheels[0].lateralFrictionCurve[0].friction = 8

    check vehicle.configuration.wheels[0].longitudinalFrictionCurve[0].friction == 0
    check vehicle.configuration.wheels[0].lateralFrictionCurve[0].friction == 0
    for _ in 0 ..< 180:
      vehicle.setInput(0, 0)
      discard world.step(dt)

    let start = chassis.position
    var groundedSamples = 0
    var maximumSlip = 0.0'f32
    var maximumCombinedLongitudinal = 0.0'f32
    var maximumCombinedLateral = 0.0'f32
    for _ in 0 ..< 360:
      vehicle.setInput(1, 0.6)
      discard world.step(dt)
      for wheel in 0 ..< vehicle.wheelCount:
        let state = vehicle.wheelState(wheel)
        if state.hasContact:
          inc groundedSamples
          maximumSlip = max(maximumSlip, abs(state.longitudinalSlip))
          maximumCombinedLongitudinal = max(
            maximumCombinedLongitudinal,
            abs(state.combinedLongitudinalFriction))
          maximumCombinedLateral = max(
            maximumCombinedLateral,
            abs(state.combinedLateralFriction))
    check groundedSamples > 0
    check maximumSlip > 0.01
    check maximumCombinedLongitudinal < 1.0e-6
    check maximumCombinedLateral < 1.0e-6
    check abs(chassis.position.z - start.z) < 0.5

  test "ray sphere and cylinder wheel collision testers support driving":
    for mode in VehicleWheelCollisionMode:
      let world = newWorld()
      let floor = world.addStaticBody(
        boxShape(vec3(80, 0.5, 80)), vec3(0, -0.5, 0))
      floor.setFriction(1)
      let chassis = world.addDynamicBody(
        boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 1.2, 0))
      var config = defaultVehicleConfig()
      config.wheelCollisionMode = mode
      config.wheelCollisionUp = vec3(0, 1, 0)
      config.wheelCollisionMaxSlopeAngle = PI.float32 * 75 / 180
      config.wheelSphereCastRadius = 0.1
      config.wheelCylinderConvexRadiusFraction = 0.25
      let vehicle = chassis.newVehicle(config)

      check vehicle.configuration.wheelCollisionMode == mode
      for _ in 0 ..< 180:
        vehicle.setInput(0, 0)
        discard world.step(dt)
      var grounded = 0
      for wheel in 0 ..< vehicle.wheelCount:
        if vehicle.wheelState(wheel).hasContact:
          inc grounded
      check grounded >= 3

      let start = chassis.position
      for _ in 0 ..< 300:
        vehicle.setInput(1, 0)
        discard world.step(dt)
      check abs(chassis.position.z - start.z) > 4
      vehicle.close()
      world.close()

  test "powertrain and differential settings reach the native controller":
    let world = newWorld()
    defer: world.close()
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 4, 0))
    chassis.setGravityFactor(0)
    var config = defaultVehicleConfig()
    config.engineMinRPM = 900
    config.engineMaxRPM = 7_200
    config.engineInertia = 0.7
    config.engineAngularDamping = 0.12
    config.engineTorqueCurve = @[
      vehicleTorquePoint(0, 0.55),
      vehicleTorquePoint(0.45, 1.1),
      vehicleTorquePoint(1, 0.65)
    ]
    config.gearRatios = @[3.2'f32, 2.1, 1.4, 0.9]
    config.reverseGearRatios = @[-3.1'f32, -1.8]
    config.frontTorqueRatio = 0.35
    config.differentialRatio = 4.1
    config.differentialLeftRightSplit = 0.4
    config.differentialLimitedSlipRatio = 2.2
    config.centerDifferentialLimitedSlipRatio = 1.8
    config.wheelInertia = 1.2
    config.wheelAngularDamping = 0.08
    config.tireLongitudinalImpulseMultiplier = 7.5
    config.tireLateralImpulseMultiplier = 1.3
    let vehicle = chassis.newVehicle(config)
    config.gearRatios[0] = 99
    var returnedConfig = vehicle.configuration
    returnedConfig.gearRatios[0] = 88

    check vehicle.differentialCount == 2
    check abs(vehicle.configuration.gearRatios[0] - 3.2) < 0.001
    let front = vehicle.differentialState(0)
    let rear = vehicle.differentialState(1)
    check front.leftWheel == 0
    check front.rightWheel == 1
    check rear.leftWheel == 2
    check rear.rightWheel == 3
    check abs(front.differentialRatio - 4.1) < 0.001
    check abs(front.leftRightSplit - 0.4) < 0.001
    check abs(front.limitedSlipRatio - 2.2) < 0.001
    check abs(front.engineTorqueRatio - 0.35) < 0.001
    check abs(rear.engineTorqueRatio - 0.65) < 0.001
    let powertrain = vehicle.powertrainState
    check abs(powertrain.engineRPM - 900) < 0.1
    check powertrain.currentGear == 0
    check abs(powertrain.transmissionRatio) < 0.001
    expect(IndexDefect): discard vehicle.differentialState(2)

  test "automatic transmission shifts and wheel slip is observable":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(100, 0.5, 100)), vec3(0, -0.5, 0))
    floor.setFriction(1)
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 1.2, 0))
    var config = defaultVehicleConfig()
    config.engineMinRPM = 1_000
    config.engineMaxRPM = 5_000
    config.shiftDownRPM = 1_100
    config.shiftUpRPM = 1_500
    config.transmissionSwitchTime = 0.05
    config.clutchReleaseTime = 0.05
    config.transmissionSwitchLatency = 0.05
    config.gearRatios = @[3.4'f32, 2.0, 1.2]
    let vehicle = chassis.newVehicle(config)

    for _ in 0 ..< 120:
      vehicle.setInput(0, 0)
      discard world.step(dt)
    var highestGear = 0
    var maximumLongitudinalSlip = 0.0'f32
    var maximumWheelSpeed = 0.0'f32
    for _ in 0 ..< 480:
      vehicle.setInput(1, 0)
      discard world.step(dt)
      let powertrain = vehicle.powertrainState
      highestGear = max(highestGear, powertrain.currentGear)
      check powertrain.engineRPM >= config.engineMinRPM - 0.1
      check powertrain.engineRPM <= config.engineMaxRPM + 0.1
      for wheel in 0 ..< vehicle.wheelCount:
        let state = vehicle.wheelState(wheel)
        maximumLongitudinalSlip = max(
          maximumLongitudinalSlip, abs(state.longitudinalSlip))
        maximumWheelSpeed = max(
          maximumWheelSpeed, abs(state.angularVelocity))
    check highestGear >= 2
    check maximumLongitudinalSlip > 0.001
    check maximumWheelSpeed > 1
    check abs(chassis.position.z) > 4

  test "manual transmission and engine RPM can be controlled":
    let world = newWorld()
    defer: world.close()
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 4, 0))
    chassis.setGravityFactor(0)
    var config = defaultVehicleConfig()
    config.transmissionMode = VehicleTransmissionMode.Manual
    config.gearRatios = @[3.2'f32, 1.8, 1.0]
    config.reverseGearRatios = @[-3.1'f32]
    let vehicle = chassis.newVehicle(config)

    vehicle.setTransmission(2, 0.35)
    vehicle.setEngineRPM(99_000)
    var state = vehicle.powertrainState
    check state.currentGear == 2
    check abs(state.clutchFriction - 0.35) < 0.001
    check abs(state.transmissionRatio - 1.8) < 0.001
    check abs(state.engineRPM - config.engineMaxRPM) < 0.1

    vehicle.setTransmission(-1, 1)
    vehicle.setEngineRPM(0)
    state = vehicle.powertrainState
    check state.currentGear == -1
    check abs(state.transmissionRatio + 3.1) < 0.001
    check abs(state.engineRPM - config.engineMinRPM) < 0.1
    expect(ValueError): vehicle.setTransmission(4, 1)
    expect(ValueError): vehicle.setTransmission(1, 1.1)

  test "motorcycle controller balances drives and exposes two-wheel state":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(80, 0.5, 80)), vec3(0, -0.5, 0))
    floor.setFriction(1)
    var bodyConfig = defaultBodyConfig()
    bodyConfig.mass = 240
    let chassis = world.addDynamicBody(
      offsetCenterOfMassShape(
        boxShape(vec3(0.2, 0.3, 0.4)), vec3(0, -0.3, 0)),
      vec3(0, 1.5, 0), config = bodyConfig)
    let motorcycle = chassis.newMotorcycle()

    check motorcycle.isMotorcycle
    check motorcycle.wheelCount == 2
    let controller = motorcycle.motorcycleControllerState
    # Jolt measures between the fully extended suspension force points.
    check abs(controller.wheelBase - 1.75) < 0.01
    check controller.leanControllerEnabled
    check controller.leanSteeringLimitEnabled
    check abs(controller.leanSpringConstant - 5_000) < 0.1

    for _ in 0 ..< 180:
      motorcycle.setInput(0, 0)
      discard world.step(dt)
    check motorcycle.wheelState(0).hasContact
    check motorcycle.wheelState(1).hasContact
    let start = chassis.position
    for _ in 0 ..< 300:
      motorcycle.setInput(1, 0.2)
      discard world.step(dt)
    let finish = chassis.position
    let dx = finish.x - start.x
    let dz = finish.z - start.z
    check sqrt(dx * dx + dz * dz) > 2
    check motorcycle.powertrainState.engineRPM >=
      motorcycle.configuration.engineMinRPM - 0.1

  test "motorcycle lean settings and rollback remain controllable":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(40, 0.5, 40)), vec3(0, -0.5, 0))
    var bodyConfig = defaultBodyConfig()
    bodyConfig.mass = 240
    let chassis = world.addDynamicBody(
      offsetCenterOfMassShape(
        boxShape(vec3(0.2, 0.3, 0.4)), vec3(0, -0.3, 0)),
      vec3(0, 1.5, 0), config = bodyConfig)
    let motorcycle = chassis.newMotorcycle()
    motorcycle.configureMotorcycleLean(
      false, false, 3_500, 750, 25, 3, 0.65)
    var controller = motorcycle.motorcycleControllerState
    check not controller.leanControllerEnabled
    check not controller.leanSteeringLimitEnabled
    check abs(controller.leanSpringConstant - 3_500) < 0.1
    check abs(controller.leanSpringDamping - 750) < 0.1
    check abs(controller.leanSpringIntegrationCoefficient - 25) < 0.1
    check abs(controller.leanSpringIntegrationCoefficientDecay - 3) < 0.1
    check abs(controller.leanSmoothingFactor - 0.65) < 0.001
    motorcycle.setMotorcycleLeanControllerEnabled(true)
    motorcycle.setMotorcycleLeanSteeringLimitEnabled(true)
    controller = motorcycle.motorcycleControllerState
    check controller.leanControllerEnabled
    check controller.leanSteeringLimitEnabled

    let saved = world.saveState()
    defer: saved.close()
    let start = chassis.position
    for _ in 0 ..< 60:
      motorcycle.setInput(1, 0)
      discard world.step(dt)
    let replayPosition = chassis.position
    let replayVelocity = chassis.linearVelocity
    world.restoreState(saved)
    check abs(chassis.position.x - start.x) < 0.001
    check abs(chassis.position.y - start.y) < 0.001
    check abs(chassis.position.z - start.z) < 0.001
    for _ in 0 ..< 60:
      motorcycle.setInput(1, 0)
      discard world.step(dt)
    check abs(chassis.position.x - replayPosition.x) < 0.001
    check abs(chassis.position.y - replayPosition.y) < 0.001
    check abs(chassis.position.z - replayPosition.z) < 0.001
    check abs(chassis.linearVelocity.x - replayVelocity.x) < 0.001
    check abs(chassis.linearVelocity.y - replayVelocity.y) < 0.001
    check abs(chassis.linearVelocity.z - replayVelocity.z) < 0.001

  test "motorcycle configuration and type-specific methods are validated":
    let world = newWorld()
    defer: world.close()
    var bodyConfig = defaultBodyConfig()
    bodyConfig.mass = 240
    let motorcycleChassis = world.addDynamicBody(
      offsetCenterOfMassShape(
        boxShape(vec3(0.2, 0.3, 0.4)), vec3(0, -0.3, 0)),
      vec3(0, 2, 0), config = bodyConfig)
    var bad = defaultMotorcycleConfig()
    bad.wheels.setLen(1)
    expect(ValueError): discard motorcycleChassis.newMotorcycle(bad)
    bad = defaultMotorcycleConfig()
    bad.maxLeanAngle = PI.float32
    expect(ValueError): discard motorcycleChassis.newMotorcycle(bad)
    bad = defaultMotorcycleConfig()
    bad.leanSmoothingFactor = 1.1
    expect(ValueError): discard motorcycleChassis.newMotorcycle(bad)
    expect(ValueError):
      discard motorcycleChassis.newMotorcycle(defaultVehicleConfig())

    let normalChassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2)), vec3(4, 2, 0))
    let normal = normalChassis.newVehicle()
    expect(ValueError): discard normal.motorcycleControllerState
    expect(ValueError): normal.setMotorcycleLeanControllerEnabled(false)

    let motorcycle = motorcycleChassis.newMotorcycle()
    expect(ValueError):
      motorcycle.configureMotorcycleLean(
        true, true, 5_000, 1_000, 0, 4, -0.1)
    motorcycle.close()
    check not motorcycle.isAlive
    check not motorcycle.isMotorcycle

  test "configuration, wheel indices and ownership are validated":
    let world = newWorld()
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 1.2, 0))

    var bad = defaultVehicleConfig()
    bad.wheelRadius = 1.1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.suspensionMaxLength = 0.1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.frontTorqueRatio = 1.1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.wheelTrack = -1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.rearBrakeTorque = -1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.engineMaxRPM = bad.engineMinRPM
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.engineTorqueCurve = @[
      vehicleTorquePoint(0.5, 1), vehicleTorquePoint(0.4, 1)]
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.gearRatios = @[2.5'f32, 0]
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.reverseGearRatios = @[2.5'f32]
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.shiftDownRPM = bad.shiftUpRPM
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.differentialLimitedSlipRatio = 1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.wheelInertia = 0
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.tireLateralImpulseMultiplier = -1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.wheelCollisionUp = vec3(0, 0, 0)
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.wheelCollisionMaxSlopeAngle = PI.float32
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.wheelSphereCastRadius = 0
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.wheelCylinderConvexRadiusFraction = 1.1
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = defaultVehicleConfig()
    bad.wheels = @[defaultVehicleWheelConfig(vec3(0, 0, 0))]
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.differentials.setLen(0)
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.differentials[0].leftWheel = 6
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.differentials[0].engineTorqueRatio = 0.5
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.wheels[0].wheelUp = vec3(0, 2, 0)
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.wheels[0].longitudinalFrictionCurve = @[
      vehicleTireFrictionPoint(0, 1)]
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.wheels[0].longitudinalFrictionCurve = @[
      vehicleTireFrictionPoint(0.2, 1),
      vehicleTireFrictionPoint(0.2, 0.8)]
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.wheels[0].lateralFrictionCurve = @[
      vehicleTireFrictionPoint(0, 1),
      vehicleTireFrictionPoint(10, -0.1)]
    expect(ValueError): discard chassis.newVehicle(bad)
    bad = sixWheelConfig()
    bad.antiRollBars[0].rightWheel = 0
    expect(ValueError): discard chassis.newVehicle(bad)
    expect(ValueError):
      discard world.addStaticBody(
        boxShape(vec3(1, 0.3, 2)), vec3(0, 1, 0)).newVehicle()
    expect(ValueError):
      discard world.addDynamicBody(
        sphereShape(1), vec3(0, 2, 0)).newVehicle()

    let vehicle = chassis.newVehicle()
    expect(IndexDefect): discard vehicle.wheelState(4)
    expect(ValueError): vehicle.setInput(1.1, 0)
    expect(ValueError): vehicle.setTransmission(1, 1)
    expect(ValueError): vehicle.setEngineRPM(-1)
    expect(JoltError): chassis.close()

    vehicle.close()
    check not vehicle.isAlive
    chassis.close()
    check not chassis.isAlive

    let remainingChassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 1.2, 0))
    let remainingVehicle = remainingChassis.newVehicle()
    world.close()
    check not remainingVehicle.isAlive
    expect(JoltError): remainingVehicle.setInput(0, 0)
