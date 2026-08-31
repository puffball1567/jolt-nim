import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc trackedChassis(world: World): Body =
  var bodyConfig = defaultBodyConfig()
  bodyConfig.mass = 4_000
  world.addDynamicBody(
    boxShape(vec3(1.7, 0.5, 3.2)), vec3(0, 1.4, 0),
    config = bodyConfig)

suite "Jolt tracked vehicle":
  test "eighteen wheels settle and tracks drive the chassis":
    let world = newWorld()
    defer: world.close()
    let groundMaterial = physicsMaterial(
      "tracked-ground", materialColor(80, 90, 70))
    let floor = world.addStaticBody(
      boxShape(vec3(100, 0.5, 100)).withMaterial(groundMaterial),
      vec3(0, -0.5, 0))
    floor.setFriction(1)
    let chassis = world.trackedChassis()
    var config = defaultTrackedVehicleConfig()
    config.engineMaxTorque = 2_000
    let vehicle = chassis.newTrackedVehicle(config)

    check vehicle.wheelCount == 18
    check vehicle.chassis == chassis
    for _ in 0 ..< 240:
      vehicle.setInput(0, 1, 1)
      discard world.step(dt)

    var grounded = 0
    var materialHits = 0
    for wheel in 0 ..< vehicle.wheelCount:
      let state = vehicle.wheelState(wheel)
      if state.hasContact:
        inc grounded
        check state.contactBodyId.get == floor.id
        check state.contactNormal.y > 0.8
        check state.combinedLongitudinalFriction > 0
        check state.combinedLateralFriction > 0
        if state.material(world).get == groundMaterial:
          inc materialHits
    check grounded >= 14
    check materialHits == grounded

    let start = chassis.position
    var highestGear = 0
    for _ in 0 ..< 600:
      vehicle.setInput(1, 1, 1)
      discard world.step(dt)
      highestGear = max(highestGear, vehicle.powertrainState.currentGear)
    check abs(chassis.position.z - start.z) > 2
    check highestGear >= 1
    let left = vehicle.trackState(TrackedVehicleSide.LeftTrack)
    let right = vehicle.trackState(TrackedVehicleSide.RightTrack)
    check left.drivenWheel == 8
    check right.drivenWheel == 17
    check abs(left.angularVelocity) > 0.1
    check abs(right.angularVelocity) > 0.1
    check abs(left.differentialRatio - 6) < 0.001

  test "opposite track ratios produce a native pivot turn":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(80, 0.5, 80)), vec3(0, -0.5, 0))
    floor.setFriction(1)
    let chassis = world.trackedChassis()
    var config = defaultTrackedVehicleConfig()
    config.engineMaxTorque = 3_000
    config.tracks[TrackedVehicleSide.LeftTrack].inertia = 12
    config.tracks[TrackedVehicleSide.RightTrack].inertia = 8
    config.wheelCollisionMode = VehicleWheelCollisionMode.CylinderCast
    config.wheelCylinderConvexRadiusFraction = 0.2
    let vehicle = chassis.newTrackedVehicle(config)

    for _ in 0 ..< 240:
      vehicle.setInput(0, 1, 1)
      discard world.step(dt)
    var maximumYawSpeed = 0.0'f32
    for _ in 0 ..< 360:
      vehicle.setInput(1, -1, 1)
      discard world.step(dt)
      maximumYawSpeed = max(maximumYawSpeed, abs(chassis.angularVelocity.y))
    let left = vehicle.trackState(TrackedVehicleSide.LeftTrack)
    let right = vehicle.trackState(TrackedVehicleSide.RightTrack)
    check left.angularVelocity * right.angularVelocity < 0
    check maximumYawSpeed > 0.1
    check vehicle.configuration.wheelCollisionMode ==
      VehicleWheelCollisionMode.CylinderCast

  test "powertrain configuration indices input and ownership are validated":
    let world = newWorld()
    let chassis = world.trackedChassis()
    var config = defaultTrackedVehicleConfig()
    config.transmissionMode = VehicleTransmissionMode.Manual
    config.gearRatios = @[4.0'f32, 2, 1]
    config.reverseGearRatios = @[-3.5'f32]
    let vehicle = chassis.newTrackedVehicle(config)
    config.gearRatios[0] = 99
    config.tracks[TrackedVehicleSide.LeftTrack].wheelIndices[0] = 17
    var returned = vehicle.configuration
    returned.tracks[TrackedVehicleSide.RightTrack].wheelIndices[0] = 0

    check abs(vehicle.configuration.gearRatios[0] - 4) < 0.001
    check vehicle.configuration.tracks[
      TrackedVehicleSide.LeftTrack].wheelIndices[0] == 0
    check vehicle.configuration.tracks[
      TrackedVehicleSide.RightTrack].wheelIndices[0] == 9
    vehicle.setTransmission(2, 0.4)
    vehicle.setEngineRPM(99_000)
    let state = vehicle.powertrainState
    check state.currentGear == 2
    check abs(state.clutchFriction - 0.4) < 0.001
    check abs(state.transmissionRatio - 2) < 0.001
    check abs(state.engineRPM - vehicle.configuration.engineMaxRPM) < 0.1
    expect(ValueError): vehicle.setInput(1, 0, 1)
    expect(ValueError): vehicle.setTransmission(4, 1)
    expect(IndexDefect): discard vehicle.wheelState(18)
    expect(JoltError): chassis.close()

    vehicle.close()
    chassis.close()
    check not vehicle.isAlive
    check not chassis.isAlive

    let invalidChassis = world.trackedChassis()
    var bad = defaultTrackedVehicleConfig()
    bad.tracks[TrackedVehicleSide.RightTrack].wheelIndices[0] = 0
    expect(ValueError): discard invalidChassis.newTrackedVehicle(bad)
    bad = defaultTrackedVehicleConfig()
    bad.tracks[TrackedVehicleSide.LeftTrack].drivenWheel = 9
    expect(ValueError): discard invalidChassis.newTrackedVehicle(bad)
    bad = defaultTrackedVehicleConfig()
    bad.tracks[TrackedVehicleSide.RightTrack].wheelIndices.setLen(8)
    expect(ValueError): discard invalidChassis.newTrackedVehicle(bad)
    bad = defaultTrackedVehicleConfig()
    bad.wheels[0].longitudinalFriction = -1
    expect(ValueError): discard invalidChassis.newTrackedVehicle(bad)
    bad = defaultTrackedVehicleConfig()
    bad.tracks[TrackedVehicleSide.LeftTrack].differentialRatio = 0
    expect(ValueError): discard invalidChassis.newTrackedVehicle(bad)
    expect(ValueError):
      discard world.addStaticBody(
        boxShape(vec3(1.7, 0.5, 3.2)), vec3(0, 1, 0)).newTrackedVehicle()
    expect(ValueError):
      discard world.addDynamicBody(
        sphereShape(1), vec3(0, 2, 0)).newTrackedVehicle()

    let remaining = invalidChassis.newTrackedVehicle()
    world.close()
    check not remaining.isAlive
    expect(JoltError): remaining.setInput(0, 1, 1)
