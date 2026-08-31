import std/[math, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc rollbackWorld(): World =
  var config = defaultWorldConfig()
  config.numThreads = 1
  config.maxBodies = 256
  config.maxBodyPairs = 2_048
  config.maxContactConstraints = 2_048
  newWorld(config)

proc checkNear(actual, expected: float32; epsilon = 1.0e-5'f32) =
  check abs(actual - expected) <= epsilon

proc checkNear(actual, expected: Vec3; epsilon = 1.0e-5'f32) =
  checkNear(actual.x, expected.x, epsilon)
  checkNear(actual.y, expected.y, epsilon)
  checkNear(actual.z, expected.z, epsilon)

suite "Jolt world state rollback":
  test "bodies, gravity, contacts and repeated deterministic replay restore":
    let world = rollbackWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(20, 0.5, 20)), vec3(0, -0.5, 0))
    let box = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 8, 0))

    for _ in 0 ..< 20:
      discard world.step(dt)
    discard world.drainEvents()
    let savedPosition = box.position
    let savedVelocity = box.linearVelocity
    let savedGravity = world.gravity
    let state = world.saveState()
    defer: state.close()

    check state.isAlive
    check state.byteSize > 0
    world.setGravity(vec3(0, 2, 0))
    box.setTransform(vec3(0, 0.25, 0), quatIdentity())
    discard world.step(dt)
    box.setLinearVelocity(vec3(7, 3, -2))
    for _ in 0 ..< 100:
      discard world.step(dt)
    check world.pendingEventCount > 0

    world.restoreState(state)
    checkNear(box.position, savedPosition)
    checkNear(box.linearVelocity, savedVelocity)
    checkNear(world.gravity, savedGravity)
    check world.pendingEventCount == 0

    for _ in 0 ..< 100:
      discard world.step(dt)
    let replayPosition = box.position
    let replayVelocity = box.linearVelocity
    world.restoreState(state)
    for _ in 0 ..< 100:
      discard world.step(dt)
    checkNear(box.position, replayPosition)
    checkNear(box.linearVelocity, replayVelocity)

  test "constraint solver state replays a pendulum deterministically":
    let world = rollbackWorld()
    defer: world.close()
    world.setGravity(vec3(0, -4, 0))
    let anchor = world.addStaticBody(
      sphereShape(0.2), vec3(0, 5, 0))
    let bob = world.addDynamicBody(
      sphereShape(0.45), vec3(0, 2, 0))
    discard addDistanceConstraint(
      anchor, bob, anchor.position, bob.position, 3, 3)
    bob.addImpulse(vec3(4, 0, 1))
    for _ in 0 ..< 25:
      discard world.step(dt)
    let savedPosition = bob.position
    let state = world.saveState()
    defer: state.close()

    for _ in 0 ..< 180:
      discard world.step(dt)
    let replayPosition = bob.position
    let replayVelocity = bob.linearVelocity
    world.restoreState(state)
    checkNear(bob.position, savedPosition)
    for _ in 0 ..< 180:
      discard world.step(dt)
    checkNear(bob.position, replayPosition)
    checkNear(bob.linearVelocity, replayVelocity)

  test "virtual character state is included in world rollback":
    let world = rollbackWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(12, 0.5, 12)), vec3(0, -0.5, 0))
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 0, 0))
    for _ in 0 ..< 30:
      character.move(vec3(0, 0, 0), dt)
      discard world.step(dt)
    character.move(vec3(2, 0, 0.5), dt, jump = true, jumpSpeed = 5)
    discard world.step(dt)
    let savedPosition = character.position
    let savedVelocity = character.linearVelocity
    let state = world.saveState()
    defer: state.close()

    for _ in 0 ..< 90:
      character.move(vec3(-3, 0, 1), dt)
      discard world.step(dt)
    world.restoreState(state)
    checkNear(character.position, savedPosition)
    checkNear(character.linearVelocity, savedVelocity)

    for _ in 0 ..< 90:
      character.move(vec3(1.5, 0, -0.5), dt)
      discard world.step(dt)
    let replayPosition = character.position
    let replayVelocity = character.linearVelocity
    world.restoreState(state)
    for _ in 0 ..< 90:
      character.move(vec3(1.5, 0, -0.5), dt)
      discard world.step(dt)
    checkNear(character.position, replayPosition, 2.0e-5)
    checkNear(character.linearVelocity, replayVelocity, 2.0e-5)

  test "wheeled and tracked controller state restore with their chassis":
    let world = rollbackWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(80, 0.5, 80)), vec3(0, -0.5, 0))
    floor.setFriction(1)
    let carBody = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2)), vec3(-6, 1.2, 0))
    let trackedBody = world.addDynamicBody(
      boxShape(vec3(1.1, 0.4, 2.2)), vec3(6, 1.3, 0))
    let car = carBody.newVehicle()
    let tracked = trackedBody.newTrackedVehicle()

    for _ in 0 ..< 150:
      car.setInput(0, 0)
      tracked.setInput(0, 1, 1)
      discard world.step(dt)
    for _ in 0 ..< 70:
      car.setInput(1, 0.25)
      tracked.setInput(0.8, 1, 0.75)
      discard world.step(dt)

    let carPosition = carBody.position
    let trackedPosition = trackedBody.position
    let carPowertrain = car.powertrainState
    let trackedPowertrain = tracked.powertrainState
    let carWheel = car.wheelState(0)
    let leftTrack = tracked.trackState(TrackedVehicleSide.LeftTrack)
    let state = world.saveState()
    defer: state.close()

    for _ in 0 ..< 120:
      car.setInput(-1, -0.8, brake = 0.5)
      tracked.setInput(-1, -1, 1)
      discard world.step(dt)
    world.restoreState(state)

    checkNear(carBody.position, carPosition)
    checkNear(trackedBody.position, trackedPosition)
    checkNear(car.powertrainState.engineRPM, carPowertrain.engineRPM)
    check car.powertrainState.currentGear == carPowertrain.currentGear
    checkNear(tracked.powertrainState.engineRPM, trackedPowertrain.engineRPM)
    check tracked.powertrainState.currentGear == trackedPowertrain.currentGear
    checkNear(car.wheelState(0).angularVelocity, carWheel.angularVelocity)
    checkNear(
      tracked.trackState(TrackedVehicleSide.LeftTrack).angularVelocity,
      leftTrack.angularVelocity)

  test "snapshot ownership, topology and lifetime are validated":
    let world = rollbackWorld()
    let other = rollbackWorld()
    defer:
      other.close()
      world.close()
    discard world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let state = world.saveState()

    expect(ValueError):
      other.restoreState(state)
    let added = world.addDynamicBody(sphereShape(0.5), vec3(2, 2, 0))
    expect(ValueError):
      world.restoreState(state)
    added.close()
    world.restoreState(state)

    state.close()
    state.close()
    check not state.isAlive
    expect(JoltError):
      discard state.byteSize
    expect(JoltError):
      world.restoreState(state)
