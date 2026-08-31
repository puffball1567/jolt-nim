import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc settle(character: RigidCharacter; world: World; frames = 180) =
  for _ in 0 ..< frames:
    discard world.step(dt)

suite "Jolt body-backed character":
  test "a rigid character settles and World.step refreshes ground state":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0),
      config = block:
        var value = defaultBodyConfig()
        value.userData = 0x1234
        value)
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 4, 0))

    character.settle(world)

    check abs(character.position.y - 0.95) < 0.08
    check character.groundState == CharacterGroundState.OnGround
    check character.isSupported
    check character.groundNormal.y > 0.95
    check character.groundBodyId == some(floor.id)
    check character.groundSubShapeId.isSome
    check character.groundUserData == 0x1234
    check character.bodyId != floor.id

  test "velocity control and impulses participate in body simulation":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(-3, 0.95, 0))
    let crate = world.addDynamicBody(
      boxShape(vec3(0.4, 0.4, 0.4)), vec3(0, 0.4, 0))
    character.settle(world, 30)

    for _ in 0 ..< 180:
      character.move(vec3(3, 0, 0))
      discard world.step(dt)

    check character.position.x > 0
    check crate.position.x > 0.35
    character.addImpulse(vec3(0, 400, 0))
    discard world.step(dt)
    check character.linearVelocity.y > 2

  test "moving ground velocity and jumping are exposed":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))
    let platform = world.addKinematicBody(
      boxShape(vec3(2, 0.25, 2)), vec3(0, 1, 0))
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2.2, 0))
    character.settle(world, 45)

    for frame in 0 ..< 60:
      platform.moveKinematic(
        vec3(float32(frame + 1) * 0.01, 1, 0), quatIdentity(), dt)
      character.move(vec3(0, 0, 0))
      discard world.step(dt)

    check character.groundVelocity.x > 0.4
    check character.position.x > 0.3
    character.move(vec3(0, 0, 0), jump = true, jumpSpeed = 6)
    discard world.step(dt)
    check character.linearVelocity.y > 5

  test "runtime transform layer slope and support settings round trip":
    let config = WorldConfig(
      maxBodies: 128,
      numBodyMutexes: 0,
      maxBodyPairs: 128,
      maxContactConstraints: 128,
      tempAllocatorBytes: 1024 * 1024,
      maxJobs: 128,
      maxBarriers: 8,
      numThreads: 1,
      maxQueuedEvents: 128,
      characterBroadPhaseCellSize: 4,
      collisionLayers: @[
        collisionLayerConfig(0),
        collisionLayerConfig(1),
        collisionLayerConfig(2)
      ],
      collisionPairs: @[
        collisionPair(CollisionLayer(0), CollisionLayer(1)),
        collisionPair(CollisionLayer(0), CollisionLayer(2))
      ])
    let world = newWorld(config)
    defer: world.close()
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0))

    character.setPosition(vec3(1, 3, 2))
    character.setRotation(quatFromAxisAngle(vec3(0, 1, 0), 0.4))
    check abs(character.position.x - 1) < 0.001
    check abs(character.centerOfMassPosition.y - 3) < 0.001
    check abs(character.rotation.w - cos(0.2'f32)) < 0.001
    character.setCollisionLayer(CollisionLayer(2))
    check character.collisionLayer == CollisionLayer(2)
    character.setMaxSlopeAngle(0.5)
    check abs(character.maxSlopeAngle - 0.5) < 0.001
    check character.isSlopeTooSteep(vec3(1, 0, 0))
    character.setSupportingHeight(0.25)
    check abs(character.supportingHeight - 0.25) < 0.001
    character.setUp(vec3(0, 2, 0))
    check abs(character.up.y - 1) < 0.001
    check abs(character.configuration.up.y - 1) < 0.001

  test "shape changes honor penetration checks and update the descriptor":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 0.95, 0))
    character.settle(world, 20)

    check character.setShape(capsuleShape(0.3, 0.3), 0.1)
    check abs(character.shape.halfHeight - 0.3) < 0.001
    character.setLinearVelocity(vec3(1, 2, 3))
    character.addLinearVelocity(vec3(2, 3, 4))
    check abs(character.linearVelocity.x - 3) < 0.001
    check abs(character.linearVelocity.y - 5) < 0.001
    check abs(character.linearVelocity.z - 7) < 0.001

  test "a rigid character contacts cooked triangle mesh terrain":
    let world = newWorld()
    defer: world.close()
    let terrain = world.addStaticBody(
      triangleMeshShape(
        [
          vec3(-5, 0, -5), vec3(-5, 0, 5),
          vec3(5, 1, 5), vec3(5, 1, -5)
        ],
        [0'u32, 1, 2, 0, 2, 3]),
      vec3(0, 0, 0))
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 4, 0))

    character.settle(world)

    check character.groundState == CharacterGroundState.OnGround
    check character.groundBodyId == some(terrain.id)
    check character.groundNormal.y > 0.98
    check character.position.y > 1.35
    check character.position.y < 1.55

  test "world snapshots preserve the rigid body and refresh character ground":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 0.95, 0))
    character.settle(world, 20)
    let state = world.saveState()
    defer: state.close()
    character.setPosition(vec3(4, 5, 0))
    discard world.step(dt)
    check character.position.x > 3

    world.restoreState(state)
    check abs(character.position.x) < 0.01
    check character.isSupported

    let extra = world.newRigidCharacter(
      capsuleShape(0.4, 0.25), vec3(2, 2, 0))
    expect(ValueError):
      world.restoreState(state)
    extra.close()

  test "validation query filtering and lifetime reject invalid use":
    let world = newWorld()
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0))
    var bad = defaultRigidCharacterConfig()
    bad.mass = 0
    expect(ValueError):
      discard world.newRigidCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 2, 0), bad)
    bad = defaultRigidCharacterConfig()
    bad.up = vec3(0, 0, 0)
    expect(ValueError):
      discard world.newRigidCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 2, 0), bad)
    bad = defaultRigidCharacterConfig()
    bad.allowedDOFs = {}
    expect(ValueError):
      discard world.newRigidCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 2, 0), bad)
    expect(ValueError): character.setMaxSlopeAngle(PI.float32)
    expect(ValueError): character.setSupportingHeight(-1)
    expect(ValueError): character.refreshGround(-1)
    expect(ValueError): character.setUp(vec3(0, 0, 0))

    var limitedConfig = defaultWorldConfig()
    limitedConfig.maxBodies = 1
    let limitedWorld = newWorld(limitedConfig)
    let blocker = limitedWorld.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))
    expect(JoltError):
      discard limitedWorld.newRigidCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 2, 0))
    blocker.close()
    let recycled = limitedWorld.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0))
    check recycled.isAlive
    limitedWorld.close()
    check not recycled.isAlive

    discard world.castRay(
      vec3(0, 4, 0), vec3(0, -1, 0), 10,
      bodyFilter = includeBodies([character.bodyId]))
    character.close()
    check not character.isAlive
    expect(JoltError): discard character.position

    let remaining = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0))
    world.close()
    check not remaining.isAlive
    expect(JoltError): remaining.activate()
