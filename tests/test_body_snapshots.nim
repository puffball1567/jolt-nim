import std/[math, options, unittest]
import jolt

proc snapshotWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  result = newWorld(config)
  result.setGravity(vec3(0, 0, 0))

proc near(left, right: float32; tolerance = 1.0e-4'f32): bool =
  abs(left - right) <= tolerance

suite "Jolt body lock snapshots":
  test "a dynamic snapshot captures complete rigid and motion state":
    let world = snapshotWorld()
    defer: world.close()
    var config = defaultBodyConfig()
    config.motionQuality = MotionQuality.LinearCast
    config.linearVelocity = vec3(3, 2, 1)
    config.angularVelocity = vec3(1, 2, 3)
    config.userData = 0x123456789abcdef0'u64
    config.allowSleeping = false
    config.collideKinematicVsNonDynamic = true
    config.useManifoldReduction = false
    config.applyGyroscopicForce = true
    config.enhancedInternalEdgeRemoval = true
    config.friction = 0.7
    config.restitution = 0.4
    config.linearDamping = 0.3
    config.angularDamping = 0.25
    config.maxLinearVelocity = 40
    config.maxAngularVelocity = 30
    config.gravityFactor = 0.5
    config.numVelocityStepsOverride = 7
    config.numPositionStepsOverride = 5
    config.massProperties = some(bodyMassProperties(12, vec3(3, 4, 5)))

    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 1, 0.75)), vec3(4, 5, 6),
      sensor = true, config = config)
    let state = body.snapshot()
    check state.bodyId == body.id
    check state.motionType == MotionType.Dynamic
    check state.collisionLayer == movingLayer
    check state.position.x.near(4)
    check state.position.y.near(5)
    check state.position.z.near(6)
    check state.centerOfMassPosition.x.near(4)
    check state.rotation.w.near(1)
    check state.linearVelocity == vec3(3, 2, 1)
    check state.angularVelocity == vec3(1, 2, 3)
    check state.active
    check state.sensor
    check state.inBroadPhase
    check not state.useManifoldReduction
    check state.friction.near(0.7)
    check state.restitution.near(0.4)
    check state.userData == 0x123456789abcdef0'u64
    check state.motion.isSome
    let motion = state.motion.get
    check motion.motionQuality == MotionQuality.LinearCast
    check motion.allowedDOFs == allAllowedDOFs()
    check motion.linearDamping.near(0.3)
    check motion.angularDamping.near(0.25)
    check motion.maxLinearVelocity.near(40)
    check motion.maxAngularVelocity.near(30)
    check motion.gravityFactor.near(0.5)
    check not motion.allowSleeping
    check motion.collideKinematicVsNonDynamic
    check motion.applyGyroscopicForce
    check motion.enhancedInternalEdgeRemoval
    check motion.numVelocityStepsOverride == 7
    check motion.numPositionStepsOverride == 5
    check motion.mass.isSome
    check motion.mass.get.near(12)
    check motion.massProperties.isSome
    check motion.massProperties.get.mass.near(12)
    let inertia = motion.massProperties.get.inertiaDiagonal
    # Jolt may reorder principal axes while retaining the same tensor.
    check (inertia.x + inertia.y + inertia.z).near(12)

  test "one multi-lock snapshot preserves mixed input order and duplicates":
    let world = snapshotWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(2, 0.5, 2)), vec3(0, -0.5, 0))
    let platform = world.addKinematicBody(
      boxShape(vec3(1, 0.25, 1)), vec3(0, 2, 0))
    let ball = world.addDynamicBody(sphereShape(0.5), vec3(0, 4, 0))

    let states = world.bodySnapshots([ball, floor, platform, ball])
    check states.len == 4
    check states[0].bodyId == ball.id
    check states[1].bodyId == floor.id
    check states[2].bodyId == platform.id
    check states[3].bodyId == ball.id
    check states[0].motionType == MotionType.Dynamic
    check states[1].motionType == MotionType.Static
    check states[2].motionType == MotionType.Kinematic
    check states[1].motion.isNone
    check states[2].motion.isSome
    check states[2].motion.get.mass.isNone
    check states[0].position == states[3].position

  test "snapshots are detached and reflect later simulation only when recaptured":
    let world = snapshotWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let before = body.snapshot()
    body.setLinearVelocity(vec3(6, 0, 0))
    check world.step(0.25) == {}
    let after = body.snapshot()

    check before.position.x.near(0)
    check before.linearVelocity.x.near(0)
    check after.position.x > 1
    check after.linearVelocity.x > 5
    check after.position.x.near(body.position.x)
    check after.centerOfMassPosition.x.near(body.centerOfMassPosition.x)

  test "velocity pairs are read changed and added under one body lock":
    let world = snapshotWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))

    body.setVelocities(vec3(1, 2, 3), vec3(4, 5, 6))
    check body.velocities == (
      linear: vec3(1, 2, 3), angular: vec3(4, 5, 6))
    body.addVelocities(vec3(0.5, -1, 2), vec3(-2, 1, 0.5))
    let changed = body.velocities
    check changed.linear == vec3(1.5, 1, 5)
    check changed.angular == vec3(2, 6, 6.5)

  test "complete transform and velocity replacement is immediately coherent":
    let world = snapshotWorld()
    defer: world.close()
    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 2, 0))

    body.setTransformAndVelocity(
      vec3(7, 8, 9), quatIdentity(), vec3(3, 2, 1), vec3(1, 2, 3))
    let state = body.snapshot()
    check state.position == vec3(7, 8, 9)
    check state.linearVelocity == vec3(3, 2, 1)
    check state.angularVelocity == vec3(1, 2, 3)
    check state.active

  test "unchanged transforms avoid activation and changed transforms wake":
    let world = snapshotWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    body.deactivate()
    check not body.isActive
    body.setTransformWhenChanged(body.position, body.rotation)
    check not body.isActive
    body.setTransformWhenChanged(vec3(1, 2, 0), body.rotation)
    check body.isActive

  test "locked degrees of freedom have explicit optional mass state":
    let world = snapshotWorld()
    defer: world.close()
    var config = defaultBodyConfig()
    config.allowedDOFs = {AllowedDOF.RotationYAxis}
    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 2, 0), config = config)
    let state = body.snapshot()

    check state.motion.isSome
    check state.motion.get.allowedDOFs == {AllowedDOF.RotationYAxis}
    check state.motion.get.mass.isNone
    check state.motion.get.massProperties.isNone

  test "invalid handles and cross-world batches fail before native locking":
    let first = snapshotWorld()
    defer: first.close()
    let second = snapshotWorld()
    defer: second.close()
    let firstBody = first.addDynamicBody(sphereShape(0.5), vec3(0, 1, 0))
    let secondBody = second.addDynamicBody(sphereShape(0.5), vec3(0, 1, 0))

    expect ValueError:
      discard first.bodySnapshots([firstBody, secondBody])
    firstBody.close()
    expect JoltError:
      discard firstBody.snapshot()
    expect JoltError:
      discard first.bodySnapshots([firstBody])

  test "compound state operations validate moving bodies and finite values":
    let world = snapshotWorld()
    defer: world.close()
    let dynamicBody = world.addDynamicBody(
      sphereShape(0.5), vec3(0, 2, 0))
    let staticBody = world.addStaticBody(
      boxShape(vec3(1, 0.5, 1)), vec3(0, -0.5, 0))

    expect JoltError:
      discard staticBody.velocities
    expect JoltError:
      staticBody.setVelocities(vec3(0, 0, 0), vec3(0, 0, 0))
    expect JoltError:
      staticBody.setTransformAndVelocity(
        vec3(0, 0, 0), quatIdentity(), vec3(0, 0, 0), vec3(0, 0, 0))
    expect ValueError:
      dynamicBody.setVelocities(
        vec3(NaN.float32, 0, 0), vec3(0, 0, 0))
    expect ValueError:
      dynamicBody.addVelocities(
        vec3(0, 0, 0), vec3(0, Inf.float32, 0))
    expect ValueError:
      dynamicBody.setTransformAndVelocity(
        vec3(0, NaN.float32, 0), quatIdentity(),
        vec3(0, 0, 0), vec3(0, 0, 0))

  test "empty batches are valid only while the world is open":
    let world = snapshotWorld()
    check world.bodySnapshots([]).len == 0
    world.close()
    expect JoltError:
      discard world.bodySnapshots([])
