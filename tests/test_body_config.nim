import std/[math, options, unittest]
import jolt

proc configWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  result = newWorld(config)
  result.setGravity(vec3(0, 0, 0))

suite "Jolt body creation configuration":
  test "creation properties reach the native body":
    let world = configWorld()
    defer: world.close()
    var config = defaultBodyConfig()
    config.motionQuality = MotionQuality.LinearCast
    config.mass = 12.5
    config.inertiaMultiplier = 2
    config.linearVelocity = vec3(3, 2, 1)
    config.angularVelocity = vec3(1, 2, 3)
    config.userData = 0x123456789abcdef0'u64
    config.allowSleeping = false
    config.collideKinematicVsNonDynamic = true
    config.useManifoldReduction = false
    config.applyGyroscopicForce = true
    config.enhancedInternalEdgeRemoval = true
    config.friction = 0.7
    config.restitution = 0.6
    config.linearDamping = 0.3
    config.angularDamping = 0.4
    config.maxLinearVelocity = 40
    config.maxAngularVelocity = 30
    config.gravityFactor = 0.25
    config.numVelocityStepsOverride = 7
    config.numPositionStepsOverride = 5

    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 2, 0), config = config)

    check abs(body.mass - 12.5) < 0.001
    check body.allowedDOFs == allAllowedDOFs()
    check body.motionQuality == MotionQuality.LinearCast
    check abs(body.linearVelocity.x - 3) < 0.001
    check abs(body.linearVelocity.y - 2) < 0.001
    check abs(body.linearVelocity.z - 1) < 0.001
    check abs(body.angularVelocity.x - 1) < 0.001
    check abs(body.angularVelocity.y - 2) < 0.001
    check abs(body.angularVelocity.z - 3) < 0.001
    check body.userData == 0x123456789abcdef0'u64
    check not body.allowsSleeping
    check body.collidesKinematicVsNonDynamic
    check body.appliesGyroscopicForce
    check body.usesEnhancedInternalEdgeRemoval
    check not body.useManifoldReduction
    check abs(body.friction - 0.7) < 0.001
    check abs(body.restitution - 0.6) < 0.001
    check abs(body.damping.linear - 0.3) < 0.001
    check abs(body.damping.angular - 0.4) < 0.001
    check abs(body.maxLinearVelocity - 40) < 0.001
    check abs(body.maxAngularVelocity - 30) < 0.001
    check abs(body.gravityFactor - 0.25) < 0.001
    check body.solverStepOverrides == (velocity: 7'u32, position: 5'u32)

    body.setMass(25)
    body.setAllowSleeping(true)
    body.setCollideKinematicVsNonDynamic(false)
    body.setApplyGyroscopicForce(false)
    body.setEnhancedInternalEdgeRemoval(false)
    body.setSolverStepOverrides(3, 2)
    check abs(body.mass - 25) < 0.001
    check body.allowsSleeping
    check not body.collidesKinematicVsNonDynamic
    check not body.appliesGyroscopicForce
    check not body.usesEnhancedInternalEdgeRemoval
    check body.solverStepOverrides == (velocity: 3'u32, position: 2'u32)

  test "explicit mass controls impulse response":
    let world = configWorld()
    defer: world.close()
    var lightConfig = defaultBodyConfig()
    lightConfig.mass = 10
    lightConfig.linearDamping = 0
    var heavyConfig = lightConfig
    heavyConfig.mass = 100
    let shape = boxShape(vec3(0.5, 0.5, 0.5))
    let light = world.addDynamicBody(shape, vec3(-2, 2, 0), config = lightConfig)
    let heavy = world.addDynamicBody(shape, vec3(2, 2, 0), config = heavyConfig)

    light.addImpulse(vec3(100, 0, 0))
    heavy.addImpulse(vec3(100, 0, 0))

    check abs(light.linearVelocity.x - 10) < 0.01
    check abs(heavy.linearVelocity.x - 1) < 0.01
    check abs(light.linearVelocity.x / heavy.linearVelocity.x - 10) < 0.02

  test "inertia multiplier controls angular impulse response":
    let world = configWorld()
    defer: world.close()
    var normalConfig = defaultBodyConfig()
    normalConfig.mass = 10
    normalConfig.inertiaMultiplier = 1
    normalConfig.angularDamping = 0
    var highInertiaConfig = normalConfig
    highInertiaConfig.inertiaMultiplier = 4
    let shape = boxShape(vec3(0.5, 0.5, 0.5))
    let normal = world.addDynamicBody(shape, vec3(-2, 2, 0), config = normalConfig)
    let highInertia = world.addDynamicBody(
      shape, vec3(2, 2, 0), config = highInertiaConfig)

    normal.addAngularImpulse(vec3(0, 10, 0))
    highInertia.addAngularImpulse(vec3(0, 10, 0))

    check normal.angularVelocity.y > highInertia.angularVelocity.y
    check abs(normal.angularVelocity.y / highInertia.angularVelocity.y - 4) < 0.05

  test "custom principal inertia and orientation control angular response":
    let world = configWorld()
    defer: world.close()
    var alignedConfig = defaultBodyConfig()
    alignedConfig.angularDamping = 0
    alignedConfig.massProperties = some(bodyMassProperties(
      12, vec3(1, 10, 10)))
    var rotatedConfig = alignedConfig
    rotatedConfig.massProperties = some(bodyMassProperties(
      12,
      vec3(1, 10, 10),
      quatFromAxisAngle(vec3(0, 0, 1), PI.float32 / 2)))
    let shape = boxShape(vec3(0.5, 0.5, 0.5))
    let aligned = world.addDynamicBody(
      shape, vec3(-2, 2, 0), config = alignedConfig)
    let rotated = world.addDynamicBody(
      shape, vec3(2, 2, 0), config = rotatedConfig)

    aligned.addAngularImpulse(vec3(10, 0, 0))
    rotated.addAngularImpulse(vec3(10, 0, 0))
    check aligned.angularVelocity.x > rotated.angularVelocity.x * 9
    check abs(aligned.mass - 12) < 1.0e-4
    let actual = aligned.massProperties
    check abs(actual.mass - 12) < 1.0e-4
    check abs(actual.inertiaDiagonal.x + actual.inertiaDiagonal.y +
              actual.inertiaDiagonal.z - 21) < 1.0e-3

  test "runtime mass properties replace the full tensor and wake the body":
    let world = configWorld()
    defer: world.close()
    var config = defaultBodyConfig()
    config.angularDamping = 0
    config.linearDamping = 0
    config.massProperties = some(bodyMassProperties(10, vec3(10, 10, 10)))
    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 2, 0), config = config)
    body.deactivate()
    body.setMassProperties(bodyMassProperties(20, vec3(2, 20, 20)))
    check body.isActive
    let actual = body.massProperties
    check abs(actual.mass - 20) < 1.0e-4
    check abs(actual.inertiaDiagonal.x + actual.inertiaDiagonal.y +
              actual.inertiaDiagonal.z - 42) < 1.0e-3

    body.addImpulse(vec3(20, 0, 0))
    body.addAngularImpulse(vec3(10, 0, 0))
    check abs(body.linearVelocity.x - 1) < 1.0e-3
    check abs(body.angularVelocity.x - 5) < 1.0e-3

  test "custom mass properties participate in native body batches":
    let world = configWorld()
    defer: world.close()
    var lowConfig = defaultBodyConfig()
    lowConfig.angularDamping = 0
    lowConfig.massProperties = some(bodyMassProperties(8, vec3(2, 8, 8)))
    var highConfig = lowConfig
    highConfig.massProperties = some(bodyMassProperties(8, vec3(8, 8, 8)))
    let bodies = world.addBodies(@[
      dynamicBodySpec(sphereShape(0.5), vec3(-1, 2, 0), config = lowConfig),
      dynamicBodySpec(sphereShape(0.5), vec3(1, 2, 0), config = highConfig)])
    bodies[0].addAngularImpulse(vec3(8, 0, 0))
    bodies[1].addAngularImpulse(vec3(8, 0, 0))
    check abs(bodies[0].angularVelocity.x - 4) < 1.0e-3
    check abs(bodies[1].angularVelocity.x - 1) < 1.0e-3

  test "custom mass property validation is atomic":
    let world = configWorld()
    defer: world.close()
    let shape = sphereShape(0.5)
    var config = defaultBodyConfig()
    config.massProperties = some(bodyMassProperties(0, vec3(1, 1, 1)))
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config.massProperties = some(bodyMassProperties(1, vec3(1, 0, 1)))
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config.massProperties = some(bodyMassProperties(
      1, vec3(1, 1, 1), Quat(x: NaN.float32, w: 1)))
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.mass = 2
    config.massProperties = some(bodyMassProperties(2, vec3(1, 1, 1)))
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.inertiaMultiplier = 2
    config.massProperties = some(bodyMassProperties(2, vec3(1, 1, 1)))
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.massProperties = some(bodyMassProperties(2, vec3(1, 1, 1)))
    expect ValueError:
      discard world.addStaticBody(shape, vec3(0, 2, 0), config = config)

    let dynamicBody = world.addDynamicBody(shape, vec3(0, 2, 0))
    let staticBody = world.addStaticBody(shape, vec3(0, 5, 0))
    expect ValueError:
      dynamicBody.setMassProperties(bodyMassProperties(1, vec3(1, -1, 1)))
    expect JoltError:
      discard staticBody.massProperties
    expect JoltError:
      staticBody.setMassProperties(bodyMassProperties(1, vec3(1, 1, 1)))

  test "plane 2D degrees of freedom filter motion":
    let world = configWorld()
    defer: world.close()
    var config = defaultBodyConfig()
    config.allowedDOFs = plane2DAllowedDOFs()
    let body = world.addDynamicBody(
      sphereShape(0.5), vec3(0, 2, 0), config = config)

    body.setLinearVelocity(vec3(3, 4, 5))
    body.setAngularVelocity(vec3(6, 7, 8))
    body.addImpulse(vec3(10, 20, 30))
    body.addAngularImpulse(vec3(40, 50, 60))

    check body.allowedDOFs == plane2DAllowedDOFs()
    check abs(body.linearVelocity.x) > 0.1
    check abs(body.linearVelocity.y) > 0.1
    check abs(body.linearVelocity.z) < 0.001
    check abs(body.angularVelocity.x) < 0.001
    check abs(body.angularVelocity.y) < 0.001
    check abs(body.angularVelocity.z) > 0.1

  test "rotation-only bodies lock translation":
    let world = configWorld()
    defer: world.close()
    var config = defaultBodyConfig()
    config.allowedDOFs = {AllowedDOF.RotationYAxis}
    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 2, 0), config = config)

    body.addImpulse(vec3(100, 100, 100))
    body.addAngularImpulse(vec3(0, 10, 0))

    check body.allowedDOFs == {AllowedDOF.RotationYAxis}
    check abs(body.linearVelocity.x) < 0.001
    check abs(body.linearVelocity.y) < 0.001
    check abs(body.linearVelocity.z) < 0.001
    check body.angularVelocity.y > 0.01
    expect JoltError:
      discard body.mass

  test "invalid creation settings fail before entering Jolt":
    let world = configWorld()
    defer: world.close()
    let shape = sphereShape(0.5)
    var config = defaultBodyConfig()
    config.allowedDOFs = {}
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.mass = -1
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.mass = 10
    expect ValueError:
      discard world.addKinematicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.inertiaMultiplier = 0
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.linearVelocity = vec3(501, 0, 0)
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.numVelocityStepsOverride = 256
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    config = defaultBodyConfig()
    config.restitution = NaN.float32
    expect ValueError:
      discard world.addDynamicBody(shape, vec3(0, 2, 0), config = config)
    let dynamicBody = world.addDynamicBody(shape, vec3(0, 2, 0))
    let staticBody = world.addStaticBody(shape, vec3(0, 5, 0))
    expect ValueError:
      dynamicBody.setMass(0)
    expect ValueError:
      dynamicBody.setSolverStepOverrides(256, 0)
    expect JoltError:
      staticBody.setMass(10)
    expect JoltError:
      staticBody.setAllowSleeping(false)
