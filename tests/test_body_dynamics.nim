import std/[math, unittest]
import jolt

proc dynamicsWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  result = newWorld(config)
  result.setGravity(vec3(0, 0, 0))

suite "Jolt body dynamics controls":
  test "torque changes angular velocity":
    let world = dynamicsWorld()
    defer: world.close()
    let body = world.addDynamicBody(boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 2, 0))
    body.setDamping(0, 0)
    body.addTorque(vec3(0, 1_000, 0))
    check world.step(1.0'f32 / 60.0'f32) == {}
    check body.angularVelocity.y > 0.05
    body.close()

  test "angular impulse changes rotation immediately":
    let world = dynamicsWorld()
    defer: world.close()
    let body = world.addDynamicBody(cylinderShape(0.5, 0.5), vec3(0, 2, 0))
    body.addAngularImpulse(vec3(0, 0, 300))
    check body.angularVelocity.z > 0.1
    for _ in 0 ..< 30:
      check world.step(1.0'f32 / 60.0'f32) == {}
    check abs(body.rotation.z) > 0.05
    body.close()

  test "linear and angular damping round trip and slow a body":
    let world = dynamicsWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    body.setDamping(3, 4)
    let configured = body.damping
    check abs(configured.linear - 3) < 0.001
    check abs(configured.angular - 4) < 0.001
    body.setLinearVelocity(vec3(10, 0, 0))
    body.setAngularVelocity(vec3(0, 8, 0))
    for _ in 0 ..< 60:
      check world.step(1.0'f32 / 60.0'f32) == {}
    check abs(body.linearVelocity.x) < 1
    check abs(body.angularVelocity.y) < 1
    body.close()

  test "explicit activation and deactivation update simulation state":
    let world = dynamicsWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    discard world.drainEvents()
    body.deactivate()
    check not body.isActive
    body.activate()
    check body.isActive
    let events = world.drainEvents()
    var sawDeactivated, sawActivated = false
    for event in events:
      if event.involves(body):
        sawDeactivated = sawDeactivated or event.kind == PhysicsEventKind.BodyDeactivated
        sawActivated = sawActivated or event.kind == PhysicsEventKind.BodyActivated
    check sawDeactivated
    check sawActivated
    body.close()

  test "point velocity includes angular motion around center of mass":
    let world = dynamicsWorld()
    defer: world.close()
    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 2, 0))
    body.setAngularVelocity(vec3(0, 0, 2))
    let velocity = body.pointVelocity(vec3(1, 2, 0))

    check abs(body.centerOfMassPosition.y - 2) < 1.0e-5
    check abs(velocity.x) < 1.0e-4
    check abs(velocity.y - 2) < 0.02
    check abs(velocity.z) < 1.0e-4

  test "motion quality velocity caps user data and manifolds round trip":
    let world = dynamicsWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))

    body.setMotionQuality(MotionQuality.LinearCast)
    body.setMaxLinearVelocity(12.5)
    body.setMaxAngularVelocity(7.25)
    body.setUserData(0xfedcba9876543210'u64)
    body.setUseManifoldReduction(false)
    body.resetSleepTimer()
    body.invalidateContactCache()

    check body.motionQuality == MotionQuality.LinearCast
    check abs(body.maxLinearVelocity - 12.5) < 1.0e-5
    check abs(body.maxAngularVelocity - 7.25) < 1.0e-5
    check body.userData == 0xfedcba9876543210'u64
    check not body.useManifoldReduction

  test "off-center force and impulse generate angular motion":
    let world = dynamicsWorld()
    defer: world.close()
    let forced = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(-2, 2, 0))
    let impulsed = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(2, 2, 0))
    forced.setDamping(0, 0)
    impulsed.setDamping(0, 0)
    forced.addForceAtPosition(vec3(2_000, 0, 0), vec3(-2, 3, 0))
    impulsed.addImpulseAtPosition(vec3(1_000, 0, 0), vec3(2, 3, 0))
    discard world.step(1.0'f32 / 60.0'f32)

    check abs(forced.angularVelocity.z) > 0.01
    check abs(impulsed.angularVelocity.z) > 0.1

  test "invalid dynamics controls fail before entering Jolt":
    let world = dynamicsWorld()
    defer: world.close()
    let dynamicBody = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let staticBody = world.addStaticBody(boxShape(vec3(1, 1, 1)), vec3(0, 5, 0))
    expect ValueError:
      dynamicBody.setDamping(-1, 0)
    expect ValueError:
      dynamicBody.addTorque(vec3(NaN.float32, 0, 0))
    expect ValueError:
      dynamicBody.addAngularImpulse(vec3(0, Inf.float32, 0))
    expect JoltError:
      discard staticBody.damping
    expect JoltError:
      staticBody.deactivate()
    expect ValueError:
      dynamicBody.setMaxLinearVelocity(0)
    expect ValueError:
      dynamicBody.setMaxAngularVelocity(Inf.float32)
    expect ValueError:
      dynamicBody.addForceAtPosition(vec3(1, 0, 0), vec3(NaN, 0, 0))
    expect ValueError:
      discard dynamicBody.pointVelocity(vec3(0, Inf.float32, 0))
    expect JoltError:
      discard staticBody.motionQuality
    expect JoltError:
      staticBody.setMaxLinearVelocity(10)
    dynamicBody.close()
    staticBody.close()
