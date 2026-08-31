import std/[math, unittest]
import jolt

proc smallWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 256
  config.maxBodyPairs = 1_024
  config.maxContactConstraints = 1_024
  config.numThreads = 1
  newWorld(config)

suite "Jolt shapes and motion":
  test "sphere and capsule settle on a floor":
    let world = smallWorld()
    defer:
      world.close()

    let floor = world.addStaticBody(
      boxShape(vec3(20, 1, 20)),
      vec3(0, -1, 0)
    )
    let sphere = world.addDynamicBody(sphereShape(0.5), vec3(-1, 3, 0))
    let capsule = world.addDynamicBody(capsuleShape(0.5, 0.5), vec3(1, 3, 0))

    world.optimizeBroadPhase()
    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}

    check sphere.position.y > 0.45
    check sphere.position.y < 0.60
    check capsule.position.y > 0.90
    check capsule.position.y < 1.10
    check sphere.shape.kind == ShapeKind.Sphere
    check capsule.shape.kind == ShapeKind.Capsule

    capsule.close()
    sphere.close()
    floor.close()

  test "gravity, material, velocity, force and impulse are controllable":
    let world = smallWorld()
    defer:
      world.close()

    world.setGravity(vec3(0, 0, 0))
    check abs(world.gravity.y) < 1.0e-6

    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    body.setFriction(0.25)
    body.setRestitution(0.8)
    body.setGravityFactor(0.5)
    check abs(body.friction - 0.25) < 1.0e-6
    check abs(body.restitution - 0.8) < 1.0e-6
    check abs(body.gravityFactor - 0.5) < 1.0e-6

    body.setLinearVelocity(vec3(0, 0, 0))
    body.setAngularVelocity(vec3(0, 1, 0))
    body.addForce(vec3(1, 0, 0))
    body.addImpulse(vec3(2, 0, 0))
    check world.step(1.0'f32 / 60.0'f32) == {}
    check body.position.x > 0
    check body.linearVelocity.x > 0
    check body.angularVelocity.y > 0

    body.close()

  test "rotation, transform and kinematic targets round trip":
    let world = smallWorld()
    defer:
      world.close()

    world.setGravity(vec3(0, 0, 0))
    let initialRotation = quatFromAxisAngle(vec3(0, 1, 0), PI.float32 * 0.25)
    let dynamicBody = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)),
      vec3(0, 1, 0),
      initialRotation
    )
    check abs(dynamicBody.rotation.y - initialRotation.y) < 1.0e-5
    check abs(dynamicBody.rotation.w - initialRotation.w) < 1.0e-5

    let movedRotation = quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)
    dynamicBody.setTransform(vec3(2, 3, 4), movedRotation)
    check abs(dynamicBody.position.x - 2) < 1.0e-5
    check abs(dynamicBody.rotation.z - movedRotation.z) < 1.0e-5

    let kinematic = world.addKinematicBody(
      boxShape(vec3(0.5, 0.5, 0.5)),
      vec3(0, 0, 0)
    )
    kinematic.moveKinematic(vec3(1, 0, 0), quatIdentity(), 1.0'f32 / 60.0'f32)
    check world.step(1.0'f32 / 60.0'f32) == {}
    check kinematic.position.x > 0.95

    kinematic.close()
    dynamicBody.close()

  test "invalid shape and world inputs fail before entering Jolt":
    expect ValueError:
      discard sphereShape(0)
    expect ValueError:
      discard capsuleShape(-1, 0.5)
    expect ValueError:
      discard boxShape(vec3(1, 0, 1))
    expect ValueError:
      discard quatFromAxisAngle(vec3(0, 0, 0), 1)

    var config = defaultWorldConfig()
    config.maxBodies = 0
    expect ValueError:
      discard newWorld(config)
