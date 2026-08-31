import std/[math, options, unittest]
import jolt

proc bodyWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 64
  config.maxBodyPairs = 256
  config.maxContactConstraints = 256
  config.numThreads = 1
  result = newWorld(config)
  result.setGravity(vec3(0, 0, 0))

suite "Jolt live body shapes and buoyancy":
  test "shape replacement updates descriptors bounds and mass":
    let world = bodyWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let sphereMass = body.mass

    body.setShape(boxShape(vec3(1, 1, 1)))

    check body.shape.kind == ShapeKind.Box
    check body.mass > sphereMass * 10
    let hit = world.castRay(vec3(-5, 2, 0), vec3(1, 0, 0), 10)
    check hit.isSome
    check abs(hit.get.distance - 4) < 0.05

  test "shape replacement can preserve existing mass properties":
    let world = bodyWorld()
    defer: world.close()
    var config = defaultBodyConfig()
    config.mass = 20
    let body = world.addDynamicBody(
      sphereShape(0.5), vec3(0, 2, 0), config = config)

    body.setShape(boxShape(vec3(1, 1, 1)), updateMassProperties = false)

    check abs(body.mass - 20) < 0.001
    check body.shape.kind == ShapeKind.Box

  test "shape replacement protects constraints and shape rules":
    let world = bodyWorld()
    defer: world.close()
    let anchor = world.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)), vec3(0, 2, 0))
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let joint = addFixedConstraint(anchor, body)
    expect JoltError:
      body.setShape(boxShape(vec3(1, 1, 1)))
    joint.close()
    expect ValueError:
      body.setShape(planeShape(vec3(0, 1, 0)))

  test "submerged bodies receive a buoyancy impulse":
    let world = bodyWorld()
    defer: world.close()
    let submerged = world.addDynamicBody(sphereShape(0.5), vec3(0, -2, 0))
    let dry = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let gravity = vec3(0, -9.81, 0)

    check submerged.applyBuoyancyImpulse(
      vec3(0, 0, 0), vec3(0, 2, 0), 1.5, 0.3, 0.05,
      vec3(0, 0, 0), gravity, 1.0'f32 / 60.0'f32)
    check submerged.linearVelocity.y > 0
    check not dry.applyBuoyancyImpulse(
      vec3(0, 0, 0), vec3(0, 1, 0), 1.5, 0.3, 0.05,
      vec3(0, 0, 0), gravity, 1.0'f32 / 60.0'f32)
    check abs(dry.linearVelocity.y) < 0.001

  test "buoyancy validates body type and fluid parameters":
    let world = bodyWorld()
    defer: world.close()
    let dynamicBody = world.addDynamicBody(sphereShape(0.5), vec3(0, -2, 0))
    let staticBody = world.addStaticBody(sphereShape(0.5), vec3(0, -2, 0))
    expect ValueError:
      discard dynamicBody.applyBuoyancyImpulse(
        vec3(0, 0, 0), vec3(0, 0, 0), 1, 0, 0,
        vec3(0, 0, 0), vec3(0, -9.81, 0), 1.0'f32 / 60.0'f32)
    expect ValueError:
      discard dynamicBody.applyBuoyancyImpulse(
        vec3(0, 0, 0), vec3(0, 1, 0), -1, 0, 0,
        vec3(0, 0, 0), vec3(0, -9.81, 0), 1.0'f32 / 60.0'f32)
    expect ValueError:
      discard dynamicBody.applyBuoyancyImpulse(
        vec3(0, 0, 0), vec3(0, 1, 0), 1, NaN.float32, 0,
        vec3(0, 0, 0), vec3(0, -9.81, 0), 1.0'f32 / 60.0'f32)
    expect ValueError:
      discard dynamicBody.applyBuoyancyImpulse(
        vec3(0, 0, 0), vec3(0, 1, 0), 1, 0, 0,
        vec3(0, 0, 0), vec3(0, -9.81, 0), 0)
    expect JoltError:
      discard staticBody.applyBuoyancyImpulse(
        vec3(0, 0, 0), vec3(0, 1, 0), 1, 0, 0,
        vec3(0, 0, 0), vec3(0, -9.81, 0), 1.0'f32 / 60.0'f32)
