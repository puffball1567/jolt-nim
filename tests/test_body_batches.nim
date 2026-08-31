import std/[math, unittest]
import jolt

proc batchWorld(maxBodies = 512'u): World =
  var config = defaultWorldConfig()
  config.maxBodies = maxBodies
  config.maxBodyPairs = max(64'u, maxBodies * 8)
  config.maxContactConstraints = max(64'u, maxBodies * 8)
  config.numThreads = 1
  newWorld(config)

suite "Jolt rigid body batches":
  test "mixed body specifications use the native batch insertion path":
    let world = batchWorld()
    defer: world.close()
    var specs = @[staticBodySpec(
      boxShape(vec3(12, 0.5, 12)), vec3(0, -0.5, 0))]
    for z in 0 ..< 6:
      for x in 0 ..< 8:
        var config = defaultBodyConfig()
        config.userData = uint64(z * 8 + x + 1)
        let shape =
          if (x + z) mod 2 == 0: boxShape(vec3(0.3, 0.3, 0.3))
          else: sphereShape(0.3)
        specs.add(dynamicBodySpec(
          shape, vec3(float32(x) - 3.5, 1 + float32(z) * 0.7, 0),
          config = config))

    let bodies = world.addBodies(specs)
    check bodies.len == 49
    check not bodies[0].isActive
    var ids: seq[uint32]
    for index, body in bodies:
      check body.isAlive
      let value = uint32(body.id)
      check value notin ids
      ids.add(value)
      if index > 0:
        check body.userData == uint64(index)

    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}
    for body in bodies[1 .. ^1]:
      check body.position.y > 0.15
      check body.position.y < 4.0

  test "batch activation is applied consistently":
    let world = batchWorld()
    defer: world.close()
    let specs = @[
      dynamicBodySpec(sphereShape(0.25), vec3(-1, 3, 0)),
      dynamicBodySpec(sphereShape(0.25), vec3(0, 3, 0)),
      dynamicBodySpec(sphereShape(0.25), vec3(1, 3, 0))]
    let bodies = world.addBodies(specs, activate = false)
    for body in bodies:
      check not body.isActive
    let before = bodies[0].position
    for _ in 0 ..< 10:
      discard world.step(1.0'f32 / 60.0'f32)
    check abs(bodies[0].position.y - before.y) < 1.0e-5

    bodies[0].activate()
    for _ in 0 ..< 10:
      discard world.step(1.0'f32 / 60.0'f32)
    check bodies[0].position.y < before.y - 0.05
    check not bodies[1].isActive
    check not bodies[2].isActive

  test "validation and allocation failures leave no partial batch":
    let world = batchWorld(4)
    defer: world.close()
    var invalidConfig = defaultBodyConfig()
    invalidConfig.mass = -1
    let invalidSpecs = @[
      dynamicBodySpec(sphereShape(0.25), vec3(0, 1, 0)),
      dynamicBodySpec(sphereShape(0.25), vec3(1, 1, 0),
                      config = invalidConfig)]
    expect ValueError:
      discard world.addBodies(invalidSpecs)

    var tooMany: seq[BodySpec]
    for index in 0 ..< 5:
      tooMany.add(dynamicBodySpec(
        sphereShape(0.2), vec3(float32(index), 1, 0)))
    expect JoltError:
      discard world.addBodies(tooMany)

    var capacity: seq[BodySpec]
    for index in 0 ..< 4:
      capacity.add(dynamicBodySpec(
        sphereShape(0.2), vec3(float32(index), 1, 0)))
    let bodies = world.addBodies(capacity)
    check bodies.len == 4
    for body in bodies:
      check body.isAlive

  test "batch close validates ownership constraints and duplicates atomically":
    let world = batchWorld(8)
    let otherWorld = batchWorld(2)
    defer:
      otherWorld.close()
      world.close()
    let bodies = world.addBodies(@[
      dynamicBodySpec(boxShape(vec3(0.2, 0.2, 0.2)), vec3(-1, 1, 0)),
      dynamicBodySpec(boxShape(vec3(0.2, 0.2, 0.2)), vec3(0, 1, 0)),
      dynamicBodySpec(boxShape(vec3(0.2, 0.2, 0.2)), vec3(1, 1, 0))])
    let foreign = otherWorld.addDynamicBody(
      sphereShape(0.2), vec3(0, 1, 0))

    expect ValueError:
      closeBodies([bodies[0], bodies[0]])
    expect JoltError:
      closeBodies([bodies[0], foreign])
    for body in bodies:
      check body.isAlive
    check foreign.isAlive

    let joint = bodies[0].addDistanceConstraint(
      bodies[1], vec3(0, 0, 0), vec3(0, 0, 0), 1, 1)
    expect JoltError:
      closeBodies(bodies)
    for body in bodies:
      check body.isAlive
    joint.close()

    closeBodies(bodies)
    for body in bodies:
      check not body.isAlive
    let replacements = world.addBodies(@[
      dynamicBodySpec(sphereShape(0.2), vec3(-1, 1, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(0, 1, 0)),
      dynamicBodySpec(sphereShape(0.2), vec3(1, 1, 0))])
    check replacements.len == 3
    for body in replacements:
      check body.isAlive
