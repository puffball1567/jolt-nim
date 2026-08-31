import std/unittest
import jolt

suite "Jolt world":
  test "a dynamic box settles on a static floor":
    var config = defaultWorldConfig()
    config.maxBodies = 1_024
    config.maxBodyPairs = 1_024
    config.maxContactConstraints = 1_024
    config.numThreads = 1

    let world = newWorld(config)
    defer:
      world.close()

    let floor = world.addStaticBody(
      boxShape(vec3(100, 1, 100)),
      vec3(0, -1, 0)
    )
    let box = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)),
      vec3(0, 2, 0)
    )

    world.optimizeBroadPhase()
    for _ in 0 ..< 120:
      check world.step(1.0'f32 / 60.0'f32) == {}

    check box.position.y > 0.45
    check box.position.y < 0.60

    box.close()
    floor.close()
    check not box.isAlive

  test "closing a world invalidates its body handles":
    var config = defaultWorldConfig()
    config.maxBodies = 16
    config.maxBodyPairs = 16
    config.maxContactConstraints = 16
    config.numThreads = 1

    let world = newWorld(config)
    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)),
      vec3(0, 2, 0)
    )

    world.close()
    world.close()
    check not world.isOpen
    check not body.isAlive
    expect JoltError:
      discard body.position

    body.close()
    body.close()

  test "multiple worlds share the process-wide Jolt lifecycle":
    var config = defaultWorldConfig()
    config.maxBodies = 16
    config.maxBodyPairs = 16
    config.maxContactConstraints = 16
    config.numThreads = 1

    let first = newWorld(config)
    let second = newWorld(config)
    first.close()

    let body = second.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)),
      vec3(0, 2, 0)
    )
    check body.isAlive
    check second.step(1.0'f32 / 60.0'f32) == {}

    body.close()
    second.close()
