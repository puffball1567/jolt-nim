import std/[math, options, sequtils, unittest]
import jolt

proc queryWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  newWorld(config)

suite "Jolt spatial queries":
  test "all-hit ray casts are sorted by distance":
    let world = queryWorld()
    defer: world.close()
    let nearBody = world.addStaticBody(sphereShape(0.5), vec3(0, 8, 0))
    let middleBody = world.addStaticBody(sphereShape(0.5), vec3(0, 5, 0))
    let farBody = world.addStaticBody(sphereShape(0.5), vec3(0, 2, 0))

    let hits = world.castRayAll(vec3(0, 10, 0), vec3(0, -1, 0), 10)
    check hits.len == 3
    check hits[0].hits(nearBody)
    check hits[1].hits(middleBody)
    check hits[2].hits(farBody)
    check hits[0].distance < hits[1].distance
    check hits[1].distance < hits[2].distance

    nearBody.close()
    middleBody.close()
    farBody.close()

  test "closest and all-hit ray casts agree on the first hit":
    let world = queryWorld()
    defer: world.close()
    let first = world.addStaticBody(boxShape(vec3(0.5, 0.5, 0.5)), vec3(2, 0, 0))
    let second = world.addStaticBody(boxShape(vec3(0.5, 0.5, 0.5)), vec3(5, 0, 0))
    let closest = world.castRay(vec3(0, 0, 0), vec3(1, 0, 0), 10)
    let all = world.castRayAll(vec3(0, 0, 0), vec3(1, 0, 0), 10)
    require closest.isSome
    require all.len == 2
    check closest.get.bodyId == all[0].bodyId
    check abs(closest.get.distance - all[0].distance) < 0.001
    check all[0].hits(first)
    check all[1].hits(second)
    first.close()
    second.close()

  test "all-hit ray capacity keeps the nearest results":
    let world = queryWorld()
    defer: world.close()
    var bodies: seq[Body]
    for index in 0 ..< 8:
      bodies.add(world.addStaticBody(sphereShape(0.25), vec3(index + 1, 0, 0)))
    let hits = world.castRayAll(vec3(0, 0, 0), vec3(1, 0, 0), 12, maxHits = 3)
    check hits.len == 3
    for index in 0 ..< 3:
      check hits[index].hits(bodies[index])
    for body in bodies:
      body.close()

  test "sphere overlap returns body, depth, point and normal":
    let world = queryWorld()
    defer: world.close()
    let first = world.addStaticBody(sphereShape(0.75), vec3(-0.5, 2, 0))
    let second = world.addStaticBody(boxShape(vec3(0.5, 0.5, 0.5)), vec3(1, 2, 0))
    discard world.addStaticBody(sphereShape(0.5), vec3(8, 2, 0))

    let hits = world.overlapSphere(vec3(0, 2, 0), 1.25)
    check hits.len == 2
    check hits.anyIt(it.hits(first))
    check hits.anyIt(it.hits(second))
    for hit in hits:
      check hit.penetrationDepth >= 0
      let normalLength = sqrt(
        hit.normal.x * hit.normal.x + hit.normal.y * hit.normal.y + hit.normal.z * hit.normal.z)
      check abs(normalLength - 1) < 0.01

  test "sphere overlap follows body transform updates":
    let world = queryWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(6, 3, 0))
    check world.overlapSphere(vec3(0, 3, 0), 1).len == 0
    body.setTransform(vec3(0.25, 3, 0), quatIdentity())
    let hits = world.overlapSphere(vec3(0, 3, 0), 1)
    check hits.len == 1
    check hits[0].hits(body)
    body.close()

  test "query misses and invalid inputs are handled":
    let world = queryWorld()
    defer: world.close()
    discard world.addStaticBody(sphereShape(0.5), vec3(0, 0, 0))
    check world.castRayAll(vec3(5, 5, 5), vec3(1, 0, 0), 2).len == 0
    check world.overlapSphere(vec3(5, 5, 5), 0.5).len == 0
    expect ValueError:
      discard world.castRayAll(vec3(0, 0, 0), vec3(0, 0, 0), 10)
    expect ValueError:
      discard world.castRayAll(vec3(0, 0, 0), vec3(1, 0, 0), 10, maxHits = 0)
    expect ValueError:
      discard world.overlapSphere(vec3(0, 0, 0), 0)
    expect ValueError:
      discard world.overlapSphere(vec3(0, 0, 0), 1, maxHits = -1)
