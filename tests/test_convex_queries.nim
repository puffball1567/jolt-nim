import std/[math, options, unittest]
import jolt

proc queryWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  newWorld(config)

suite "Jolt convex shape queries":
  test "a rotated box cast reports the closest contact":
    let world = queryWorld()
    defer: world.close()
    let obstacle = world.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))
    let hit = world.castShape(
      boxShape(vec3(0.75, 0.25, 0.25)),
      vec3(-5, 0, 0),
      vec3(1, 0, 0),
      10,
      rotation = quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5))

    require hit.isSome
    check hit.get.hits(obstacle)
    check abs(hit.get.distance - 3.75) < 0.06
    check hit.get.normal.x < -0.9
    check abs(hit.get.contactPoint.x + 1) < 0.06

  test "capsule, cylinder and convex hull casts return sorted hits":
    let world = queryWorld()
    defer: world.close()
    let first = world.addStaticBody(sphereShape(0.6), vec3(0, 0, 0))
    let second = world.addStaticBody(sphereShape(0.6), vec3(4, 0, 0))
    let queryShapes = @[
      capsuleShape(0.45, 0.3),
      cylinderShape(0.45, 0.3),
      convexHullShape([
        vec3(-0.4, -0.3, -0.3), vec3(0.4, -0.3, -0.3),
        vec3(0, 0.45, -0.3), vec3(0, 0, 0.45)
      ])
    ]
    for shape in queryShapes:
      let hits = world.castShapeAll(
        shape, vec3(-5, 0, 0), vec3(1, 0, 0), 12)
      require hits.len == 2
      check hits[0].hits(first)
      check hits[1].hits(second)
      check hits[0].distance < hits[1].distance
      let limited = world.castShapeAll(
        shape, vec3(-5, 0, 0), vec3(1, 0, 0), 12,
        maxHits = 1)
      require limited.len == 1
      check limited[0].hits(first)

  test "overlap shape applies rotation and returns penetration metadata":
    let world = queryWorld()
    defer: world.close()
    let horizontal = world.addStaticBody(sphereShape(0.25), vec3(1.2, 0, 0))
    let vertical = world.addStaticBody(sphereShape(0.25), vec3(0, 1.2, 0))
    let query = boxShape(vec3(1.5, 0.2, 0.2))

    let unrotated = world.overlapShape(query, vec3(0, 0, 0))
    require unrotated.len == 1
    check unrotated[0].hits(horizontal)
    check unrotated[0].penetrationDepth > 0

    let rotated = world.overlapShape(
      query,
      vec3(0, 0, 0),
      rotation = quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5))
    require rotated.len == 1
    check rotated[0].hits(vertical)
    check rotated[0].penetrationDepth > 0

  test "general convex queries honor exact object-layer filters":
    const selectedLayer = CollisionLayer(2)
    var config = defaultWorldConfig()
    config.numThreads = 1
    config.collisionLayers.add collisionLayerConfig(1)
    let world = newWorld(config)
    defer: world.close()
    let ignored = world.addStaticBody(
      sphereShape(0.5), vec3(2, 0, 0), layer = nonMovingLayer)
    let selected = world.addStaticBody(
      sphereShape(0.5), vec3(5, 0, 0), layer = selectedLayer)

    let filteredHit = world.castShape(
      capsuleShape(0.4, 0.25),
      vec3(0, 0, 0),
      vec3(1, 0, 0),
      10,
      layer = some(selectedLayer))
    require filteredHit.isSome
    check filteredHit.get.hits(selected)
    check not filteredHit.get.hits(ignored)

    check world.overlapShape(
      boxShape(vec3(1, 1, 1)),
      vec3(2, 0, 0),
      layer = some(selectedLayer)).len == 0

  test "non-convex and invalid query inputs are rejected":
    let world = queryWorld()
    defer: world.close()
    let mesh = triangleMeshShape(
      [vec3(-1, 0, -1), vec3(1, 0, -1), vec3(0, 0, 1)],
      [0'u32, 2, 1])
    expect ValueError:
      discard world.castShape(
        mesh, vec3(0, 1, 0), vec3(0, -1, 0), 2)
    expect ValueError:
      discard world.castShape(
        sphereShape(0.5), vec3(0, 0, 0), vec3(0, 0, 0), 2)
    expect ValueError:
      discard world.castShapeAll(
        sphereShape(0.5), vec3(0, 0, 0), vec3(1, 0, 0), 2,
        maxHits = 0)
    expect ValueError:
      discard world.overlapShape(
        mesh, vec3(0, 0, 0))
    expect ValueError:
      discard world.overlapShape(
        sphereShape(0.5), vec3(0, 0, 0), maxHits = 0)
