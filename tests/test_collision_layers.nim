import std/[options, sequtils, unittest]
import jolt

const
  queryLayer = CollisionLayer(2)
  ignoredLayer = CollisionLayer(3)

proc layeredWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  config.collisionLayers = @[
    collisionLayerConfig(0),
    collisionLayerConfig(1),
    collisionLayerConfig(1),
    collisionLayerConfig(1)
  ]
  config.collisionPairs = @[
    collisionPair(nonMovingLayer, movingLayer),
    collisionPair(movingLayer, movingLayer)
  ]
  newWorld(config)

suite "Jolt collision layers and query filters":
  test "collision matrix controls simulation pairs":
    let world = layeredWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)),
      vec3(0, -0.5, 0)
    )
    let colliding = world.addDynamicBody(
      sphereShape(0.5), vec3(-2, 3, 0), layer = movingLayer)
    let ignored = world.addDynamicBody(
      sphereShape(0.5), vec3(2, 3, 0), layer = queryLayer)

    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}

    check colliding.position.y > 0.4
    check colliding.position.y < 0.7
    check ignored.position.y < -5

  test "body collision layer can be read and changed at runtime":
    let world = layeredWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)),
      vec3(0, -0.5, 0)
    )
    let body = world.addDynamicBody(
      sphereShape(0.5), vec3(0, 3, 0), layer = queryLayer)
    check body.collisionLayer == queryLayer

    body.setCollisionLayer(movingLayer)
    check body.collisionLayer == movingLayer
    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}
    check body.position.y > 0.4
    check body.position.y < 0.7

  test "ray, sphere cast and overlap filters select one layer":
    let world = layeredWorld()
    defer: world.close()
    let nearBody = world.addStaticBody(
      sphereShape(0.5), vec3(2, 0, 0), layer = queryLayer)
    let farBody = world.addStaticBody(
      sphereShape(0.5), vec3(5, 0, 0), layer = ignoredLayer)

    let allRays = world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10)
    check allRays.len == 2
    check allRays.anyIt(it.hits(nearBody))
    check allRays.anyIt(it.hits(farBody))

    let filteredRay = world.castRay(
      vec3(0, 0, 0), vec3(1, 0, 0), 10,
      layer = some(ignoredLayer))
    require filteredRay.isSome
    check filteredRay.get.hits(farBody)

    let filteredRays = world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10,
      layer = some(queryLayer))
    check filteredRays.len == 1
    check filteredRays[0].hits(nearBody)

    let sphereHit = world.castSphere(
      0.25, vec3(0, 0, 0), vec3(1, 0, 0), 10,
      layer = some(ignoredLayer))
    require sphereHit.isSome
    check sphereHit.get.hits(farBody)

    let sphereHits = world.castSphereAll(
      0.25, vec3(0, 0, 0), vec3(1, 0, 0), 10,
      layer = some(queryLayer))
    check sphereHits.len == 1
    check sphereHits[0].hits(nearBody)

    let overlap = world.overlapSphere(
      vec3(2, 0, 0), 1, layer = some(queryLayer))
    check overlap.len == 1
    check overlap[0].hits(nearBody)
    check world.overlapSphere(
      vec3(2, 0, 0), 1, layer = some(ignoredLayer)).len == 0

  test "reusable layer sets filter every spatial query family":
    let world = layeredWorld()
    defer: world.close()
    let nearBody = world.addStaticBody(
      sphereShape(0.5), vec3(2, 0, 0), layer = queryLayer)
    let farBody = world.addStaticBody(
      sphereShape(0.5), vec3(5, 0, 0), layer = ignoredLayer)
    let excludedBody = world.addStaticBody(
      sphereShape(0.5), vec3(8, 0, 0), layer = nonMovingLayer)
    world.optimizeBroadPhase()

    let layers = queryLayerSet([queryLayer, ignoredLayer, queryLayer])
    check layers.len == 2
    check layers[0] == queryLayer
    check layers[1] == ignoredLayer

    let closestRay = world.castRay(
      vec3(0, 0, 0), vec3(1, 0, 0), 10, layers)
    require closestRay.isSome
    check closestRay.get.hits(nearBody)
    let rays = world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10, layers)
    check rays.len == 2
    check rays[0].hits(nearBody)
    check rays[1].hits(farBody)
    check not rays.anyIt(it.hits(excludedBody))

    let closestSphere = world.castSphere(
      0.1, vec3(0, 0, 0), vec3(1, 0, 0), 10, layers)
    require closestSphere.isSome
    check closestSphere.get.hits(nearBody)
    let spheres = world.castSphereAll(
      0.1, vec3(0, 0, 0), vec3(1, 0, 0), 10, layers)
    check spheres.len == 2

    let queryShape = boxShape(vec3(0.1, 0.1, 0.1))
    let closestShape = world.castShape(
      queryShape, vec3(0, 0, 0), vec3(1, 0, 0), 10, layers)
    require closestShape.isSome
    check closestShape.get.hits(nearBody)
    check world.castShapeAll(
      queryShape, vec3(0, 0, 0), vec3(1, 0, 0), 10,
      layers).len == 2

    check world.overlapSphere(
      vec3(2, 0, 0), 0.75, layers).anyIt(it.hits(nearBody))
    check world.overlapShape(
      sphereShape(0.75), vec3(5, 0, 0), layers).anyIt(it.hits(farBody))
    check world.collidePoint(
      vec3(2, 0, 0), layers).anyIt(it == nearBody.id)

    check world.broadPhaseQueryBox(
      vec3(1, -1, -1), vec3(6, 1, 1), layers).len == 2
    check world.broadPhaseQuerySphere(
      vec3(3.5, 0, 0), 3, layers).len == 2
    check world.broadPhaseQueryPoint(
      vec3(5, 0, 0), layers).anyIt(it == farBody.id)
    check world.broadPhaseQueryOrientedBox(
      vec3(3.5, 0, 0), vec3(3, 1, 1), layers).len == 2
    let broadRays = world.broadPhaseCastRay(
      vec3(0, 0, 0), vec3(1, 0, 0), 10, layers)
    let broadBoxes = world.broadPhaseCastBox(
      vec3(0, 0, 0), vec3(0.1, 0.1, 0.1),
      vec3(1, 0, 0), 10, layers)
    check broadRays.len == 2
    check broadRays[0].bodyId == nearBody.id
    check broadBoxes.len == 2

    check world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10,
      layers, maxHits = 1).len == 1

    expect ValueError:
      discard queryLayerSet(newSeq[CollisionLayer]())
    expect ValueError:
      discard world.castRay(
        vec3(0, 0, 0), vec3(1, 0, 0), 10,
        queryLayerSet([CollisionLayer(9)]))

  test "native body sets include or exclude bodies from narrow-phase queries":
    let world = layeredWorld()
    defer: world.close()
    let nearBody = world.addStaticBody(
      sphereShape(0.5), vec3(2, 0, 0), layer = queryLayer)
    let farBody = world.addStaticBody(
      sphereShape(0.5), vec3(5, 0, 0), layer = ignoredLayer)
    let layers = queryLayerSet([queryLayer, ignoredLayer])
    let excludeNear = excludeBodies([nearBody.id, nearBody.id])
    let includeFar = includeBodies([farBody.id])

    check excludeNear.len == 1
    check excludeNear.filterMode == QueryBodyFilterMode.Exclude
    check includeFar.filterMode == QueryBodyFilterMode.IncludeOnly

    let ray = world.castRay(
      vec3(0, 0, 0), vec3(1, 0, 0), 10, layers,
      bodyFilter = excludeNear)
    require ray.isSome
    check ray.get.hits(farBody)
    let rays = world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10, layers,
      bodyFilter = includeFar)
    check rays.len == 1
    check rays[0].hits(farBody)

    let sphereHit = world.castSphere(
      0.1, vec3(0, 0, 0), vec3(1, 0, 0), 10, layers,
      bodyFilter = excludeNear)
    require sphereHit.isSome
    check sphereHit.get.hits(farBody)
    check world.castSphereAll(
      0.1, vec3(0, 0, 0), vec3(1, 0, 0), 10, layers,
      bodyFilter = includeFar).len == 1

    let queryShape = boxShape(vec3(0.1, 0.1, 0.1))
    let shapeHit = world.castShape(
      queryShape, vec3(0, 0, 0), vec3(1, 0, 0), 10, layers,
      bodyFilter = excludeNear)
    require shapeHit.isSome
    check shapeHit.get.hits(farBody)
    check world.castShapeAll(
      queryShape, vec3(0, 0, 0), vec3(1, 0, 0), 10, layers,
      bodyFilter = includeFar).len == 1

    let sphereOverlaps = world.overlapSphere(
      vec3(3.5, 0, 0), 3, layers,
      bodyFilter = includeFar)
    let shapeOverlaps = world.overlapShape(
      sphereShape(3), vec3(3.5, 0, 0), layers,
      bodyFilter = includeFar)
    check sphereOverlaps.len == 1
    check sphereOverlaps.allIt(it.hits(farBody))
    check shapeOverlaps.len == 1
    check shapeOverlaps.allIt(it.hits(farBody))
    check world.collidePoint(
      vec3(2, 0, 0), layers, bodyFilter = excludeNear).len == 0
    check world.collidePoint(
      vec3(2, 0, 0), bodyFilter = includeBodies([nearBody.id])).len == 1

    expect ValueError:
      discard includeBodies(newSeq[BodyId]())
    let stale = world.addStaticBody(sphereShape(0.25), vec3(20, 0, 0))
    let staleFilter = excludeBodies([stale.id])
    stale.close()
    expect ValueError:
      discard world.castRay(
        vec3(0, 0, 0), vec3(1, 0, 0), 10,
        bodyFilter = staleFilter)

  test "exact sub-shape sets filter every narrow-phase query family":
    let world = layeredWorld()
    defer: world.close()
    let compound = world.addStaticBody(
      staticCompoundShape([
        compoundChild(sphereShape(0.5), vec3(2, 0, 0)),
        compoundChild(sphereShape(0.5), vec3(5, 0, 0))
      ]),
      vec3(0, 0, 0),
      layer = queryLayer)
    let layers = queryLayerSet([queryLayer, ignoredLayer])

    let nearProbe = world.castRay(vec3(2, 2, 0), vec3(0, -1, 0), 4)
    let farProbe = world.castRay(vec3(5, 2, 0), vec3(0, -1, 0), 4)
    require nearProbe.isSome
    require farProbe.isSome
    require nearProbe.get.subShapeId != farProbe.get.subShapeId
    let nearPart = querySubShape(nearProbe.get)
    let farPart = querySubShape(farProbe.get)
    let excludeNear = excludeSubShapes([nearPart, nearPart])
    let includeFar = includeSubShapes([farPart])

    check excludeNear.len == 1
    check excludeNear.filterMode == QuerySubShapeFilterMode.ExcludeSubShapes
    check includeFar.filterMode == QuerySubShapeFilterMode.IncludeOnlySubShapes

    let ray = world.castRay(
      vec3(0, 0, 0), vec3(1, 0, 0), 8, layers,
      bodyFilter = includeBodies([compound.id]),
      subShapeFilter = excludeNear)
    require ray.isSome
    check ray.get.subShapeId == farPart.subShapeId
    let rays = world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 8,
      subShapeFilter = includeFar)
    check rays.len == 1
    check rays[0].subShapeId == farPart.subShapeId

    let sphere = world.castSphere(
      0.1, vec3(0, 0, 0), vec3(1, 0, 0), 8,
      subShapeFilter = excludeNear)
    require sphere.isSome
    check sphere.get.subShapeId == farPart.subShapeId
    check querySubShape(sphere.get) == farPart
    let spheres = world.castSphereAll(
      0.1, vec3(0, 0, 0), vec3(1, 0, 0), 8, layers,
      subShapeFilter = includeFar)
    check spheres.len == 1
    check spheres[0].subShapeId == farPart.subShapeId

    let queryShape = boxShape(vec3(0.1, 0.1, 0.1))
    let shapeHit = world.castShape(
      queryShape, vec3(0, 0, 0), vec3(1, 0, 0), 8,
      subShapeFilter = excludeNear)
    require shapeHit.isSome
    check shapeHit.get.subShapeId == farPart.subShapeId
    let shapeHits = world.castShapeAll(
      queryShape, vec3(0, 0, 0), vec3(1, 0, 0), 8, layers,
      subShapeFilter = includeFar)
    check shapeHits.len == 1
    check shapeHits[0].subShapeId == farPart.subShapeId

    let sphereOverlaps = world.overlapSphere(
      vec3(3.5, 0, 0), 3, subShapeFilter = includeFar)
    let shapeOverlaps = world.overlapShape(
      sphereShape(3), vec3(3.5, 0, 0), layers,
      subShapeFilter = includeFar)
    check sphereOverlaps.len == 1
    check sphereOverlaps[0].subShapeId == farPart.subShapeId
    check querySubShape(sphereOverlaps[0]) == farPart
    check shapeOverlaps.len == 1
    check shapeOverlaps[0].subShapeId == farPart.subShapeId
    check world.collidePoint(
      vec3(5, 0, 0), subShapeFilter = includeFar) == @[compound.id]
    check world.collidePoint(
      vec3(2, 0, 0), layers, subShapeFilter = includeFar).len == 0

    expect ValueError:
      discard includeSubShapes(newSeq[QuerySubShape]())
    let stalePart = querySubShape(compound.id, farPart.subShapeId)
    compound.close()
    expect ValueError:
      discard world.castRay(
        vec3(0, 0, 0), vec3(1, 0, 0), 8,
        subShapeFilter = includeSubShapes([stalePart]))

  test "undefined layers and invalid configurations are rejected":
    var config = defaultWorldConfig()
    config.collisionLayers.setLen(1)
    expect ValueError:
      discard newWorld(config)

    config = defaultWorldConfig()
    config.collisionPairs.add collisionPair(nonMovingLayer, CollisionLayer(9))
    expect ValueError:
      discard newWorld(config)

    expect ValueError:
      discard collisionLayerConfig(256)

    let world = layeredWorld()
    defer: world.close()
    expect ValueError:
      discard world.addStaticBody(
        sphereShape(0.5), vec3(0, 0, 0), layer = CollisionLayer(9))
    expect ValueError:
      discard world.castRay(
        vec3(0, 0, 0), vec3(1, 0, 0), 10,
        layer = some(CollisionLayer(9)))

  test "character and vehicle helper queries use configured layers":
    let world = layeredWorld()
    defer: world.close()
    let character = world.newCharacter(
      capsuleShape(0.9, 0.35),
      vec3(0, 2, 0),
      layer = queryLayer
    )
    check character.collisionLayer == queryLayer

    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2.0)), vec3(0, 2, 4))
    let vehicle = chassis.newVehicle(wheelCollisionLayer = ignoredLayer)
    check vehicle.wheelCollisionLayer == ignoredLayer

    expect ValueError:
      discard world.newCharacter(
        capsuleShape(0.9, 0.35),
        vec3(0, 2, 0),
        layer = CollisionLayer(9)
      )
