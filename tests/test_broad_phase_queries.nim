import std/[math, options, unittest]
import jolt

proc contains(ids: openArray[BodyId]; body: Body): bool =
  for id in ids:
    if id == body.id:
      return true

suite "Jolt point and broad-phase queries":
  test "narrow point queries reject an AABB-only false positive":
    let world = newWorld()
    defer: world.close()
    let sphere = world.addStaticBody(sphereShape(1), vec3(0, 0, 0))
    world.optimizeBroadPhase()
    let point = vec3(0.9, 0.9, 0)

    check world.broadPhaseQueryPoint(point).contains(sphere)
    check not world.collidePoint(point).contains(sphere)
    check world.collidePoint(vec3(0.25, 0.25, 0)).contains(sphere)

  test "box sphere and oriented-box broad-phase queries return candidates":
    let world = newWorld()
    defer: world.close()
    let left = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(-3, 1, 0))
    let middle = world.addStaticBody(
      sphereShape(0.75), vec3(0, 1, 0))
    let right = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(3, 1, 0))
    world.optimizeBroadPhase()

    let boxHits = world.broadPhaseQueryBox(
      vec3(-3.6, 0.4, -0.6), vec3(0.8, 1.8, 0.6))
    let sphereHits = world.broadPhaseQuerySphere(vec3(3, 1, 0), 1.2)
    let orientedHits = world.broadPhaseQueryOrientedBox(
      vec3(0, 1, 0), vec3(2.4, 0.4, 0.4),
      quatFromAxisAngle(vec3(0, 1, 0), PI.float32 * 0.25))

    check boxHits.contains(left)
    check boxHits.contains(middle)
    check not boxHits.contains(right)
    check sphereHits.contains(right)
    check not sphereHits.contains(left)
    check orientedHits.contains(middle)

  test "broad-phase ray and box casts are sorted and bounded":
    let world = newWorld()
    defer: world.close()
    let nearBody = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(-2, 1, 0))
    let farBody = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(3, 1, 0))
    world.optimizeBroadPhase()

    let rayHits = world.broadPhaseCastRay(
      vec3(-8, 1, 0), vec3(1, 0, 0), 20)
    let boxHits = world.broadPhaseCastBox(
      vec3(-8, 1, 0), vec3(0.25, 0.25, 0.25), vec3(1, 0, 0), 20)
    let bounds = world.broadPhaseBounds()

    check rayHits.len == 2
    check rayHits[0].bodyId == nearBody.id
    check rayHits[1].bodyId == farBody.id
    check rayHits[0].distance < rayHits[1].distance
    check boxHits.len == 2
    check boxHits[0].bodyId == nearBody.id
    check bounds.minimum.x <= -2.5
    check bounds.maximum.x >= 3.5

  test "broad-phase queries honor object layers and capacity":
    const diagnosticLayer = CollisionLayer(2)
    var config = defaultWorldConfig()
    config.collisionLayers.add collisionLayerConfig(1)
    config.collisionPairs.add collisionPair(nonMovingLayer, diagnosticLayer)
    let world = newWorld(config)
    defer: world.close()
    let ordinary = world.addStaticBody(
      sphereShape(1), vec3(-1, 0, 0))
    let diagnostic = world.addStaticBody(
      sphereShape(1), vec3(1, 0, 0), layer = diagnosticLayer)
    world.optimizeBroadPhase()

    let filtered = world.broadPhaseQuerySphere(
      vec3(0, 0, 0), 5, layer = some(diagnosticLayer))
    let limited = world.broadPhaseQuerySphere(vec3(0, 0, 0), 5, maxHits = 1)
    check filtered.len == 1
    check filtered.contains(diagnostic)
    check not filtered.contains(ordinary)
    check limited.len == 1

  test "body sets filter every broad-phase query family":
    let world = newWorld()
    defer: world.close()
    let nearBody = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(-2, 1, 0))
    let farBody = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(3, 1, 0))
    world.optimizeBroadPhase()
    let layers = queryLayerSet([nonMovingLayer])
    let includeFar = includeBodies([farBody.id])
    let excludeNear = excludeBodies([nearBody.id])

    let boxHits = world.broadPhaseQueryBox(
      vec3(-3, 0, -1), vec3(4, 2, 1),
      bodyFilter = excludeNear)
    let sphereHits = world.broadPhaseQuerySphere(
      vec3(0.5, 1, 0), 5, layers, bodyFilter = includeFar)
    let orientedHits = world.broadPhaseQueryOrientedBox(
      vec3(0.5, 1, 0), vec3(4, 1, 1), layers,
      bodyFilter = excludeNear)
    check boxHits == @[farBody.id]
    check sphereHits == @[farBody.id]
    check orientedHits == @[farBody.id]

    check world.broadPhaseQueryPoint(
      vec3(-2, 1, 0), bodyFilter = includeFar).len == 0
    check world.broadPhaseQueryPoint(
      vec3(-2, 1, 0), layers,
      bodyFilter = includeBodies([nearBody.id])) == @[nearBody.id]

    let rayHits = world.broadPhaseCastRay(
      vec3(-8, 1, 0), vec3(1, 0, 0), 20,
      bodyFilter = includeFar)
    let castBoxHits = world.broadPhaseCastBox(
      vec3(-8, 1, 0), vec3(0.25, 0.25, 0.25),
      vec3(1, 0, 0), 20, layers,
      bodyFilter = excludeNear)
    check rayHits.len == 1
    check rayHits[0].bodyId == farBody.id
    check castBoxHits.len == 1
    check castBoxHits[0].bodyId == farBody.id

    let staleFilter = includeBodies([farBody.id])
    farBody.close()
    expect ValueError:
      discard world.broadPhaseQuerySphere(
        vec3(0, 0, 0), 10, bodyFilter = staleFilter)

  test "invalid broad-phase inputs are rejected":
    let world = newWorld()
    defer: world.close()
    expect(ValueError):
      discard world.broadPhaseQueryBox(vec3(1, 0, 0), vec3(-1, 1, 1))
    expect(ValueError): discard world.broadPhaseQuerySphere(vec3(0, 0, 0), 0)
    expect(ValueError): discard world.broadPhaseQueryPoint(vec3(NaN, 0, 0))
    expect(ValueError):
      discard world.broadPhaseQueryOrientedBox(
        vec3(0, 0, 0), vec3(0, 1, 1))
    expect(ValueError):
      discard world.broadPhaseCastRay(
        vec3(0, 0, 0), vec3(0, 0, 0), 10)
    expect(ValueError): discard world.collidePoint(vec3(0, 0, 0), maxHits = 0)
