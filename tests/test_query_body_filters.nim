import std/[options, sequtils, unittest]
import jolt

proc queryFilterWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  newWorld(config)

proc findInfo(infos: openArray[QueryBodyInfo]; bodyId: BodyId): QueryBodyInfo =
  for info in infos:
    if info.bodyId == bodyId:
      return info
  raise newException(ValueError, "body query info was not found")

suite "Jolt property and predicate body query filters":
  test "world body queries return detached rigid and soft properties":
    let world = queryFilterWorld()
    defer: world.close()

    var floorConfig = defaultBodyConfig()
    floorConfig.userData = 11
    let floor = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(1, 0, 0),
      sensor = true, config = floorConfig)

    var dynamicConfig = defaultBodyConfig()
    dynamicConfig.userData = 22
    let dynamicBody = world.addDynamicBody(
      sphereShape(0.5), vec3(3, 0, 0), config = dynamicConfig)

    var softConfig = defaultSoftBodyConfig()
    softConfig.userData = 33
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(3, 3, 0.25), vec3(0, 4, 0), config = softConfig)

    let infos = world.queryBodies()
    check infos.len == 3

    let floorInfo = infos.findInfo(floor.id)
    check floorInfo.motionType == MotionType.Static
    check floorInfo.collisionLayer == nonMovingLayer
    check floorInfo.sensor
    check not floorInfo.softBody
    check floorInfo.userData == 11
    check floorInfo.position == vec3(1, 0, 0)

    let dynamicInfo = infos.findInfo(dynamicBody.id)
    check dynamicInfo.motionType == MotionType.Dynamic
    check dynamicInfo.collisionLayer == movingLayer
    check dynamicInfo.active
    check not dynamicInfo.sensor
    check not dynamicInfo.softBody
    check dynamicInfo.userData == 22

    let clothInfo = infos.findInfo(cloth.id)
    check clothInfo.motionType == MotionType.Dynamic
    check clothInfo.softBody
    check clothInfo.userData == 33

  test "declarative criteria compose body properties":
    let world = queryFilterWorld()
    defer: world.close()

    var firstConfig = defaultBodyConfig()
    firstConfig.userData = 101
    let first = world.addDynamicBody(
      sphereShape(0.4), vec3(2, 0, 0), config = firstConfig)

    var secondConfig = defaultBodyConfig()
    secondConfig.userData = 202
    let second = world.addDynamicBody(
      sphereShape(0.4), vec3(4, 0, 0), sensor = true,
      config = secondConfig)

    var staticConfig = defaultBodyConfig()
    staticConfig.userData = 202
    let staticBody = world.addStaticBody(
      sphereShape(0.4), vec3(6, 0, 0), sensor = true,
      config = staticConfig)

    let criteria = bodyQueryCriteria(
      motionTypes = {MotionType.Dynamic},
      layers = @[movingLayer],
      sensor = some(true),
      softBody = some(false),
      userData = some(202'u64))
    let selected = world.queryBodies(criteria)
    check selected.len == 1
    check selected[0].bodyId == second.id

    let filter = world.queryBodyFilter(criteria)
    check filter.isEnabled
    check filter.len == 1
    let hits = world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10, bodyFilter = filter)
    check hits.len == 1
    check hits[0].hits(second)
    check not hits.anyIt(it.hits(first) or it.hits(staticBody))

  test "Nim predicates run once on the caller thread and produce native filters":
    let world = queryFilterWorld()
    defer: world.close()

    var bodies: seq[Body]
    for index in 0 ..< 5:
      var config = defaultBodyConfig()
      config.userData = uint64(index)
      bodies.add(world.addStaticBody(
        sphereShape(0.3), vec3(index + 1, 0, 0), config = config))

    var calls = 0
    let evenFilter = world.queryBodyFilter(
      proc(info: QueryBodyInfo): bool =
        inc calls
        info.userData mod 2 == 0)
    check calls == bodies.len
    check evenFilter.len == 3

    let hits = world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10, bodyFilter = evenFilter)
    check calls == bodies.len
    check hits.len == 3
    check hits[0].hits(bodies[0])
    check hits[1].hits(bodies[2])
    check hits[2].hits(bodies[4])

  test "body-backed characters are included outside the regular body list":
    let world = queryFilterWorld()
    defer: world.close()
    var config = defaultRigidCharacterConfig()
    config.userData = 404
    let character = world.newRigidCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0), config = config)

    let infos = world.queryBodies()
    check infos.len == 1
    check infos[0].bodyId == character.bodyId
    check infos[0].motionType == MotionType.Dynamic
    check infos[0].collisionLayer == movingLayer
    check not infos[0].softBody
    check infos[0].userData == 404

  test "an empty resolved include filter rejects every body":
    let world = queryFilterWorld()
    defer: world.close()
    discard world.addStaticBody(sphereShape(0.5), vec3(2, 0, 0))

    let filter = world.queryBodyFilter(
      bodyQueryCriteria(userData = some(high(uint64))))
    check filter.isEnabled
    check filter.len == 0
    check world.castRayAll(
      vec3(0, 0, 0), vec3(1, 0, 0), 10,
      bodyFilter = filter).len == 0
    check world.broadPhaseCastRay(
      vec3(0, 0, 0), vec3(1, 0, 0), 10,
      bodyFilter = filter).len == 0

  test "criteria and predicate validation fail before entering Jolt":
    let world = queryFilterWorld()
    expect ValueError:
      discard world.queryBodies(bodyQueryCriteria(
        layers = @[CollisionLayer(99)]))
    let nilPredicate: QueryBodyPredicate = nil
    expect ValueError:
      discard world.queryBodies(nilPredicate)
    world.close()
    expect JoltError:
      discard world.queryBodies()
