import std/[options, unittest]
import jolt

proc materialWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  result = newWorld(config)
  result.setGravity(vec3(0, 0, 0))

proc rayMaterial(world: World; x, z: float32): PhysicsMaterial =
  let hit = world.castRay(vec3(x, 5, z), vec3(0, -1, 0), 10)
  check hit.isSome
  let material = hit.get.material(world)
  check material.isSome
  material.get

suite "Jolt physics materials":
  test "convex material name color and sub-shape ID round trip":
    let world = materialWorld()
    defer: world.close()
    let rubber = physicsMaterial(
      "rubber", materialColor(220, 35, 45, 240))
    let body = world.addStaticBody(
      sphereShape(1).withMaterial(rubber), vec3(0, 0, 0))
    let hit = world.castRay(vec3(0, 5, 0), vec3(0, -1, 0), 10)

    check hit.isSome
    check hit.get.hits(body)
    let fromHit = hit.get.material(world)
    let fromBody = body.materialAt(hit.get.subShapeId)
    check fromHit.isSome
    check fromBody.isSome
    check fromHit.get == rubber
    check fromBody.get == rubber

  test "unassigned shapes report no custom material":
    let world = materialWorld()
    defer: world.close()
    let body = world.addStaticBody(boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))
    let hit = world.castRay(vec3(0, 5, 0), vec3(0, -1, 0), 10).get

    check hit.hits(body)
    check hit.material(world).isNone
    check body.materialAt(hit.subShapeId).isNone

  test "compound children retain independent materials":
    let world = materialWorld()
    defer: world.close()
    let wood = physicsMaterial("wood", materialColor(155, 95, 45))
    let metal = physicsMaterial("metal", materialColor(165, 180, 195))
    let body = world.addStaticBody(
      staticCompoundShape([
        compoundChild(
          boxShape(vec3(0.8, 0.5, 0.8)).withMaterial(wood), vec3(-2, 0, 0)),
        compoundChild(
          sphereShape(0.8).withMaterial(metal), vec3(2, 0, 0))
      ]),
      vec3(0, 0, 0))
    let leftHit = world.castRay(vec3(-2, 5, 0), vec3(0, -1, 0), 10).get
    let rightHit = world.castRay(vec3(2, 5, 0), vec3(0, -1, 0), 10).get

    check leftHit.hits(body)
    check rightHit.hits(body)
    check leftHit.subShapeId != rightHit.subShapeId
    check leftHit.material(world).get == wood
    check rightHit.material(world).get == metal

  test "sphere and convex casts and overlaps expose target materials":
    let world = materialWorld()
    defer: world.close()
    let wood = physicsMaterial("wood", materialColor(155, 95, 45))
    let metal = physicsMaterial("metal", materialColor(165, 180, 195))
    let body = world.addStaticBody(
      staticCompoundShape([
        compoundChild(
          boxShape(vec3(0.8, 0.5, 0.8)).withMaterial(wood), vec3(-2, 0, 0)),
        compoundChild(
          sphereShape(0.8).withMaterial(metal), vec3(2, 0, 0))
      ]), vec3(0, 0, 0))

    let sphereCast = world.castSphere(
      0.1, vec3(-2, 5, 0), vec3(0, -1, 0), 10).get
    let convexCast = world.castShape(
      boxShape(vec3(0.1, 0.1, 0.1)),
      vec3(2, 5, 0), vec3(0, -1, 0), 10).get
    let sphereOverlap = world.overlapSphere(vec3(-2, 0, 0), 0.4)
    let convexOverlap = world.overlapShape(
      sphereShape(0.4), vec3(2, 0, 0))

    check sphereCast.hits(body)
    check convexCast.hits(body)
    check sphereCast.material(world).get == wood
    check convexCast.material(world).get == metal
    check sphereOverlap.len == 1
    check convexOverlap.len == 1
    check sphereOverlap[0].material(world).get == wood
    check convexOverlap[0].material(world).get == metal

  test "contact events expose both sub-shape materials":
    let world = materialWorld()
    defer: world.close()
    world.setGravity(vec3(0, -9.81, 0))
    let wood = physicsMaterial("wood", materialColor(155, 95, 45))
    let metal = physicsMaterial("metal", materialColor(165, 180, 195))
    let rubber = physicsMaterial("rubber", materialColor(220, 35, 45))
    let floor = world.addStaticBody(
      staticCompoundShape([
        compoundChild(
          boxShape(vec3(1.5, 0.5, 1.5)).withMaterial(wood),
          vec3(-2, -0.5, 0)),
        compoundChild(
          boxShape(vec3(1.5, 0.5, 1.5)).withMaterial(metal),
          vec3(2, -0.5, 0))
      ]), vec3(0, 0, 0))
    let ball = world.addDynamicBody(
      sphereShape(0.5).withMaterial(rubber), vec3(2, 3, 0))
    discard world.drainEvents()

    var found = false
    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}
      for event in world.drainEvents():
        if event.kind == PhysicsEventKind.ContactAdded and
            event.involves(floor) and event.involves(ball):
          found = true
          check event.subShapeId1.isSome
          check event.subShapeId2.isSome
          let first = event.material1(world)
          let second = event.material2(world)
          check first.isSome
          check second.isSome
          if event.body1 == floor.id:
            check first.get == metal
            check second.get == rubber
          else:
            check first.get == rubber
            check second.get == metal
      if found:
        break
    check found

  test "triangle mesh resolves per-triangle materials":
    let world = materialWorld()
    defer: world.close()
    let grass = physicsMaterial("grass", materialColor(45, 175, 70))
    let stone = physicsMaterial("stone", materialColor(115, 125, 135))
    let mesh = triangleMeshShape(
      [
        vec3(-4, 0, -2), vec3(0, 0, -2), vec3(0, 0, 2), vec3(-4, 0, 2),
        vec3(0, 0, -2), vec3(4, 0, -2), vec3(4, 0, 2), vec3(0, 0, 2)
      ],
      [0'u32, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6]
    ).withMaterials([grass, stone], [0'u32, 0, 1, 1])
    let body = world.addStaticBody(mesh, vec3(0, 0, 0))

    check world.rayMaterial(-2, 0) == grass
    check world.rayMaterial(2, 0) == stone
    let grassHit = world.castRay(vec3(-2, 5, 0), vec3(0, -1, 0), 10).get
    let stoneHit = world.castRay(vec3(2, 5, 0), vec3(0, -1, 0), 10).get
    check grassHit.subShapeId != stoneHit.subShapeId
    let stoneOnly = includeSubShapes([
      querySubShape(body.id, stoneHit.subShapeId)])
    check world.castRay(
      vec3(-2, 5, 0), vec3(0, -1, 0), 10,
      subShapeFilter = stoneOnly).isNone
    check world.castRay(
      vec3(2, 5, 0), vec3(0, -1, 0), 10,
      subShapeFilter = stoneOnly).get.material(world).get == stone

  test "height field resolves per-cell materials":
    let world = materialWorld()
    defer: world.close()
    let sand = physicsMaterial("sand", materialColor(220, 190, 95))
    let mud = physicsMaterial("mud", materialColor(90, 60, 35))
    var indices = newSeq[uint32](9)
    for z in 0 ..< 3:
      for x in 0 ..< 3:
        indices[x + z * 3] = if x == 0: 0 else: 1
    let terrain = heightFieldShape(
      newSeq[float32](16), 4,
      offset = vec3(-1.5, 0, -1.5),
      scale = vec3(1, 1, 1)).withMaterials([sand, mud], indices)
    let body = world.addStaticBody(terrain, vec3(0, 0, 0))

    check world.rayMaterial(-1, 0) == sand
    check world.rayMaterial(1, 0) == mud
    let sandHit = world.castRay(vec3(-1, 5, 0), vec3(0, -1, 0), 10).get
    let mudHit = world.castRay(vec3(1, 5, 0), vec3(0, -1, 0), 10).get
    check sandHit.subShapeId != mudHit.subShapeId
    let mudOnly = includeSubShapes([
      querySubShape(body.id, mudHit.subShapeId)])
    check world.castRay(
      vec3(-1, 5, 0), vec3(0, -1, 0), 10,
      subShapeFilter = mudOnly).isNone
    check world.castRay(
      vec3(1, 5, 0), vec3(0, -1, 0), 10,
      subShapeFilter = mudOnly).get.material(world).get == mud

  test "recursive material application survives decoration and replacement":
    let world = materialWorld()
    defer: world.close()
    let ceramic = physicsMaterial("ceramic", materialColor(235, 230, 210))
    let body = world.addDynamicBody(
      scaledShape(sphereShape(0.5), vec3(2, 1, 1)).withMaterial(ceramic),
      vec3(0, 0, 0))
    check world.rayMaterial(0, 0) == ceramic

    body.setShape(
      rotatedTranslatedShape(
        boxShape(vec3(0.5, 0.5, 0.5)), vec3(0.5, 0, 0)).withMaterial(ceramic))
    check world.rayMaterial(0.5, 0) == ceramic

  test "invalid material definitions and mappings are rejected":
    let grass = physicsMaterial("grass", materialColor(40, 160, 65))
    expect ValueError:
      discard physicsMaterial("")
    expect ValueError:
      discard physicsMaterial("bad\0name")
    expect ValueError:
      discard materialColor(256, 0, 0)
    expect ValueError:
      discard emptyShape().withMaterial(grass)
    let mesh = triangleMeshShape(
      [vec3(-1, 0, -1), vec3(1, 0, -1), vec3(0, 0, 1)],
      [0'u32, 2, 1])
    expect ValueError:
      discard mesh.withMaterials([grass], [])
    expect ValueError:
      discard mesh.withMaterials([grass], [1'u32])
    expect ValueError:
      discard sphereShape(1).withMaterials([grass], [0'u32])
