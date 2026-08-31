import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

let rampVertices = [
  vec3(-5, 0, -5), vec3(5, 0, -5),
  vec3(5, 2, 5), vec3(-5, 2, 5)
]
let quadIndices = [0'u32, 3, 2, 0, 2, 1]

suite "Jolt triangle mesh":
  test "a cooked static ramp supports bodies and spatial queries":
    let world = newWorld()
    defer: world.close()
    let terrain = world.addStaticBody(
      triangleMeshShape(rampVertices, quadIndices), vec3(0, 0, 0))
    let box = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 4, 0))
    terrain.setFriction(1)
    box.setFriction(1)

    for _ in 0 ..< 240:
      discard world.step(dt)

    check box.position.y > 1.35
    check box.position.y < 1.7
    check terrain.shape.kind == ShapeKind.TriangleMesh
    check terrain.shape.vertices.len == rampVertices.len
    check terrain.shape.triangleIndices.len == quadIndices.len
    let ray = world.castRay(vec3(3, 5, 0), vec3(0, -1, 0), 10)
    check ray.isSome
    check ray.get.hits(terrain)
    check world.overlapSphere(vec3(3, 1.1, 0), 0.3).len > 0

  test "a virtual character traverses an inclined mesh":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      triangleMeshShape(rampVertices, quadIndices), vec3(0, 0, 0))
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 0.2, -4))

    for _ in 0 ..< 30:
      character.move(vec3(0, 0, 0), dt)
      discard world.step(dt)
    for _ in 0 ..< 210:
      character.move(vec3(0, 0, 2.2), dt)
      discard world.step(dt)

    check character.position.z > 3
    check character.position.y > 1.5
    check character.groundState == CharacterGroundState.OnGround

  test "vehicle wheel casts contact a triangle mesh floor":
    let world = newWorld()
    defer: world.close()
    let floorVertices = [
      vec3(-30, 0, -30), vec3(30, 0, -30),
      vec3(30, 0, 30), vec3(-30, 0, 30)
    ]
    let terrain = world.addStaticBody(
      triangleMeshShape(floorVertices, quadIndices), vec3(0, 0, 0))
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2)), vec3(0, 1.2, -8))
    let vehicle = chassis.newVehicle()

    for _ in 0 ..< 150:
      vehicle.setInput(0, 0)
      discard world.step(dt)
    var contacts = 0
    for wheel in 0 ..< vehicle.wheelCount:
      let state = vehicle.wheelState(wheel)
      if state.hasContact:
        inc contacts
        check state.contactBodyId.isSome
        check state.contactBodyId.get == terrain.id
    check contacts >= 3

    let startZ = chassis.position.z
    for _ in 0 ..< 240:
      vehicle.setInput(1, 0)
      discard world.step(dt)
    check abs(chassis.position.z - startZ) > 3

  test "mesh descriptions and static-only use are validated":
    expect(ValueError):
      discard triangleMeshShape(rampVertices[0 .. 1], quadIndices)
    expect(ValueError):
      discard triangleMeshShape(rampVertices, [0'u32, 1])
    expect(ValueError):
      discard triangleMeshShape(rampVertices, [0'u32, 1, 8])
    expect(ValueError):
      discard triangleMeshShape(
        [vec3(0, 0, 0), vec3(NaN, 0, 0), vec3(0, 0, 1)],
        [0'u32, 1, 2])

    let world = newWorld()
    defer: world.close()
    expect(ValueError):
      discard world.addDynamicBody(
        triangleMeshShape(rampVertices, quadIndices), vec3(0, 0, 0))
    expect(JoltError):
      discard world.addStaticBody(
        triangleMeshShape(
          [vec3(0, 0, 0), vec3(1, 0, 0), vec3(2, 0, 0)],
          [0'u32, 1, 2]),
        vec3(0, 0, 0))
