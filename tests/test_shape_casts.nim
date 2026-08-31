import std/[math, options, unittest]
import jolt

suite "Jolt sphere shape casts":
  test "closest cast stops the sphere before a box":
    let world = newWorld()
    defer: world.close()
    let obstacle = world.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))

    let hit = world.castSphere(
      0.5, vec3(-5, 0, 0), vec3(1, 0, 0), 12)
    check hit.isSome
    check hit.get.hits(obstacle)
    check abs(hit.get.distance - 3.5) < 0.05
    check abs(hit.get.position.x + 1.5) < 0.05
    check abs(hit.get.contactPoint.x + 1.0) < 0.05
    check hit.get.normal.x < -0.9

  test "all-hit casts are sorted and honor capacity":
    let world = newWorld()
    defer: world.close()
    var obstacles: seq[Body]
    for x in [0.0'f32, 4.0'f32, 8.0'f32]:
      obstacles.add world.addStaticBody(
        boxShape(vec3(1, 1, 1)), vec3(x, 0, 0))

    let hits = world.castSphereAll(
      0.5, vec3(-5, 0, 0), vec3(1, 0, 0), 20)
    check hits.len == 3
    check hits[0].hits(obstacles[0])
    check hits[1].hits(obstacles[1])
    check hits[2].hits(obstacles[2])
    check hits[0].distance < hits[1].distance
    check hits[1].distance < hits[2].distance

    let limited = world.castSphereAll(
      0.5, vec3(-5, 0, 0), vec3(1, 0, 0), 20, maxHits = 2)
    check limited.len == 2
    check limited[0].hits(obstacles[0])
    check limited[1].hits(obstacles[1])

  test "sphere casts collide with triangle mesh terrain":
    let world = newWorld()
    defer: world.close()
    let terrain = world.addStaticBody(
      triangleMeshShape(
        [
          vec3(-10, 0, -10), vec3(10, 0, -10),
          vec3(10, 0, 10), vec3(-10, 0, 10)
        ],
        [0'u32, 3, 2, 0, 2, 1]),
      vec3(0, 0, 0))
    let hit = world.castSphere(
      0.5, vec3(0, 5, 0), vec3(0, -1, 0), 10)
    check hit.isSome
    check hit.get.hits(terrain)
    check abs(hit.get.distance - 4.5) < 0.05
    check hit.get.normal.y > 0.9

  test "misses, initial overlaps and invalid casts are handled":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))
    check world.castSphere(
      0.5, vec3(-5, 5, 0), vec3(1, 0, 0), 3).isNone
    let overlap = world.castSphere(
      0.5, vec3(0, 0, 0), vec3(1, 0, 0), 3)
    check overlap.isSome
    check overlap.get.fraction <= 0

    expect(ValueError):
      discard world.castSphere(0, vec3(0, 0, 0), vec3(1, 0, 0), 1)
    expect(ValueError):
      discard world.castSphere(1, vec3(0, 0, 0), vec3(0, 0, 0), 1)
    expect(ValueError):
      discard world.castSphereAll(
        1, vec3(0, 0, 0), vec3(1, 0, 0), 1, maxHits = 0)
