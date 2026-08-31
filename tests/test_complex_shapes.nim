import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

let tetrahedron = [
  vec3(-0.8, -0.6, -0.7),
  vec3(0.9, -0.6, -0.7),
  vec3(0, -0.6, 0.9),
  vec3(0, 1.0, 0)
]

suite "Jolt complex shapes":
  test "a dynamic convex hull settles and participates in queries":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let hull = world.addDynamicBody(
      convexHullShape(tetrahedron), vec3(0, 4, 0))

    for _ in 0 ..< 240:
      discard world.step(dt)

    check hull.position.y < 1.2
    check hull.shape.kind == ShapeKind.ConvexHull
    check hull.shape.points.len == tetrahedron.len
    let ray = world.castRay(vec3(0, 5, 0), vec3(0, -1, 0), 10)
    check ray.isSome
    check ray.get.hits(hull)
    check world.overlapSphere(hull.position, 1.5).len > 0

  test "interior points are accepted and discarded by hull construction":
    var points = @tetrahedron
    points.add vec3(0, 0, 0)
    points.add vec3(0.1, -0.2, 0.1)
    let world = newWorld()
    defer: world.close()
    let hull = world.addDynamicBody(
      convexHullShape(points, maxConvexRadius = 0), vec3(0, 2, 0))
    check hull.isAlive
    check hull.shape.points.len == points.len

  test "invalid hull descriptions fail before or during native cooking":
    expect(ValueError):
      discard convexHullShape(tetrahedron[0 .. 2])
    expect(ValueError):
      discard convexHullShape(tetrahedron, maxConvexRadius = -0.1)
    expect(ValueError):
      discard convexHullShape([
        vec3(0, 0, 0), vec3(1, 0, 0),
        vec3(0, NaN, 0), vec3(0, 0, 1)])

    let coplanar = [
      vec3(-1, 0, -1), vec3(1, 0, -1),
      vec3(1, 0, 1), vec3(-1, 0, 1)
    ]
    let world = newWorld()
    defer: world.close()
    let flatHull = world.addDynamicBody(
      convexHullShape(coplanar), vec3(0, 2, 0))
    check flatHull.isAlive

    let coincident = [
      vec3(0, 0, 0), vec3(0, 0, 0),
      vec3(0, 0, 0), vec3(0, 0, 0)
    ]
    expect(JoltError):
      discard world.addDynamicBody(
        convexHullShape(coincident), vec3(0, 2, 0))
