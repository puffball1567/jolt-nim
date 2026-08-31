import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

suite "Jolt additional built-in shapes":
  test "tapered capsule and cylinder settle on a plane":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      planeShape(vec3(0, 1, 0), halfExtent = 20), vec3(0, 0, 0))
    let capsule = world.addDynamicBody(
      taperedCapsuleShape(0.7, 0.25, 0.5), vec3(-1.5, 5, 0))
    let cylinder = world.addDynamicBody(
      taperedCylinderShape(0.8, 0.3, 0.65), vec3(1.5, 5, 0))

    for _ in 0 ..< 240:
      discard world.step(dt)

    check floor.shape.kind == ShapeKind.Plane
    check capsule.shape.kind == ShapeKind.TaperedCapsule
    check cylinder.shape.kind == ShapeKind.TaperedCylinder
    check capsule.position.y > 0.8
    check capsule.position.y < 1.5
    check cylinder.position.y > 0.7
    check cylinder.position.y < 1.4

  test "triangle bodies participate in ray and point queries":
    let world = newWorld()
    defer: world.close()
    let triangle = world.addStaticBody(
      triangleShape(
        vec3(-2, 0, -2), vec3(-2, 0, 2), vec3(2, 0, -2), 0.02),
      vec3(0, 2, 0))
    let ray = world.castRay(vec3(0, 5, 0), vec3(0, -1, 0), 10)

    check ray.isSome
    check ray.get.hits(triangle)
    check abs(ray.get.position.y - 2) < 0.05

  test "empty kinematic bodies anchor constraints without colliding":
    let world = newWorld()
    defer: world.close()
    let anchor = world.addKinematicBody(
      emptyShape(vec3(0, 0.25, 0)), vec3(0, 4, 0))
    let payload = world.addDynamicBody(sphereShape(0.4), vec3(0, 2, 0))
    let joint = addDistanceConstraint(
      anchor, payload, vec3(0, 4, 0), vec3(0, 2, 0), 2, 2)

    for _ in 0 ..< 120:
      discard world.step(dt)

    check anchor.shape.kind == ShapeKind.Empty
    check world.collidePoint(anchor.position).len == 0
    check abs((anchor.position.y - payload.position.y) - 2) < 0.15
    check joint.isAlive

  test "both tapered shapes work as convex query shapes":
    let world = newWorld()
    defer: world.close()
    let target = world.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 2, 0))

    let taperedHit = world.castShape(
      taperedCylinderShape(0.5, 0.2, 0.5),
      vec3(-5, 2, 0), vec3(1, 0, 0), 10)
    let capsuleHit = world.castShape(
      taperedCapsuleShape(0.5, 0.25, 0.45),
      vec3(-5, 2, 0), vec3(1, 0, 0), 10)

    check taperedHit.isSome
    check taperedHit.get.hits(target)
    check capsuleHit.isSome
    check capsuleHit.get.hits(target)

  test "additional shape descriptions reject invalid settings":
    expect(ValueError): discard taperedCapsuleShape(-1, 0.2, 0.4)
    expect(ValueError): discard taperedCylinderShape(1, 0, 0.4)
    expect(ValueError):
      discard triangleShape(vec3(0, 0, 0), vec3(1, 0, 0), vec3(2, 0, 0))
    expect(ValueError): discard planeShape(vec3(0, 0, 0))
    expect(ValueError): discard emptyShape(vec3(NaN, 0, 0))

    let world = newWorld()
    defer: world.close()
    expect(ValueError):
      discard world.addDynamicBody(planeShape(vec3(0, 1, 0)), vec3(0, 0, 0))
    expect(ValueError):
      discard world.addDynamicBody(emptyShape(), vec3(0, 0, 0))
