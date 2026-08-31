import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

suite "Jolt decorated and mutable shapes":
  test "non-uniform scaled shapes participate in collision and ray queries":
    let world = newWorld()
    defer: world.close()
    let body = world.addStaticBody(
      scaledShape(boxShape(vec3(0.5, 0.5, 0.5)), vec3(4, 1, 2)),
      vec3(0, 0, 0))

    let hit = world.castRay(vec3(1.7, 3, 0), vec3(0, -1, 0), 6)
    let miss = world.castRay(vec3(2.3, 3, 0), vec3(0, -1, 0), 6)
    check hit.isSome
    check hit.get.hits(body)
    check miss.isNone
    check body.shape.shapeScale == vec3(4, 1, 2)

  test "rotated-translated child transforms move collision geometry":
    let world = newWorld()
    defer: world.close()
    let decorated = rotatedTranslatedShape(
      boxShape(vec3(1.2, 0.25, 0.35)),
      vec3(2, 0.5, 0),
      quatFromAxisAngle(vec3(0, 1, 0), PI.float32 * 0.5))
    let body = world.addStaticBody(decorated, vec3(0, 0, 0))

    check world.castRay(vec3(2, 4, 0), vec3(0, -1, 0), 8).get.hits(body)
    check world.castRay(vec3(0, 4, 0), vec3(0, -1, 0), 8).isNone
    check body.shape.innerShape.kind == ShapeKind.Box

  test "center-of-mass offsets remain stable on dynamic bodies":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let body = world.addDynamicBody(
      offsetCenterOfMassShape(
        boxShape(vec3(0.8, 0.8, 0.8)), vec3(0, -0.55, 0)),
      vec3(0, 5, 0),
      quatFromAxisAngle(vec3(0, 0, 1), 0.35))

    for _ in 0 ..< 300:
      discard world.step(dt)
    check body.isAlive
    check body.position.y > 0.5
    check body.position.y < 1.4
    check body.shape.centerOfMassOffset == vec3(0, -0.55, 0)

  test "decorators compose and work as convex query shapes":
    let world = newWorld()
    defer: world.close()
    let target = world.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(5, 1, 0))
    let queryShape = rotatedTranslatedShape(
      scaledShape(sphereShape(0.3), vec3(2, 1, 1)),
      vec3(0.2, 0, 0))

    let shapeHit = world.castShape(
      queryShape, vec3(0, 1, 0), vec3(1, 0, 0), 10)
    check shapeHit.isSome
    check shapeHit.get.hits(target)
    check world.overlapShape(queryShape, vec3(4.2, 1, 0)).len > 0

  test "mutable compounds add and remove live collision children":
    let world = newWorld()
    defer: world.close()
    let body = world.addStaticBody(
      mutableCompoundShape([
        compoundChild(boxShape(vec3(0.4, 0.4, 0.4)), vec3(-2, 0.4, 0))
      ]),
      vec3(0, 0, 0))
    check world.castRay(vec3(2, 3, 0), vec3(0, -1, 0), 6).isNone

    let index = body.addMutableChild(
      compoundChild(sphereShape(0.6), vec3(2, 0.6, 0)))
    check index == 1
    check world.castRay(vec3(2, 3, 0), vec3(0, -1, 0), 6).get.hits(body)
    body.removeMutableChild(index)
    check world.castRay(vec3(2, 3, 0), vec3(0, -1, 0), 6).isNone
    check body.shape.children.len == 1

  test "mutable compounds move and replace children in place":
    let world = newWorld()
    defer: world.close()
    let body = world.addStaticBody(
      mutableCompoundShape([
        compoundChild(boxShape(vec3(0.4, 0.4, 0.4)), vec3(-2, 0.4, 0))
      ]),
      vec3(0, 0, 0))

    body.setMutableChildTransform(0, vec3(2, 0.4, 0))
    check world.castRay(vec3(-2, 3, 0), vec3(0, -1, 0), 6).isNone
    check world.castRay(vec3(2, 3, 0), vec3(0, -1, 0), 6).get.hits(body)

    body.replaceMutableChild(
      0, compoundChild(sphereShape(1.1), vec3(0, 1.1, 0)))
    check world.castRay(vec3(0.8, 3, 0), vec3(0, -1, 0), 6).get.hits(body)
    check body.shape.children[0].shape.kind == ShapeKind.Sphere

  test "mutable compound batches clone and swap one body independently":
    let world = newWorld()
    defer: world.close()
    let sharedDescription = mutableCompoundShape([
      compoundChild(sphereShape(0.4), vec3(-3, 0.4, 0)),
      compoundChild(sphereShape(0.4), vec3(0, 0.4, 0)),
      compoundChild(sphereShape(0.4), vec3(3, 0.4, 0))
    ])
    let changed = world.addStaticBody(sharedDescription, vec3(0, 0, 0))
    let unchanged = world.addStaticBody(sharedDescription, vec3(0, 0, 10))

    changed.setMutableChildTransforms(1, [
      compoundChildTransform(vec3(4, 0.4, 0)),
      compoundChildTransform(
        vec3(7, 0.4, 0),
        quatFromAxisAngle(vec3(0, 1, 0), PI.float32 * 0.25))
    ])

    check world.castRay(vec3(0, 3, 0), vec3(0, -1, 0), 6).isNone
    check world.castRay(vec3(4, 3, 0), vec3(0, -1, 0), 6).get.hits(changed)
    check world.castRay(vec3(7, 3, 0), vec3(0, -1, 0), 6).get.hits(changed)
    check world.castRay(vec3(0, 3, 10), vec3(0, -1, 0), 6).get.hits(unchanged)
    check changed.shape.children[1].position == vec3(4, 0.4, 0)
    check unchanged.shape.children[1].position == vec3(0, 0.4, 0)

    expect ValueError:
      changed.setMutableChildTransforms(
        0, newSeq[CompoundChildTransform]())
    expect IndexDefect:
      changed.setMutableChildTransforms(2, [
        compoundChildTransform(vec3(0, 0, 0)),
        compoundChildTransform(vec3(1, 0, 0))
      ])

  test "dynamic mutable compounds update mass and broadphase bounds":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let body = world.addDynamicBody(
      mutableCompoundShape([
        compoundChild(boxShape(vec3(0.5, 0.5, 0.5)))
      ]),
      vec3(0, 6, 0))
    discard body.addMutableChild(
      compoundChild(sphereShape(0.45), vec3(1.2, 0, 0)))
    body.setAngularVelocity(vec3(0, 2, 0.5))

    for _ in 0 ..< 300:
      discard world.step(dt)
    check body.isAlive
    check body.position.y > 0.3
    check world.overlapSphere(body.position, 2).len > 0

  test "invalid decorated and mutable operations are rejected":
    expect(ValueError):
      discard scaledShape(sphereShape(1), vec3(1, 0, 1))
    expect(ValueError):
      discard rotatedTranslatedShape(sphereShape(1), vec3(NaN, 0, 0))
    expect(ValueError):
      discard offsetCenterOfMassShape(sphereShape(1), vec3(0, Inf, 0))
    expect(ValueError):
      discard mutableCompoundShape(newSeq[CompoundChild]())

    let world = newWorld()
    defer: world.close()
    let ordinary = world.addDynamicBody(sphereShape(0.5), vec3(0, 3, 0))
    expect(ValueError):
      discard ordinary.addMutableChild(compoundChild(sphereShape(0.2)))
    let mutableBody = world.addDynamicBody(
      mutableCompoundShape([compoundChild(sphereShape(0.5))]),
      vec3(0, 4, 0))
    expect(ValueError):
      mutableBody.removeMutableChild(0)
    expect(IndexDefect):
      mutableBody.setMutableChildTransform(4, vec3(0, 0, 0))
