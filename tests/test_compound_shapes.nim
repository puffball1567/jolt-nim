import std/[math, options, sequtils, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc dumbbellShape(): Shape =
  staticCompoundShape([
    compoundChild(sphereShape(0.5), vec3(-1.2, 0, 0)),
    compoundChild(boxShape(vec3(0.5, 0.5, 0.5))),
    compoundChild(sphereShape(0.5), vec3(1.2, 0, 0))
  ])

suite "Jolt static compound shapes":
  test "a dynamic compound collides and participates in queries":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))
    let compound = world.addDynamicBody(dumbbellShape(), vec3(0, 5, 0))
    compound.setFriction(0.9)

    for _ in 0 ..< 240:
      discard world.step(dt)

    check compound.position.y > 0.4
    check compound.position.y < 0.8
    check compound.shape.kind == ShapeKind.StaticCompound
    check compound.shape.children.len == 3
    let ray = world.castRay(vec3(0, 4, 0), vec3(0, -1, 0), 8)
    check ray.isSome
    check ray.get.hits(compound)
    let overlaps = world.overlapSphere(compound.position, 0.7)
    check overlaps.anyIt(it.hits(compound))

    let startX = compound.position.x
    compound.setLinearVelocity(vec3(3, 0, 0))
    for _ in 0 ..< 90:
      discard world.step(dt)
    check compound.position.x > startX + 0.5

  test "child transforms affect the compound collision geometry":
    let world = newWorld()
    defer: world.close()
    let shape = staticCompoundShape([
      compoundChild(
        cylinderShape(1.5, 0.25),
        vec3(0, 2, 0),
        quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)),
      compoundChild(boxShape(vec3(0.4, 0.5, 0.4)), vec3(0, 0.5, 0))
    ])
    let body = world.addStaticBody(shape, vec3(0, 0, 0))

    let endHit = world.castRay(vec3(1.2, 5, 0), vec3(0, -1, 0), 8)
    let miss = world.castRay(vec3(0, 5, 1.0), vec3(0, -1, 0), 8)
    check endHit.isSome
    check endHit.get.hits(body)
    check miss.isNone

  test "nested compounds cook and retain every child":
    let inner = staticCompoundShape([
      compoundChild(boxShape(vec3(0.4, 0.4, 0.4)), vec3(-0.7, 0, 0)),
      compoundChild(boxShape(vec3(0.4, 0.4, 0.4)), vec3(0.7, 0, 0))
    ])
    let outer = staticCompoundShape([
      compoundChild(inner),
      compoundChild(sphereShape(0.5), vec3(0, 1.2, 0))
    ])
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let body = world.addDynamicBody(outer, vec3(0, 4, 0))

    for _ in 0 ..< 240:
      discard world.step(dt)
    check body.isAlive
    check body.shape.children.len == 2
    check body.shape.children[0].shape.kind == ShapeKind.StaticCompound
    check body.position.y > 0.35

  test "static-only children make the whole compound static-only":
    let mesh = triangleMeshShape(
      [vec3(-2, 0, -2), vec3(2, 0, -2), vec3(0, 0, 2)],
      [0'u32, 2, 1])
    let mixed = staticCompoundShape([
      compoundChild(mesh),
      compoundChild(boxShape(vec3(0.3, 0.3, 0.3)), vec3(0, 0.3, 0))
    ])
    let world = newWorld()
    defer: world.close()
    let body = world.addStaticBody(mixed, vec3(0, 0, 0))
    check world.castRay(vec3(0, 3, 0), vec3(0, -1, 0), 6).get.hits(body)
    expect(ValueError):
      discard world.addDynamicBody(mixed, vec3(0, 3, 0))

  test "compound descriptions reject invalid children":
    expect(ValueError):
      discard staticCompoundShape(newSeq[CompoundChild]())
    expect(ValueError):
      discard staticCompoundShape([
        compoundChild(sphereShape(0.5))
      ])
    expect(ValueError):
      discard staticCompoundShape([
        CompoundChild(
          shape: sphereShape(0.5),
          position: vec3(NaN, 0, 0),
          rotation: quatIdentity()),
        compoundChild(boxShape(vec3(0.5, 0.5, 0.5)))
      ])
    expect(ValueError):
      discard staticCompoundShape([
        CompoundChild(
          shape: sphereShape(0.5),
          position: vec3(0, 0, 0),
          rotation: Quat()),
        compoundChild(boxShape(vec3(0.5, 0.5, 0.5)))
      ])
