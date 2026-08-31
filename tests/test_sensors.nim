import std/unittest
import jolt

const dt = 1.0'f32 / 60.0'f32

suite "Jolt sensor bodies":
  test "a sensor emits enter, stay and exit without blocking motion":
    let world = newWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let sensor = world.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 1, 0), sensor = true)
    let mover = world.addDynamicBody(
      sphereShape(0.25), vec3(-4, 1, 0))
    mover.setLinearVelocity(vec3(3, 0, 0))

    var added = 0
    var persisted = 0
    var removed = 0
    for _ in 0 ..< 180:
      discard world.step(dt)
      for event in world.drainEvents():
        if event.involves(sensor) and event.involves(mover):
          case event.kind
          of PhysicsEventKind.ContactAdded: inc added
          of PhysicsEventKind.ContactPersisted: inc persisted
          of PhysicsEventKind.ContactRemoved: inc removed
          else: discard

    check sensor.isSensor
    check mover.position.x > 3
    check added >= 1
    check persisted >= 1
    check removed >= 1

  test "the same shape blocks motion when it is not a sensor":
    let world = newWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let obstacle = world.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 1, 0))
    let mover = world.addDynamicBody(
      sphereShape(0.25), vec3(-4, 1, 0))
    mover.setLinearVelocity(vec3(3, 0, 0))

    for _ in 0 ..< 180:
      discard world.step(dt)

    check not obstacle.isSensor
    check mover.position.x < -1

  test "sensor mode can be changed while a body is alive":
    let world = newWorld()
    let body = world.addKinematicBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 2, 0))
    check not body.isSensor
    body.setSensor(true)
    check body.isSensor
    check world.overlapSphere(vec3(0, 2, 0), 0.5).len > 0
    body.setSensor(false)
    check not body.isSensor

    body.close()
    expect(JoltError): discard body.isSensor
    world.close()
