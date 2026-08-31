import std/[math, options, sequtils, unittest]
import jolt

proc eventWorld(maxQueuedEvents = 1_024'u; numThreads = 2'i32): World =
  var config = defaultWorldConfig()
  config.maxBodies = 256
  config.maxBodyPairs = 1_024
  config.maxContactConstraints = 1_024
  config.maxQueuedEvents = maxQueuedEvents
  config.numThreads = numThreads
  newWorld(config)

suite "Jolt polled physics events":
  test "activation events are delivered without a Nim worker callback":
    let world = eventWorld()
    defer:
      world.close()

    check world.pendingEventCount == 0
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 3, 0))
    let events = world.drainEvents()
    check events.len >= 1
    check events.anyIt(it.kind == PhysicsEventKind.BodyActivated and it.involves(body))
    for event in events:
      if event.kind == PhysicsEventKind.BodyActivated:
        check event.subShapeId1.isNone
        check event.subShapeId2.isNone
        check event.material1(world).isNone
        check event.material2(world).isNone
    check world.pendingEventCount == 0

    body.close()

  test "contact lifecycle includes manifold data and removal":
    let world = eventWorld()
    defer:
      world.close()

    let floor = world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)),
      vec3(0, -0.5, 0)
    )
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 3, 0))
    let bodyId = body.id
    let floorId = floor.id
    discard world.drainEvents()

    var added, persisted, removed, deactivated = false
    var contactPoint: Vec3
    var contactNormal: Vec3
    for _ in 0 ..< 240:
      check world.step(1.0'f32 / 60.0'f32) == {}
      for event in world.drainEvents():
        case event.kind
        of PhysicsEventKind.ContactAdded:
          if event.involves(body) and event.involves(floor):
            added = true
            check event.hasManifold
            check event.body2.isSome
            check event.subShapeId1.isSome
            check event.subShapeId2.isSome
            contactPoint = event.contactPoint
            contactNormal = event.contactNormal
        of PhysicsEventKind.ContactPersisted:
          if event.involves(body) and event.involves(floor):
            persisted = true
        of PhysicsEventKind.ContactRemoved:
          if event.body2.isSome and
              ((event.body1 == bodyId and event.body2.get == floorId) or
               (event.body1 == floorId and event.body2.get == bodyId)):
            removed = true
        of PhysicsEventKind.BodyDeactivated:
          if event.body1 == bodyId:
            deactivated = true
        else:
          discard

    check added
    check persisted
    check removed
    check deactivated
    check abs(contactPoint.y) < 0.1
    check abs(contactNormal.y) > 0.9

    body.close()
    check world.step(1.0'f32 / 60.0'f32) == {}
    discard world.drainEvents()
    floor.close()

  test "bounded queue reports dropped events and keeps recent ones":
    let world = eventWorld(maxQueuedEvents = 3, numThreads = 1)
    defer:
      world.close()

    var bodies: seq[Body]
    for index in 0 ..< 12:
      bodies.add(world.addDynamicBody(
        sphereShape(0.2),
        vec3(index * 2, 5, 0)
      ))

    check world.pendingEventCount == 3
    check world.droppedEventCount() >= 9
    check world.droppedEventCount(reset = true) >= 9
    check world.droppedEventCount() == 0
    check world.drainEvents(limit = 2).len == 2
    check world.pendingEventCount == 1
    check world.drainEvents().len == 1

    for body in bodies:
      body.close()

  test "event configuration and drain limits are validated":
    var config = defaultWorldConfig()
    config.maxQueuedEvents = 0
    expect ValueError:
      discard newWorld(config)

    let world = eventWorld()
    defer:
      world.close()
    expect ValueError:
      discard world.drainEvents(limit = -1)
