import std/[math, options, sequtils, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc configuredWorld(policy: ContactPolicy): World =
  var config = defaultWorldConfig()
  config.contactPolicies.add(policy)
  newWorld(config)

suite "Jolt declarative contact policies":
  test "rejected layer pairs pass through without contact events":
    let world = configuredWorld(contactPolicy(
      nonMovingLayer, movingLayer, response = ContactPolicyReject))
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(5, 0.5, 5)), vec3(0, -0.5, 0))
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    for step in 0 ..< 90:
      check world.step(dt) == {}
    check body.position.y < -2
    check not world.drainEvents().anyIt(
      it.kind in {PhysicsEventKind.ContactAdded,
        PhysicsEventKind.ContactPersisted} and it.involves(body))
    body.close()
    floor.close()

  test "sensor policies report contacts without collision response":
    let world = configuredWorld(contactPolicy(
      nonMovingLayer, movingLayer, response = ContactPolicySensor))
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(5, 0.5, 5)), vec3(0, -0.5, 0))
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    for step in 0 ..< 90:
      check world.step(dt) == {}
    check body.position.y < -2
    check world.drainEvents().anyIt(
      it.kind == PhysicsEventKind.ContactAdded and it.involves(body))
    body.close()
    floor.close()

  test "restitution and friction overrides change solver response":
    let bounceWorld = configuredWorld(contactPolicy(
      nonMovingLayer, movingLayer,
      friction = some(0.0'f32), restitution = some(1.0'f32)))
    defer: bounceWorld.close()
    let bounceFloor = bounceWorld.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    bounceFloor.setFriction(2)
    let bouncing = bounceWorld.addDynamicBody(
      sphereShape(0.5), vec3(0, 3, 0))
    bouncing.setFriction(2)
    var upwardSpeed = 0.0'f32
    for step in 0 ..< 100:
      check bounceWorld.step(dt) == {}
      upwardSpeed = max(upwardSpeed, bouncing.linearVelocity.y)
    check upwardSpeed > 5

    bouncing.setLinearVelocity(vec3(4, 0, 0))
    for step in 0 ..< 60:
      check bounceWorld.step(dt) == {}
    check abs(bouncing.linearVelocity.x) > 3.5

  test "linear surface velocity drives a resting body like a conveyor":
    let world = configuredWorld(contactPolicy(
      nonMovingLayer, movingLayer,
      friction = some(1.0'f32),
      linearSurfaceVelocity = vec3(3, 0, 0)))
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let body = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 0.6, 0))
    for step in 0 ..< 180:
      check world.step(dt) == {}
    check abs(body.position.x) > 2
    check floor.isAlive
    body.close()
    floor.close()

  test "directional inverse mass scales alter two-body impulses":
    let world = configuredWorld(contactPolicy(
      movingLayer, movingLayer,
      inverseMassScale1 = 0, inverseInertiaScale1 = 0))
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let first = world.addDynamicBody(sphereShape(0.5), vec3(-1, 2, 0))
    let second = world.addDynamicBody(sphereShape(0.5), vec3(1, 2, 0))
    first.setLinearVelocity(vec3(2, 0, 0))
    second.setLinearVelocity(vec3(-2, 0, 0))
    for step in 0 ..< 45:
      check world.step(dt) == {}
    check first.linearVelocity.x > 1.5
    check second.linearVelocity.x > 1

  test "world owns an immutable copy of contact policy configuration":
    var config = defaultWorldConfig()
    config.contactPolicies.add(contactPolicy(
      nonMovingLayer, movingLayer, response = ContactPolicyReject))
    let world = newWorld(config)
    defer: world.close()

    config.contactPolicies[0].response = ContactPolicyCollide
    config.contactPolicies[0].restitution = some(1.0'f32)

    let floor = world.addStaticBody(
      boxShape(vec3(5, 0.5, 5)), vec3(0, -0.5, 0))
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    for step in 0 ..< 90:
      check world.step(dt) == {}
    check body.position.y < -2
    check not world.drainEvents().anyIt(
      it.kind in {PhysicsEventKind.ContactAdded,
        PhysicsEventKind.ContactPersisted} and it.involves(body))
    body.close()
    floor.close()

  test "rejected policies also suppress soft body contacts":
    let world = configuredWorld(contactPolicy(
      nonMovingLayer, movingLayer, response = ContactPolicyReject))
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(5, 0.5, 5)), vec3(0, -0.5, 0))
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(5, 5, 0.4), vec3(0, 3, 0))
    for step in 0 ..< 150:
      check world.step(dt) == {}
    check cloth.vertexState(12).position.y < -2
    check not world.drainSoftBodyContactEvents().anyIt(
      it.softBody == cloth.id and it.otherBody == floor.id)
    cloth.close()
    floor.close()

  test "sensor policies convert soft body contacts without rigid sensors":
    let world = configuredWorld(contactPolicy(
      nonMovingLayer, movingLayer, response = ContactPolicySensor))
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let rigid = world.addStaticBody(
      boxShape(vec3(3, 0.5, 3)), vec3(0, 2, 0))
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(5, 5, 0.4), vec3(0, 2, 0))
    let before = cloth.vertexState(12).position
    for step in 0 ..< 8:
      check world.step(dt) == {}
    check abs(cloth.vertexState(12).position.y - before.y) < 0.02
    check world.drainSoftBodyContactEvents().anyIt(
      it.softBody == cloth.id and it.otherBody == rigid.id and
        it.isSensor and it.vertex.isNone)
    cloth.close()
    rigid.close()

  test "directional mass policies pin soft contact response to one side":
    let world = configuredWorld(contactPolicy(
      movingLayer, movingLayer,
      inverseMassScale1 = 0, inverseInertiaScale1 = 0))
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(5, 5, 0.5), vec3(0, 2, 0))
    var rigidConfig = defaultBodyConfig()
    rigidConfig.gravityFactor = 0
    let rigid = world.addDynamicBody(
      boxShape(vec3(0.6, 0.6, 0.6)), vec3(0, 1.7, 0),
      config = rigidConfig)
    let clothBefore = cloth.vertexState(12).position
    let rigidBefore = rigid.position
    for step in 0 ..< 30:
      check world.step(dt) == {}
    check abs(cloth.vertexState(12).position.y - clothBefore.y) < 0.05
    check abs(rigid.position.y - rigidBefore.y) > 0.1
    cloth.close()
    rigid.close()

  test "exact body-pair policies override layers and can be removed live":
    let world = configuredWorld(contactPolicy(
      nonMovingLayer, movingLayer, response = ContactPolicySensor))
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(6, 0.5, 6)), vec3(0, -0.5, 0))
    let protected = world.addDynamicBody(
      sphereShape(0.5), vec3(-1.5, 2, 0))
    let sensorOnly = world.addDynamicBody(
      sphereShape(0.5), vec3(1.5, 2, 0))

    world.setBodyPairContactPolicy(
      floor, protected, bodyPairContactPolicy())
    check world.bodyPairContactPolicyCount == 1
    check world.hasBodyPairContactPolicy(protected, floor)
    for step in 0 ..< 90:
      check world.step(dt) == {}
    check protected.position.y > 0.4
    check sensorOnly.position.y < -2

    check world.removeBodyPairContactPolicy(protected, floor)
    check not world.removeBodyPairContactPolicy(protected, floor)
    check world.bodyPairContactPolicyCount == 0
    protected.setTransform(vec3(-1.5, 2, 0), quatIdentity())
    protected.setLinearVelocity(vec3(0, 0, 0))
    for step in 0 ..< 90:
      check world.step(dt) == {}
    check protected.position.y < -2
    sensorOnly.close()
    protected.close()
    floor.close()

  test "exact reject affects only its selected body on a shared layer":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(6, 0.5, 6)), vec3(0, -0.5, 0))
    let rejected = world.addDynamicBody(
      sphereShape(0.5), vec3(-1.5, 2, 0))
    let colliding = world.addDynamicBody(
      sphereShape(0.5), vec3(1.5, 2, 0))
    world.setBodyPairContactPolicy(
      floor, rejected,
      bodyPairContactPolicy(response = ContactPolicyReject))
    for step in 0 ..< 90:
      check world.step(dt) == {}
    check rejected.position.y < -2
    check colliding.position.y > 0.4
    colliding.close()
    rejected.close()
    floor.close()

  test "exact directional settings preserve the caller's body order":
    let world = newWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let first = world.addDynamicBody(sphereShape(0.5), vec3(-1, 2, 0))
    let second = world.addDynamicBody(sphereShape(0.5), vec3(1, 2, 0))
    first.setLinearVelocity(vec3(2, 0, 0))
    second.setLinearVelocity(vec3(-2, 0, 0))
    world.setBodyPairContactPolicy(
      second, first,
      bodyPairContactPolicy(
        inverseMassScale1 = 0, inverseInertiaScale1 = 0))
    for step in 0 ..< 45:
      check world.step(dt) == {}
    check second.linearVelocity.x < -1.5
    check first.linearVelocity.x < -1
    second.close()
    first.close()

  test "sub-shape rules override a body rule for only one compound child":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      staticCompoundShape([
        compoundChild(boxShape(vec3(1.5, 0.5, 2)), vec3(-2, 0, 0)),
        compoundChild(boxShape(vec3(1.5, 0.5, 2)), vec3(2, 0, 0))
      ]),
      vec3(0, -0.5, 0))
    let leftFloorHit = world.castRay(
      vec3(-2, 5, 0), vec3(0, -1, 0), 10).get
    let rightFloorHit = world.castRay(
      vec3(2, 5, 0), vec3(0, -1, 0), 10).get
    check leftFloorHit.subShapeId != rightFloorHit.subShapeId

    let sphere = world.addDynamicBody(
      sphereShape(0.5), vec3(-2, 3, 0))
    let sphereHit = world.castRay(
      vec3(-2, 5, 0), vec3(0, -1, 0), 10).get
    check sphereHit.hits(sphere)
    world.setBodyPairContactPolicy(
      floor, sphere,
      bodyPairContactPolicy(response = ContactPolicyReject))
    world.setSubShapePairContactPolicy(
      floor, rightFloorHit.subShapeId,
      sphere, sphereHit.subShapeId,
      bodyPairContactPolicy())
    check world.subShapePairContactPolicyCount == 1
    check world.hasSubShapePairContactPolicy(
      sphere, sphereHit.subShapeId,
      floor, rightFloorHit.subShapeId)

    for step in 0 ..< 100:
      check world.step(dt) == {}
    check sphere.position.y < -2

    sphere.setTransform(vec3(2, 3, 0), quatIdentity())
    sphere.setLinearVelocity(vec3(0, 0, 0))
    for step in 0 ..< 100:
      check world.step(dt) == {}
    check sphere.position.y > 0.4
    check world.drainEvents().anyIt(
      it.kind == PhysicsEventKind.ContactAdded and
        it.involves(sphere) and
        (it.subShapeId1 == some(rightFloorHit.subShapeId) or
          it.subShapeId2 == some(rightFloorHit.subShapeId)))

    check world.removeSubShapePairContactPolicy(
      floor, rightFloorHit.subShapeId,
      sphere, sphereHit.subShapeId)
    check world.subShapePairContactPolicyCount == 0
    sphere.setTransform(vec3(2, 3, 0), quatIdentity())
    sphere.setLinearVelocity(vec3(0, 0, 0))
    for step in 0 ..< 100:
      check world.step(dt) == {}
    check sphere.position.y < -2
    sphere.close()
    floor.close()

  test "sub-shape contact settings modify one manifold but not its sibling":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      staticCompoundShape([
        compoundChild(boxShape(vec3(1.5, 0.5, 2)), vec3(-2, 0, 0)),
        compoundChild(boxShape(vec3(1.5, 0.5, 2)), vec3(2, 0, 0))
      ]),
      vec3(0, -0.5, 0))
    floor.setUseManifoldReduction(false)
    let leftFloorHit = world.castRay(
      vec3(-2, 5, 0), vec3(0, -1, 0), 10).get
    let rightFloorHit = world.castRay(
      vec3(2, 5, 0), vec3(0, -1, 0), 10).get
    check leftFloorHit.subShapeId != rightFloorHit.subShapeId
    let box = world.addDynamicBody(
      boxShape(vec3(0.45, 0.45, 0.45)), vec3(-2, 0.7, 0))
    box.setUseManifoldReduction(false)
    box.setFriction(1)
    let boxHit = world.castRay(
      vec3(-2, 3, 0), vec3(0, -1, 0), 5).get
    check boxHit.hits(box)
    world.setSubShapePairContactPolicy(
      floor, leftFloorHit.subShapeId,
      box, boxHit.subShapeId,
      bodyPairContactPolicy(
        friction = some(1.0'f32),
        linearSurfaceVelocity = vec3(1, 0, 0)))

    let leftStart = box.position.x
    for step in 0 ..< 60:
      check world.step(dt) == {}
    check abs(box.position.x - leftStart) > 0.35

    box.setTransform(vec3(2, 0.7, 0), quatIdentity())
    box.setLinearVelocity(vec3(0, 0, 0))
    let rightStart = box.position.x
    for step in 0 ..< 60:
      check world.step(dt) == {}
    check abs(box.position.x - rightStart) < 0.15
    check world.subShapePairContactPolicyCount == 1
    box.setShape(sphereShape(0.45))
    check world.subShapePairContactPolicyCount == 0
    box.close()
    floor.close()

  test "exact policies support soft-rigid sensor contacts":
    let world = newWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let rigid = world.addStaticBody(
      boxShape(vec3(3, 0.5, 3)), vec3(0, 2, 0))
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(5, 5, 0.4), vec3(0, 2, 0))
    let before = cloth.vertexState(12).position
    world.setBodyPairContactPolicy(
      cloth, rigid,
      bodyPairContactPolicy(response = ContactPolicySensor))
    for step in 0 ..< 8:
      check world.step(dt) == {}
    check abs(cloth.vertexState(12).position.y - before.y) < 0.02
    check world.drainSoftBodyContactEvents().anyIt(
      it.softBody == cloth.id and it.otherBody == rigid.id and
        it.isSensor and it.vertex.isNone)
    cloth.close()
    check world.bodyPairContactPolicyCount == 0
    rigid.close()

  test "exact policy values ownership identity and cleanup are validated":
    let firstWorld = newWorld()
    defer: firstWorld.close()
    let secondWorld = newWorld()
    defer: secondWorld.close()
    let first = firstWorld.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let second = firstWorld.addDynamicBody(sphereShape(0.5), vec3(2, 2, 0))
    let foreign = secondWorld.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    expect ValueError:
      firstWorld.setBodyPairContactPolicy(first, first, bodyPairContactPolicy())
    expect ValueError:
      firstWorld.setBodyPairContactPolicy(
        first, second, bodyPairContactPolicy(friction = some(-0.1'f32)))
    expect ValueError:
      firstWorld.setBodyPairContactPolicy(
        first, second,
        bodyPairContactPolicy(linearSurfaceVelocity = vec3(Inf.float32, 0, 0)))
    expect JoltError:
      firstWorld.setBodyPairContactPolicy(first, foreign, bodyPairContactPolicy())
    expect ValueError:
      firstWorld.setSubShapePairContactPolicy(
        first, 0, first, 0, bodyPairContactPolicy())
    expect JoltError:
      firstWorld.setSubShapePairContactPolicy(
        first, 0, foreign, 0, bodyPairContactPolicy())
    firstWorld.setBodyPairContactPolicy(first, second, bodyPairContactPolicy())
    check firstWorld.bodyPairContactPolicyCount == 1
    second.close()
    check firstWorld.bodyPairContactPolicyCount == 0
    first.close()
    foreign.close()

  test "policy layers values duplicates and enabled pairs are validated":
    var config = defaultWorldConfig()
    config.contactPolicies.add(contactPolicy(
      nonMovingLayer, CollisionLayer(9)))
    expect ValueError:
      discard newWorld(config)

    config = defaultWorldConfig()
    config.collisionLayers.add(collisionLayerConfig(1))
    config.contactPolicies.add(contactPolicy(
      nonMovingLayer, CollisionLayer(2)))
    expect ValueError:
      discard newWorld(config)

    config = defaultWorldConfig()
    config.contactPolicies = @[
      contactPolicy(nonMovingLayer, movingLayer),
      contactPolicy(movingLayer, nonMovingLayer)]
    expect ValueError:
      discard newWorld(config)

    config = defaultWorldConfig()
    config.contactPolicies.add(contactPolicy(
      nonMovingLayer, movingLayer, friction = some(-0.1'f32)))
    expect ValueError:
      discard newWorld(config)

    config = defaultWorldConfig()
    config.contactPolicies.add(contactPolicy(
      nonMovingLayer, movingLayer, restitution = some(1.1'f32)))
    expect ValueError:
      discard newWorld(config)

    config = defaultWorldConfig()
    config.contactPolicies.add(contactPolicy(
      nonMovingLayer, movingLayer, inverseMassScale2 = -1))
    expect ValueError:
      discard newWorld(config)

    config = defaultWorldConfig()
    config.contactPolicies.add(contactPolicy(
      nonMovingLayer, movingLayer,
      angularSurfaceVelocity = vec3(NaN.float32, 0, 0)))
    expect ValueError:
      discard newWorld(config)
