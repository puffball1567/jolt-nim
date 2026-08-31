import std/[math, options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc softWorld(): World =
  var config = defaultWorldConfig()
  config.numThreads = 1
  config.maxBodies = 256
  config.maxBodyPairs = 2_048
  config.maxContactConstraints = 2_048
  newWorld(config)

proc tetrahedronMesh(): SoftBodyMesh =
  result.vertices = @[
    softBodyVertex(vec3(0, 1, 0)),
    softBodyVertex(vec3(-1, -1, -1)),
    softBodyVertex(vec3(1, -1, -1)),
    softBodyVertex(vec3(0, -1, 1))
  ]
  result.faces = @[
    softBodyFace(0, 2, 1),
    softBodyFace(0, 3, 2),
    softBodyFace(0, 1, 3),
    softBodyFace(1, 2, 3)
  ]

proc skinnedCloth(maxDistance: float32;
                  weights = @[softBodySkinWeight(0, 1)]): SoftBodyMesh =
  result = clothSoftBodyMesh(4, 4, 0.5)
  result.skinBindPose = @[softBodyJointTransform(vec3(0, 0, 0))]
  for vertex in 0 ..< result.vertices.len:
    result.skinConstraints.add(softBodySkinConstraint(
      vertex, weights, maxDistance = maxDistance))

proc distance(first, second: Vec3): float32 =
  let dx = second.x - first.x
  let dy = second.y - first.y
  let dz = second.z - first.z
  sqrt(dx * dx + dy * dy + dz * dz)

proc tetrahedronVolume(body: SoftBody): float32 =
  let p1 = body.vertexState(0).position
  let p2 = body.vertexState(1).position
  let p3 = body.vertexState(2).position
  let p4 = body.vertexState(3).position
  let e1 = vec3(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
  let e2 = vec3(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z)
  let e3 = vec3(p4.x - p1.x, p4.y - p1.y, p4.z - p1.z)
  abs(
    (e1.y * e2.z - e1.z * e2.y) * e3.x +
    (e1.z * e2.x - e1.x * e2.z) * e3.y +
    (e1.x * e2.y - e1.y * e2.x) * e3.z) / 6.0'f32

suite "Jolt soft bodies":
  test "cloth topology, material and creation settings reach Jolt":
    let world = softWorld()
    defer: world.close()
    let silk = physicsMaterial("silk", materialColor(170, 70, 190))
    let mesh = clothSoftBodyMesh(6, 5, 0.4, [0, 5])
    var config = defaultSoftBodyConfig()
    config.material = some(silk)
    config.facesDoubleSided = true
    config.numIterations = 8
    config.vertexRadius = 0.03
    let cloth = world.addSoftBody(mesh, vec3(0, 5, 0), config = config)

    check cloth.isAlive
    check cloth.vertexCount == 30
    check cloth.faceCount == 40
    check cloth.face(0).vertices == [0'u32, 6'u32, 7'u32]
    check cloth.configuration.material.get == silk
    check cloth.configuration.numIterations == 8
    check not cloth.configuration.updatePosition
    check cloth.runtimeState.numIterations == 8
    check cloth.runtimeState.facesDoubleSided
    check abs(cloth.runtimeState.vertexRadius - 0.03) < 0.0001
    check cloth.vertexState(0).inverseMass == 0
    check cloth.vertexState(1).inverseMass == 1

  test "faces retain independent native materials":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let fabric = physicsMaterial("fabric", materialColor(35, 160, 220))
    let leather = physicsMaterial("leather", materialColor(125, 70, 35))
    var mesh: SoftBodyMesh
    mesh.vertices = @[
      softBodyVertex(vec3(-3, 0, -1), inverseMass = 0),
      softBodyVertex(vec3(-2, 0, 1), inverseMass = 0),
      softBodyVertex(vec3(-1, 0, -1), inverseMass = 0),
      softBodyVertex(vec3(1, 0, -1), inverseMass = 0),
      softBodyVertex(vec3(2, 0, 1), inverseMass = 0),
      softBodyVertex(vec3(3, 0, -1), inverseMass = 0)
    ]
    mesh.faces = @[softBodyFace(0, 1, 2, 0), softBodyFace(3, 4, 5, 1)]
    mesh = mesh.withMaterials([fabric, leather])
    let surface = world.addSoftBody(mesh, vec3(0, 2, 0))
    discard world.step(dt)

    let left = world.castRay(vec3(-2, 5, 0), vec3(0, -1, 0), 6)
    let right = world.castRay(vec3(2, 5, 0), vec3(0, -1, 0), 6)
    check left.isSome
    check right.isSome
    check left.get.hits(surface)
    check right.get.hits(surface)
    check left.get.subShapeId != right.get.subShapeId
    check left.get.material(world).get == fabric
    check right.get.material(world).get == leather
    check surface.materialAt(left.get.subShapeId).get == fabric
    check surface.materialAt(right.get.subShapeId).get == leather
    check surface.face(0).materialIndex == 0
    check surface.face(1).materialIndex == 1
    check surface.mesh.materials == @[fabric, leather]

    var invalid = mesh
    invalid.faces[0].materialIndex = 2
    expect(ValueError):
      discard world.addSoftBody(invalid, vec3(0, 2, 0))
    var conflicting = defaultSoftBodyConfig()
    conflicting.material = some(fabric)
    expect(ValueError):
      discard world.addSoftBody(mesh, vec3(0, 2, 0), config = conflicting)

  test "free cloth falls and collides with a rigid floor":
    let world = softWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(12, 0.5, 12)), vec3(0, -0.5, 0))
    var config = defaultSoftBodyConfig()
    config.vertexRadius = 0.05
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(8, 8, 0.35), vec3(0, 5, 0), config = config)
    let initialY = cloth.vertexState(0).position.y

    for _ in 0 ..< 240:
      check world.step(dt) == {}
    var minimumY = high(float32)
    var maximumY = -high(float32)
    for vertex in cloth.vertices:
      minimumY = min(minimumY, vertex.position.y)
      maximumY = max(maximumY, vertex.position.y)
    check maximumY < initialY - 2
    check minimumY > -0.1
    check maximumY < 0.4
    let softContacts = world.drainSoftBodyContactEvents()
    check softContacts.len > 0
    var sawFloorVertex = false
    for contact in softContacts:
      if contact.softBody == cloth.id and contact.otherBody == floor.id and
          contact.vertex.isSome and not contact.isSensor:
        sawFloorVertex = true
        check int(contact.vertex.get) < cloth.vertexCount
        check abs(contact.contactNormal.y) > 0.5
        break
    check sawFloorVertex

  test "custom updates settle a cloth without stepping the world":
    let world = softWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    var config = defaultSoftBodyConfig()
    config.vertexRadius = 0.04
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(6, 6, 0.35), vec3(0, 4, 0), config = config)
    let initial = cloth.vertexState(0).position

    cloth.customUpdate(dt)
    check cloth.vertexState(0).position.y < initial.y
    cloth.settle(179, dt)
    var minimumY = high(float32)
    var maximumY = -high(float32)
    for vertex in cloth.vertices:
      minimumY = min(minimumY, vertex.position.y)
      maximumY = max(maximumY, vertex.position.y)
    check minimumY > -0.15
    check maximumY < 0.5
    check world.step(dt) == {}
    cloth.customUpdate(dt)

    expect(ValueError): cloth.customUpdate(0)
    expect(ValueError): cloth.settle(-1)
    cloth.close()
    expect(JoltError): cloth.customUpdate(dt)

  test "soft body sensor contacts are queued without collision response":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let sensor = world.addStaticBody(
      boxShape(vec3(3, 1, 3)), vec3(0, 5, 0), sensor = true)
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(5, 5, 0.4), vec3(0, 5, 0))
    let before = cloth.vertexState(12).position
    for _ in 0 ..< 8:
      discard world.step(dt)
    let events = world.drainSoftBodyContactEvents()
    var sawSensor = false
    for event in events:
      if event.softBody == cloth.id and event.otherBody == sensor.id and
          event.isSensor:
        sawSensor = true
        check event.vertex.isNone
    check sawSensor
    check distance(cloth.vertexState(12).position, before) < 0.02

  test "soft body contact queue is bounded and reports overflow":
    var config = defaultWorldConfig()
    config.numThreads = 1
    config.maxBodies = 64
    config.maxQueuedEvents = 3
    let world = newWorld(config)
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    discard world.addStaticBody(
      boxShape(vec3(3, 1, 3)), vec3(0, 5, 0), sensor = true)
    discard world.addSoftBody(
      clothSoftBodyMesh(4, 4, 0.4), vec3(0, 5, 0))
    discard world.drainEvents()
    discard world.drainSoftBodyContactEvents()
    discard world.droppedEventCount(reset = true)
    for _ in 0 ..< 10:
      discard world.step(dt)
    check world.pendingSoftBodyContactEventCount == 3
    check world.droppedEventCount() >= 7
    check world.drainSoftBodyContactEvents().len == 3

  test "fixed cloth corners remain attached while the center sags":
    let world = softWorld()
    defer: world.close()
    let columns = 9
    let rows = 9
    let last = columns * rows - 1
    let mesh = clothSoftBodyMesh(
      columns, rows, 0.5, [0, columns - 1, last - columns + 1, last])
    let cloth = world.addSoftBody(mesh, vec3(0, 6, 0))
    let fixedBefore = [
      cloth.vertexState(0).position,
      cloth.vertexState(columns - 1).position,
      cloth.vertexState(last - columns + 1).position,
      cloth.vertexState(last).position
    ]
    for _ in 0 ..< 180:
      discard world.step(dt)

    let center = cloth.vertexState((rows div 2) * columns + columns div 2)
    check center.position.y < fixedBefore[0].y - 0.2
    for index, vertex in [0, columns - 1, last - columns + 1, last]:
      let current = cloth.vertexState(vertex).position
      check abs(current.x - fixedBefore[index].x) < 0.001
      check abs(current.y - fixedBefore[index].y) < 0.001
      check abs(current.z - fixedBefore[index].z) < 0.001

  test "runtime vertex and pressure controls update native motion properties":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var creation = defaultSoftBodyConfig()
    creation.userData = 0x1234_5678'u64
    creation.allowSleeping = false
    let soft = world.addSoftBody(
      tetrahedronMesh(), vec3(0, 4, 0), config = creation)

    check abs(soft.runtimeState.volume) > 0.1
    check soft.userData == 0x1234_5678'u64
    check not soft.allowsSleeping
    soft.setUserData(0xfeed_beef'u64)
    soft.setAllowSleeping(true)
    check soft.userData == 0xfeed_beef'u64
    check soft.allowsSleeping
    let groupFilter = newCollisionGroupFilter(3)
    defer: groupFilter.close()
    let assignedGroup = groupFilter.bodyCollisionGroup(77, 2)
    soft.setCollisionGroup(assignedGroup)
    check soft.collisionGroup.isSome
    check soft.collisionGroup.get.groupId == 77
    check soft.collisionGroup.get.subgroupId == 2
    soft.clearCollisionGroup()
    check soft.collisionGroup.isNone
    soft.setNumIterations(11)
    soft.setPressure(250)
    soft.setVertexRadius(0.08)
    soft.setFacesDoubleSided(true)
    soft.setFriction(0.75)
    soft.setRestitution(0.35)
    soft.setGravityFactor(0.4)
    soft.setMaxLinearVelocity(42)
    soft.setLinearDamping(0.25)
    soft.setVertexVelocity(0, vec3(0, 3, 0))
    soft.setVertexInverseMass(1, 0)
    let runtime = soft.runtimeState
    check runtime.numIterations == 11
    check runtime.pressure == 250
    check abs(runtime.vertexRadius - 0.08) < 0.0001
    check runtime.facesDoubleSided
    check abs(soft.friction - 0.75) < 0.0001
    check abs(soft.restitution - 0.35) < 0.0001
    check abs(soft.gravityFactor - 0.4) < 0.0001
    check abs(soft.maxLinearVelocity - 42) < 0.0001
    check abs(soft.linearDamping - 0.25) < 0.0001
    check not runtime.updatePosition
    check soft.vertexState(1).inverseMass == 0
    check soft.vertexState(0).velocity.y > 2.9
    expect(ValueError): soft.setUpdatePosition(true)
    expect(IndexDefect): discard soft.vertexState(4)
    expect(ValueError): soft.setPressure(-1)
    expect(ValueError): soft.setNumIterations(0)

  test "world snapshots restore soft body vertices and lifetime is safe":
    let world = softWorld()
    let cloth = world.addSoftBody(
      clothSoftBodyMesh(6, 6, 0.4), vec3(0, 5, 0))
    for _ in 0 ..< 30:
      discard world.step(dt)
    let before = cloth.vertexState(10)
    let state = world.saveState()
    for _ in 0 ..< 90:
      discard world.step(dt)
    check abs(cloth.vertexState(10).position.y - before.position.y) > 0.5
    world.restoreState(state)
    let restored = cloth.vertexState(10)
    check abs(restored.position.x - before.position.x) < 0.0001
    check abs(restored.position.y - before.position.y) < 0.0001
    check abs(restored.position.z - before.position.z) < 0.0001
    check abs(restored.velocity.y - before.velocity.y) < 0.0001
    state.close()

    cloth.close()
    cloth.close()
    check not cloth.isAlive
    expect(JoltError): discard cloth.vertexCount
    let remaining = world.addSoftBody(
      clothSoftBodyMesh(3, 3), vec3(0, 2, 0))
    world.close()
    check not remaining.isAlive
    expect(JoltError): discard remaining.vertexState(0)

  test "volume constraints preserve a tetrahedron under deformation":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var constrainedMesh = tetrahedronMesh()
    constrainedMesh.volumeConstraints = @[
      softBodyVolumeConstraint(0, 1, 2, 3)]
    var looseMesh = tetrahedronMesh()
    var config = defaultSoftBodyConfig()
    config.bendType = SoftBodyBendType.NoBend
    config.edgeCompliance = 1.0'f32
    config.shearCompliance = 1.0'f32
    config.numIterations = 12
    config.linearDamping = 0
    let constrained = world.addSoftBody(
      constrainedMesh, vec3(-3, 4, 0), config = config)
    let loose = world.addSoftBody(looseMesh, vec3(3, 4, 0), config = config)
    let initialConstrained = constrained.tetrahedronVolume()
    let initialLoose = loose.tetrahedronVolume()
    constrained.setVertexVelocity(0, vec3(0, -12, 0))
    loose.setVertexVelocity(0, vec3(0, -12, 0))
    for _ in 0 ..< 90:
      discard world.step(dt)
    let constrainedError =
      abs(constrained.tetrahedronVolume() - initialConstrained)
    let looseError = abs(loose.tetrahedronVolume() - initialLoose)
    check constrained.volumeConstraintCount == 1
    check constrained.volumeConstraint(0).vertices ==
      [0'u32, 1'u32, 2'u32, 3'u32]
    check constrainedError < initialConstrained * 0.1
    check looseError > constrainedError * 3

  test "long range attachments prevent highly compliant cloth stretching":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    const columns = 5
    const rows = 9
    let fixedTop = [0, 1, 2, 3, 4]
    let mesh = clothSoftBodyMesh(columns, rows, 0.4, fixedTop)
    var looseConfig = defaultSoftBodyConfig()
    looseConfig.bendType = SoftBodyBendType.NoBend
    looseConfig.edgeCompliance = 0.1'f32
    looseConfig.shearCompliance = 0.1'f32
    looseConfig.linearDamping = 0
    looseConfig.maxLinearVelocity = 100
    let loose = world.addSoftBody(mesh, vec3(-3, 7, 0), config = looseConfig)
    var attachedBodies: seq[SoftBody]
    for index, lraType in [
        SoftBodyLRAType.EuclideanLRA,
        SoftBodyLRAType.GeodesicLRA]:
      var attachedConfig = looseConfig
      attachedConfig.lraType = lraType
      attachedBodies.add(world.addSoftBody(
        mesh, vec3(float32(index) * 3, 7, 0), config = attachedConfig))
    let bottom = columns * rows - (columns div 2) - 1
    let looseBefore = loose.vertexState(bottom).position
    var attachedBefore: seq[Vec3]
    for attached in attachedBodies:
      attachedBefore.add(attached.vertexState(bottom).position)
    loose.setVertexVelocity(bottom, vec3(0, -40, 0))
    for attached in attachedBodies:
      attached.setVertexVelocity(bottom, vec3(0, -40, 0))
    for _ in 0 ..< 90:
      discard world.step(dt)
    let looseStretch = looseBefore.y - loose.vertexState(bottom).position.y
    for index, attached in attachedBodies:
      let attachedStretch = attachedBefore[index].y -
        attached.vertexState(bottom).position.y
      check attached.configuration.lraType == [
        SoftBodyLRAType.EuclideanLRA,
        SoftBodyLRAType.GeodesicLRA][index]
      check looseStretch > attachedStretch + 0.5
      check attachedStretch < 1.0

  test "face-free Cosserat rod chains retain segment lengths and sag":
    let world = softWorld()
    defer: world.close()
    var points: seq[Vec3]
    for index in 0 ..< 9:
      points.add(vec3(float32(index) * 0.5, 0, 0))
    let mesh = rodSoftBodyMesh(points, inverseMass = 0.05,
      rodCompliance = 1.0e-6, bendTwistCompliance = 1.0e-4)
    var config = defaultSoftBodyConfig()
    config.numIterations = 12
    config.vertexRadius = 0.02
    let rod = world.addSoftBody(mesh, vec3(-2, 8, 0), config = config)
    let initialCounts = rod.constraintCounts
    check initialCounts.rods == points.len - 1
    check initialCounts.rodBendTwists == points.len - 2
    let initialBounds = rod.localBounds
    check initialBounds.minimum.x <= initialBounds.maximum.x
    check initialBounds.minimum.y <= initialBounds.maximum.y
    check initialBounds.minimum.z <= initialBounds.maximum.z
    let lastBefore = rod.vertexState(points.len - 1).position
    for _ in 0 ..< 180:
      discard world.step(dt)
    let lastAfter = rod.vertexState(points.len - 1).position
    check rod.faceCount == 0
    check rod.rodCount == points.len - 1
    check rod.rodBendTwistConstraintCount == points.len - 2
    check rod.rod(0).vertices == [0'u32, 1'u32]
    check rod.rodBendTwistConstraint(0).rods == [0'u32, 1'u32]
    let nativeRod = rod.rodState(0)
    let rotationLength = sqrt(
      nativeRod.rotation.x * nativeRod.rotation.x +
      nativeRod.rotation.y * nativeRod.rotation.y +
      nativeRod.rotation.z * nativeRod.rotation.z +
      nativeRod.rotation.w * nativeRod.rotation.w)
    check abs(rotationLength - 1) < 0.001
    check nativeRod.angularVelocity.x.classify notin {fcNan, fcInf, fcNegInf}
    check nativeRod.angularVelocity.y.classify notin {fcNan, fcInf, fcNegInf}
    check nativeRod.angularVelocity.z.classify notin {fcNan, fcInf, fcNegInf}
    expect(IndexDefect): discard rod.rodState(points.len)
    check lastAfter.y < lastBefore.y - 0.5
    for constraint in mesh.rods:
      let first = rod.vertexState(int(constraint.vertices[0])).position
      let second = rod.vertexState(int(constraint.vertices[1])).position
      check abs(distance(first, second) - 0.5) < 0.08

  test "rod diagnostics retain authored indices after native optimization":
    let world = softWorld()
    defer: world.close()
    var mesh: SoftBodyMesh
    mesh.vertices = @[
      softBodyVertex(vec3(0, 0, 0), inverseMass = 0),
      softBodyVertex(vec3(1, 0, 0)),
      softBodyVertex(vec3(1, 1, 0)),
      softBodyVertex(vec3(2, 1, 0)),
      softBodyVertex(vec3(2, 1, 1))]
    mesh.rods = @[
      softBodyRodConstraint(3, 4),
      softBodyRodConstraint(0, 1),
      softBodyRodConstraint(2, 3),
      softBodyRodConstraint(1, 2)]
    mesh.rodBendTwistConstraints = @[
      softBodyRodBendTwistConstraint(1, 3),
      softBodyRodBendTwistConstraint(3, 2),
      softBodyRodBendTwistConstraint(2, 0)]
    let body = world.addSoftBody(mesh, vec3(0, 4, 0))
    var remap = body.rodOptimizationRemap
    check remap.stretchShear.len == mesh.rods.len
    check remap.bendTwist.len == mesh.rodBendTwistConstraints.len
    var seenRods = newSeq[bool](mesh.rods.len)
    var seenPairs = newSeq[bool](mesh.rodBendTwistConstraints.len)
    var reordered = false
    for authored, nativeIndex in remap.stretchShear:
      check int(nativeIndex) < seenRods.len
      check not seenRods[int(nativeIndex)]
      seenRods[int(nativeIndex)] = true
      check body.rodNativeIndex(authored) == int(nativeIndex)
      reordered = reordered or authored != int(nativeIndex)
      discard body.rodState(authored)
    for authored, nativeIndex in remap.bendTwist:
      check int(nativeIndex) < seenPairs.len
      check not seenPairs[int(nativeIndex)]
      seenPairs[int(nativeIndex)] = true
      check body.rodBendTwistNativeIndex(authored) == int(nativeIndex)
      reordered = reordered or authored != int(nativeIndex)
    check reordered
    remap.stretchShear[0] = 99
    check body.rodNativeIndex(0) != 99
    expect(IndexDefect): discard body.rodNativeIndex(-1)
    expect(IndexDefect): discard body.rodBendTwistNativeIndex(3)
    body.close()
    expect(JoltError): discard body.rodOptimizationRemap

  test "hard skinning follows an animated joint and resets velocities":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var config = defaultSoftBodyConfig()
    config.updatePosition = false
    let cloth = world.addSoftBody(
      skinnedCloth(0), vec3(0, 6, 0), config = config)
    let before = cloth.vertices
    cloth.setVertexVelocity(15, vec3(0, -30, 0))
    cloth.skinVertices(
      [softBodyJointTransform(vec3(0, 2, 0))], hardSkinAll = true)
    let reset = cloth.vertices
    for index in 0 ..< reset.len:
      check abs(reset[index].position.x - before[index].position.x) < 0.001
      check abs(reset[index].position.y - before[index].position.y - 2) < 0.001
      check abs(reset[index].position.z - before[index].position.z) < 0.001
      check distance(reset[index].velocity, vec3(0, 0, 0)) < 0.001

    cloth.skinVertices([softBodyJointTransform(vec3(0, 3, 0))])
    for _ in 0 ..< 5:
      discard world.step(dt)
    check abs(cloth.vertexState(15).position.y - before[15].position.y - 3) < 0.05
    check cloth.skinJointCount == 1
    check cloth.skinConstraintCount == 16
    check cloth.skinConstraint(0).maxDistance == 0
    check cloth.runtimeState.skinConstraintsEnabled

  test "normalized multi-joint weights blend poses and copies stay independent":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var mesh = clothSoftBodyMesh(3, 3, 0.5)
    mesh.skinBindPose = @[
      softBodyJointTransform(vec3(0, 0, 0)),
      softBodyJointTransform(vec3(0, 0, 0))]
    let blendedWeights = [
      softBodySkinWeight(0, 1), softBodySkinWeight(1, 3)]
    for vertex in 0 ..< mesh.vertices.len:
      mesh.skinConstraints.add(softBodySkinConstraint(
        vertex, blendedWeights, maxDistance = 0))
    var config = defaultSoftBodyConfig()
    config.updatePosition = false
    let cloth = world.addSoftBody(mesh, vec3(0, 5, 0), config = config)
    let before = cloth.vertexState(4).position
    cloth.skinVertices([
      softBodyJointTransform(vec3(0, 0, 0)),
      softBodyJointTransform(vec3(0, 4, 0))], hardSkinAll = true)
    check abs(cloth.vertexState(4).position.y - before.y - 3) < 0.001
    check abs(cloth.skinConstraint(0).weights[0].weight - 0.25) < 0.0001
    check abs(cloth.skinConstraint(0).weights[1].weight - 0.75) < 0.0001
    mesh.skinConstraints[0].weights[0].weight = 1
    check abs(cloth.skinConstraint(0).weights[0].weight - 0.25) < 0.0001

  test "skin enable and distance multiplier control native constraint limits":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var config = defaultSoftBodyConfig()
    config.updatePosition = false
    config.linearDamping = 0
    let cloth = world.addSoftBody(
      skinnedCloth(0.5), vec3(0, 6, 0), config = config)
    let target = cloth.vertexState(15).position
    cloth.setSkinConstraintsEnabled(false)
    cloth.setVertexVelocity(15, vec3(0, -12, 0))
    for _ in 0 ..< 30:
      discard world.step(dt)
    check distance(cloth.vertexState(15).position, target) > 1

    cloth.setSkinConstraintsEnabled(true)
    cloth.setSkinnedMaxDistanceMultiplier(1)
    cloth.skinVertices([softBodyJointTransform(vec3(0, 0, 0))])
    for _ in 0 ..< 4:
      discard world.step(dt)
    check distance(cloth.vertexState(15).position, target) < 0.55
    cloth.setSkinnedMaxDistanceMultiplier(0)
    discard world.step(dt)
    check distance(cloth.vertexState(15).position, target) < 0.02
    check cloth.runtimeState.skinnedMaxDistanceMultiplier == 0

  test "skin back-stop prevents vertices crossing behind the animated surface":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var stoppedMesh = skinnedCloth(2)
    for constraint in stoppedMesh.skinConstraints.mitems:
      constraint.backStopDistance = 0.1
      constraint.backStopRadius = 40
    let looseMesh = skinnedCloth(2)
    var config = defaultSoftBodyConfig()
    config.updatePosition = false
    config.linearDamping = 0
    let stopped = world.addSoftBody(
      stoppedMesh, vec3(-2, 6, 0), config = config)
    let loose = world.addSoftBody(
      looseMesh, vec3(2, 6, 0), config = config)
    let stoppedTarget = stopped.vertexState(5).position
    let looseTarget = loose.vertexState(5).position
    for vertex in 0 ..< stopped.vertexCount:
      stopped.setVertexVelocity(vertex, vec3(0, -8, 0))
      loose.setVertexVelocity(vertex, vec3(0, -8, 0))
    stopped.skinVertices([softBodyJointTransform(vec3(0, 0, 0))])
    loose.skinVertices([softBodyJointTransform(vec3(0, 0, 0))])
    for _ in 0 ..< 12:
      discard world.step(dt)
    let stoppedDrop = stoppedTarget.y - stopped.vertexState(5).position.y
    let looseDrop = looseTarget.y - loose.vertexState(5).position.y
    check stoppedDrop < 0.15
    check looseDrop > 0.5
    check looseDrop > stoppedDrop + 0.4

  test "world snapshots restore skin state for deterministic replay":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    var config = defaultSoftBodyConfig()
    config.updatePosition = false
    let cloth = world.addSoftBody(
      skinnedCloth(0.25), vec3(0, 5, 0), config = config)
    cloth.skinVertices(
      [softBodyJointTransform(vec3(0, 1, 0))], hardSkinAll = true)
    discard world.step(dt)
    let savedVertex = cloth.vertexState(15)
    let snapshot = world.saveState()
    defer: snapshot.close()

    cloth.skinVertices([softBodyJointTransform(vec3(0, 3, 0))])
    for _ in 0 ..< 5:
      discard world.step(dt)
    check distance(cloth.vertexState(15).position, savedVertex.position) > 1
    world.restoreState(snapshot)
    check distance(cloth.vertexState(15).position, savedVertex.position) < 0.001

    cloth.skinVertices([softBodyJointTransform(vec3(0, 2, 0))])
    for _ in 0 ..< 4:
      discard world.step(dt)
    let firstReplay = cloth.vertexState(15)
    world.restoreState(snapshot)
    cloth.skinVertices([softBodyJointTransform(vec3(0, 2, 0))])
    for _ in 0 ..< 4:
      discard world.step(dt)
    let secondReplay = cloth.vertexState(15)
    check distance(firstReplay.position, secondReplay.position) < 0.0001
    check distance(firstReplay.velocity, secondReplay.velocity) < 0.0001

  test "explicit edge, dihedral and long-range constraints reach Jolt":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))

    var particles: SoftBodyMesh
    particles.vertices = @[
      softBodyVertex(vec3(0, 0, 0), inverseMass = 0),
      softBodyVertex(vec3(1, 0, 0), velocity = vec3(8, 0, 0)),
      softBodyVertex(vec3(0, 2, 0))
    ]
    particles.edgeConstraints = @[softBodyEdgeConstraint(0, 1)]
    particles.longRangeConstraints = @[
      softBodyLongRangeConstraint(0, 1, 1.25)]
    var particleConfig = defaultSoftBodyConfig()
    particleConfig.bendType = SoftBodyBendType.NoBend
    particleConfig.lraType = SoftBodyLRAType.NoLRA
    particleConfig.linearDamping = 0
    let tether = world.addSoftBody(
      particles, vec3(-2, 4, 0), config = particleConfig)
    let tetherCounts = tether.constraintCounts
    check tetherCounts.edges == 1
    check tetherCounts.dihedralBends == 0
    check tetherCounts.longRangeAttachments == 1
    check tether.mesh.edgeConstraints.len == 1
    check tether.mesh.longRangeConstraints.len == 1
    for _ in 0 ..< 30:
      discard world.step(dt)
    check distance(tether.vertexState(0).position,
      tether.vertexState(1).position) < 1.05

    var folded: SoftBodyMesh
    folded.vertices = @[
      softBodyVertex(vec3(-1, 0, 0), inverseMass = 0),
      softBodyVertex(vec3(1, 0, 0), inverseMass = 0),
      softBodyVertex(vec3(0, 1, 0)),
      softBodyVertex(vec3(0, -1, 0))
    ]
    folded.faces = @[softBodyFace(0, 2, 1), softBodyFace(0, 1, 3)]
    folded.dihedralBendConstraints = @[
      softBodyDihedralBendConstraint(0, 1, 2, 3)]
    var foldConfig = defaultSoftBodyConfig()
    foldConfig.bendType = SoftBodyBendType.NoBend
    foldConfig.edgeCompliance = softBodyDisabledCompliance
    foldConfig.shearCompliance = softBodyDisabledCompliance
    let fold = world.addSoftBody(folded, vec3(2, 4, 0), config = foldConfig)
    let foldCounts = fold.constraintCounts
    check foldCounts.dihedralBends == 1
    check fold.mesh.dihedralBendConstraints.len == 1
    fold.setVertexVelocity(3, vec3(0, 0, 4))
    for _ in 0 ..< 12:
      discard world.step(dt)
    let foldedPosition = fold.vertexState(3).position
    check foldedPosition.x.classify notin {fcNan, fcInf, fcNegInf}
    check foldedPosition.y.classify notin {fcNan, fcInf, fcNegInf}
    check foldedPosition.z.classify notin {fcNan, fcInf, fcNegInf}

    expect(ValueError):
      discard softBodyLongRangeConstraint(0, 0, 1)
    var invalid = particles
    invalid.longRangeConstraints = @[
      SoftBodyLongRangeConstraint(vertices: [1'u32, 0'u32], maxDistance: 1)]
    expect(ValueError):
      discard world.addSoftBody(invalid, vec3(0, 2, 0))

  test "per-vertex attributes create reinforced and unconstrained regions":
    let world = softWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let uniform = softBodyVertexAttributes(
      edgeCompliance = 1.0e-5,
      shearCompliance = 1.0e-5,
      bendCompliance = 1.0e-5)
    var reinforcedMesh = clothSoftBodyMesh(4, 4, 0.5)
    reinforcedMesh.vertexAttributes = @[uniform]
    var selectiveMesh = clothSoftBodyMesh(4, 4, 0.5)
    selectiveMesh.vertexAttributes = newSeq[SoftBodyVertexAttributes](
      selectiveMesh.vertices.len)
    for index in 0 ..< selectiveMesh.vertexAttributes.len:
      selectiveMesh.vertexAttributes[index] = uniform
    selectiveMesh.vertexAttributes[0] = softBodyVertexAttributes(
      edgeCompliance = softBodyDisabledCompliance,
      shearCompliance = softBodyDisabledCompliance,
      bendCompliance = softBodyDisabledCompliance)
    var config = defaultSoftBodyConfig()
    config.linearDamping = 0
    config.updatePosition = false
    let reinforced = world.addSoftBody(
      reinforcedMesh, vec3(-3, 5, 0), config = config)
    let selective = world.addSoftBody(
      selectiveMesh, vec3(3, 5, 0), config = config)

    let reinforcedCounts = reinforced.constraintCounts
    let selectiveCounts = selective.constraintCounts
    check reinforcedCounts.edges > 0
    check selectiveCounts.edges < reinforcedCounts.edges
    check reinforced.mesh.vertexAttributes.len == 1
    check selective.mesh.vertexAttributes.len == selectiveMesh.vertices.len

    let reinforcedBefore = reinforced.vertexState(0).position
    let selectiveBefore = selective.vertexState(0).position
    reinforced.setVertexVelocity(0, vec3(0, 4, 0))
    selective.setVertexVelocity(0, vec3(0, 4, 0))
    for _ in 0 ..< 30:
      discard world.step(dt)
    let reinforcedTravel = distance(
      reinforcedBefore, reinforced.vertexState(0).position)
    let selectiveTravel = distance(
      selectiveBefore, selective.vertexState(0).position)
    check selectiveTravel > reinforcedTravel + 0.5

    var attachedMesh = clothSoftBodyMesh(4, 4, 0.5, [0, 3])
    attachedMesh.vertexAttributes = @[softBodyVertexAttributes(
      edgeCompliance = 0.5,
      shearCompliance = 0.5,
      bendCompliance = 0.5,
      lraType = SoftBodyLRAType.EuclideanLRA,
      lraMaxDistanceMultiplier = 1.02)]
    let attached = world.addSoftBody(
      attachedMesh, vec3(0, 5, 4), config = config)
    check attached.configuration.lraType == SoftBodyLRAType.NoLRA
    check attached.constraintCounts.longRangeAttachments > 0

    var invalid = reinforcedMesh
    invalid.vertexAttributes = newSeq[SoftBodyVertexAttributes](
      invalid.vertices.len + 1)
    expect(ValueError):
      discard world.addSoftBody(invalid, vec3(0, 3, 0))
    invalid = reinforcedMesh
    invalid.vertexAttributes[0].edgeCompliance = -1
    expect(ValueError):
      discard world.addSoftBody(invalid, vec3(0, 3, 0))
    invalid = reinforcedMesh
    invalid.vertexAttributes[0].lraType = SoftBodyLRAType.EuclideanLRA
    expect(ValueError):
      discard world.addSoftBody(invalid, vec3(0, 3, 0))

  test "invalid meshes and settings are rejected before native creation":
    let world = softWorld()
    defer: world.close()
    expect(ValueError): discard clothSoftBodyMesh(1, 4)
    expect(IndexDefect): discard clothSoftBodyMesh(3, 3, fixedVertices = [9])
    var badMesh = tetrahedronMesh()
    badMesh.faces[0] = SoftBodyFace(vertices: [0'u32, 1'u32, 9'u32])
    expect(ValueError): discard world.addSoftBody(badMesh, vec3(0, 2, 0))
    badMesh = tetrahedronMesh()
    badMesh.vertices[0].inverseMass = -1
    expect(ValueError): discard world.addSoftBody(badMesh, vec3(0, 2, 0))
    var badConfig = defaultSoftBodyConfig()
    badConfig.bendCompliance = -1
    expect(ValueError):
      discard world.addSoftBody(tetrahedronMesh(), vec3(0, 2, 0),
        config = badConfig)
    badConfig = defaultSoftBodyConfig()
    badConfig.lraType = SoftBodyLRAType.GeodesicLRA
    expect(ValueError):
      discard world.addSoftBody(tetrahedronMesh(), vec3(0, 2, 0),
        config = badConfig)
    badMesh = tetrahedronMesh()
    badMesh.volumeConstraints = @[
      SoftBodyVolumeConstraint(
        vertices: [0'u32, 1'u32, 2'u32, 9'u32], compliance: 0)]
    expect(ValueError): discard world.addSoftBody(badMesh, vec3(0, 2, 0))
    expect(ValueError):
      discard rodSoftBodyMesh([vec3(0, 0, 0), vec3(1, 0, 0)])
    expect(ValueError): discard softBodySkinWeight(0, 0)
    expect(ValueError):
      discard softBodySkinConstraint(0, newSeq[SoftBodySkinWeight]())
    expect(ValueError):
      discard softBodySkinConstraint(0, [
        softBodySkinWeight(0, 1), softBodySkinWeight(1, 1),
        softBodySkinWeight(2, 1), softBodySkinWeight(3, 1),
        softBodySkinWeight(4, 1)])
    expect(ValueError):
      discard softBodySkinConstraint(
        0, [softBodySkinWeight(0, 1)], backStopRadius = 0)
    badMesh = tetrahedronMesh()
    badMesh.skinConstraints = @[
      softBodySkinConstraint(0, [softBodySkinWeight(0, 1)])]
    expect(ValueError): discard world.addSoftBody(badMesh, vec3(0, 2, 0))
    badMesh.skinBindPose = @[softBodyJointTransform(vec3(0, 0, 0))]
    badMesh.skinConstraints[0].weights[0].joint = 1
    expect(ValueError): discard world.addSoftBody(badMesh, vec3(0, 2, 0))
    badConfig = defaultSoftBodyConfig()
    badConfig.skinnedMaxDistanceMultiplier = -1
    expect(ValueError):
      discard world.addSoftBody(skinnedCloth(0.5), vec3(0, 2, 0),
        config = badConfig)
    var skinConfig = defaultSoftBodyConfig()
    skinConfig.updatePosition = false
    let skinned = world.addSoftBody(
      skinnedCloth(0.5), vec3(0, 3, 0), config = skinConfig)
    expect(ValueError): skinned.skinVertices(newSeq[SoftBodyJointTransform]())
    skinned.close()
    expect(JoltError):
      skinned.skinVertices([softBodyJointTransform(vec3(0, 0, 0))])
