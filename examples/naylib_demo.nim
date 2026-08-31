## Optional 3D validation demo using Naylib's typed raylib wrapper.

import std/[math, options, os, strformat, strutils]
import jolt
import raylib as rl
import rlgl

type VisualBody = object
  body: Body
  color: rl.Color

const
  dt = 1.0'f32 / 60.0'f32
  diagnosticLayer = CollisionLayer(2)
  palette = [rl.Red, rl.Blue, rl.Green, rl.Gold, rl.Violet, rl.Orange]

proc rv(value: Vec3): rl.Vector3 =
  rl.Vector3(x: value.x, y: value.y, z: value.z)

proc raylibColor(color: MaterialColor): rl.Color =
  rl.Color(r: color.r, g: color.g, b: color.b, a: color.a)

proc axisAngle(rotation: Quat): tuple[axis: Vec3, degrees: float32] =
  let q = rotation.normalized
  let w = clamp(q.w, -1.0'f32, 1.0'f32)
  let angle = 2.0'f32 * arccos(w)
  let denominator = sqrt(max(0.0'f32, 1.0'f32 - w * w))
  result.axis = if denominator < 1.0e-5'f32:
      vec3(0, 1, 0)
    else:
      vec3(q.x / denominator, q.y / denominator, q.z / denominator)
  result.degrees = angle * 180.0'f32 / PI.float32

proc drawShape(shape: Shape; fallbackColor: rl.Color) =
  let color = if shape.material.isSome:
      shape.material.get.debugColor.raylibColor
    else:
      fallbackColor
  case shape.kind
  of ShapeKind.Box:
    let size = rl.Vector3(
      x: shape.halfExtent.x * 2,
      y: shape.halfExtent.y * 2,
      z: shape.halfExtent.z * 2)
    rl.drawCube(rl.Vector3(), size, color)
    rl.drawCubeWires(rl.Vector3(), size, rl.DarkGray)
  of ShapeKind.Sphere:
    rl.drawSphere(rl.Vector3(), shape.radius, 12, 16, color)
    rl.drawSphereWires(rl.Vector3(), shape.radius, 8, 12, rl.DarkGray)
  of ShapeKind.Capsule:
    rl.drawCapsule(
      rl.Vector3(y: -shape.halfHeight),
      rl.Vector3(y: shape.halfHeight),
      shape.radius, 12, 6, color)
  of ShapeKind.Cylinder:
    rl.drawCylinder(
      rl.Vector3(), shape.radius, shape.radius,
      shape.halfHeight * 2, 18, color)
    rl.drawCylinderWires(
      rl.Vector3(), shape.radius, shape.radius,
      shape.halfHeight * 2, 18, rl.DarkGray)
  of ShapeKind.TaperedCapsule:
    rl.drawCylinder(
      rl.Vector3(), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, color)
    rl.drawCylinderWires(
      rl.Vector3(), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, rl.DarkGray)
    rl.drawSphere(
      rl.Vector3(y: -shape.halfHeight), shape.bottomRadius, 10, 14, color)
    rl.drawSphereWires(
      rl.Vector3(y: -shape.halfHeight), shape.bottomRadius,
      8, 12, rl.DarkGray)
    rl.drawSphere(
      rl.Vector3(y: shape.halfHeight), shape.topRadius, 10, 14, color)
    rl.drawSphereWires(
      rl.Vector3(y: shape.halfHeight), shape.topRadius,
      8, 12, rl.DarkGray)
  of ShapeKind.TaperedCylinder:
    rl.drawCylinder(
      rl.Vector3(), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, color)
    rl.drawCylinderWires(
      rl.Vector3(), shape.bottomRadius, shape.topRadius,
      shape.halfHeight * 2, 18, rl.DarkGray)
  of ShapeKind.Triangle:
    let a = shape.points[0].rv
    let b = shape.points[1].rv
    let c = shape.points[2].rv
    rl.drawTriangle3D(a, b, c, color)
    rl.drawLine3D(a, b, rl.DarkGray)
    rl.drawLine3D(b, c, rl.DarkGray)
    rl.drawLine3D(c, a, rl.DarkGray)
  of ShapeKind.Plane:
    let extent = min(shape.planeHalfExtent, 20.0'f32)
    rl.drawCube(
      rl.Vector3(y: -0.015), rl.Vector3(x: extent * 2, y: 0.03, z: extent * 2),
      color)
  of ShapeKind.Empty:
    rl.drawSphereWires(shape.centerOfMass.rv, 0.16, 6, 8, color)
  of ShapeKind.ConvexHull:
    for point in shape.points:
      rl.drawSphere(point.rv, 0.055, 4, 6, color)
    for first in 0 ..< shape.points.len:
      for second in first + 1 ..< shape.points.len:
        rl.drawLine3D(shape.points[first].rv, shape.points[second].rv, rl.DarkGray)
  of ShapeKind.TriangleMesh:
    for triangle in 0 ..< shape.triangleIndices.len div 3:
      let triangleColor = if triangle < shape.materialIndices.len and
          int(shape.materialIndices[triangle]) < shape.materials.len:
          shape.materials[int(shape.materialIndices[triangle])].debugColor.raylibColor
        else:
          color
      let offset = triangle * 3
      let a = shape.vertices[int(shape.triangleIndices[offset])]
      let b = shape.vertices[int(shape.triangleIndices[offset + 1])]
      let c = shape.vertices[int(shape.triangleIndices[offset + 2])]
      rl.drawTriangle3D(a.rv, b.rv, c.rv, triangleColor)
      rl.drawLine3D(a.rv, b.rv, rl.DarkGray)
      rl.drawLine3D(b.rv, c.rv, rl.DarkGray)
      rl.drawLine3D(c.rv, a.rv, rl.DarkGray)
  of ShapeKind.HeightField:
    let sampleCount = int(shape.sampleCount)
    for z in 0 ..< sampleCount - 1:
      for x in 0 ..< sampleCount - 1:
        let cell = x + z * (sampleCount - 1)
        let cellColor = if cell < shape.materialIndices.len and
            int(shape.materialIndices[cell]) < shape.materials.len:
            shape.materials[int(shape.materialIndices[cell])].debugColor.raylibColor
          else:
            color
        let a = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[z * sampleCount + x],
          shape.heightOffset.z + shape.heightScale.z * float32(z))
        let b = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x + 1),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[z * sampleCount + x + 1],
          shape.heightOffset.z + shape.heightScale.z * float32(z))
        let c = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x + 1),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[(z + 1) * sampleCount + x + 1],
          shape.heightOffset.z + shape.heightScale.z * float32(z + 1))
        let d = vec3(
          shape.heightOffset.x + shape.heightScale.x * float32(x),
          shape.heightOffset.y + shape.heightScale.y *
            shape.heightSamples[(z + 1) * sampleCount + x],
          shape.heightOffset.z + shape.heightScale.z * float32(z + 1))
        rl.drawTriangle3D(a.rv, d.rv, c.rv, cellColor)
        rl.drawTriangle3D(a.rv, c.rv, b.rv, cellColor)
        rl.drawLine3D(a.rv, b.rv, rl.DarkGray)
        rl.drawLine3D(a.rv, d.rv, rl.DarkGray)
  of ShapeKind.StaticCompound, ShapeKind.MutableCompound:
    for child in shape.children:
      let rotation = child.rotation.axisAngle
      pushMatrix()
      translatef(child.position.x, child.position.y, child.position.z)
      rotatef(
        rotation.degrees,
        rotation.axis.x,
        rotation.axis.y,
        rotation.axis.z)
      drawShape(child.shape, color)
      popMatrix()
  of ShapeKind.Scaled:
    pushMatrix()
    scalef(shape.shapeScale.x, shape.shapeScale.y, shape.shapeScale.z)
    drawShape(shape.innerShape, color)
    popMatrix()
  of ShapeKind.RotatedTranslated:
    let rotation = shape.shapeRotation.axisAngle
    pushMatrix()
    translatef(
      shape.shapePosition.x, shape.shapePosition.y, shape.shapePosition.z)
    rotatef(
      rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawShape(shape.innerShape, color)
    popMatrix()
  of ShapeKind.OffsetCenterOfMass:
    drawShape(shape.innerShape, color)

proc draw(visual: VisualBody) =
  let rotation = visual.body.rotation.axisAngle
  pushMatrix()
  let position = visual.body.position
  translatef(position.x, position.y, position.z)
  rotatef(rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
  drawShape(visual.body.shape, visual.color)
  popMatrix()

proc draw(ragdoll: Ragdoll) =
  if ragdoll.isNil or not ragdoll.isAlive:
    return
  var positions = newSeq[jolt.Vec3](ragdoll.partCount)
  for index in 0 ..< ragdoll.partCount:
    positions[index] = ragdoll.partPosition(index)
    let rotation = ragdoll.partRotation(index).axisAngle
    pushMatrix()
    translatef(positions[index].x, positions[index].y, positions[index].z)
    rotatef(rotation.degrees, rotation.axis.x, rotation.axis.y, rotation.axis.z)
    drawShape(ragdoll.partShape(index), palette[index mod palette.len])
    popMatrix()
  for index in 1 ..< ragdoll.partCount:
    let parent = ragdoll.partParent(index).get
    rl.drawLine3D(positions[parent].rv, positions[index].rv, rl.DarkGray)

proc draw(body: SoftBody) =
  if body.isNil or not body.isAlive:
    return
  let vertices = body.vertices
  let material = body.configuration.material
  let color = if material.isSome:
      material.get.debugColor.raylibColor
    else:
      rl.SkyBlue
  for faceIndex in 0 ..< body.faceCount:
    let face = body.face(faceIndex)
    let a = vertices[int(face.vertices[0])].position.rv
    let b = vertices[int(face.vertices[1])].position.rv
    let c = vertices[int(face.vertices[2])].position.rv
    rl.drawTriangle3D(a, b, c, color)
    rl.drawTriangle3D(c, b, a, color)
    rl.drawLine3D(a, b, rl.DarkGray)
    rl.drawLine3D(b, c, rl.DarkGray)
    rl.drawLine3D(c, a, rl.DarkGray)
  for rodIndex in 0 ..< body.rodCount:
    let rod = body.rod(rodIndex)
    let a = vertices[int(rod.vertices[0])].position.rv
    let b = vertices[int(rod.vertices[1])].position.rv
    rl.drawLine3D(a, b, color)
    rl.drawSphere(a, 0.08, 6, 8, color)
    if rodIndex == body.rodCount - 1:
      rl.drawSphere(b, 0.08, 6, 8, color)

proc skinnedRibbonMesh(columns, rows: int; spacing: float32): SoftBodyMesh =
  result = clothSoftBodyMesh(columns, rows, spacing)
  let minimumZ = -0.5'f32 * spacing * float32(rows - 1)
  let maximumZ = -minimumZ
  result.skinBindPose = @[
    softBodyJointTransform(vec3(0, 0, minimumZ)),
    softBodyJointTransform(vec3(0, 0, maximumZ))]
  for row in 0 ..< rows:
    let fraction = float32(row) / float32(rows - 1)
    for column in 0 ..< columns:
      var weights: seq[SoftBodySkinWeight]
      if fraction < 1.0'f32:
        weights.add(softBodySkinWeight(0, 1.0'f32 - fraction))
      if fraction > 0.0'f32:
        weights.add(softBodySkinWeight(1, fraction))
      result.skinConstraints.add(softBodySkinConstraint(
        column + row * columns, weights,
        maxDistance = 0.45, backStopDistance = 0.08))

proc naylibRagdollConfig(origin: Vec3): RagdollConfig =
  let sideways = quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)
  template p(dx, dy, dz: float32): Vec3 =
    vec3(origin.x + dx, origin.y + dy, origin.z + dz)
  ragdollConfig(@[
    ragdollPart(
      "pelvis", capsuleShape(0.24, 0.28), p(0, 1.2, 0),
      ragdollRootJoint(), rotation = sideways),
    ragdollPart(
      "torso", capsuleShape(0.42, 0.3), p(0, 1.85, 0),
      ragdollJoint(
        0, p(0, 1.5, 0), twistAxis = vec3(0, 1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.3,
        planeHalfConeAngle = 0.3, twistMinAngle = -0.2,
        twistMaxAngle = 0.2, maxMotorTorque = 500)),
    ragdollPart(
      "head", capsuleShape(0.12, 0.23), p(0, 2.55, 0),
      ragdollJoint(
        1, p(0, 2.3, 0), twistAxis = vec3(0, 1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.4,
        planeHalfConeAngle = 0.4, twistMinAngle = -0.4,
        twistMaxAngle = 0.4)),
    ragdollPart(
      "left arm", capsuleShape(0.4, 0.14), p(-0.72, 1.98, 0),
      ragdollHingeJoint(
        1, p(-0.34, 2.0, 0), hingeAxis = vec3(0, 0, 1),
        normalAxis = vec3(1, 0, 0), minAngle = -1.8,
        maxAngle = 1.8), rotation = sideways),
    ragdollPart(
      "right arm", capsuleShape(0.4, 0.14), p(0.72, 1.98, 0),
      ragdollHingeJoint(
        1, p(0.34, 2.0, 0), hingeAxis = vec3(0, 0, 1),
        normalAxis = vec3(1, 0, 0), minAngle = -1.8,
        maxAngle = 1.8), rotation = sideways),
    ragdollPart(
      "left leg", capsuleShape(0.5, 0.18), p(-0.22, 0.32, 0),
      ragdollJoint(
        0, p(-0.22, 0.88, 0), twistAxis = vec3(0, -1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.6,
        planeHalfConeAngle = 0.4, twistMinAngle = -0.3,
        twistMaxAngle = 0.3)),
    ragdollPart(
      "right leg", capsuleShape(0.5, 0.18), p(0.22, 0.32, 0),
      ragdollJoint(
        0, p(0.22, 0.88, 0), twistAxis = vec3(0, -1, 0),
        planeAxis = vec3(0, 0, 1), normalHalfConeAngle = 0.6,
        planeHalfConeAngle = 0.4, twistMinAngle = -0.3,
        twistMaxAngle = 0.3))
  ], groupId = 401, distanceConstraints = @[
    ragdollDistanceConstraint(
      3, 4, p(-0.72, 1.98, 0), p(0.72, 1.98, 0), 1.2, 1.8)
  ])

proc main() =
  let frames = if paramCount() >= 1: parseInt(paramStr(1)) else: 0
  let screenshot = if paramCount() >= 2: paramStr(2) else: ""
  if screenshot.len > 0:
    rl.setConfigFlags(rl.flags(rl.ConfigFlags.WindowHidden, rl.ConfigFlags.Msaa4xHint))
  else:
    rl.setConfigFlags(rl.flags(rl.ConfigFlags.WindowResizable, rl.ConfigFlags.Msaa4xHint))
  rl.initWindow(1280, 720, "jolt-nim + Naylib")
  defer: rl.closeWindow()
  if frames == 0:
    rl.setTargetFPS(60)

  var config = defaultWorldConfig()
  config.maxBodies = 512
  config.collisionLayers.add collisionLayerConfig(1)
  config.collisionPairs.add collisionPair(nonMovingLayer, diagnosticLayer)
  config.collisionPairs.add collisionPair(movingLayer, diagnosticLayer)
  config.collisionPairs.add collisionPair(diagnosticLayer, diagnosticLayer)
  let world = newWorld(config)
  defer: world.close()
  var visuals: seq[VisualBody]
  var constraints: seq[Constraint]
  let grass = physicsMaterial("grass", materialColor(55, 165, 75))
  let stone = physicsMaterial("stone", materialColor(125, 135, 145))
  let rubber = physicsMaterial("rubber", materialColor(220, 55, 65))
  let steel = physicsMaterial("steel", materialColor(135, 160, 185))
  const terrainSamples = 16
  var heights = newSeq[float32](terrainSamples * terrainSamples)
  var terrainMaterials = newSeq[uint32](
    (terrainSamples - 1) * (terrainSamples - 1))
  for z in 0 ..< terrainSamples:
    for x in 0 ..< terrainSamples:
      heights[z * terrainSamples + x] =
        sin(float32(x) * 0.7'f32) * 0.12'f32 +
        cos(float32(z) * 0.6'f32) * 0.08'f32
  for z in 0 ..< terrainSamples - 1:
    for x in 0 ..< terrainSamples - 1:
      terrainMaterials[x + z * (terrainSamples - 1)] =
        if (x div 3 + z div 3) mod 2 == 0: 0 else: 1
  let floor = world.addStaticBody(
    heightFieldShape(
      heights, terrainSamples,
      offset = vec3(-12, 0, -12),
      scale = vec3(1.6, 1, 1.6)).withMaterials(
        [grass, stone], terrainMaterials),
    vec3(0, 0, 0))
  visuals.add VisualBody(body: floor, color: rl.Gray)

  var clothConfig = defaultSoftBodyConfig()
  clothConfig.material = some(physicsMaterial(
    "naylib cloth", materialColor(40, 175, 220)))
  clothConfig.facesDoubleSided = true
  clothConfig.vertexRadius = 0.04
  clothConfig.edgeCompliance = 0.06
  clothConfig.shearCompliance = 0.06
  clothConfig.lraType = SoftBodyLRAType.EuclideanLRA
  clothConfig.lraMaxDistanceMultiplier = 1.05
  let clothColumns = 12
  let softCloth = world.addSoftBody(
    clothSoftBodyMesh(clothColumns, 10, 0.42,
      [0, clothColumns - 1]),
    vec3(0, 9, -5), config = clothConfig)

  var volumeMesh: SoftBodyMesh
  volumeMesh.vertices = @[
    softBodyVertex(vec3(0, 1.35, 0)),
    softBodyVertex(vec3(-1.15, -0.75, -0.85)),
    softBodyVertex(vec3(1.15, -0.75, -0.85)),
    softBodyVertex(vec3(0, -0.75, 1.25))
  ]
  volumeMesh.faces = @[
    softBodyFace(0, 2, 1), softBodyFace(0, 3, 2),
    softBodyFace(0, 1, 3), softBodyFace(1, 2, 3)
  ]
  volumeMesh.volumeConstraints.add(
    softBodyVolumeConstraint(0, 1, 2, 3))
  var volumeConfig = defaultSoftBodyConfig()
  volumeConfig.material = some(physicsMaterial(
    "Naylib volume", materialColor(175, 85, 215)))
  volumeConfig.facesDoubleSided = true
  volumeConfig.edgeCompliance = 0.05
  volumeConfig.numIterations = 10
  let softVolume = world.addSoftBody(
    volumeMesh, vec3(5, 8, -2), config = volumeConfig)
  softVolume.setVertexVelocity(0, vec3(4, -10, 2))

  var rodPoints: seq[Vec3]
  for index in 0 ..< 18:
    let t = float32(index)
    rodPoints.add(vec3(t * 0.4, 0.3 * sin(t * 0.65),
      0.4 * cos(t * 0.45)))
  var rodConfig = defaultSoftBodyConfig()
  rodConfig.material = some(physicsMaterial(
    "Naylib rod", materialColor(35, 185, 190)))
  rodConfig.numIterations = 12
  rodConfig.vertexRadius = 0.07
  let softRod = world.addSoftBody(
    rodSoftBodyMesh(
      rodPoints, rodCompliance = 1.0e-6,
      bendTwistCompliance = 2.0e-4),
    vec3(-4, 9, 5), config = rodConfig)
  var skinConfig = defaultSoftBodyConfig()
  skinConfig.material = some(physicsMaterial(
    "Naylib skin", materialColor(245, 205, 55)))
  skinConfig.facesDoubleSided = true
  skinConfig.updatePosition = false
  skinConfig.edgeCompliance = 0.015
  skinConfig.shearCompliance = 0.015
  skinConfig.numIterations = 10
  let softSkin = world.addSoftBody(
    skinnedRibbonMesh(8, 12, 0.32), vec3(7, 10, 3),
    rotation = quatFromAxisAngle(vec3(1, 0, 0), PI.float32 * 0.5),
    config = skinConfig)
  let softBodies = @[softCloth, softVolume, softRod, softSkin]
  let ragdoll = world.addRagdoll(naylibRagdollConfig(vec3(-8, 7, -1)))
  ragdoll.addImpulse(vec3(3, 0, 1.5))

  let vehicleChassis = world.addDynamicBody(
    boxShape(vec3(0.85, 0.3, 1.8)), vec3(7, 1.4, 7))
  vehicleChassis.setFriction(0.9)
  visuals.add VisualBody(body: vehicleChassis, color: rl.Blue)
  var vehicleConfig = defaultVehicleConfig()
  vehicleConfig.fourWheelDrive = false
  vehicleConfig.frontWheelDrive = false
  vehicleConfig.wheelTrack = 1.45
  vehicleConfig.frontAxleOffset = 1.25
  vehicleConfig.rearAxleOffset = 1.15
  vehicleConfig.rearMaxSteerAngle = PI.float32 / 48
  vehicleConfig.frontBrakeTorque = 1_000
  vehicleConfig.rearBrakeTorque = 1_800
  vehicleConfig.engineMaxTorque = 900
  vehicleConfig.engineMaxRPM = 6_500
  vehicleConfig.gearRatios = @[3.0'f32, 1.9, 1.3, 0.9]
  vehicleConfig.shiftUpRPM = 5_200
  vehicleConfig.differentialRatio = 3.9
  vehicleConfig.differentialLimitedSlipRatio = 1.8
  vehicleConfig.wheelCollisionMode = VehicleWheelCollisionMode.SphereCast
  vehicleConfig.wheelSphereCastRadius = 0.08
  let vehicle = vehicleChassis.newVehicle(vehicleConfig)

  let compound = world.addDynamicBody(
    staticCompoundShape([
      compoundChild(boxShape(vec3(0.7, 0.25, 0.25)).withMaterial(steel)),
      compoundChild(
        sphereShape(0.45).withMaterial(rubber), vec3(-1.1, 0, 0)),
      compoundChild(
        sphereShape(0.45).withMaterial(grass), vec3(1.1, 0, 0)),
      compoundChild(
        cylinderShape(0.7, 0.2).withMaterial(stone),
        vec3(0, 0.8, 0),
        quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5))
    ]),
    vec3(-4, 6, -2))
  compound.setAngularVelocity(vec3(0.6, 1.1, 0.4))
  visuals.add VisualBody(body: compound, color: rl.Violet)

  let mutableBody = world.addDynamicBody(
    mutableCompoundShape([
      compoundChild(boxShape(vec3(0.65, 0.25, 0.25)).withMaterial(steel)),
      compoundChild(
        sphereShape(0.42).withMaterial(rubber), vec3(-1.1, 0, 0)),
      compoundChild(
        sphereShape(0.42).withMaterial(grass), vec3(1.1, 0, 0))
    ]),
    vec3(-7, 8, -5))
  mutableBody.setAngularVelocity(vec3(0.4, 1.2, 0.3))
  visuals.add VisualBody(body: mutableBody, color: rl.Orange)

  for index in 0 ..< 4:
    let decorated = scaledShape(
      rotatedTranslatedShape(
        boxShape(vec3(0.45, 0.8, 0.45)),
        vec3(0.45, 0, 0),
        quatFromAxisAngle(vec3(0, 0, 1), 0.35)),
      vec3(0.7 + float32(index) * 0.25, 1, 1))
    let body = world.addDynamicBody(
      decorated, vec3(-3 + float32(index) * 2, 11, -5))
    visuals.add VisualBody(body: body, color: palette[index])

  for index in 0 ..< 6:
    let shape = if index mod 2 == 0:
        taperedCapsuleShape(0.65, 0.25, 0.5)
      else:
        taperedCylinderShape(0.72, 0.3, 0.62)
    let body = world.addDynamicBody(
      shape, vec3(-6 + float32(index) * 2.4, 12, 5))
    body.setAngularVelocity(vec3(0.35, 0.8, 0.25))
    visuals.add VisualBody(body: body, color: palette[index mod palette.len])

  let triangle = world.addStaticBody(
    triangleShape(
      vec3(-2.5, 0, -2), vec3(-2.5, 0, 2), vec3(2.5, 2, -2),
      convexRadius = 0.025).withMaterial(stone),
    vec3(8, 0.1, -5))
  visuals.add VisualBody(body: triangle, color: rl.Green)

  let emptyAnchor = world.addKinematicBody(
    emptyShape(vec3(0, 0.2, 0)), vec3(-9, 7, 7))
  let anchoredPayload = world.addDynamicBody(
    sphereShape(0.5), vec3(-9, 4.5, 7))
  visuals.add VisualBody(body: emptyAnchor, color: rl.Red)
  visuals.add VisualBody(body: anchoredPayload, color: rl.SkyBlue)
  constraints.add addDistanceConstraint(
    emptyAnchor, anchoredPayload, vec3(-9, 7, 7), vec3(-9, 4.5, 7),
    2.5, 2.5)

  let core = world.addDynamicBody(boxShape(vec3(0.6, 0.6, 0.6)), vec3(0, 8, 0))
  visuals.add VisualBody(body: core, color: rl.SkyBlue)
  for index, offset in [vec3(1.5, 0, 0), vec3(-1.5, 0, 0), vec3(0, 1.5, 0), vec3(0, -1.5, 0)]:
    let part = world.addDynamicBody(cylinderShape(0.6, 0.32), vec3(offset.x, 8 + offset.y, 0))
    visuals.add VisualBody(body: part, color: palette[index])
    constraints.add addFixedConstraint(core, part)
  core.addAngularImpulse(vec3(600, 800, 450))

  let motorAnchor = world.addStaticBody(
    boxShape(vec3(0.3, 0.3, 0.3)), vec3(5, 8, -3))
  let motorBody = world.addDynamicBody(
    boxShape(vec3(0.3, 1.0, 0.3)), vec3(5, 6.5, -3))
  visuals.add VisualBody(body: motorAnchor, color: rl.Gray)
  visuals.add VisualBody(body: motorBody, color: rl.Gold)
  var sixConfig = defaultSixDOFConfig()
  sixConfig.limits[SixDOFAxis.RotationY] = freeAxis()
  let motorJoint = addSixDOFConstraint(
    motorAnchor, motorBody, vec3(5, 7.7, -3), sixConfig)
  var motorSettings = defaultMotorSettings()
  motorSettings.minTorque = -600
  motorSettings.maxTorque = 600
  motorJoint.configureAxisMotor(SixDOFAxis.RotationY, motorSettings)
  motorJoint.setSixDOFMotorTargets(
    vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0),
    quatFromAxisAngle(vec3(0, 1, 0), 0.8))
  motorJoint.setAxisMotorState(SixDOFAxis.RotationY, MotorState.Position)
  constraints.add motorJoint

  for index in 0 ..< 36:
    let layer = if index mod 3 == 0: diagnosticLayer else: movingLayer
    let body = world.addDynamicBody(
      cylinderShape(0.42, 0.36),
      vec3(-7 + float32(index mod 9) * 1.7, 1.2 + float32(index div 9) * 1.0, 2),
      quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5),
      layer = layer)
    body.setFriction(0.85)
    visuals.add VisualBody(
      body: body,
      color: if layer == diagnosticLayer: rl.SkyBlue else: palette[index mod palette.len])
  world.optimizeBroadPhase()

  var camera = rl.Camera3D(
    position: rl.Vector3(x: 18, y: 13, z: 18),
    target: rl.Vector3(x: 0, y: 3, z: 0),
    up: rl.Vector3(y: 1),
    fovy: 45,
    projection: rl.CameraProjection.Perspective)
  var frame = 0
  var contacts = 0
  var softContacts = 0
  var mutablePhase = -1
  while not rl.windowShouldClose():
    let phase = (frame div 60) mod 4
    if phase != mutablePhase:
      let spread = if phase mod 2 == 0: 1.1'f32 else: 1.8'f32
      mutableBody.setMutableChildTransform(1, vec3(-spread, 0, 0))
      mutableBody.replaceMutableChild(
        2,
        compoundChild(
          (if phase < 2:
              sphereShape(0.42)
            else:
              boxShape(vec3(0.45, 0.45, 0.45))).withMaterial(grass),
          vec3(spread, 0, 0)))
      mutablePhase = phase
    if frame < 180:
      vehicle.setInput(0, 0, brake = 0.15)
    else:
      vehicle.setInput(0.12, 0.2)
    let skinExtent = 0.5'f32 * 0.32'f32 * 11.0'f32
    let skinAngle = 0.45'f32 * sin(float32(frame) * dt * 1.7'f32)
    softSkin.skinVertices([
      softBodyJointTransform(
        vec3(0, 0, -skinExtent),
        quatFromAxisAngle(vec3(1, 0, 0), skinAngle)),
      softBodyJointTransform(
        vec3(0, 0, skinExtent),
        quatFromAxisAngle(vec3(1, 0, 0), -skinAngle))])
    if world.step(dt) != {}:
      raise newException(JoltError, "physics capacity exceeded")
    for event in world.drainEvents():
      if event.kind in {PhysicsEventKind.ContactAdded, PhysicsEventKind.ContactPersisted}:
        inc contacts
    softContacts += world.drainSoftBodyContactEvents().len
    if frames == 0:
      rl.updateCamera(camera, rl.CameraMode.Orbital)

    rl.beginDrawing()
    rl.clearBackground(rl.RayWhite)
    rl.beginMode3D(camera)
    rl.drawGrid(28, 1)
    for visual in visuals:
      visual.draw()
    for body in softBodies:
      body.draw()
    ragdoll.draw()
    var groundedWheels = 0
    for wheelIndex in 0 ..< vehicle.wheelCount:
      let wheel = vehicle.wheelState(wheelIndex)
      rl.drawSphere(wheel.position.rv, vehicleConfig.wheelRadius, 8, 12, rl.Black)
      rl.drawSphereWires(
        wheel.position.rv, vehicleConfig.wheelRadius, 6, 10, rl.DarkGray)
      if wheel.hasContact:
        inc groundedWheels
        rl.drawLine3D(
          wheel.contactPosition.rv,
          vec3(
            wheel.contactPosition.x + wheel.contactNormal.x * 0.6'f32,
            wheel.contactPosition.y + wheel.contactNormal.y * 0.6'f32,
            wheel.contactPosition.z + wheel.contactNormal.z * 0.6'f32).rv,
          rl.Green)
    let center = vec3(sin(float32(frame) * dt) * 5, 2, 0)
    rl.drawSphereWires(center.rv, 1.2, 10, 14, rl.Magenta)
    for hit in world.overlapSphere(center, 1.2):
      let material = hit.material(world)
      rl.drawSphere(
        hit.contactPoint.rv, 0.1,
        if material.isSome: material.get.debugColor.raylibColor else: rl.Magenta)
    for hit in world.overlapSphere(
        center, 1.2, layer = some(diagnosticLayer)):
      rl.drawSphereWires(hit.contactPoint.rv, 0.18, 6, 8, rl.SkyBlue)
    let boxCenter = vec3(center.x, 1.1, 2)
    let boxRotation = quatFromAxisAngle(vec3(0, 1, 0), PI.float32 * 0.2)
    pushMatrix()
    translatef(boxCenter.x, boxCenter.y, boxCenter.z)
    rotatef(36, 0, 1, 0)
    rl.drawCubeWires(rl.Vector3(), rl.Vector3(x: 3, y: 0.8, z: 1.2), rl.Lime)
    popMatrix()
    for hit in world.overlapShape(
        boxShape(vec3(1.5, 0.4, 0.6)),
        boxCenter,
        rotation = boxRotation,
        layer = some(diagnosticLayer)):
      rl.drawSphereWires(hit.contactPoint.rv, 0.2, 6, 8, rl.Lime)
    let broadOrigin = vec3(-11, 3, 3.5)
    let broadEnd = vec3(11, 3, 3.5)
    rl.drawLine3D(broadOrigin.rv, broadEnd.rv, rl.Blue)
    for hit in world.broadPhaseCastRay(broadOrigin, vec3(1, 0, 0), 22):
      rl.drawCubeWires(
        vec3(broadOrigin.x + hit.distance, broadOrigin.y, broadOrigin.z).rv,
        rl.Vector3(x: 0.22, y: 0.22, z: 0.22), rl.Blue)
    let broadRotation = quatFromAxisAngle(
      vec3(0, 1, 0), float32(frame) * dt * 0.35)
    let broadHits = world.broadPhaseQueryOrientedBox(
      center, vec3(1.7, 0.8, 0.7), broadRotation)
    let broadAxisAngle = broadRotation.axisAngle
    pushMatrix()
    translatef(center.x, center.y, center.z)
    rotatef(
      broadAxisAngle.degrees,
      broadAxisAngle.axis.x,
      broadAxisAngle.axis.y,
      broadAxisAngle.axis.z)
    rl.drawCubeWires(
      rl.Vector3(), rl.Vector3(x: 3.4, y: 1.6, z: 1.4),
      if broadHits.len > 0: rl.Orange else: rl.Gray)
    popMatrix()
    rl.endMode3D()
    let powertrain = vehicle.powertrainState
    rl.drawText(
      &"jolt-nim + Naylib | {visuals.len + ragdoll.partCount} rigid + {softBodies.len} soft bodies | {ragdoll.constraintCount} ragdoll + {constraints.len} other joints | RWD {groundedWheels}/4 | {contacts} rigid + {softContacts} soft contacts",
      24, 22, 24, rl.DarkGray)
    rl.drawText(
      &"engine {powertrain.engineRPM:.0f} rpm | gear {powertrain.currentGear} | clutch {powertrain.clutchFriction:.2f} | wheel speed {powertrain.wheelSpeedAtClutch:.1f} rad/s",
      24, 52, 20, rl.DarkGray)
    rl.endDrawing()

    inc frame
    if frames > 0 and frame >= frames:
      if screenshot.len > 0:
        let image = rl.loadImageFromScreen()
        if not image.exportImage(screenshot):
          raise newException(IOError, "could not write screenshot: " & screenshot)
        echo "captured ", screenshot
      break

main()
