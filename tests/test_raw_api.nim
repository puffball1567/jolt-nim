import jolt/raw as api

proc jobCallback(userData: pointer) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc threadInitExitCallback(userData: pointer; threadIndex: cint) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc simCollideCallback(
  userData: pointer;
  body1, body2: ptr api.Body;
  transform1, transform2: ptr api.Mat44;
  settings: ptr api.CollideShapeSettings;
  collector: ptr api.CollideShapeCollector;
  shapeFilter: ptr api.ShapeFilter,
) {.cdecl.} =
  discard

proc shaderLoaderCallback(
  userData: pointer;
  name: cstring;
  data: ptr api.ComputeByteArray;
  error: ptr api.String,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  doAssert $name == "raw-shader"
  data[].push_back(11'u8)
  data[].push_back(22'u8)
  data[].push_back(33'u8)
  api.setJoltString(error, "loaded")
  true

proc hairRenderCallback(
  userData: pointer;
  buffer: ptr api.ComputeBuffer;
  positions: ptr api.Float3;
  count: api.uint,
) {.cdecl.} =
  discard

proc vehicleCombineCallback(
  userData: pointer;
  wheelIndex: api.uint;
  longitudinalFriction, lateralFriction: ptr cfloat;
  body: ptr api.Body;
  subShapeID: ptr api.SubShapeID,
) {.cdecl.} =
  discard

proc vehicleStepCallback(
  userData: pointer;
  vehicle: ptr api.VehicleConstraint;
  context: ptr api.PhysicsStepListenerContext,
) {.cdecl.} =
  discard

proc tireMaxImpulseCallback(
  userData: pointer;
  wheelIndex: api.uint;
  longitudinalImpulse, lateralImpulse: ptr cfloat;
  suspensionImpulse, longitudinalFriction, lateralFriction: cfloat;
  longitudinalSlip, lateralSlip, deltaTime: cfloat,
) {.cdecl.} =
  discard

proc bodyActivationCallback(
  userData: pointer;
  bodyID: ptr api.BodyID;
  bodyUserData: uint64,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc physicsStepCallback(
  userData: pointer;
  context: ptr api.PhysicsStepListenerContext,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc contactValidateCallback(
  userData: pointer;
  body1, body2: ptr api.Body;
  baseOffset: ptr api.RVec3;
  collisionResult: ptr api.CollideShapeResult,
): api.ValidateResult {.cdecl.} =
  api.ValidateResult.AcceptAllContactsForThisBodyPair

proc contactCallback(
  userData: pointer;
  body1, body2: ptr api.Body;
  manifold: ptr api.ContactManifold;
  settings: ptr api.ContactSettings,
) {.cdecl.} =
  discard

proc contactRemovedCallback(
  userData: pointer;
  pair: ptr api.SubShapeIDPair,
) {.cdecl.} =
  discard

proc softBodyValidateCallback(
  userData: pointer;
  softBody, otherBody: ptr api.Body;
  settings: ptr api.SoftBodyContactSettings,
): api.SoftBodyValidateResult {.cdecl.} =
  api.SoftBodyValidateResult.AcceptContact

proc softBodyContactAddedCallback(
  userData: pointer;
  softBody: ptr api.Body;
  manifold: ptr api.SoftBodyManifold,
) {.cdecl.} =
  discard

proc simShapeCallback(
  userData: pointer;
  body1: ptr api.Body;
  shape1: ptr api.Shape;
  subShapeID1: ptr api.SubShapeID;
  body2: ptr api.Body;
  shape2: ptr api.Shape;
  subShapeID2: ptr api.SubShapeID,
): bool {.cdecl.} =
  true

proc contactCombineCallback(
  userData: pointer;
  body1: ptr api.Body;
  subShapeID1: ptr api.SubShapeID;
  body2: ptr api.Body;
  subShapeID2: ptr api.SubShapeID,
): cfloat {.cdecl.} =
  0.25

proc rayCastResultCallback(
  userData: pointer;
  result: ptr api.RayCastResult,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc shapeCastResultCallback(
  userData: pointer;
  result: ptr api.ShapeCastResult,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc collidePointResultCallback(
  userData: pointer;
  result: ptr api.CollidePointResult,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc collideShapeResultCallback(
  userData: pointer;
  result: ptr api.CollideShapeResult,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc transformedShapeCallback(
  userData: pointer;
  result: ptr api.TransformedShape,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc broadPhaseCastResultCallback(
  userData: pointer;
  result: ptr api.BroadPhaseCastResult,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc bodyIDCallback(
  userData: pointer;
  result: ptr api.BodyID,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc bodyPairCallback(
  userData: pointer;
  result: ptr api.BodyPair,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc broadPhaseLayerFilterCallback(
  userData: pointer;
  layer: ptr api.BroadPhaseLayer,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc objectLayerFilterCallback(
  userData: pointer;
  layer: api.ObjectLayer,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc objectLayerPairFilterCallback(
  userData: pointer;
  layer1, layer2: api.ObjectLayer,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  layer1 != layer2

proc objectVsBroadPhaseLayerFilterCallback(
  userData: pointer;
  layer1: api.ObjectLayer;
  layer2: ptr api.BroadPhaseLayer,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc bodyIDFilterCallback(
  userData: pointer;
  bodyID: ptr api.BodyID,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  true

proc bodyFilterLockedCallback(
  userData: pointer;
  body: ptr api.Body,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc shapeFilterSingleCallback(
  userData: pointer;
  shape: ptr api.Shape;
  subShapeID: ptr api.SubShapeID,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc shapeFilterPairCallback(
  userData: pointer;
  shape1: ptr api.Shape;
  subShapeID1: ptr api.SubShapeID;
  shape2: ptr api.Shape;
  subShapeID2: ptr api.SubShapeID,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc stateRecorderBodyCallback(
  userData: pointer;
  body: ptr api.Body,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc stateRecorderConstraintCallback(
  userData: pointer;
  constraint: ptr api.Constraint,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc stateRecorderContactCallback(
  userData: pointer;
  body1, body2: ptr api.BodyID,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  false

proc broadPhaseLayerCountCallback(userData: pointer): api.uint {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  2

proc broadPhaseLayerMapCallback(
  userData: pointer;
  layer: api.ObjectLayer,
): uint8 {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  uint8(layer)

proc characterContactValidateCallback(
  userData: pointer;
  character: ptr api.CharacterVirtual;
  contact: ptr api.CharacterContact,
): bool {.cdecl.} =
  inc cast[ptr uint32](userData)[]
  true

proc characterContactCallback(
  userData: pointer;
  character: ptr api.CharacterVirtual;
  contact: ptr api.CharacterContact;
  settings: ptr api.CharacterContactSettings,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc characterCollideCallback(
  userData: pointer;
  character: ptr api.CharacterVirtual;
  centerOfMassTransform: ptr api.RMat44;
  settings: ptr api.CollideShapeSettings;
  baseOffset: ptr api.RVec3;
  collector: ptr api.CollideShapeCollector,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc characterCastCallback(
  userData: pointer;
  character: ptr api.CharacterVirtual;
  centerOfMassTransform: ptr api.RMat44;
  direction: ptr api.Vec3;
  settings: ptr api.ShapeCastSettings;
  baseOffset: ptr api.RVec3;
  collector: ptr api.CastShapeCollector,
) {.cdecl.} =
  inc cast[ptr uint32](userData)[]

proc vehicleCollideRawCallback(
  userData: pointer;
  physicsSystem: ptr api.PhysicsSystem;
  vehicleConstraint: ptr api.VehicleConstraint;
  wheelIndex: api.uint;
  origin: ptr api.RVec3;
  direction: ptr api.Vec3;
  vehicleBodyID: ptr api.BodyID;
  body: ptr ptr api.Body;
  subShapeID: ptr api.SubShapeID;
  contactPosition: ptr api.RVec3;
  contactNormal: ptr api.Vec3;
  suspensionLength: ptr cfloat,
): bool {.cdecl.} =
  false

static:
  doAssert sizeof(api.EActivation) == 4
  doAssert sizeof(api.EMotionType) == 1
  doAssert sizeof(api.EMotionQuality) == 1
  doAssert sizeof(api.EBodyType) == 1
  doAssert sizeof(api.EBackFaceMode) == 1
  doAssert sizeof(api.EShapeType) == 1
  doAssert sizeof(api.EShapeSubType) == 1
  doAssert sizeof(api.EAllowedDOFs) == 1
  doAssert sizeof(api.EAccess) == 1
  doAssert sizeof(api.ECanSleep) == 4
  doAssert sizeof(api.EPhysicsLockTypes) == 4
  doAssert sizeof(api.ValidateResult) == 4
  doAssert sizeof(api.EPhysicsUpdateError) == 4
  doAssert sizeof(api.ESupportMode) == 4
  doAssert sizeof(api.EStateRecorderState) == 1
  doAssert sizeof(api.EConstraintType) == 4
  doAssert sizeof(api.EConstraintSubType) == 4
  doAssert sizeof(api.EConstraintSpace) == 4
  doAssert sizeof(api.LargeIslandSplitter_EStatus) == 4
  doAssert sizeof(api.EIterationStatus) == 8
  doAssert sizeof(api.EOverrideMassProperties) == 1
  doAssert sizeof(api.EBendType) == 4
  doAssert sizeof(api.ELRAType) == 4
  doAssert sizeof(api.EActiveEdgeMode) == 1
  doAssert sizeof(api.ECollectFacesMode) == 1
  doAssert sizeof(api.EBuildQuality) == 4
  doAssert sizeof(api.ERoundingMode) == 4
  doAssert sizeof(api.ESpringMode) == 1
  doAssert sizeof(api.EMotorState) == 4
  doAssert sizeof(api.EPathRotationConstraintType) == 4
  doAssert sizeof(api.ESwingType) == 1
  doAssert sizeof(api.EAxis) == 4
  doAssert sizeof(api.EGroundState) == 4
  doAssert sizeof(api.EState) == 4
  doAssert sizeof(api.SoftBodyMotionProperties_EStatus) == 4
  doAssert sizeof(api.EType) == 4
  doAssert sizeof(api.EMode) == 4
  doAssert sizeof(api.EBarrier) == 4
  doAssert sizeof(api.ETransmissionMode) == 1
  doAssert sizeof(api.ETrackSide) == 4
  doAssert sizeof(api.SoftBodyValidateResult) == 4

proc testLowLevelApi() =
  doAssert api.JoltApi.VerifyJoltVersionID()
  doAssert api.joltFactoryInstance[].Find("BoxShapeSettings") != nil
  doAssert not api.joltFactoryInstance[].GetAllClasses().empty()

  var semaphore = api.constructSemaphore()
  doAssert semaphore.GetValue() == 0
  semaphore.Release()
  doAssert semaphore.GetValue() == 1
  semaphore.Acquire()
  doAssert semaphore.GetValue() == 0

  var compoundVector = api.constructVec3(1.0, 2.0, 3.0)
  discard compoundVector.mulAssign(2.0)
  discard compoundVector.addAssign(api.constructVec3(1.0, 1.0, 1.0))
  discard compoundVector.subAssign(api.constructVec3(1.0, 1.0, 1.0))
  discard compoundVector.divAssign(2.0)
  doAssert compoundVector == api.constructVec3(1.0, 2.0, 3.0)

  template assertNil(value: untyped) =
    doAssert value == nil

  assertNil api.asShape(cast[ptr api.BoxShape](nil))
  assertNil api.asShape(cast[ptr api.CapsuleShape](nil))
  assertNil api.asShape(cast[ptr api.ConvexHullShape](nil))
  assertNil api.asShape(cast[ptr api.CylinderShape](nil))
  assertNil api.asShape(cast[ptr api.EmptyShape](nil))
  assertNil api.asShape(cast[ptr api.HeightFieldShape](nil))
  assertNil api.asShape(cast[ptr api.MeshShape](nil))
  assertNil api.asShape(cast[ptr api.MutableCompoundShape](nil))
  assertNil api.asShape(cast[ptr api.OffsetCenterOfMassShape](nil))
  assertNil api.asShape(cast[ptr api.PlaneShape](nil))
  assertNil api.asShape(cast[ptr api.RotatedTranslatedShape](nil))
  assertNil api.asShape(cast[ptr api.ScaledShape](nil))
  assertNil api.asShape(cast[ptr api.SphereShape](nil))
  assertNil api.asShape(cast[ptr api.StaticCompoundShape](nil))
  assertNil api.asShape(cast[ptr api.TaperedCapsuleShape](nil))
  assertNil api.asShape(cast[ptr api.TaperedCylinderShape](nil))
  assertNil api.asShape(cast[ptr api.TriangleShape](nil))
  assertNil api.asShape(cast[ptr api.SoftBodyShape](nil))

  assertNil api.asShapeSettings(cast[ptr api.BoxShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.CapsuleShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.ConvexHullShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.CylinderShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.EmptyShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.HeightFieldShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.MeshShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.MutableCompoundShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.OffsetCenterOfMassShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.PlaneShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.RotatedTranslatedShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.ScaledShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.SphereShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.StaticCompoundShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.TaperedCapsuleShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.TaperedCylinderShapeSettings](nil))
  assertNil api.asShapeSettings(cast[ptr api.TriangleShapeSettings](nil))

  assertNil api.asConvexShape(cast[ptr api.BoxShape](nil))
  assertNil api.asConvexShape(cast[ptr api.CapsuleShape](nil))
  assertNil api.asConvexShape(cast[ptr api.ConvexHullShape](nil))
  assertNil api.asConvexShape(cast[ptr api.CylinderShape](nil))
  assertNil api.asConvexShape(cast[ptr api.SphereShape](nil))
  assertNil api.asConvexShape(cast[ptr api.TaperedCapsuleShape](nil))
  assertNil api.asConvexShape(cast[ptr api.TaperedCylinderShape](nil))
  assertNil api.asConvexShape(cast[ptr api.TriangleShape](nil))
  assertNil api.asCompoundShape(cast[ptr api.MutableCompoundShape](nil))
  assertNil api.asCompoundShape(cast[ptr api.StaticCompoundShape](nil))
  assertNil api.asDecoratedShape(cast[ptr api.OffsetCenterOfMassShape](nil))
  assertNil api.asDecoratedShape(cast[ptr api.RotatedTranslatedShape](nil))
  assertNil api.asDecoratedShape(cast[ptr api.ScaledShape](nil))

  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.ConeConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.DistanceConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.FixedConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.GearConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.HingeConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.PathConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.PointConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.PulleyConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.RackAndPinionConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.SixDOFConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.SliderConstraintSettings](nil))
  assertNil api.asTwoBodyConstraintSettings(cast[ptr api.SwingTwistConstraintSettings](nil))

  assertNil api.asVehicleCollisionTester(cast[ptr api.VehicleCollisionTesterRay](nil))
  assertNil api.asVehicleCollisionTester(cast[ptr api.VehicleCollisionTesterCastSphere](nil))
  assertNil api.asVehicleCollisionTester(cast[ptr api.VehicleCollisionTesterCastCylinder](nil))
  assertNil api.asBodyLockInterface(cast[ptr api.BodyLockInterfaceLocking](nil))
  assertNil api.asBodyLockInterface(cast[ptr api.BodyLockInterfaceNoLock](nil))
  assertNil api.asCharacterVsCharacterCollision(cast[ptr api.CharacterVsCharacterCollisionSimple](nil))
  assertNil api.asGroupFilter(cast[ptr api.GroupFilterTable](nil))

  let vector = api.constructVec3(1.0, 2.0, 3.0)
  doAssert vector.GetX() == 1.0
  doAssert vector.GetY() == 2.0
  doAssert vector.GetZ() == 3.0

  let bodyId = api.constructBodyID(42'u32, 3'u8)
  doAssert not bodyId.IsInvalid()
  doAssert bodyId.GetIndex() == 42'u32
  doAssert bodyId.GetSequenceNumber() == 3'u8

  let sphere = api.constructSphereShape(1.25, nil)
  doAssert sphere.GetRadius() == 1.25

  let box = api.constructBoxShape(api.constructVec3(0.5, 1.0, 1.5), 0.05, nil)
  doAssert box.GetHalfExtent().GetY() == 1.0
  doAssert api.asShape(addr box) != nil

  template exerciseOwnedShape(shapeExpression: untyped) =
    block:
      let ownedShape = shapeExpression
      doAssert ownedShape != nil
      let ownedSettings = api.constructBodyCreationSettings(
        api.asShape(ownedShape),
        api.Vec3.sZero(),
        api.Quat.sIdentity(),
        api.EMotionType.Static,
        api.ObjectLayer(0),
      )
      doAssert ownedSettings.GetShape() != nil

  exerciseOwnedShape api.newCapsuleShape(0.5, 0.25)
  exerciseOwnedShape api.newCylinderShape(0.5, 0.25, 0.02)
  exerciseOwnedShape api.newEmptyShape(api.Vec3.sZero())
  exerciseOwnedShape api.newPlaneShape(
    api.constructPlane(api.Vec3.sAxisY(), 0.0),
    nil,
    10.0,
  )
  exerciseOwnedShape api.newTriangleShape(
    api.constructVec3(-1.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
  )
  exerciseOwnedShape api.newOffsetCenterOfMassShape(
    api.asShape(api.newSphereShape(0.5)),
    api.constructVec3(0.0, 0.25, 0.0),
  )
  exerciseOwnedShape api.newRotatedTranslatedShape(
    api.constructVec3(0.0, 0.25, 0.0),
    api.Quat.sIdentity(),
    api.asShape(api.newSphereShape(0.5)),
  )
  exerciseOwnedShape api.newScaledShape(
    api.asShape(api.newSphereShape(0.5)),
    api.constructVec3(1.0, 2.0, 1.0),
  )

  let ray = api.constructRayCast(
    api.constructVec3(1.0, 2.0, 3.0),
    api.constructVec3(0.0, -4.0, 0.0),
  )
  doAssert ray.GetPointOnRay(0.5).GetY() == 0.0
  doAssert ray.Translated(api.constructVec3(2.0, 0.0, 0.0))
    .GetPointOnRay(0.0).GetX() == 3.0

  let identity = api.Mat44.sIdentity()
  let shapeCast = api.constructShapeCast(
    api.asShape(addr sphere),
    api.constructVec3(1.0, 1.0, 1.0),
    identity,
    api.constructVec3(0.0, -2.0, 0.0),
  )
  doAssert shapeCast.GetPointOnRay(0.5).GetY() == -1.0

  let realRay = api.constructRRayCast(ray)
  doAssert realRay.toRayCast().GetPointOnRay(0.5).GetY() == 0.0
  let realShapeCast = api.constructRShapeCast(shapeCast)
  doAssert realShapeCast.toShapeCast().GetPointOnRay(0.5).GetY() == -1.0

  var broadPhaseLayer = api.constructBroadPhaseLayer(1'u8)
  let broadPhaseLayerCopy = api.constructBroadPhaseLayer(2'u8)
  doAssert broadPhaseLayer.assign(broadPhaseLayerCopy) == addr broadPhaseLayer
  doAssert broadPhaseLayer.toBroadPhaseLayerType() == 2'u8

  var allowedDofs = api.bitOr(
    api.EAllowedDOFs.TranslationX,
    api.EAllowedDOFs.TranslationY,
  )
  doAssert allowedDofs == api.bitAnd(
    api.EAllowedDOFs.Plane2D,
    api.bitNot(api.EAllowedDOFs.RotationZ),
  )
  doAssert api.bitXor(allowedDofs, api.EAllowedDOFs.TranslationY) ==
    api.EAllowedDOFs.TranslationX
  doAssert api.bitOrAssign(allowedDofs, api.EAllowedDOFs.RotationZ) ==
    addr allowedDofs
  doAssert allowedDofs == api.EAllowedDOFs.Plane2D
  doAssert api.bitAndAssign(allowedDofs, api.EAllowedDOFs.TranslationX) ==
    addr allowedDofs
  doAssert api.bitXorAssign(allowedDofs, api.EAllowedDOFs.TranslationY) ==
    addr allowedDofs

  var recorderState = api.bitOr(
    api.EStateRecorderState.Global,
    api.EStateRecorderState.Bodies,
  )
  doAssert api.bitAnd(recorderState, api.EStateRecorderState.Bodies) ==
    api.EStateRecorderState.Bodies
  doAssert api.bitXor(recorderState, api.EStateRecorderState.Global) ==
    api.EStateRecorderState.Bodies
  discard api.bitNot(recorderState)
  doAssert api.bitOrAssign(recorderState, api.EStateRecorderState.Contacts) ==
    addr recorderState
  doAssert api.bitAndAssign(recorderState, api.EStateRecorderState.All) ==
    addr recorderState
  doAssert api.bitXorAssign(recorderState, api.EStateRecorderState.Global) ==
    addr recorderState

  var combinedUpdateError = api.bitOr(
    api.EPhysicsUpdateError.ManifoldCacheFull,
    api.EPhysicsUpdateError.BodyPairCacheFull,
  )
  doAssert api.bitAnd(
    combinedUpdateError,
    api.EPhysicsUpdateError.BodyPairCacheFull,
  ) == api.EPhysicsUpdateError.BodyPairCacheFull
  discard api.bitOrAssign(
    combinedUpdateError,
    api.EPhysicsUpdateError.ContactConstraintsFull,
  )

  var softVertices = [
    api.constructSoftBodyVertex(),
    api.constructSoftBodyVertex(),
  ]
  softVertices[0].mPosition = api.constructVec3(1.0, 2.0, 3.0)
  softVertices[1].mPosition = api.constructVec3(4.0, 5.0, 6.0)
  var softVertexIterator = api.constructCollideSoftBodyVertexIterator(
    addr softVertices[0],
  )
  var softVertexIteratorCopy = api.constructCollideSoftBodyVertexIterator()
  doAssert softVertexIteratorCopy.assign(softVertexIterator) ==
    addr softVertexIteratorCopy
  doAssert softVertexIteratorCopy.GetPosition().GetX() == 1.0
  doAssert softVertexIteratorCopy.advance() == addr softVertexIteratorCopy
  doAssert softVertexIteratorCopy.GetPosition().GetX() == 4.0

  var springSettings = api.constructSpringSettings()
  let springSettingsCopy = api.constructSpringSettings()
  doAssert springSettings.assign(springSettingsCopy) == addr springSettings
  var motorSettings = api.constructMotorSettings()
  let motorSettingsCopy = api.constructMotorSettings()
  doAssert motorSettings.assign(motorSettingsCopy) == addr motorSettings

  var baseCharacterSettings = api.constructCharacterBaseSettings()
  let baseCharacterSettingsCopy = api.constructCharacterBaseSettings()
  doAssert baseCharacterSettings.assign(baseCharacterSettingsCopy) ==
    addr baseCharacterSettings
  var bodyCharacterSettingsCopyTarget = api.constructCharacterSettings()
  let bodyCharacterSettingsCopySource = api.constructCharacterSettings()
  doAssert bodyCharacterSettingsCopyTarget.assign(
    bodyCharacterSettingsCopySource,
  ) == addr bodyCharacterSettingsCopyTarget
  var virtualSettingsCopyTarget = api.constructCharacterVirtualSettings()
  let virtualSettingsCopySource = api.constructCharacterVirtualSettings()
  doAssert virtualSettingsCopyTarget.assign(virtualSettingsCopySource) ==
    addr virtualSettingsCopyTarget
  var contactKeyCopyTarget = api.constructCharacterContactKey()
  let contactKeyCopySource = api.constructCharacterContactKey()
  doAssert contactKeyCopyTarget.assign(contactKeyCopySource) ==
    addr contactKeyCopyTarget
  var subShapePairCopyTarget = api.constructSubShapeIDPair()
  let subShapePairCopySource = api.constructSubShapeIDPair()
  doAssert subShapePairCopyTarget.assign(subShapePairCopySource) ==
    addr subShapePairCopyTarget

  let dummySolverSteps = api.constructDummyCalculateSolverSteps()
  dummySolverSteps.apply(cast[ptr api.Body](nil))
  var physicsSettingsForSolver = api.constructPhysicsSettings()
  var solverSteps = api.constructCalculateSolverSteps(physicsSettingsForSolver)
  var motionPropertiesForSolver = api.constructMotionProperties()
  solverSteps.apply(addr motionPropertiesForSolver)
  solverSteps.Finalize()
  doAssert solverSteps.GetNumVelocitySteps() ==
    physicsSettingsForSolver.mNumVelocitySteps
  doAssert solverSteps.GetNumPositionSteps() ==
    physicsSettingsForSolver.mNumPositionSteps

  var hingeSettings = api.constructHingeConstraintSettings()
  hingeSettings.mMaxFrictionTorque = 4.0
  doAssert hingeSettings.mMaxFrictionTorque == 4.0
  doAssert api.asTwoBodyConstraintSettings(addr hingeSettings) != nil

  var characterSettings = api.constructCharacterVirtualSettings()
  characterSettings.mMass = 75.0
  doAssert characterSettings.mMass == 75.0

  block:
    var bodyCharacterSettings = api.constructCharacterSettings()
    var virtualCharacterSettings = api.constructCharacterVirtualSettings()
    let bodyCharacterShape = api.newCapsuleShape(0.75, 0.3)
    let virtualCharacterShape = api.newCapsuleShape(0.8, 0.35)
    let innerBodyShape = api.newSphereShape(0.25)
    let bodyCharacterBase = api.asCharacterBaseSettings(
      addr bodyCharacterSettings,
    )
    let virtualCharacterBase = api.asCharacterBaseSettings(
      addr virtualCharacterSettings,
    )
    api.setCharacterSettingsShape(
      bodyCharacterBase,
      api.asShape(bodyCharacterShape),
    )
    api.setCharacterSettingsShape(
      virtualCharacterBase,
      api.asShape(virtualCharacterShape),
    )
    api.setCharacterVirtualInnerBodyShape(
      addr virtualCharacterSettings,
      api.asShape(innerBodyShape),
    )
    doAssert api.getCharacterSettingsShape(bodyCharacterBase) != nil
    doAssert api.getCharacterSettingsShape(virtualCharacterBase) != nil
    doAssert api.getCharacterVirtualInnerBodyShape(
      addr virtualCharacterSettings,
    ) != nil

  var vehicleSettings = api.constructVehicleConstraintSettings()
  vehicleSettings.mMaxPitchRollAngle = 0.75
  doAssert vehicleSettings.mMaxPitchRollAngle == 0.75

  block:
    var pathSettings = api.constructPathConstraintSettings()
    let path = api.newPathConstraintPathHermite()
    path[].AddPoint(
      api.constructVec3(0.0, 0.0, 0.0),
      api.constructVec3(1.0, 0.0, 0.0),
      api.constructVec3(0.0, 1.0, 0.0),
    )
    path[].AddPoint(
      api.constructVec3(2.0, 0.0, 0.0),
      api.constructVec3(1.0, 0.0, 0.0),
      api.constructVec3(0.0, 1.0, 0.0),
    )
    let pathBase = api.asPathConstraintPath(path)
    doAssert pathBase != nil
    api.setPathConstraintSettingsPath(addr pathSettings, pathBase)
    doAssert api.getPathConstraintSettingsPath(addr pathSettings) == pathBase
    doAssert path[].GetPathMaxFraction() == 1.0

  block:
    var ownedVehicleSettings = api.constructVehicleConstraintSettings()
    let wheelWV = api.newWheelSettingsWV()
    let wheelTV = api.newWheelSettingsTV()
    api.asWheelSettings(wheelWV)[].mRadius = 0.41
    api.asWheelSettings(wheelTV)[].mRadius = 0.52
    api.addVehicleWheelSettings(
      addr ownedVehicleSettings,
      api.asWheelSettings(wheelWV),
    )
    api.addVehicleWheelSettings(
      addr ownedVehicleSettings,
      api.asWheelSettings(wheelTV),
    )
    doAssert api.getVehicleWheelSettingsCount(addr ownedVehicleSettings) == 2
    doAssert abs(api.getVehicleWheelSettings(addr ownedVehicleSettings, 0)[].mRadius - 0.41) < 0.001
    doAssert abs(api.getVehicleWheelSettings(addr ownedVehicleSettings, 1)[].mRadius - 0.52) < 0.001

    let wheeled = api.newWheeledVehicleControllerSettings()
    let motorcycle = api.newMotorcycleControllerSettings()
    let tracked = api.newTrackedVehicleControllerSettings()
    doAssert api.asVehicleControllerSettings(wheeled) != nil
    doAssert api.asVehicleControllerSettings(motorcycle) != nil
    doAssert api.asVehicleControllerSettings(tracked) != nil
    doAssert api.asWheeledVehicleControllerSettings(motorcycle) != nil

    var differential = api.constructVehicleDifferentialSettings()
    differential.mLeftWheel = 0
    differential.mRightWheel = 1
    api.addVehicleDifferentialSettings(wheeled, differential)
    doAssert api.getVehicleDifferentialSettingsCount(wheeled) == 1
    doAssert api.getVehicleDifferentialSettings(wheeled, 0)[].mRightWheel == 1

    let leftTrack = api.getTrackedVehicleTrackSettings(tracked, 0)
    let rightTrack = api.getTrackedVehicleTrackSettings(tracked, 1)
    leftTrack[].mDrivenWheel = 0
    rightTrack[].mDrivenWheel = 2
    api.addVehicleTrackWheel(leftTrack, 0)
    api.addVehicleTrackWheel(leftTrack, 1)
    api.addVehicleTrackWheel(rightTrack, 2)
    api.addVehicleTrackWheel(rightTrack, 3)
    doAssert api.getVehicleTrackWheelCount(leftTrack) == 2
    doAssert api.getVehicleTrackWheel(leftTrack, 1) == 1
    doAssert api.getVehicleTrackWheelCount(rightTrack) == 2
    doAssert api.getVehicleTrackWheel(rightTrack, 0) == 2
    api.setVehicleControllerSettings(
      addr ownedVehicleSettings,
      api.asVehicleControllerSettings(wheeled),
    )
    doAssert api.getVehicleControllerSettings(addr ownedVehicleSettings) != nil
    api.setVehicleControllerSettings(
      addr ownedVehicleSettings,
      api.asVehicleControllerSettings(motorcycle),
    )
    api.setVehicleControllerSettings(
      addr ownedVehicleSettings,
      api.asVehicleControllerSettings(tracked),
    )
    api.clearVehicleDifferentialSettings(wheeled)
    doAssert api.getVehicleDifferentialSettingsCount(wheeled) == 0
    api.clearVehicleTrackWheels(leftTrack)
    api.clearVehicleTrackWheels(rightTrack)
    api.clearVehicleWheelSettings(addr ownedVehicleSettings)
    doAssert api.getVehicleWheelSettingsCount(addr ownedVehicleSettings) == 0

  block:
    var ragdollSettings = api.constructRagdollSettings()
    let skeleton = api.newSkeleton()
    api.setRagdollSkeleton(addr ragdollSettings, skeleton)
    doAssert ragdollSettings.GetSkeleton() == skeleton

    var part = api.constructRagdollSettings_Part()
    let partBodySettings = api.asBodyCreationSettings(addr part)
    partBodySettings[].SetShape(
      api.asShape(api.newSphereShape(0.35)),
    )
    doAssert partBodySettings[].GetShape() != nil

    template exerciseOwnedParentConstraint(newSettings: untyped) =
      block:
        let settings = newSettings
        api.setRagdollPartToParent(
          addr part,
          api.asTwoBodyConstraintSettings(settings),
        )
        doAssert api.getRagdollPartToParent(addr part) != nil

    exerciseOwnedParentConstraint api.newConeConstraintSettings()
    exerciseOwnedParentConstraint api.newDistanceConstraintSettings()
    exerciseOwnedParentConstraint api.newFixedConstraintSettings()
    exerciseOwnedParentConstraint api.newGearConstraintSettings()
    exerciseOwnedParentConstraint api.newHingeConstraintSettings()
    exerciseOwnedParentConstraint api.newPathConstraintSettings()
    exerciseOwnedParentConstraint api.newPointConstraintSettings()
    exerciseOwnedParentConstraint api.newPulleyConstraintSettings()
    exerciseOwnedParentConstraint api.newRackAndPinionConstraintSettings()
    exerciseOwnedParentConstraint api.newSixDOFConstraintSettings()
    exerciseOwnedParentConstraint api.newSliderConstraintSettings()
    exerciseOwnedParentConstraint api.newSwingTwistConstraintSettings()

    api.addRagdollPart(addr ragdollSettings, part)
    doAssert api.getRagdollPartCount(addr ragdollSettings) == 1
    doAssert ragdollSettings.mParts.size() == 1
    doAssert api.getRagdollPart(addr ragdollSettings, 0) != nil
    let additionalSettings = api.newPointConstraintSettings()
    api.addRagdollAdditionalConstraint(
      addr ragdollSettings,
      2,
      5,
      api.asTwoBodyConstraintSettings(additionalSettings),
    )
    doAssert api.getRagdollAdditionalConstraintCount(addr ragdollSettings) == 1
    doAssert ragdollSettings.mAdditionalConstraints.size() == 1
    let additional = api.getRagdollAdditionalConstraint(
      addr ragdollSettings,
      0,
    )
    doAssert api.getRagdollAdditionalConstraintBodyIndex(additional, 0) == 2
    doAssert api.getRagdollAdditionalConstraintBodyIndex(additional, 1) == 5
    doAssert api.getRagdollAdditionalConstraintSettings(additional) != nil
    api.clearRagdollAdditionalConstraints(addr ragdollSettings)
    api.clearRagdollParts(addr ragdollSettings)
    doAssert api.getRagdollAdditionalConstraintCount(addr ragdollSettings) == 0
    doAssert api.getRagdollPartCount(addr ragdollSettings) == 0

  block:
    var scene = api.constructPhysicsScene()
    let sceneShape = api.newSphereShape(0.4)
    let body = api.constructBodyCreationSettings(
      api.asShape(sceneShape),
      api.constructVec3(0.0, 1.0, 0.0),
      api.Quat.sIdentity(),
      api.EMotionType.Dynamic,
      api.ObjectLayer(1),
    )
    scene.AddBody(body)
    doAssert scene.GetNumBodies() == 1
    doAssert scene.GetBodies()[].size() == 1
    doAssert api.getPhysicsSceneBody(addr scene, 0)[].GetShape() != nil

    let sceneConstraint = api.newPointConstraintSettings()
    scene.AddConstraint(
      api.asTwoBodyConstraintSettings(sceneConstraint),
      api.physicsSceneFixedToWorld(),
      0,
    )
    doAssert scene.GetNumConstraints() == 1
    doAssert scene.GetConstraints()[].size() == 1
    let connected = api.getPhysicsSceneConstraint(addr scene, 0)
    doAssert connected[].mBody1 == api.physicsSceneFixedToWorld()
    doAssert connected[].mBody2 == 0
    doAssert api.getPhysicsSceneConstraintSettings(connected) != nil

    let sharedSettingsRef = api.SoftBodySharedSettings.sCreateCube(2, 0.5)
    let sharedSettings = sharedSettingsRef.GetPtr()
    doAssert sharedSettings != nil
    let softBody = api.constructSoftBodyCreationSettings(
      sharedSettings,
      api.constructVec3(0.0, 2.0, 0.0),
      api.Quat.sIdentity(),
      api.ObjectLayer(1),
    )
    scene.AddSoftBody(softBody)
    doAssert scene.GetNumSoftBodies() == 1
    doAssert scene.GetSoftBodies()[].size() == 1
    doAssert api.getPhysicsSceneSoftBody(addr scene, 0) != nil

  block:
    var animation = api.constructSkeletalAnimation()
    let joint = api.addSkeletalAnimationJoint(addr animation, "root")
    doAssert joint != nil
    doAssert api.getSkeletalAnimationJointCount(addr animation) == 1
    doAssert animation.GetAnimatedJoints()[].size() == 1
    doAssert $api.getSkeletalAnimationJointName(joint) == "root"
    discard api.addSkeletalAnimationKeyframe(
      joint,
      0.0,
      api.Quat.sIdentity(),
      api.Vec3.sZero(),
    )
    discard api.addSkeletalAnimationKeyframe(
      joint,
      1.5,
      api.Quat.sRotation(api.Vec3.sAxisY(), 0.5),
      api.constructVec3(2.0, 0.0, 0.0),
    )
    doAssert api.getSkeletalAnimationKeyframeCount(joint) == 2
    doAssert joint[].mKeyframes.size() == 2
    let lastKeyframe = api.getSkeletalAnimationKeyframe(joint, 1)
    doAssert lastKeyframe[].mTime == 1.5
    let lastState = api.asSkeletalAnimationJointState(lastKeyframe)
    doAssert lastState[].mTranslation.GetX() == 2.0
    doAssert animation.GetDuration() == 1.5
    animation.SetIsLooping(false)
    doAssert not animation.IsLooping()
    animation.ScaleJoints(2.0)
    doAssert api.asSkeletalAnimationJointState(
      api.getSkeletalAnimationKeyframe(
        api.getSkeletalAnimationJoint(addr animation, 0),
        1,
      ),
    )[].mTranslation.GetX() == 4.0
    api.clearSkeletalAnimationKeyframes(joint)
    doAssert api.getSkeletalAnimationKeyframeCount(joint) == 0
    api.clearSkeletalAnimationJoints(addr animation)
    doAssert api.getSkeletalAnimationJointCount(addr animation) == 0

  block:
    var shapeToID = api.constructObjectToIDMap[api.Shape]()
    var idToShape = api.constructIDToObjectMap[api.Shape]()
    var materialToID = api.constructObjectToIDMap[api.PhysicsMaterial]()
    var idToMaterial = api.constructIDToObjectMap[api.PhysicsMaterial]()
    var groupFilterToID = api.constructObjectToIDMap[api.GroupFilter]()
    var idToGroupFilter = api.constructIDToObjectMap[api.GroupFilter]()
    var sharedSettingsToID = api.constructObjectToIDMap[
      api.SoftBodySharedSettings,
    ]()
    var idToSharedSettings = api.constructIDToObjectMap[
      api.SoftBodySharedSettings,
    ]()
    discard addr shapeToID
    discard addr idToShape
    discard addr materialToID
    discard addr idToMaterial
    discard addr groupFilterToID
    discard addr idToGroupFilter
    discard addr sharedSettingsToID
    discard addr idToSharedSettings

  let combinedHash = api.JoltApi.HashCombineArgs(1'u32, 2'u32, 3'u32)
  doAssert combinedHash != 0'u64

  var contactSettings = api.constructSoftBodyContactSettings()
  contactSettings.mInvMassScale1 = 0.5
  doAssert contactSettings.mInvMassScale1 == 0.5

  block:
    var hairSettings = api.constructHairSettings()
    hairSettings.mNumIterationsPerSecond = 120
    doAssert hairSettings.mNumIterationsPerSecond == 120

    let simVertices = api.getHairSimVertices(addr hairSettings)
    simVertices[].push_back(api.constructHairSettings_SVertex(
      api.constructFloat3(0.0, 0.0, 0.0),
    ))
    simVertices[].push_back(api.constructHairSettings_SVertex(
      api.constructFloat3(0.0, 1.0, 0.0),
    ))
    doAssert simVertices[].size() == 2

    let simStrands = api.getHairSimStrands(addr hairSettings)
    simStrands[].push_back(api.constructHairSettings_SStrand(0, 2, 0))
    doAssert simStrands[].size() == 1
    doAssert simStrands[].at(0)[].mStartVtx == 0
    doAssert simStrands[].at(0)[].mEndVtx == 2
    doAssert api.asHairRenderStrand(simStrands[].at(0))[].VertexCount() == 2

    let renderVertices = api.getHairRenderVertices(addr hairSettings)
    renderVertices[].push_back(api.constructHairSettings_RVertex())
    let influence = api.getHairRenderVertexInfluence(
      renderVertices[].at(0),
      0,
    )
    influence[].mVertexIndex = 1
    influence[].mRelativePosition = api.constructFloat3(0.0, 0.25, 0.0)
    influence[].mWeight = 0.75
    doAssert influence[].mVertexIndex == 1
    doAssert influence[].mWeight == 0.75
    doAssert api.hairNumSVertexInfluences() == 3
    doAssert api.hairNoInfluence() == high(uint32)

    let renderStrands = api.getHairRenderStrands(addr hairSettings)
    renderStrands[].push_back(api.constructHairSettings_RStrand(0, 1))
    doAssert renderStrands[].at(0)[].VertexCount() == 1

    let scalpVertices = api.getHairScalpVertices(addr hairSettings)
    scalpVertices[].push_back(api.constructFloat3(0.0, 0.0, 0.0))
    scalpVertices[].push_back(api.constructFloat3(1.0, 0.0, 0.0))
    scalpVertices[].push_back(api.constructFloat3(0.0, 0.0, 1.0))
    doAssert scalpVertices[].size() == 3

    let scalpTriangles = api.getHairScalpTriangles(addr hairSettings)
    scalpTriangles[].push_back(api.constructIndexedTriangleNoMaterial(0, 1, 2))
    doAssert scalpTriangles[].size() == 1

    let inverseBindPose = api.getHairScalpInverseBindPose(addr hairSettings)
    inverseBindPose[].push_back(api.Mat44.sIdentity())
    doAssert inverseBindPose[].size() == 1

    var skinWeight = api.constructHairSettings_SkinWeight()
    skinWeight.mJointIdx = 4
    skinWeight.mWeight = 0.8
    let scalpSkinWeights = api.getHairScalpSkinWeights(addr hairSettings)
    scalpSkinWeights[].push_back(skinWeight)
    doAssert scalpSkinWeights[].at(0)[].mJointIdx == 4
    doAssert abs(scalpSkinWeights[].at(0)[].mWeight - 0.8) < 0.001

    let materials = api.getHairMaterials(addr hairSettings)
    materials[].push_back(api.constructHairSettings_Material())
    doAssert materials[].size() == 1

    var skinPoint = api.constructHairSettings_SkinPoint()
    skinPoint.mTriangleIndex = 0
    skinPoint.mU = 0.25
    skinPoint.mV = 0.5
    skinPoint.mToBishop = 7
    let skinPoints = api.getHairSkinPoints(addr hairSettings)
    skinPoints[].push_back(skinPoint)
    doAssert skinPoints[].at(0)[].mV == 0.5
    doAssert skinPoints[].at(0)[].mToBishop == 7

    let neutralDensity = api.getHairNeutralDensity(addr hairSettings)
    neutralDensity[].push_back(0.625)
    doAssert neutralDensity[].at(0)[] == 0.625
    neutralDensity[].clear()
    doAssert neutralDensity[].empty()

  const
    NonMoving = api.ObjectLayer(0)
    Moving = api.ObjectLayer(1)

  var objectLayerPairs = api.constructObjectLayerPairFilterTable(2)
  objectLayerPairs.EnableCollision(NonMoving, Moving)
  objectLayerPairs.EnableCollision(Moving, Moving)

  var broadPhaseLayers = api.constructBroadPhaseLayerInterfaceTable(2, 2)
  broadPhaseLayers.MapObjectToBroadPhaseLayer(
    NonMoving,
    api.constructBroadPhaseLayer(0'u8),
  )
  broadPhaseLayers.MapObjectToBroadPhaseLayer(
    Moving,
    api.constructBroadPhaseLayer(1'u8),
  )

  let objectVsBroadPhase = api.constructObjectVsBroadPhaseLayerFilterTable(
    broadPhaseLayers,
    2,
    objectLayerPairs,
    2,
  )

  var physics = api.constructPhysicsSystem()
  physics.Init(
    1_024,
    0,
    1_024,
    1_024,
    broadPhaseLayers,
    objectVsBroadPhase,
    objectLayerPairs,
  )

  doAssert physics.GetPhysicsSettings() != nil
  doAssert physics.GetBodyInterface() != nil
  doAssert physics.GetBodyInterfaceNoLock() != nil
  doAssert physics.GetBroadPhaseQuery() != nil
  doAssert physics.GetNarrowPhaseQuery() != nil
  doAssert physics.GetNarrowPhaseQueryNoLock() != nil
  doAssert physics.GetBodyLockInterface() != nil
  doAssert physics.GetBodyLockInterfaceNoLock() != nil
  doAssert physics.GetBroadPhaseLayerInterface() != nil
  doAssert physics.GetObjectVsBroadPhaseLayerFilter() != nil
  doAssert physics.GetObjectLayerPairFilter() != nil
  doAssert physics.GetSimCollideBodyVsBody() != nil
  let combineFriction = physics.GetCombineFriction()
  let combineRestitution = physics.GetCombineRestitution()
  let customCombine = api.newContactCombineFunctionAdapter(
    contactCombineCallback,
  )
  doAssert customCombine.IsValid()
  physics.SetCombineFriction(customCombine.Get())
  physics.SetCombineRestitution(customCombine.Get())
  doAssert physics.GetMaxBodies() == 1_024

  var listenerCallbackCount = 0'u32
  let activationListener = api.newBodyActivationListenerAdapter(
    bodyActivationCallback,
    bodyActivationCallback,
    addr listenerCallbackCount,
  )
  physics.SetBodyActivationListener(
    api.asBodyActivationListener(activationListener),
  )
  doAssert physics.GetBodyActivationListener() != nil
  api.invokeBodyActivated(activationListener, bodyId, 0)
  doAssert listenerCallbackCount == 1

  let stepListener = api.newPhysicsStepListenerAdapter(
    physicsStepCallback,
    addr listenerCallbackCount,
  )
  physics.AddStepListener(api.asPhysicsStepListener(stepListener))
  var stepContext = api.constructPhysicsStepListenerContext()
  api.invokePhysicsStep(stepListener, stepContext)
  doAssert listenerCallbackCount == 2

  let contactListener = api.newContactListenerAdapter(
    contactValidateCallback,
    contactCallback,
    contactCallback,
    contactRemovedCallback,
  )
  physics.SetContactListener(api.asContactListener(contactListener))
  doAssert physics.GetContactListener() != nil

  let softBodyListener = api.newSoftBodyContactListenerAdapter(
    softBodyValidateCallback,
    softBodyContactAddedCallback,
  )
  physics.SetSoftBodyContactListener(
    api.asSoftBodyContactListener(softBodyListener),
  )
  doAssert physics.GetSoftBodyContactListener() != nil

  let simShapeFilter = api.newSimShapeFilterAdapter(simShapeCallback)
  physics.SetSimShapeFilter(api.asSimShapeFilter(simShapeFilter))
  doAssert physics.GetSimShapeFilter() != nil

  var characterCallbackCount = 0'u32
  let characterListener = api.newCharacterContactListenerAdapter(
    nil,
    characterContactValidateCallback,
    characterContactCallback,
    characterContactCallback,
    nil,
    characterContactValidateCallback,
    characterContactCallback,
    characterContactCallback,
    nil,
    nil,
    nil,
    addr characterCallbackCount,
  )
  var characterContact = api.constructCharacterContact()
  var characterContactSettings = api.constructCharacterContactSettings()
  doAssert api.asCharacterContactListener(characterListener)[].OnContactValidate(
    nil,
    characterContact,
  )
  api.asCharacterContactListener(characterListener)[].OnContactAdded(
    nil,
    characterContact,
    characterContactSettings,
  )
  doAssert api.asCharacterContactListener(characterListener)[].OnCharacterContactValidate(
    nil,
    characterContact,
  )
  doAssert characterCallbackCount == 3

  let bodies = physics.GetBodyInterface()
  doAssert bodies != nil
  let fallingSphere = api.newSphereShape(0.5)
  doAssert fallingSphere != nil
  let fallingSettings = api.constructBodyCreationSettings(
    api.asShape(fallingSphere),
    api.constructVec3(0.0, 2.0, 0.0),
    api.Quat.sIdentity(),
    api.EMotionType.Dynamic,
    Moving,
  )
  let fallingBody = bodies[].CreateAndAddBody(
    fallingSettings,
    api.EActivation.Activate,
  )
  doAssert not fallingBody.IsInvalid()

  let fallingBoxShape = api.newBoxShape(
    api.constructVec3(0.5, 0.5, 0.5),
    0.05,
  )
  doAssert fallingBoxShape != nil
  let fallingBoxSettings = api.constructBodyCreationSettings(
    api.asShape(fallingBoxShape),
    api.constructVec3(3.0, 3.0, 0.0),
    api.Quat.sIdentity(),
    api.EMotionType.Dynamic,
    Moving,
  )
  let fallingBoxBody = bodies[].CreateAndAddBody(
    fallingBoxSettings,
    api.EActivation.Activate,
  )
  doAssert not fallingBoxBody.IsInvalid()

  var rayHitCount = 0'u32
  var broadPhaseHitCount = 0'u32
  var unusedCollectorHitCount = 0'u32
  let rayCollector = api.newCastRayCollectorAdapter(
    rayCastResultCallback,
    addr rayHitCount,
  )
  let shapeCastCollector = api.newCastShapeCollectorAdapter(
    shapeCastResultCallback,
    addr unusedCollectorHitCount,
  )
  let pointCollector = api.newCollidePointCollectorAdapter(
    collidePointResultCallback,
    addr unusedCollectorHitCount,
  )
  let shapeCollector = api.newCollideShapeCollectorAdapter(
    collideShapeResultCallback,
    addr unusedCollectorHitCount,
  )
  let transformedCollector = api.newTransformedShapeCollectorAdapter(
    transformedShapeCallback,
    addr unusedCollectorHitCount,
  )
  let rayBodyCollector = api.newRayCastBodyCollectorAdapter(
    broadPhaseCastResultCallback,
    addr unusedCollectorHitCount,
  )
  let shapeBodyCollector = api.newCastShapeBodyCollectorAdapter(
    broadPhaseCastResultCallback,
    addr unusedCollectorHitCount,
  )
  let collideBodyCollector = api.newCollideShapeBodyCollectorAdapter(
    bodyIDCallback,
    addr broadPhaseHitCount,
  )
  let bodyPairCollector = api.newBodyPairCollectorAdapter(
    bodyPairCallback,
    addr unusedCollectorHitCount,
  )
  doAssert api.asCastRayCollector(rayCollector) != nil
  doAssert api.asCastShapeCollector(shapeCastCollector) != nil
  doAssert api.asCollidePointCollector(pointCollector) != nil
  doAssert api.asCollideShapeCollector(shapeCollector) != nil
  doAssert api.asTransformedShapeCollector(transformedCollector) != nil
  doAssert api.asRayCastBodyCollector(rayBodyCollector) != nil
  doAssert api.asCastShapeBodyCollector(shapeBodyCollector) != nil
  doAssert api.asCollideShapeBodyCollector(collideBodyCollector) != nil
  doAssert api.asBodyPairCollector(bodyPairCollector) != nil

  var characterCollisionCallbackCount = 0'u32
  let characterCollision = api.newCharacterVsCharacterCollisionAdapter(
    characterCollideCallback,
    characterCastCallback,
    addr characterCollisionCallbackCount,
  )
  api.asCharacterVsCharacterCollision(characterCollision)[].CollideCharacter(
    nil,
    api.Mat44.sIdentity(),
    api.constructCollideShapeSettings(),
    api.Vec3.sZero(),
    api.asCollideShapeCollector(shapeCollector)[],
  )
  api.asCharacterVsCharacterCollision(characterCollision)[].CastCharacter(
    nil,
    api.Mat44.sIdentity(),
    api.Vec3.sAxisY(),
    api.constructShapeCastSettings(),
    api.Vec3.sZero(),
    api.asCastShapeCollector(shapeCastCollector)[],
  )
  doAssert characterCollisionCallbackCount == 2

  var filterCallbackCount = 0'u32
  let broadPhaseFilter = api.newBroadPhaseLayerFilterAdapter(
    broadPhaseLayerFilterCallback,
    addr filterCallbackCount,
  )
  let objectFilter = api.newObjectLayerFilterAdapter(
    objectLayerFilterCallback,
    addr filterCallbackCount,
  )
  let objectPairFilter = api.newObjectLayerPairFilterAdapter(
    objectLayerPairFilterCallback,
    addr filterCallbackCount,
  )
  let objectVsBroadPhaseFilter = api.newObjectVsBroadPhaseLayerFilterAdapter(
    objectVsBroadPhaseLayerFilterCallback,
    addr filterCallbackCount,
  )
  let bodyFilter = api.newBodyFilterAdapter(
    bodyIDFilterCallback,
    bodyFilterLockedCallback,
    addr filterCallbackCount,
  )
  let shapeFilter = api.newShapeFilterAdapter(
    shapeFilterSingleCallback,
    shapeFilterPairCallback,
    addr filterCallbackCount,
  )
  var stateRecorderCallbackCount = 0'u32
  let stateRecorderFilter = api.newStateRecorderFilterAdapter(
    stateRecorderBodyCallback,
    stateRecorderConstraintCallback,
    stateRecorderContactCallback,
    stateRecorderContactCallback,
    addr stateRecorderCallbackCount,
  )
  var broadPhaseInterfaceCallbackCount = 0'u32
  let broadPhaseInterface = api.newBroadPhaseLayerInterfaceAdapter(
    broadPhaseLayerCountCallback,
    broadPhaseLayerMapCallback,
    nil,
    addr broadPhaseInterfaceCallbackCount,
  )
  doAssert not api.asBroadPhaseLayerFilter(broadPhaseFilter)[].ShouldCollide(
    api.constructBroadPhaseLayer(1'u8),
  )
  doAssert not api.asObjectLayerFilter(objectFilter)[].ShouldCollide(Moving)
  doAssert api.asObjectLayerPairFilter(objectPairFilter)[].ShouldCollide(
    NonMoving,
    Moving,
  )
  doAssert not api.asObjectVsBroadPhaseLayerFilter(objectVsBroadPhaseFilter)[].ShouldCollide(
    Moving,
    api.constructBroadPhaseLayer(1'u8),
  )
  doAssert api.asBodyFilter(bodyFilter)[].ShouldCollide(fallingBody)
  doAssert not api.asShapeFilter(shapeFilter)[].ShouldCollide(
    api.asShape(fallingSphere),
    api.constructSubShapeID(),
  )
  doAssert filterCallbackCount == 6
  doAssert api.asBroadPhaseLayerInterface(broadPhaseInterface)[].GetNumBroadPhaseLayers() == 2
  doAssert api.asBroadPhaseLayerInterface(broadPhaseInterface)[].GetBroadPhaseLayer(Moving).GetValue() == 1'u8
  doAssert broadPhaseInterfaceCallbackCount == 2

  block:
    var callbackPhysics = api.constructPhysicsSystem()
    callbackPhysics.Init(
      16,
      0,
      16,
      16,
      api.asBroadPhaseLayerInterface(broadPhaseInterface)[],
      api.asObjectVsBroadPhaseLayerFilter(objectVsBroadPhaseFilter)[],
      api.asObjectLayerPairFilter(objectPairFilter)[],
    )
    doAssert callbackPhysics.GetMaxBodies() == 16

  let vehicleCollisionTester = api.newVehicleCollisionTesterAdapter(
    Moving,
    vehicleCollideRawCallback,
    nil,
  )
  let vehicleCollisionTesterBase = api.asVehicleCollisionTester(
    vehicleCollisionTester,
  )
  doAssert vehicleCollisionTesterBase != nil
  doAssert vehicleCollisionTesterBase[].GetObjectLayer() == Moving
  vehicleCollisionTesterBase[].SetBroadPhaseLayerFilter(
    api.asBroadPhaseLayerFilter(broadPhaseFilter),
  )
  vehicleCollisionTesterBase[].SetObjectLayerFilter(
    api.asObjectLayerFilter(objectFilter),
  )
  vehicleCollisionTesterBase[].SetBodyFilter(api.asBodyFilter(bodyFilter))
  doAssert vehicleCollisionTesterBase[].GetBroadPhaseLayerFilter() != nil
  doAssert vehicleCollisionTesterBase[].GetObjectLayerFilter() != nil
  doAssert vehicleCollisionTesterBase[].GetBodyFilter() != nil

  var scene = api.constructPhysicsScene()
  doAssert scene.GetBodies() != nil
  doAssert scene.GetConstraints() != nil
  doAssert scene.GetSoftBodies() != nil
  doAssert scene.GetBodies()[].empty()
  doAssert scene.GetConstraints()[].empty()
  doAssert scene.GetSoftBodies()[].empty()

  let compute = api.JoltApi.CreateComputeSystemCPU()
  doAssert compute.IsValid()

  var callbackCount = 0'u32
  let jobFunction = api.makeJobFunction(jobCallback, addr callbackCount)
  api.invokeJobFunction(jobFunction)
  doAssert callbackCount == 1

  var singleThreadedJobs = api.constructJobSystemSingleThreaded(16)
  doAssert singleThreadedJobs.GetMaxConcurrency() == 1
  let immediateJob = singleThreadedJobs.CreateJob(
    "raw-single-threaded",
    api.constructColor(),
    jobFunction,
  )
  doAssert callbackCount == 2
  var copiedImmediateJob = api.constructJobSystem_JobHandle()
  discard copiedImmediateJob.assign(immediateJob)

  var threadCallbackCount = 0'u32
  var jobs = api.constructJobSystemThreadPool()
  jobs.SetThreadInitFunction(
    api.makeThreadInitExitFunction(threadInitExitCallback, addr threadCallbackCount),
  )
  jobs.SetThreadExitFunction(
    api.makeThreadInitExitFunction(threadInitExitCallback, addr threadCallbackCount),
  )
  jobs.Init(1_024, 8, 0)
  doAssert jobs.GetMaxConcurrency() == 1

  var allocator = api.constructTempAllocatorImpl(4 * 1_024 * 1_024)
  let updateError = physics.Update(
    1.0 / 60.0,
    1,
    api.asTempAllocator(addr allocator),
    api.asJobSystem(addr jobs),
  )
  doAssert updateError == api.EPhysicsUpdateError.None
  doAssert bodies[].GetPosition(fallingBody).GetY() < 2.0
  doAssert bodies[].GetPosition(fallingBoxBody).GetY() < 3.0

  physics.GetBroadPhaseQuery()[].CollideSphere(
    api.constructVec3(0.0, 2.0, 0.0),
    2.0,
    api.asCollideShapeBodyCollector(collideBodyCollector)[],
    api.constructBroadPhaseLayerFilter(),
    api.constructObjectLayerFilter(),
  )
  doAssert broadPhaseHitCount >= 1

  physics.GetNarrowPhaseQuery()[].CastRay(
    api.constructRRayCast(
      api.constructRayCast(
        api.constructVec3(0.0, 5.0, 0.0),
        api.constructVec3(0.0, -10.0, 0.0),
      ),
    ),
    api.constructRayCastSettings(),
    api.asCastRayCollector(rayCollector)[],
    api.constructBroadPhaseLayerFilter(),
    api.constructObjectLayerFilter(),
    api.constructBodyFilter(),
    api.constructShapeFilter(),
  )
  doAssert rayHitCount >= 1

  let rayHitsBeforeFilteredQuery = rayHitCount
  physics.GetNarrowPhaseQuery()[].CastRay(
    api.constructRRayCast(
      api.constructRayCast(
        api.constructVec3(0.0, 5.0, 0.0),
        api.constructVec3(0.0, -10.0, 0.0),
      ),
    ),
    api.constructRayCastSettings(),
    api.asCastRayCollector(rayCollector)[],
    api.constructBroadPhaseLayerFilter(),
    api.constructObjectLayerFilter(),
    api.asBodyFilter(bodyFilter)[],
    api.constructShapeFilter(),
  )
  doAssert rayHitCount == rayHitsBeforeFilteredQuery
  doAssert filterCallbackCount >= 8

  var recorder = api.constructStateRecorderImpl()
  physics.SaveState(
    api.asStateRecorder(addr recorder)[],
    api.EStateRecorderState.All,
    api.asStateRecorderFilter(stateRecorderFilter),
  )
  doAssert stateRecorderCallbackCount >= 1

  bodies[].RemoveBody(fallingBody)
  bodies[].DestroyBody(fallingBody)
  bodies[].RemoveBody(fallingBoxBody)
  bodies[].DestroyBody(fallingBoxBody)

  api.delete(stateRecorderFilter)
  api.delete(broadPhaseInterface)
  api.delete(vehicleCollisionTester)
  api.delete(shapeFilter)
  api.delete(bodyFilter)
  api.delete(objectVsBroadPhaseFilter)
  api.delete(objectPairFilter)
  api.delete(objectFilter)
  api.delete(broadPhaseFilter)

  api.delete(bodyPairCollector)
  api.delete(characterCollision)
  api.delete(collideBodyCollector)
  api.delete(shapeBodyCollector)
  api.delete(rayBodyCollector)
  api.delete(transformedCollector)
  api.delete(shapeCollector)
  api.delete(pointCollector)
  api.delete(shapeCastCollector)
  api.delete(rayCollector)

  physics.SetSimShapeFilter(nil)
  physics.SetSoftBodyContactListener(nil)
  physics.SetContactListener(nil)
  physics.RemoveStepListener(api.asPhysicsStepListener(stepListener))
  physics.SetBodyActivationListener(nil)
  physics.SetCombineFriction(combineFriction)
  physics.SetCombineRestitution(combineRestitution)
  api.delete(customCombine)
  api.delete(simShapeFilter)
  api.delete(characterListener)
  api.delete(softBodyListener)
  api.delete(contactListener)
  api.delete(stepListener)
  api.delete(activationListener)

  let simCollide = api.makeSimCollideBodyVsBody(simCollideCallback)
  physics.SetSimCollideBodyVsBody(simCollide)
  doAssert physics.GetSimCollideBodyVsBody() != nil

  var shaderLoaderCount = 0'u32
  let shaderLoader = api.makeShaderLoader(
    shaderLoaderCallback,
    addr shaderLoaderCount,
  )
  var shaderData = api.constructComputeByteArray()
  var shaderError = api.joltInitCppString_String("")
  doAssert api.invokeShaderLoader(
    shaderLoader,
    "raw-shader",
    addr shaderData,
    addr shaderError,
  )
  doAssert shaderLoaderCount == 1
  doAssert shaderData.size() == 3
  doAssert shaderData.at(0)[] == 11'u8
  doAssert shaderData.at(2)[] == 33'u8
  doAssert $api.joltCppStringCStr_String(shaderError) == "loaded"
  let computeRef = compute.Get()
  doAssert computeRef != nil
  let computeSystem = computeRef[].GetPtr()
  doAssert computeSystem != nil
  computeSystem[].mShaderLoader = shaderLoader

  var tracking = api.constructQuadTreeTrackingVector()
  tracking.resize(2)
  doAssert tracking.size() == 2
  tracking.clear()
  doAssert tracking.empty()

  var sizedValues = api.constructArrayWithLength[uint32](3)
  doAssert sizedValues.size() == 3
  var filledValues = api.constructArrayWithValue[uint32](2, 7'u32)
  doAssert filledValues.size() == 2
  doAssert filledValues[0][] == 7'u32
  doAssert filledValues.front()[] == 7'u32
  doAssert filledValues.back()[] == 7'u32
  doAssert filledValues.data() == filledValues.begin()
  filledValues[0][] = 6'u32
  doAssert filledValues.at(0)[] == 6'u32
  var movedPushValue = 8'u32
  filledValues.pushBackMove(movedPushValue)
  doAssert filledValues.back()[] == 8'u32
  doAssert filledValues.emplace_back(9'u32)[] == 9'u32
  discard filledValues.emplace_back()
  doAssert filledValues.size() == 5
  filledValues.pop_back()
  doAssert filledValues.size() == 4

  var initializerValues = api.constructArrayFromInitializerList[uint32](
    1'u32,
    2'u32,
    3'u32,
    4'u32,
  )
  doAssert initializerValues.size() == 4
  doAssert initializerValues.front()[] == 1'u32
  doAssert initializerValues.back()[] == 4'u32
  initializerValues.assignInitializerList(6'u32, 7'u32, 8'u32, 9'u32)
  doAssert initializerValues.size() == 4
  doAssert initializerValues.front()[] == 6'u32
  doAssert initializerValues.assignInitializerListOperator(
    10'u32,
    11'u32,
    12'u32,
  ) == addr initializerValues
  doAssert initializerValues.size() == 3
  doAssert initializerValues.back()[] == 12'u32

  var pairValues = api.constructArray[api.StdPair[uint32, uint32]]()
  discard pairValues.emplace_back(1'u32, 2'u32)
  doAssert pairValues.size() == 1
  var float3Values = api.constructArray[api.Float3]()
  doAssert api.`[]`(float3Values.emplace_back(1.0, 2.0, 3.0)[], 1) == 2.0
  var float4Values = api.constructArray[api.Float4]()
  doAssert api.`[]`(
    float4Values.emplace_back(1.0, 2.0, 3.0, 4.0)[],
    3,
  ) == 4.0

  var copiedValues = api.constructArray(filledValues)
  doAssert copiedValues == filledValues
  var assignedValues = api.constructArray[uint32]()
  doAssert assignedValues.assign(copiedValues) == addr assignedValues
  doAssert assignedValues == copiedValues
  var moveConstructSource = api.constructArray(copiedValues)
  var moveConstructedValues = api.constructArrayMove(moveConstructSource)
  doAssert moveConstructSource.empty()
  doAssert moveConstructedValues.size() == copiedValues.size()
  var moveAssignSource = api.constructArray(copiedValues)
  doAssert assignedValues.assignMove(moveAssignSource) == addr assignedValues
  doAssert moveAssignSource.empty()
  doAssert assignedValues == copiedValues

  let immutableValues = api.constructArrayWithValue[uint32](2, 12'u32)
  doAssert immutableValues.get_allocator() != nil
  doAssert immutableValues.data() != nil
  doAssert immutableValues.at(1)[] == 12'u32
  doAssert immutableValues.front()[] == 12'u32
  doAssert immutableValues.back()[] == 12'u32
  doAssert immutableValues[1][] == 12'u32
  doAssert immutableValues.begin() != immutableValues.end_call()

  var reverseValues = api.constructArray[uint32]()
  reverseValues.push_back(10'u32)
  reverseValues.push_back(20'u32)
  var rangedValues = api.constructArrayFromRange[uint32](
    reverseValues.begin(),
    reverseValues.end_call(),
  )
  doAssert rangedValues == reverseValues
  rangedValues.insert(rangedValues.cbegin(), 5'u32)
  doAssert rangedValues.front()[] == 5'u32
  var insertionValues = api.constructArrayWithValue[uint32](2, 30'u32)
  rangedValues.insert(
    rangedValues.cend(),
    insertionValues.cbegin(),
    insertionValues.cend(),
  )
  doAssert rangedValues.size() == 5
  doAssert rangedValues.erase(rangedValues.cbegin()) == rangedValues.begin()
  doAssert rangedValues.front()[] == 10'u32
  discard rangedValues.erase(
    cast[api.Array_const_iterator[uint32, api.STLAllocator[uint32]]](
      rangedValues.at(2)
    ),
    rangedValues.cend(),
  )
  doAssert rangedValues.size() == 2
  var reverseIt = reverseValues.rbegin()
  doAssert api.`*`(reverseIt)[] == 20'u32
  let reverseNext = reverseIt + 1
  doAssert api.`*`(reverseNext)[] == 10'u32
  doAssert reverseIt.advance() == addr reverseIt
  doAssert api.`*`(reverseIt)[] == 10'u32
  doAssert reverseIt.retreat() == addr reverseIt
  doAssert api.`*`(reverseIt)[] == 20'u32
  let reverseBeforeAdvance = reverseIt.advancePost()
  doAssert api.`*`(reverseBeforeAdvance)[] == 20'u32
  doAssert api.`*`(reverseIt)[] == 10'u32
  let reverseBeforeRetreat = reverseIt.retreatPost()
  doAssert api.`*`(reverseBeforeRetreat)[] == 10'u32
  doAssert api.`*`(reverseIt)[] == 20'u32
  doAssert reverseIt.advanceBy(1) == addr reverseIt
  doAssert api.`*`(reverseIt)[] == 10'u32
  doAssert reverseIt.retreatBy(1) == addr reverseIt
  doAssert api.`*`(reverseIt)[] == 20'u32
  var reverseAssigned = api.constructArray_rev_it[uint32]()
  doAssert reverseAssigned.assign(reverseNext) == addr reverseAssigned
  doAssert api.`*`(reverseAssigned)[] == 10'u32

  var constReverseIt = api.constructArray_crev_it(reverseValues.at(1))
  doAssert api.`*`(constReverseIt)[] == 20'u32
  doAssert constReverseIt.advance() == addr constReverseIt
  doAssert api.`*`(constReverseIt)[] == 10'u32
  doAssert constReverseIt.retreat() == addr constReverseIt
  doAssert api.`*`(constReverseIt)[] == 20'u32
  let constBeforeAdvance = constReverseIt.advancePost()
  doAssert api.`*`(constBeforeAdvance)[] == 20'u32
  doAssert api.`*`(constReverseIt)[] == 10'u32
  let constBeforeRetreat = constReverseIt.retreatPost()
  doAssert api.`*`(constBeforeRetreat)[] == 10'u32
  doAssert api.`*`(constReverseIt)[] == 20'u32
  doAssert constReverseIt.advanceBy(1) == addr constReverseIt
  doAssert api.`*`(constReverseIt)[] == 10'u32
  doAssert constReverseIt.retreatBy(1) == addr constReverseIt
  doAssert api.`*`(constReverseIt)[] == 20'u32
  var constReverseAssigned = api.constructArray_crev_it[uint32]()
  doAssert constReverseAssigned.assign(constReverseIt) == addr constReverseAssigned
  doAssert api.`*`(constReverseAssigned)[] == 20'u32

  let integerHash = api.constructHash[uint32]()
  doAssert integerHash.hashValue(7'u32) == integerHash.hashValue(7'u32)
  let arrayHash = api.constructStdHash[
    uint32,
    api.STLAllocator[uint32],
  ]()
  doAssert arrayHash.hashValue(reverseValues) == reverseValues.GetHash().csize_t
  let bodyHash = api.constructHash_JPH_BodyID()
  doAssert bodyHash.hashValue(api.constructBodyID()) ==
    bodyHash.hashValue(api.constructBodyID())
  let subShapePairHash = api.constructStdHashJPHSubShapeIDPair()
  doAssert subShapePairHash.hashValue(api.constructSubShapeIDPair()) ==
    subShapePairHash.hashValue(api.constructSubShapeIDPair())

  let referencedSphere = api.newSphereShape(0.25)
  var shapeRef = api.constructRef[api.SphereShape](referencedSphere)
  doAssert shapeRef.GetPtr() == referencedSphere
  doAssert api.`*`(shapeRef) == referencedSphere
  doAssert shapeRef.InternalGetPointer()[] == cast[pointer](referencedSphere)
  var shapeRefCopy = api.constructRef(shapeRef)
  doAssert shapeRefCopy == shapeRef
  var shapeRefAssigned = api.constructRef[api.SphereShape]()
  doAssert shapeRefAssigned.assign(shapeRefCopy) == addr shapeRefAssigned
  doAssert shapeRefAssigned == referencedSphere
  doAssert shapeRefAssigned.assign(cast[ptr api.SphereShape](nil)) ==
    addr shapeRefAssigned
  doAssert shapeRefAssigned.GetPtr() == nil
  var shapeRefMoveSource = api.constructRef(shapeRef)
  var shapeRefMoved = api.constructRefMove(shapeRefMoveSource)
  doAssert shapeRefMoveSource.GetPtr() == nil
  doAssert shapeRefMoved.GetPtr() == referencedSphere
  doAssert shapeRefAssigned.assignMove(shapeRefMoved) == addr shapeRefAssigned
  doAssert shapeRefMoved.GetPtr() == nil
  doAssert shapeRefAssigned.GetPtr() == referencedSphere

  var shapeRefConst = api.constructRefConst[api.SphereShape](referencedSphere)
  doAssert shapeRefConst.GetPtr() == referencedSphere
  doAssert api.`*`(shapeRefConst) == referencedSphere
  doAssert shapeRefConst.InternalGetPointer()[] == cast[pointer](referencedSphere)
  var shapeRefConstCopy = api.constructRefConst(shapeRefConst)
  var shapeRefConstFromRef = api.constructRefConst(shapeRef)
  doAssert shapeRefConstCopy == shapeRefConstFromRef
  var shapeRefConstAssigned = api.constructRefConst[api.SphereShape]()
  doAssert shapeRefConstAssigned.assign(shapeRefConstCopy) ==
    addr shapeRefConstAssigned
  doAssert shapeRefConstAssigned.assign(shapeRef) == addr shapeRefConstAssigned
  doAssert shapeRefConstAssigned.assign(cast[ptr api.SphereShape](nil)) ==
    addr shapeRefConstAssigned
  var shapeRefConstMoveSource = api.constructRefConst(shapeRefConst)
  var shapeRefConstMoved = api.constructRefConstMove(shapeRefConstMoveSource)
  doAssert shapeRefConstMoveSource.GetPtr() == nil
  doAssert shapeRefConstMoved.GetPtr() == referencedSphere
  doAssert shapeRefConstAssigned.assignMove(shapeRefConstMoved) ==
    addr shapeRefConstAssigned
  var refForConstMove = api.constructRef(shapeRef)
  var shapeRefConstMovedFromRef = api.constructRefConstMove(refForConstMove)
  doAssert refForConstMove.GetPtr() == nil
  doAssert shapeRefConstMovedFromRef.GetPtr() == referencedSphere
  doAssert shapeRefConstAssigned.assignMove(shapeRefConstMovedFromRef) ==
    addr shapeRefConstAssigned
  var refForConstAssignMove = api.constructRef(shapeRef)
  doAssert shapeRefConstAssigned.assign(cast[ptr api.SphereShape](nil)) ==
    addr shapeRefConstAssigned
  doAssert shapeRefConstAssigned.assignMove(refForConstAssignMove) ==
    addr shapeRefConstAssigned
  doAssert refForConstAssignMove.GetPtr() == nil

  let shapeRefHash = api.constructStdHashRef[api.SphereShape]()
  doAssert shapeRefHash.hashValue(shapeRef) == shapeRef.GetHash().csize_t
  let shapeRefConstHash = api.constructStdHashRefConst[api.SphereShape]()
  doAssert shapeRefConstHash.hashValue(shapeRefConst) ==
    shapeRefConst.GetHash().csize_t

  let errorView = api.constructStdStringView("view-error")
  doAssert errorView.size() == 10
  doAssert $errorView.data() == "view-error"
  var rawResult = api.constructResult[uint32]()
  doAssert rawResult.IsEmpty()
  rawResult.Set(41'u32)
  doAssert rawResult.IsValid()
  doAssert rawResult.Get()[] == 41'u32
  var movedValue = 42'u32
  rawResult.SetMove(movedValue)
  doAssert rawResult.Get()[] == 42'u32
  var copiedResult = api.constructResult(rawResult)
  doAssert copiedResult.Get()[] == 42'u32
  var assignedResult = api.constructResult[uint32]()
  doAssert assignedResult.assign(copiedResult) == addr assignedResult
  doAssert assignedResult.Get()[] == 42'u32
  var movedResult = api.constructResultMove(copiedResult)
  doAssert movedResult.Get()[] == 42'u32
  doAssert assignedResult.assignMove(movedResult) == addr assignedResult
  doAssert assignedResult.Get()[] == 42'u32
  rawResult.SetError(errorView)
  doAssert rawResult.HasError()
  doAssert $api.joltCppStringCStr_String(rawResult.GetError()[]) == "view-error"
  var ownedError = api.joltInitCppString_String("owned-error")
  rawResult.SetError(ownedError)
  doAssert $api.joltCppStringCStr_String(rawResult.GetError()[]) == "owned-error"
  rawResult.SetError("cstring-error")
  doAssert $api.joltCppStringCStr_String(rawResult.GetError()[]) == "cstring-error"
  rawResult.Clear()
  doAssert rawResult.IsEmpty()

  type
    RawHashPair = api.StdPair[uint32, uint32]
    RawHashDetail = api.UnorderedMapDetail[uint32, uint32]
    RawHashFunction = api.Hash[uint32]
    RawHashEqual = api.StdEqualTo[uint32]
  var rawHashTable = api.constructHashTable[
    uint32,
    RawHashPair,
    RawHashDetail,
    RawHashFunction,
    RawHashEqual,
  ]()
  doAssert rawHashTable.empty()
  var rawHashBegin = rawHashTable.begin()
  let rawHashEnd = rawHashTable.end_call()
  doAssert rawHashBegin == rawHashEnd

  var staticValues = api.constructStaticArray[uint32, 8]()
  staticValues.push_back(2'u32)
  staticValues.emplace_back(3'u32)
  staticValues.emplace_back()
  doAssert staticValues.size() == 3
  staticValues.pop_back()
  doAssert staticValues.size() == 2
  doAssert staticValues.begin() == staticValues.data()
  doAssert staticValues.front()[] == 2'u32
  doAssert staticValues.back()[] == 3'u32
  staticValues[0][] = 1'u32
  doAssert staticValues.at(0)[] == 1'u32
  var staticInitializerValues =
    api.constructStaticArrayFromInitializerList[uint32, 8](
      20'u32,
      21'u32,
      22'u32,
      23'u32,
    )
  doAssert staticInitializerValues.size() == 4
  doAssert staticInitializerValues.front()[] == 20'u32
  doAssert staticInitializerValues.back()[] == 23'u32
  var staticCopy = api.constructStaticArray(staticValues)
  doAssert staticCopy == staticValues
  var staticAssigned = api.constructStaticArray[uint32, 8]()
  doAssert staticAssigned.assign(staticCopy) == addr staticAssigned
  var staticLarger = api.constructStaticArray[uint32, 16]()
  doAssert staticLarger.assignFromCapacity(staticAssigned) == addr staticLarger
  doAssert staticLarger == api.constructStaticArray[uint32, 16](staticLarger)
  staticLarger.eraseAt(staticLarger.begin())
  doAssert staticLarger.front()[] == 3'u32
  staticLarger.push_back(4'u32)
  staticLarger.push_back(5'u32)
  staticLarger.eraseRange(staticLarger.at(1), staticLarger.end_call())
  doAssert staticLarger.size() == 1

  let immutableStatic = api.constructStaticArray(staticValues)
  doAssert immutableStatic.begin() != immutableStatic.end_call()
  doAssert immutableStatic.data() != nil
  doAssert immutableStatic.at(0)[] == 1'u32
  doAssert immutableStatic.front()[] == 1'u32
  doAssert immutableStatic.back()[] == 3'u32
  doAssert immutableStatic[1][] == 3'u32
  let staticHash = api.constructStdHashStaticArray[uint32, 8]()
  doAssert staticHash.hashValue(staticValues) == staticValues.GetHash().csize_t

  var staticPairs = api.constructStaticArray[api.StdPair[uint32, uint32], 2]()
  staticPairs.emplace_back(1'u32, 2'u32)
  doAssert staticPairs.size() == 1
  var staticFloat3 = api.constructStaticArray[api.Float3, 2]()
  staticFloat3.emplace_back(4.0, 5.0, 6.0)
  doAssert api.`[]`(staticFloat3.front()[], 2) == 6.0
  var staticFloat4 = api.constructStaticArray[api.Float4, 2]()
  staticFloat4.emplace_back(7.0, 8.0, 9.0, 10.0)
  doAssert api.`[]`(staticFloat4.front()[], 3) == 10.0

  var unorderedMap = api.constructUnorderedMap[
    uint32,
    uint32,
    RawHashFunction,
    RawHashEqual,
  ]()
  unorderedMap[7'u32][] = 11'u32
  doAssert unorderedMap[7'u32][] == 11'u32
  let emplaced = unorderedMap.try_emplace(9'u32, 13'u32)
  doAssert emplaced.second

  var lockFreeAllocator = api.constructLFHMAllocator()
  lockFreeAllocator.Init(4_096)
  var lockFreeContext = api.constructLFHMAllocatorContext(
    lockFreeAllocator,
    256,
  )
  var lockFreeMap = api.constructLockFreeHashMap[uint32, uint32](
    lockFreeAllocator,
  )
  lockFreeMap.Init(16)
  let lockFreeHash = 123'u64
  let lockFreeEntry = lockFreeMap.Create(
    lockFreeContext,
    7'u32,
    lockFreeHash,
    0,
    42'u32,
  )
  doAssert lockFreeEntry != nil
  doAssert lockFreeEntry[].GetKey()[] == 7'u32
  doAssert lockFreeEntry[].GetValue()[] == 42'u32
  doAssert lockFreeMap.Find(7'u32, lockFreeHash) == lockFreeEntry
  let lockFreeHandle = lockFreeMap.ToHandle(lockFreeEntry)
  doAssert lockFreeMap.FromHandle(lockFreeHandle) == lockFreeEntry
  var lockFreeEntries = api.constructArray[
    api.LockFreeHashMap_ConstKeyValuePtr[uint32, uint32]
  ]()
  lockFreeMap.GetAllKeyValues(lockFreeEntries)
  doAssert lockFreeEntries.size() == 1
  var lockFreeIt = lockFreeMap.begin()
  let lockFreeEnd = lockFreeMap.end_call()
  doAssert lockFreeIt != lockFreeEnd
  doAssert api.`*`(lockFreeIt)[].GetValue()[] == 42'u32

  doAssert api.PhysicsMaterial.sInternalGetRefCountOffset() >= 0

  var freeList = api.constructFixedSizeFreeList[uint32]()
  freeList.Init(4, 2)
  let freeListIndex = freeList.ConstructObject(99'u32)
  doAssert freeList.Get(freeListIndex)[] == 99'u32
  var freeListBatch = api.constructFixedSizeFreeList_Batch[uint32]()
  freeList.AddObjectToBatch(freeListBatch, freeListIndex)
  freeList.DestructObjectBatch(freeListBatch)

  var stdString = api.constructStdBasicString[cchar]()
  stdString.push_back(120.cchar)
  var inputStream = api.constructStdBasicIstringstream(stdString)
  var emptyInputStream = api.constructStdBasicIstringstream[cchar]()
  inputStream.swap(emptyInputStream)

  var vector2 = api.constructVector[2]()
  vector2.SetZero()
  doAssert vector2.GetRows() == 2
  doAssert vector2.IsZero()

  var matrix22 = api.constructMatrix[2, 2]()
  matrix22.SetIdentity()
  doAssert matrix22.GetRows() == 2
  doAssert matrix22.GetCols() == 2
  doAssert matrix22.IsIdentity()

  var alignedAllocator = api.constructSTLAlignedAllocator[uint8, 64]()
  let alignedMemory = alignedAllocator.allocate(8)
  doAssert alignedMemory != nil
  alignedAllocator.deallocate(alignedMemory, 8)

  var localAllocator = api.constructSTLLocalAllocator[uint8, 64]()
  let localMemory = localAllocator.allocate(8)
  doAssert localMemory != nil
  doAssert api.is_local[uint8, 64](localAllocator, localMemory)
  localAllocator.deallocate(localMemory, 8)

  discard api.constructFPControlWord[0, 0]()

  var jobHandles = api.constructStaticArray[api.JobHandle, 32]()
  doAssert jobHandles.empty()
  var bodyPairQueues = api.constructStaticArray[
    api.PhysicsUpdateContext_BodyPairQueue,
    32,
  ]()
  bodyPairQueues.resize(1)
  doAssert bodyPairQueues.size() == 1

  discard api.makeHairRenderPositions(hairRenderCallback)
  discard api.makeVehicleCombineFriction(vehicleCombineCallback)
  discard api.makeVehicleStepCallback(vehicleStepCallback)
  discard api.makeTireMaxImpulseCallback(tireMaxImpulseCallback)

  var mutex = api.constructMutex()
  when defined(joltEnableAsserts):
    doAssert api.BodyAccess.sCheckRights(api.EAccess.ReadWrite, api.EAccess.Read)
    doAssert api.BodyAccess.sVelocityAccess() != nil
    doAssert api.BodyAccess.sPositionAccess() != nil
    api.PhysicsLock.sLock(mutex, nil, api.EPhysicsLockTypes.BroadPhaseQuery)
    api.PhysicsLock.sUnlock(mutex, nil, api.EPhysicsLockTypes.BroadPhaseQuery)
  else:
    api.PhysicsLock.sLock(mutex)
    api.PhysicsLock.sUnlock(mutex)

api.JoltApi.RegisterDefaultAllocator()
doAssert api.joltFactoryInstance == nil
api.joltFactoryInstance = api.newJoltFactory()
doAssert api.joltFactoryInstance != nil
api.JoltApi.RegisterTypes()
testLowLevelApi()
api.JoltApi.UnregisterTypes()
api.deleteJoltFactory(api.joltFactoryInstance)
api.joltFactoryInstance = nil
