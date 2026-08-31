## Thin native bindings for the Jolt Physics C++ API.
##
## Compile this module with Nim's C++ backend. The Jolt headers, library,
## preprocessor definitions, and CPU options must match the linked Jolt build.

import std/os

const joltBindingsHeader = currentSourcePath.parentDir / "private" / "jolt_bindings.hpp"

type
  ObjectLayer* = uint16

  EActivation* {.importcpp: "JPH::EActivation", pure, size: sizeof(cint).} = enum
    Activate = 0
    DontActivate = 1

  EMotionType* {.importcpp: "JPH::EMotionType", pure, size: sizeof(uint8).} = enum
    Static = 0
    Kinematic = 1
    Dynamic = 2

  EMotionQuality* {.importcpp: "JPH::EMotionQuality", pure, size: sizeof(uint8).} = enum
    Discrete = 0
    LinearCast = 1

  Vec3* {.importcpp: "JPH::Vec3", header: joltBindingsHeader.} = object
  Quat* {.importcpp: "JPH::Quat", header: joltBindingsHeader.} = object
  TempAllocatorImpl* {.importcpp: "JPH::TempAllocatorImpl", header: joltBindingsHeader.} = object
  JobSystemThreadPool* {.importcpp: "JPH::JobSystemThreadPool", header: joltBindingsHeader.} = object
  BodyID* {.importcpp: "JPH::BodyID", header: joltBindingsHeader.} = object
  Body* {.importcpp: "JPH::Body", header: joltBindingsHeader.} = object
  Shape* {.importcpp: "JPH::Shape", header: joltBindingsHeader.} = object
  PhysicsMaterial* {.importcpp: "JPH::PhysicsMaterial", header: joltBindingsHeader.} = object
  Constraint* {.importcpp: "JPH::Constraint", header: joltBindingsHeader.} = object
  RagdollPartData* {.importcpp: "joltnim_detail::RagdollPartData", header: joltBindingsHeader.} = object
  RagdollDistanceConstraintData* {.importcpp: "joltnim_detail::RagdollDistanceConstraintData", header: joltBindingsHeader.} = object
  RagdollPointConstraintData* {.importcpp: "joltnim_detail::RagdollPointConstraintData", header: joltBindingsHeader.} = object
  RagdollFixedConstraintData* {.importcpp: "joltnim_detail::RagdollFixedConstraintData", header: joltBindingsHeader.} = object
  RagdollHingeConstraintData* {.importcpp: "joltnim_detail::RagdollHingeConstraintData", header: joltBindingsHeader.} = object
  RagdollSliderConstraintData* {.importcpp: "joltnim_detail::RagdollSliderConstraintData", header: joltBindingsHeader.} = object
  RagdollSwingTwistConstraintData* {.importcpp: "joltnim_detail::RagdollSwingTwistConstraintData", header: joltBindingsHeader.} = object
  RagdollSixDOFConstraintData* {.importcpp: "joltnim_detail::RagdollSixDOFConstraintData", header: joltBindingsHeader.} = object
  RagdollConeConstraintData* {.importcpp: "joltnim_detail::RagdollConeConstraintData", header: joltBindingsHeader.} = object
  RagdollHandle* {.importcpp: "joltnim_detail::RagdollHandle", header: joltBindingsHeader.} = object
  SkeletonMapperHandle* {.importcpp: "joltnim_detail::SkeletonMapperHandle", header: joltBindingsHeader.} = object
  SkeletalAnimationHandle* {.importcpp: "joltnim_detail::SkeletalAnimationHandle", header: joltBindingsHeader.} = object
  PhysicsSceneHandle* {.importcpp: "joltnim_detail::PhysicsSceneHandle", header: joltBindingsHeader.} = object
  PhysicsSceneInstanceHandle* {.importcpp: "joltnim_detail::PhysicsSceneInstanceHandle", header: joltBindingsHeader.} = object
  AuthoredCompoundSettingsHandle* {.importcpp: "joltnim_detail::AuthoredCompoundSettingsHandle", header: joltBindingsHeader.} = object
  CharacterHandle* {.importcpp: "joltnim_detail::CharacterHandle", header: joltBindingsHeader.} = object
  CharacterBroadPhase* {.importcpp: "joltnim_detail::CharacterBroadPhase", header: joltBindingsHeader.} = object
  RigidCharacterHandle* {.importcpp: "joltnim_detail::RigidCharacterHandle", header: joltBindingsHeader.} = object
  VehicleHandle* {.importcpp: "joltnim_detail::VehicleHandle", header: joltBindingsHeader.} = object
  VehicleWheelConfigData* {.importcpp: "joltnim_detail::VehicleWheelConfigData", header: joltBindingsHeader.} = object
  TrackedVehicleWheelConfigData* {.importcpp: "joltnim_detail::TrackedVehicleWheelConfigData", header: joltBindingsHeader.} = object
  VehicleDifferentialConfigData* {.importcpp: "joltnim_detail::VehicleDifferentialConfigData", header: joltBindingsHeader.} = object
  VehicleAntiRollBarConfigData* {.importcpp: "joltnim_detail::VehicleAntiRollBarConfigData", header: joltBindingsHeader.} = object
  EventBridge* {.importcpp: "joltnim_detail::EventBridge", header: joltBindingsHeader.} = object
  WorldStateHandle* {.importcpp: "joltnim_detail::WorldStateHandle", header: joltBindingsHeader.} = object
  BodyCreationSettings* {.importcpp: "JPH::BodyCreationSettings", header: joltBindingsHeader.} = object
  BodyInterface* {.importcpp: "JPH::BodyInterface", header: joltBindingsHeader.} = object
  PhysicsSystem* {.importcpp: "JPH::PhysicsSystem", header: joltBindingsHeader.} = object
  PhysicsSettings* {.importcpp: "JPH::PhysicsSettings", header: joltBindingsHeader.} = object
    mMaxInFlightBodyPairs*: cint
    mStepListenersBatchSize*: cint
    mStepListenerBatchesPerJob*: cint
    mBaumgarte*: cfloat
    mSpeculativeContactDistance*: cfloat
    mPenetrationSlop*: cfloat
    mLinearCastThreshold*: cfloat
    mLinearCastMaxPenetration*: cfloat
    mManifoldTolerance*: cfloat
    mMaxPenetrationDistance*: cfloat
    mBodyPairCacheMaxDeltaPositionSq*: cfloat
    mBodyPairCacheCosMaxDeltaRotationDiv2*: cfloat
    mContactNormalCosMaxDeltaRotation*: cfloat
    mContactPointPreserveLambdaMaxDistSq*: cfloat
    mInternalEdgeRemovalVertexToleranceSq*: cfloat
    mNumVelocitySteps*: uint
    mNumPositionSteps*: uint
    mMinVelocityForRestitution*: cfloat
    mTimeBeforeSleep*: cfloat
    mPointVelocitySleepThreshold*: cfloat
    mDeterministicSimulation*: bool
    mConstraintWarmStart*: bool
    mUseBodyPairContactCache*: bool
    mUseManifoldReduction*: bool
    mUseLargeIslandSplitter*: bool
    mAllowSleeping*: bool
    mCheckActiveEdges*: bool
  BodySnapshotData* {.importcpp: "joltnim_detail::BodySnapshotData",
                      header: joltBindingsHeader.} = object
    mSucceeded*: bool
    mMotionType*: uint8
    mObjectLayer*: uint16
    mPosition*: Vec3
    mCenterOfMassPosition*: Vec3
    mRotation*: Quat
    mLinearVelocity*: Vec3
    mAngularVelocity*: Vec3
    mActive*: bool
    mSensor*: bool
    mInBroadPhase*: bool
    mCollisionCacheInvalid*: bool
    mUseManifoldReduction*: bool
    mFriction*: cfloat
    mRestitution*: cfloat
    mUserData*: uint64
    mHasMotionProperties*: bool
    mMotionQuality*: uint8
    mAllowedDOFs*: uint8
    mLinearDamping*: cfloat
    mAngularDamping*: cfloat
    mMaxLinearVelocity*: cfloat
    mMaxAngularVelocity*: cfloat
    mGravityFactor*: cfloat
    mAllowSleeping*: bool
    mCollideKinematicVsNonDynamic*: bool
    mApplyGyroscopicForce*: bool
    mEnhancedInternalEdgeRemoval*: bool
    mNumVelocityStepsOverride*: uint32
    mNumPositionStepsOverride*: uint32
    mHasMass*: bool
    mMass*: cfloat
    mHasMassProperties*: bool
    mInertiaDiagonal*: Vec3
    mInertiaRotation*: Quat
  BroadPhaseLayer* {.importcpp: "JPH::BroadPhaseLayer", header: joltBindingsHeader.} = object
  ObjectLayerPairFilterTable* {.importcpp: "JPH::ObjectLayerPairFilterTable", header: joltBindingsHeader.} = object
  BroadPhaseLayerInterfaceTable* {.importcpp: "JPH::BroadPhaseLayerInterfaceTable", header: joltBindingsHeader.} = object
  ObjectVsBroadPhaseLayerFilterTable* {.importcpp: "JPH::ObjectVsBroadPhaseLayerFilterTable", header: joltBindingsHeader.} = object
  GroupFilterTable* {.importcpp: "JPH::GroupFilterTable", header: joltBindingsHeader.} = object

proc acquireJolt*(): bool {.importcpp: "joltnim_detail::AcquireJolt()", header: joltBindingsHeader.}
proc releaseJolt*() {.importcpp: "joltnim_detail::ReleaseJolt()", header: joltBindingsHeader.}
proc verifyJoltVersionID*(): bool {.importcpp: "JPH::VerifyJoltVersionID()", header: joltBindingsHeader.}

proc vec3*(x, y, z: cfloat): Vec3 {.importcpp: "JPH::Vec3(@)", constructor, header: joltBindingsHeader.}
proc x*(self: Vec3): cfloat {.importcpp: "#.GetX()", noSideEffect, header: joltBindingsHeader.}
proc y*(self: Vec3): cfloat {.importcpp: "#.GetY()", noSideEffect, header: joltBindingsHeader.}
proc z*(self: Vec3): cfloat {.importcpp: "#.GetZ()", noSideEffect, header: joltBindingsHeader.}
proc identity*(_: type Quat): Quat {.importcpp: "JPH::Quat::sIdentity()", header: joltBindingsHeader.}
proc quat*(x, y, z, w: cfloat): Quat {.importcpp: "JPH::Quat(@)", constructor, header: joltBindingsHeader.}
proc x*(self: Quat): cfloat {.importcpp: "#.GetX()", noSideEffect, header: joltBindingsHeader.}
proc y*(self: Quat): cfloat {.importcpp: "#.GetY()", noSideEffect, header: joltBindingsHeader.}
proc z*(self: Quat): cfloat {.importcpp: "#.GetZ()", noSideEffect, header: joltBindingsHeader.}
proc w*(self: Quat): cfloat {.importcpp: "#.GetW()", noSideEffect, header: joltBindingsHeader.}

proc bodyID*(value: uint32): BodyID {.importcpp: "JPH::BodyID(@)", constructor, header: joltBindingsHeader.}
proc value*(self: BodyID): uint32 {.importcpp: "#.GetIndexAndSequenceNumber()", noSideEffect, header: joltBindingsHeader.}
proc isInvalid*(self: BodyID): bool {.importcpp: "#.IsInvalid()", noSideEffect, header: joltBindingsHeader.}

proc broadPhaseLayer*(value: uint8): BroadPhaseLayer {.importcpp: "JPH::BroadPhaseLayer(@)", constructor, header: joltBindingsHeader.}

proc newTempAllocator*(size: csize_t): ptr TempAllocatorImpl {.importcpp: "new JPH::TempAllocatorImpl(@)", header: joltBindingsHeader.}
proc delete*(self: ptr TempAllocatorImpl) {.importcpp: "delete #", header: joltBindingsHeader.}

proc newJobSystemThreadPool*(maxJobs, maxBarriers: uint; numThreads: cint = -1): ptr JobSystemThreadPool {.importcpp: "new JPH::JobSystemThreadPool(@)", header: joltBindingsHeader.}
proc delete*(self: ptr JobSystemThreadPool) {.importcpp: "delete #", header: joltBindingsHeader.}

proc newObjectLayerPairFilterTable*(numLayers: uint): ptr ObjectLayerPairFilterTable {.importcpp: "new JPH::ObjectLayerPairFilterTable(@)", header: joltBindingsHeader.}
proc delete*(self: ptr ObjectLayerPairFilterTable) {.importcpp: "delete #", header: joltBindingsHeader.}
proc enableCollision*(self: ptr ObjectLayerPairFilterTable; layer1, layer2: ObjectLayer) {.importcpp: "#->EnableCollision(@)", header: joltBindingsHeader.}

proc newBroadPhaseLayerInterfaceTable*(numObjectLayers, numBroadPhaseLayers: uint): ptr BroadPhaseLayerInterfaceTable {.importcpp: "new JPH::BroadPhaseLayerInterfaceTable(@)", header: joltBindingsHeader.}
proc delete*(self: ptr BroadPhaseLayerInterfaceTable) {.importcpp: "delete #", header: joltBindingsHeader.}
proc mapObjectToBroadPhaseLayer*(self: ptr BroadPhaseLayerInterfaceTable; objectLayer: ObjectLayer; broadPhaseLayer: BroadPhaseLayer) {.importcpp: "#->MapObjectToBroadPhaseLayer(@)", header: joltBindingsHeader.}

proc newObjectVsBroadPhaseLayerFilterTable*(broadPhaseInterface: ptr BroadPhaseLayerInterfaceTable; numBroadPhaseLayers: uint; objectPairFilter: ptr ObjectLayerPairFilterTable; numObjectLayers: uint): ptr ObjectVsBroadPhaseLayerFilterTable {.importcpp: "joltnim_detail::CreateObjectVsBroadPhaseLayerFilter(@)", header: joltBindingsHeader.}
proc delete*(self: ptr ObjectVsBroadPhaseLayerFilterTable) {.importcpp: "delete #", header: joltBindingsHeader.}

proc newPhysicsSystem*(): ptr PhysicsSystem {.importcpp: "new JPH::PhysicsSystem()", header: joltBindingsHeader.}
proc delete*(self: ptr PhysicsSystem) {.importcpp: "delete #", header: joltBindingsHeader.}
proc constructPhysicsSettings*(): PhysicsSettings {.importcpp: "JPH::PhysicsSettings()", constructor, header: joltBindingsHeader.}
proc GetPhysicsSettings*(self: ptr PhysicsSystem): ptr PhysicsSettings {.importcpp: "&(#->GetPhysicsSettings())", noSideEffect, header: joltBindingsHeader.}
proc SetPhysicsSettings*(self: ptr PhysicsSystem; settings: PhysicsSettings) {.importcpp: "#->SetPhysicsSettings(@)", header: joltBindingsHeader.}
proc init*(self: ptr PhysicsSystem; maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints: uint; broadPhaseInterface: ptr BroadPhaseLayerInterfaceTable; objectVsBroadPhaseFilter: ptr ObjectVsBroadPhaseLayerFilterTable; objectPairFilter: ptr ObjectLayerPairFilterTable) {.importcpp: "joltnim_detail::InitializePhysicsSystem(@)", header: joltBindingsHeader.}
proc bodyInterface*(self: ptr PhysicsSystem): ptr BodyInterface {.importcpp: "joltnim_detail::GetBodyInterface(@)", noSideEffect, header: joltBindingsHeader.}
proc optimizeBroadPhase*(self: ptr PhysicsSystem) {.importcpp: "#->OptimizeBroadPhase()", header: joltBindingsHeader.}
proc update*(self: ptr PhysicsSystem; deltaTime: cfloat; collisionSteps: cint; allocator: ptr TempAllocatorImpl; jobs: ptr JobSystemThreadPool): uint32 {.importcpp: "joltnim_detail::Update(@)", header: joltBindingsHeader.}
proc gravity*(self: ptr PhysicsSystem): Vec3 {.importcpp: "#->GetGravity()", noSideEffect, header: joltBindingsHeader.}
proc setGravity*(self: ptr PhysicsSystem; gravity: Vec3) {.importcpp: "#->SetGravity(@)", header: joltBindingsHeader.}

proc newGroupFilterTable*(numSubGroups: uint32): ptr GroupFilterTable
  {.importcpp: "joltnim_detail::CreateGroupFilterTable(@)", header: joltBindingsHeader.}
proc release*(self: ptr GroupFilterTable)
  {.importcpp: "joltnim_detail::ReleaseGroupFilterTable(@)", header: joltBindingsHeader.}
proc disableCollision*(self: ptr GroupFilterTable; subgroup1, subgroup2: uint32)
  {.importcpp: "#->DisableCollision(@)", header: joltBindingsHeader.}
proc enableCollision*(self: ptr GroupFilterTable; subgroup1, subgroup2: uint32)
  {.importcpp: "#->EnableCollision(@)", header: joltBindingsHeader.}
proc isCollisionEnabled*(self: ptr GroupFilterTable; subgroup1, subgroup2: uint32): bool
  {.importcpp: "#->IsCollisionEnabled(@)", noSideEffect, header: joltBindingsHeader.}
proc setCollisionGroup*(self: ptr PhysicsSystem; id: BodyID;
                        filter: ptr GroupFilterTable;
                        groupId, subgroupId: uint32)
  {.importcpp: "joltnim_detail::SetBodyCollisionGroup(@)", header: joltBindingsHeader.}
proc clearCollisionGroup*(self: ptr PhysicsSystem; id: BodyID)
  {.importcpp: "joltnim_detail::ClearBodyCollisionGroup(@)", header: joltBindingsHeader.}
proc collisionGroup*(self: ptr PhysicsSystem; id: BodyID;
                     groupId, subgroupId: ptr uint32): bool
  {.importcpp: "joltnim_detail::GetBodyCollisionGroup(@)", noSideEffect, header: joltBindingsHeader.}

proc newPhysicsMaterial*(name: cstring; red, green, blue,
                         alpha: uint8): ptr PhysicsMaterial
  {.importcpp: "joltnim_detail::CreatePhysicsMaterial(@)", header: joltBindingsHeader.}
proc release*(self: ptr PhysicsMaterial)
  {.importcpp: "joltnim_detail::ReleasePhysicsMaterial(@)", header: joltBindingsHeader.}
proc bodyMaterial*(self: ptr PhysicsSystem; id: BodyID; subShapeId: uint32;
                   name: ptr cstring; red, green, blue,
                   alpha: ptr uint8): bool
  {.importcpp: "joltnim_detail::GetBodyMaterial(@)", noSideEffect, header: joltBindingsHeader.}

proc newBoxShape*(halfExtent: Vec3; convexRadius: cfloat;
                  material: ptr PhysicsMaterial): ptr Shape {.importcpp: "new JPH::BoxShape(@)", header: joltBindingsHeader.}
proc newSphereShape*(radius: cfloat; material: ptr PhysicsMaterial): ptr Shape {.importcpp: "new JPH::SphereShape(@)", header: joltBindingsHeader.}
proc newCapsuleShape*(halfHeight, radius: cfloat;
                      material: ptr PhysicsMaterial): ptr Shape {.importcpp: "new JPH::CapsuleShape(@)", header: joltBindingsHeader.}
proc newCylinderShape*(halfHeight, radius, convexRadius: cfloat;
                       material: ptr PhysicsMaterial): ptr Shape {.importcpp: "new JPH::CylinderShape(@)", header: joltBindingsHeader.}
proc newTaperedCapsuleShape*(halfHeight, topRadius,
                             bottomRadius: cfloat;
                             material: ptr PhysicsMaterial): ptr Shape
  {.importcpp: "joltnim_detail::CreateTaperedCapsuleShape(@)", header: joltBindingsHeader.}
proc newTaperedCylinderShape*(halfHeight, topRadius, bottomRadius,
                              convexRadius: cfloat;
                              material: ptr PhysicsMaterial): ptr Shape
  {.importcpp: "joltnim_detail::CreateTaperedCylinderShape(@)", header: joltBindingsHeader.}
proc newTriangleShape*(v1, v2, v3: Vec3; convexRadius: cfloat;
                       material: ptr PhysicsMaterial): ptr Shape
  {.importcpp: "joltnim_detail::CreateTriangleShape(@)", header: joltBindingsHeader.}
proc newPlaneShape*(normal: Vec3; constant, halfExtent: cfloat;
                    material: ptr PhysicsMaterial): ptr Shape
  {.importcpp: "joltnim_detail::CreatePlaneShape(@)", header: joltBindingsHeader.}
proc newEmptyShape*(centerOfMass: Vec3): ptr Shape
  {.importcpp: "joltnim_detail::CreateEmptyShape(@)", header: joltBindingsHeader.}
proc newConvexHullShape*(points: ptr Vec3; pointCount: uint32;
                         maxConvexRadius: cfloat;
                         material: ptr PhysicsMaterial): ptr Shape
  {.importcpp: "joltnim_detail::CreateConvexHullShape(@)", header: joltBindingsHeader.}
proc newTriangleMeshShape*(vertices: ptr Vec3; vertexCount: uint32;
                           indices: ptr uint32;
                           triangleCount: uint32;
                           materials: ptr ptr PhysicsMaterial;
                           materialCount: uint32;
                           materialIndices: ptr uint32): ptr Shape
  {.importcpp: "joltnim_detail::CreateTriangleMeshShape(@)", header: joltBindingsHeader.}
proc newHeightFieldShape*(samples: ptr cfloat; sampleCount: uint32;
                          offset, scale: Vec3; blockSize,
                          bitsPerSample: uint32;
                          materialIndices: ptr uint8;
                          materials: ptr ptr PhysicsMaterial;
                          materialCount: uint32): ptr Shape
  {.importcpp: "joltnim_detail::CreateHeightFieldShape(@)", header: joltBindingsHeader.}
proc newStaticCompoundShape*(shapes: ptr ptr Shape; positions: ptr Vec3;
                             rotations: ptr Quat;
                             shapeCount: uint32): ptr Shape
  {.importcpp: "joltnim_detail::CreateStaticCompoundShape(@)", header: joltBindingsHeader.}
proc newMutableCompoundShape*(shapes: ptr ptr Shape; positions: ptr Vec3;
                              rotations: ptr Quat;
                              shapeCount: uint32): ptr Shape
  {.importcpp: "joltnim_detail::CreateMutableCompoundShape(@)", header: joltBindingsHeader.}
proc newScaledShape*(innerShape: ptr Shape; scale: Vec3): ptr Shape
  {.importcpp: "joltnim_detail::CreateScaledShape(@)", header: joltBindingsHeader.}
proc newRotatedTranslatedShape*(innerShape: ptr Shape; position: Vec3;
                                rotation: Quat): ptr Shape
  {.importcpp: "joltnim_detail::CreateRotatedTranslatedShape(@)", header: joltBindingsHeader.}
proc newOffsetCenterOfMassShape*(innerShape: ptr Shape; offset: Vec3): ptr Shape
  {.importcpp: "joltnim_detail::CreateOffsetCenterOfMassShape(@)", header: joltBindingsHeader.}
proc addRef*(self: ptr Shape) {.importcpp: "#->AddRef()", header: joltBindingsHeader.}
proc release*(self: ptr Shape) {.importcpp: "#->Release()", header: joltBindingsHeader.}

proc ragdollPartData*(shape: ptr Shape; position: Vec3; rotation: Quat;
                      parent: int32; jointType: uint8;
                      layer: ObjectLayer; motionType: uint8;
                      jointPosition, twistAxis, planeAxis: Vec3;
                      sixDOFSwingType: uint8;
                      normalHalfConeAngle, planeHalfConeAngle,
                      twistMinAngle, twistMaxAngle, maxFrictionTorque,
                      motorFrequency, motorDamping, maxMotorTorque: cfloat;
                      linearLimitMin, linearLimitMax,
                      angularLimitMin, angularLimitMax,
                      linearFriction, angularFriction: Vec3;
                      allowedDOFs, motionQuality: uint8;
                      mass, inertiaMultiplier: cfloat;
                      linearVelocity, angularVelocity: Vec3;
                      userData: uint64;
                      allowSleeping, collideKinematicVsNonDynamic,
                      useManifoldReduction, applyGyroscopicForce,
                      enhancedInternalEdgeRemoval: bool;
                      friction, restitution, linearDamping, angularDamping,
                      maxLinearVelocity, maxAngularVelocity,
                      gravityFactor: cfloat;
                      numVelocityStepsOverride,
                      numPositionStepsOverride: uint32): RagdollPartData
  {.importcpp: "joltnim_detail::MakeRagdollPartData(@)",
    header: joltBindingsHeader.}
proc ragdollDistanceConstraintData*(part1, part2: uint32;
                                    point1, point2: Vec3;
                                    minDistance, maxDistance: cfloat):
                                    RagdollDistanceConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollDistanceConstraintData(@)",
    header: joltBindingsHeader.}
proc ragdollPointConstraintData*(part1, part2: uint32;
                                 point: Vec3): RagdollPointConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollPointConstraintData(@)",
    header: joltBindingsHeader.}
proc ragdollFixedConstraintData*(part1, part2: uint32):
                                 RagdollFixedConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollFixedConstraintData(@)",
    header: joltBindingsHeader.}
proc ragdollHingeConstraintData*(part1, part2: uint32; point,
                                 hingeAxis, normalAxis: Vec3;
                                 minAngle, maxAngle,
                                 maxFrictionTorque: cfloat):
                                 RagdollHingeConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollHingeConstraintData(@)",
    header: joltBindingsHeader.}
proc ragdollSliderConstraintData*(part1, part2: uint32; point,
                                  sliderAxis, normalAxis: Vec3;
                                  minPosition, maxPosition,
                                  maxFrictionForce: cfloat):
                                  RagdollSliderConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollSliderConstraintData(@)",
    header: joltBindingsHeader.}
proc ragdollSwingTwistConstraintData*(part1, part2: uint32; point,
                                      twistAxis, planeAxis: Vec3;
                                      normalHalfConeAngle,
                                      planeHalfConeAngle, twistMinAngle,
                                      twistMaxAngle,
                                      maxFrictionTorque: cfloat):
                                      RagdollSwingTwistConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollSwingTwistConstraintData(@)",
    header: joltBindingsHeader.}
proc ragdollSixDOFConstraintData*(part1, part2: uint32; point,
                                  axisX, axisY: Vec3; swingType: uint8;
                                  linearLimitMin, linearLimitMax,
                                  angularLimitMin, angularLimitMax,
                                  linearFriction, angularFriction: Vec3):
                                  RagdollSixDOFConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollSixDOFConstraintData(@)",
    header: joltBindingsHeader.}
proc ragdollConeConstraintData*(part1, part2: uint32; point,
                                twistAxis1, twistAxis2: Vec3;
                                halfConeAngle: cfloat):
                                RagdollConeConstraintData
  {.importcpp: "joltnim_detail::MakeRagdollConeConstraintData(@)",
    header: joltBindingsHeader.}
proc newRagdoll*(system: ptr PhysicsSystem; parts: ptr RagdollPartData;
                 partCount: uint32;
                 distanceConstraints: ptr RagdollDistanceConstraintData;
                 distanceConstraintCount: uint32;
                 pointConstraints: ptr RagdollPointConstraintData;
                 pointConstraintCount: uint32;
                 fixedConstraints: ptr RagdollFixedConstraintData;
                 fixedConstraintCount: uint32;
                 hingeConstraints: ptr RagdollHingeConstraintData;
                 hingeConstraintCount: uint32;
                 sliderConstraints: ptr RagdollSliderConstraintData;
                 sliderConstraintCount: uint32;
                 swingTwistConstraints: ptr RagdollSwingTwistConstraintData;
                 swingTwistConstraintCount: uint32;
                 sixDOFConstraints: ptr RagdollSixDOFConstraintData;
                 sixDOFConstraintCount: uint32;
                 coneConstraints: ptr RagdollConeConstraintData;
                 coneConstraintCount, groupId: uint32;
                 disableParentChildCollisions, stabilize,
                 calculatePriorities, activate: bool): ptr RagdollHandle
  {.importcpp: "joltnim_detail::CreateRagdoll(@)", header: joltBindingsHeader.}
proc delete*(self: ptr RagdollHandle)
  {.importcpp: "joltnim_detail::DestroyRagdoll(@)", header: joltBindingsHeader.}
proc partCount*(self: ptr RagdollHandle): uint32
  {.importcpp: "joltnim_detail::GetRagdollPartCount(@)", noSideEffect,
    header: joltBindingsHeader.}
proc constraintCount*(self: ptr RagdollHandle): uint32
  {.importcpp: "joltnim_detail::GetRagdollConstraintCount(@)", noSideEffect,
    header: joltBindingsHeader.}
proc constraint*(self: ptr RagdollHandle; index: uint32): ptr Constraint
  {.importcpp: "joltnim_detail::GetRagdollConstraint(@)", noSideEffect,
    header: joltBindingsHeader.}
proc constraintBodyIndices*(self: ptr RagdollHandle; index: uint32;
                            body1, body2: ptr uint32): bool
  {.importcpp: "joltnim_detail::GetRagdollConstraintBodyIndices(@)",
    noSideEffect, header: joltBindingsHeader.}
proc bodyId*(self: ptr RagdollHandle; index: uint32): uint32
  {.importcpp: "joltnim_detail::GetRagdollBodyID(@)", noSideEffect,
    header: joltBindingsHeader.}
proc setPose*(self: ptr RagdollHandle; positions: ptr Vec3;
              rotations: ptr Quat)
  {.importcpp: "joltnim_detail::SetRagdollPose(@)", header: joltBindingsHeader.}
proc pose*(self: ptr RagdollHandle; positions: ptr Vec3;
           rotations: ptr Quat)
  {.importcpp: "joltnim_detail::GetRagdollPose(@)", noSideEffect,
    header: joltBindingsHeader.}
proc driveKinematic*(self: ptr RagdollHandle; positions: ptr Vec3;
                     rotations: ptr Quat; deltaTime: cfloat)
  {.importcpp: "joltnim_detail::DriveRagdollKinematic(@)",
    header: joltBindingsHeader.}
proc driveMotors*(self: ptr RagdollHandle; translations: ptr Vec3;
                  rotations: ptr Quat)
  {.importcpp: "joltnim_detail::DriveRagdollMotors(@)",
    header: joltBindingsHeader.}
proc driveMotorsWithVelocity*(self: ptr RagdollHandle;
                              previousTranslations: ptr Vec3;
                              previousRotations: ptr Quat;
                              translations: ptr Vec3; rotations: ptr Quat;
                              deltaTime: cfloat)
  {.importcpp: "joltnim_detail::DriveRagdollMotorsWithVelocity(@)",
    header: joltBindingsHeader.}
proc activate*(self: ptr RagdollHandle)
  {.importcpp: "joltnim_detail::ActivateRagdoll(@)", header: joltBindingsHeader.}
proc isActive*(self: ptr RagdollHandle): bool
  {.importcpp: "joltnim_detail::IsRagdollActive(@)", noSideEffect,
    header: joltBindingsHeader.}
proc setGroupId*(self: ptr RagdollHandle; groupId: uint32)
  {.importcpp: "joltnim_detail::SetRagdollGroupID(@)", header: joltBindingsHeader.}
proc setVelocity*(self: ptr RagdollHandle; linear, angular: Vec3)
  {.importcpp: "joltnim_detail::SetRagdollVelocity(@)", header: joltBindingsHeader.}
proc addLinearVelocity*(self: ptr RagdollHandle; velocity: Vec3)
  {.importcpp: "joltnim_detail::AddRagdollLinearVelocity(@)",
    header: joltBindingsHeader.}
proc addImpulse*(self: ptr RagdollHandle; impulse: Vec3)
  {.importcpp: "joltnim_detail::AddRagdollImpulse(@)", header: joltBindingsHeader.}
proc resetWarmStart*(self: ptr RagdollHandle)
  {.importcpp: "joltnim_detail::ResetRagdollWarmStart(@)",
    header: joltBindingsHeader.}

proc newSkeletonMapper*(sourceNames: ptr cstring; sourceParents: ptr int32;
                        sourceNeutralPositions: ptr Vec3;
                        sourceNeutralRotations: ptr Quat;
                        sourceCount: uint32; targetNames: ptr cstring;
                        targetParents: ptr int32;
                        targetNeutralPositions: ptr Vec3;
                        targetNeutralRotations: ptr Quat;
                        targetCount: uint32): ptr SkeletonMapperHandle
  {.importcpp: "joltnim_detail::CreateSkeletonMapper(@)",
    header: joltBindingsHeader.}
proc delete*(self: ptr SkeletonMapperHandle)
  {.importcpp: "joltnim_detail::DestroySkeletonMapper(@)",
    header: joltBindingsHeader.}
proc mappingCount*(self: ptr SkeletonMapperHandle): uint32
  {.importcpp: "joltnim_detail::GetSkeletonMapperMappingCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc chainCount*(self: ptr SkeletonMapperHandle): uint32
  {.importcpp: "joltnim_detail::GetSkeletonMapperChainCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc unmappedCount*(self: ptr SkeletonMapperHandle): uint32
  {.importcpp: "joltnim_detail::GetSkeletonMapperUnmappedCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc mappedJoint*(self: ptr SkeletonMapperHandle; sourceJoint: int32): int32
  {.importcpp: "joltnim_detail::GetSkeletonMapperMappedJoint(@)",
    noSideEffect, header: joltBindingsHeader.}
proc isTranslationLocked*(self: ptr SkeletonMapperHandle;
                          targetJoint: int32): bool
  {.importcpp: "joltnim_detail::IsSkeletonMapperTranslationLocked(@)",
    noSideEffect, header: joltBindingsHeader.}
proc lockTranslations*(self: ptr SkeletonMapperHandle; locked: ptr bool;
                       targetNeutralPositions: ptr Vec3;
                       targetNeutralRotations: ptr Quat)
  {.importcpp: "joltnim_detail::LockSkeletonMapperTranslations(@)",
    header: joltBindingsHeader.}
proc lockAllTranslations*(self: ptr SkeletonMapperHandle;
                          targetNeutralPositions: ptr Vec3;
                          targetNeutralRotations: ptr Quat)
  {.importcpp: "joltnim_detail::LockAllSkeletonMapperTranslations(@)",
    header: joltBindingsHeader.}
proc mapPose*(self: ptr SkeletonMapperHandle;
              sourceModelPositions: ptr Vec3;
              sourceModelRotations: ptr Quat;
              targetLocalPositions: ptr Vec3;
              targetLocalRotations: ptr Quat;
              targetModelPositions: ptr Vec3;
              targetModelRotations: ptr Quat)
  {.importcpp: "joltnim_detail::MapSkeletonPose(@)",
    header: joltBindingsHeader.}
proc reverseMapPose*(self: ptr SkeletonMapperHandle;
                     targetModelPositions: ptr Vec3;
                     targetModelRotations: ptr Quat;
                     sourceModelPositions: ptr Vec3;
                     sourceModelRotations: ptr Quat)
  {.importcpp: "joltnim_detail::ReverseMapSkeletonPose(@)",
    header: joltBindingsHeader.}
proc newSkeletalAnimation*(jointNames: ptr cstring; jointParents: ptr int32;
                           neutralPositions: ptr Vec3;
                           neutralRotations: ptr Quat; jointCount: uint32;
                           trackNames: ptr cstring; trackOffsets: ptr uint32;
                           trackCount: uint32; times: ptr cfloat;
                           translations: ptr Vec3; rotations: ptr Quat;
                           keyframeCount: uint32;
                           looping: bool): ptr SkeletalAnimationHandle
  {.importcpp: "joltnim_detail::CreateSkeletalAnimation(@)",
    header: joltBindingsHeader.}
proc restoreSkeletalAnimation*(jointNames: ptr cstring;
                               jointParents: ptr int32;
                               neutralPositions: ptr Vec3;
                               neutralRotations: ptr Quat;
                               jointCount: uint32; data: ptr uint8;
                               size: csize_t): ptr SkeletalAnimationHandle
  {.importcpp: "joltnim_detail::RestoreSkeletalAnimation(@)",
    header: joltBindingsHeader.}
proc serialize*(self: ptr SkeletalAnimationHandle): bool
  {.importcpp: "joltnim_detail::SerializeSkeletalAnimation(@)",
    header: joltBindingsHeader.}
proc serializedSize*(self: ptr SkeletalAnimationHandle): csize_t
  {.importcpp: "joltnim_detail::GetSkeletalAnimationSerializedSize(@)",
    noSideEffect, header: joltBindingsHeader.}
proc copySerializedData*(self: ptr SkeletalAnimationHandle; data: ptr uint8)
  {.importcpp: "joltnim_detail::CopySkeletalAnimationSerializedData(@)",
    noSideEffect, header: joltBindingsHeader.}
proc delete*(self: ptr SkeletalAnimationHandle)
  {.importcpp: "joltnim_detail::DestroySkeletalAnimation(@)",
    header: joltBindingsHeader.}
proc duration*(self: ptr SkeletalAnimationHandle): cfloat
  {.importcpp: "joltnim_detail::GetSkeletalAnimationDuration(@)",
    noSideEffect, header: joltBindingsHeader.}
proc trackCount*(self: ptr SkeletalAnimationHandle): uint32
  {.importcpp: "joltnim_detail::GetSkeletalAnimationTrackCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc keyframeCount*(self: ptr SkeletalAnimationHandle): uint32
  {.importcpp: "joltnim_detail::GetSkeletalAnimationKeyframeCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc isLooping*(self: ptr SkeletalAnimationHandle): bool
  {.importcpp: "joltnim_detail::IsSkeletalAnimationLooping(@)",
    noSideEffect, header: joltBindingsHeader.}
proc setLooping*(self: ptr SkeletalAnimationHandle; looping: bool)
  {.importcpp: "joltnim_detail::SetSkeletalAnimationLooping(@)",
    header: joltBindingsHeader.}
proc scaleJoints*(self: ptr SkeletalAnimationHandle; scale: cfloat)
  {.importcpp: "joltnim_detail::ScaleSkeletalAnimationJoints(@)",
    header: joltBindingsHeader.}
proc sample*(self: ptr SkeletalAnimationHandle; time: cfloat;
             modelSpace: bool; positions: ptr Vec3; rotations: ptr Quat)
  {.importcpp: "joltnim_detail::SampleSkeletalAnimation(@)",
    noSideEffect, header: joltBindingsHeader.}
proc capturePhysicsScene*(system: ptr PhysicsSystem): ptr PhysicsSceneHandle
  {.importcpp: "joltnim_detail::CapturePhysicsScene(@)",
    header: joltBindingsHeader.}
proc newPhysicsScene*(): ptr PhysicsSceneHandle
  {.importcpp: "joltnim_detail::CreatePhysicsScene()",
    header: joltBindingsHeader.}
proc physicsScenePrimitiveBodySettings*(shapeKind: uint8;
                                        halfExtent: Vec3;
                                        halfHeight, radius, topRadius,
                                        bottomRadius, convexRadius: cfloat;
                                        points: ptr Vec3; pointCount: uint32;
                                        planeNormal: Vec3;
                                        planeConstant, planeHalfExtent: cfloat;
                                        centerOfMass: Vec3;
                                        material: ptr PhysicsMaterial;
                                        position: Vec3; rotation: Quat;
                                        motionType: EMotionType;
                                        objectLayer: ObjectLayer): BodyCreationSettings
  {.importcpp: "joltnim_detail::CreatePhysicsScenePrimitiveBodySettings(@)",
    header: joltBindingsHeader.}
proc physicsSceneMeshBodySettings*(vertices: ptr Vec3; vertexCount: uint32;
                                   indices: ptr uint32; triangleCount: uint32;
                                   materials: ptr ptr PhysicsMaterial;
                                   materialCount: uint32;
                                   materialIndices: ptr uint32;
                                   position: Vec3; rotation: Quat;
                                   motionType: EMotionType;
                                   objectLayer: ObjectLayer): BodyCreationSettings
  {.importcpp: "joltnim_detail::CreatePhysicsSceneMeshBodySettings(@)",
    header: joltBindingsHeader.}
proc physicsSceneHeightFieldBodySettings*(
    samples: ptr cfloat; sampleCount: uint32;
    offset, scale: Vec3; blockSize, bitsPerSample: uint32;
    materialIndices: ptr uint8;
    materials: ptr ptr PhysicsMaterial; materialCount: uint32;
    position: Vec3; rotation: Quat; motionType: EMotionType;
    objectLayer: ObjectLayer): BodyCreationSettings
  {.importcpp: "joltnim_detail::CreatePhysicsSceneHeightFieldBodySettings(@)",
    header: joltBindingsHeader.}
proc newPhysicsSceneCompoundSettings*(mutable: bool): ptr AuthoredCompoundSettingsHandle
  {.importcpp: "joltnim_detail::CreatePhysicsSceneCompoundSettings(@)",
    header: joltBindingsHeader.}
proc delete*(self: ptr AuthoredCompoundSettingsHandle)
  {.importcpp: "joltnim_detail::DestroyPhysicsSceneCompoundSettings(@)",
    header: joltBindingsHeader.}
proc addChild*(self: ptr AuthoredCompoundSettingsHandle;
               child: BodyCreationSettings; position: Vec3;
               rotation: Quat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneCompoundChild(@)",
    header: joltBindingsHeader.}
proc physicsSceneCompoundBodySettings*(
    compound: ptr AuthoredCompoundSettingsHandle;
    position: Vec3; rotation: Quat; motionType: EMotionType;
    objectLayer: ObjectLayer): BodyCreationSettings
  {.importcpp: "joltnim_detail::CreatePhysicsSceneCompoundBodySettings(@)",
    header: joltBindingsHeader.}
proc physicsSceneDecoratedBodySettings*(
    child: BodyCreationSettings; decoratorKind: uint8;
    scale, shapePosition: Vec3; shapeRotation: Quat;
    centerOfMassOffset, position: Vec3; rotation: Quat;
    motionType: EMotionType;
    objectLayer: ObjectLayer): BodyCreationSettings
  {.importcpp: "joltnim_detail::CreatePhysicsSceneDecoratedBodySettings(@)",
    header: joltBindingsHeader.}
proc addPointConstraint*(self: ptr PhysicsSceneHandle;
                         body1, body2: uint32;
                         point1, point2: Vec3): bool
  {.importcpp: "joltnim_detail::AddPhysicsScenePointConstraint(@)",
    header: joltBindingsHeader.}
proc addDistanceConstraint*(self: ptr PhysicsSceneHandle;
                            body1, body2: uint32;
                            point1, point2: Vec3;
                            minDistance, maxDistance: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneDistanceConstraint(@)",
    header: joltBindingsHeader.}
proc addFixedConstraint*(self: ptr PhysicsSceneHandle;
                         body1, body2: uint32): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneFixedConstraint(@)",
    header: joltBindingsHeader.}
proc addFixedConstraint*(self: ptr PhysicsSceneHandle;
                         body1, body2: uint32;
                         point1, point2, axisX1, axisY1,
                         axisX2, axisY2: Vec3): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneFixedConstraint(@)",
    header: joltBindingsHeader.}
proc addHingeConstraint*(self: ptr PhysicsSceneHandle;
                         body1, body2: uint32;
                         point1, point2, axis1, axis2: Vec3;
                         minAngle, maxAngle: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneHingeConstraint(@)",
    header: joltBindingsHeader.}
proc addSliderConstraint*(self: ptr PhysicsSceneHandle;
                          body1, body2: uint32;
                          point1, point2, axis: Vec3;
                          minPosition, maxPosition: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneSliderConstraint(@)",
    header: joltBindingsHeader.}
proc addConeConstraint*(self: ptr PhysicsSceneHandle;
                        body1, body2: uint32;
                        point1, point2, axis1, axis2: Vec3;
                        halfConeAngle: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneConeConstraint(@)",
    header: joltBindingsHeader.}
proc addSwingTwistConstraint*(self: ptr PhysicsSceneHandle;
                              body1, body2: uint32;
                              point1, point2, twistAxis,
                              planeAxis: Vec3;
                              normalHalfConeAngle,
                              planeHalfConeAngle,
                              twistMinAngle,
                              twistMaxAngle: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneSwingTwistConstraint(@)",
    header: joltBindingsHeader.}
proc addSixDOFConstraint*(self: ptr PhysicsSceneHandle;
                          body1, body2: uint32;
                          point1, point2, axisX, axisY: Vec3;
                          swingType: uint8;
                          limitMin, limitMax: ptr cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneSixDOFConstraint(@)",
    header: joltBindingsHeader.}
proc addGearConstraint*(self: ptr PhysicsSceneHandle;
                        body1, body2: uint32;
                        axis1, axis2: Vec3; ratio: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneGearConstraint(@)",
    header: joltBindingsHeader.}
proc addPulleyConstraint*(self: ptr PhysicsSceneHandle;
                          body1, body2: uint32;
                          bodyPoint1, fixedPoint1,
                          bodyPoint2, fixedPoint2: Vec3;
                          ratio, minLength, maxLength: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsScenePulleyConstraint(@)",
    header: joltBindingsHeader.}
proc addRackAndPinionConstraint*(self: ptr PhysicsSceneHandle;
                                 pinionBody, rackBody: uint32;
                                 hingeAxis, sliderAxis: Vec3;
                                 ratio: cfloat): bool
  {.importcpp: "joltnim_detail::AddPhysicsSceneRackAndPinionConstraint(@)",
    header: joltBindingsHeader.}
proc addPathConstraint*(self: ptr PhysicsSceneHandle;
                        pathBody, movingBody: uint32;
                        positions, tangents, normals: ptr Vec3;
                        pointCount: uint32; looping: bool;
                        pathPosition: Vec3; pathRotation: Quat;
                        pathFraction, maxFrictionForce: cfloat;
                        rotationConstraintType: uint8): bool
  {.importcpp: "joltnim_detail::AddPhysicsScenePathConstraint(@)",
    header: joltBindingsHeader.}
proc configureConstraint*(self: ptr PhysicsSceneHandle;
                          constraintIndex: uint32;
                          enabled: bool; priority: uint32;
                          velocityStepsOverride,
                          positionStepsOverride: uint32;
                          drawConstraintSize: cfloat;
                          userData: uint64): bool
  {.importcpp: "joltnim_detail::ConfigurePhysicsSceneConstraint(@)",
    header: joltBindingsHeader.}
proc configureHingeTuning*(self: ptr PhysicsSceneHandle;
                           constraintIndex: uint32;
                           maxFrictionTorque: cfloat;
                           springMode: uint8;
                           springValue, springDamping: cfloat): bool
  {.importcpp: "joltnim_detail::ConfigurePhysicsSceneHingeTuning(@)",
    header: joltBindingsHeader.}
proc configureDistanceSpring*(self: ptr PhysicsSceneHandle;
                              constraintIndex: uint32;
                              springMode: uint8;
                              springValue, springDamping: cfloat): bool
  {.importcpp: "joltnim_detail::ConfigurePhysicsSceneDistanceSpring(@)",
    header: joltBindingsHeader.}
proc configureSliderTuning*(self: ptr PhysicsSceneHandle;
                            constraintIndex: uint32;
                            maxFrictionForce: cfloat;
                            springMode: uint8;
                            springValue, springDamping: cfloat): bool
  {.importcpp: "joltnim_detail::ConfigurePhysicsSceneSliderTuning(@)",
    header: joltBindingsHeader.}
proc setSwingTwistFriction*(self: ptr PhysicsSceneHandle;
                            constraintIndex: uint32;
                            maxFrictionTorque: cfloat): bool
  {.importcpp: "joltnim_detail::SetPhysicsSceneSwingTwistFriction(@)",
    header: joltBindingsHeader.}
proc setSixDOFFriction*(self: ptr PhysicsSceneHandle;
                        constraintIndex: uint32; axis: uint8;
                        maxFriction: cfloat): bool
  {.importcpp: "joltnim_detail::SetPhysicsSceneSixDOFFriction(@)",
    header: joltBindingsHeader.}
proc setSixDOFTranslationSpring*(self: ptr PhysicsSceneHandle;
                                 constraintIndex: uint32;
                                 axis, springMode: uint8;
                                 springValue,
                                 springDamping: cfloat): bool
  {.importcpp: "joltnim_detail::SetPhysicsSceneSixDOFTranslationSpring(@)",
    header: joltBindingsHeader.}
proc configureMotor*(self: ptr PhysicsSceneHandle;
                     constraintIndex: uint32; motorKind, axis,
                     springMode: uint8; springValue,
                     springDamping, minForce, maxForce,
                     minTorque, maxTorque: cfloat): bool
  {.importcpp: "joltnim_detail::ConfigurePhysicsSceneMotor(@)",
    header: joltBindingsHeader.}
proc addBody*(self: ptr PhysicsSceneHandle;
              settings: BodyCreationSettings): uint32
  {.importcpp: "joltnim_detail::AddPhysicsSceneBody(@)",
    header: joltBindingsHeader.}
proc restorePhysicsScene*(data: ptr uint8;
                          size: csize_t): ptr PhysicsSceneHandle
  {.importcpp: "joltnim_detail::RestorePhysicsScene(@)",
    header: joltBindingsHeader.}
proc restorePhysicsSceneObjectStream*(data: ptr uint8;
                                      size: csize_t): ptr PhysicsSceneHandle
  {.importcpp: "joltnim_detail::RestorePhysicsSceneObjectStream(@)",
    header: joltBindingsHeader.}
proc delete*(self: ptr PhysicsSceneHandle)
  {.importcpp: "joltnim_detail::DestroyPhysicsScene(@)",
    header: joltBindingsHeader.}
proc bodyCount*(self: ptr PhysicsSceneHandle): uint32
  {.importcpp: "joltnim_detail::GetPhysicsSceneBodyCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc constraintCount*(self: ptr PhysicsSceneHandle): uint32
  {.importcpp: "joltnim_detail::GetPhysicsSceneConstraintCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc softBodyCount*(self: ptr PhysicsSceneHandle): uint32
  {.importcpp: "joltnim_detail::GetPhysicsSceneSoftBodyCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc fixInvalidScales*(self: ptr PhysicsSceneHandle): bool
  {.importcpp: "joltnim_detail::FixPhysicsSceneInvalidScales(@)",
    header: joltBindingsHeader.}
proc serialize*(self: ptr PhysicsSceneHandle): bool
  {.importcpp: "joltnim_detail::SerializePhysicsScene(@)",
    header: joltBindingsHeader.}
proc objectStreamSerializable*(self: ptr PhysicsSceneHandle): bool
  {.importcpp: "joltnim_detail::IsPhysicsSceneObjectStreamSerializable(@)",
    noSideEffect, header: joltBindingsHeader.}
proc serializeObjectStream*(self: ptr PhysicsSceneHandle; binary: bool): bool
  {.importcpp: "joltnim_detail::SerializePhysicsSceneObjectStream(@)",
    header: joltBindingsHeader.}
proc serializedSize*(self: ptr PhysicsSceneHandle): csize_t
  {.importcpp: "joltnim_detail::GetPhysicsSceneSerializedSize(@)",
    noSideEffect, header: joltBindingsHeader.}
proc copySerializedData*(self: ptr PhysicsSceneHandle; data: ptr uint8)
  {.importcpp: "joltnim_detail::CopyPhysicsSceneSerializedData(@)",
    noSideEffect, header: joltBindingsHeader.}
proc instantiate*(self: ptr PhysicsSceneHandle; system: ptr PhysicsSystem;
                  layerCount: uint32): ptr PhysicsSceneInstanceHandle
  {.importcpp: "joltnim_detail::InstantiatePhysicsScene(@)",
    header: joltBindingsHeader.}
proc delete*(self: ptr PhysicsSceneInstanceHandle)
  {.importcpp: "joltnim_detail::DestroyPhysicsSceneInstance(@)",
    header: joltBindingsHeader.}
proc abandon*(self: ptr PhysicsSceneInstanceHandle)
  {.importcpp: "joltnim_detail::AbandonPhysicsSceneInstance(@)",
    header: joltBindingsHeader.}
proc bodyCount*(self: ptr PhysicsSceneInstanceHandle): uint32
  {.importcpp: "joltnim_detail::GetPhysicsSceneInstanceBodyCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc constraintCount*(self: ptr PhysicsSceneInstanceHandle): uint32
  {.importcpp: "joltnim_detail::GetPhysicsSceneInstanceConstraintCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc bodyId*(self: ptr PhysicsSceneInstanceHandle; index: uint32): uint32
  {.importcpp: "joltnim_detail::GetPhysicsSceneInstanceBodyID(@)",
    noSideEffect, header: joltBindingsHeader.}
proc constraint*(self: ptr PhysicsSceneInstanceHandle;
                 index: uint32): ptr Constraint
  {.importcpp: "joltnim_detail::GetPhysicsSceneInstanceConstraint(@)",
    noSideEffect, header: joltBindingsHeader.}
proc isSoftBody*(self: ptr PhysicsSceneInstanceHandle; index: uint32): bool
  {.importcpp: "joltnim_detail::IsPhysicsSceneInstanceSoftBody(@)",
    noSideEffect, header: joltBindingsHeader.}
proc motionType*(self: ptr PhysicsSceneInstanceHandle; index: uint32): uint8
  {.importcpp: "joltnim_detail::GetPhysicsSceneInstanceMotionType(@)",
    noSideEffect, header: joltBindingsHeader.}
proc rootTransform*(self: ptr RagdollHandle; position: ptr Vec3;
                    rotation: ptr Quat)
  {.importcpp: "joltnim_detail::GetRagdollRootTransform(@)", noSideEffect,
    header: joltBindingsHeader.}

proc newCharacter*(system: ptr PhysicsSystem; shape: ptr Shape;
                   position: Vec3; rotation: Quat; layer: ObjectLayer;
                   centerOffset, supportingHeight, maxSlopeAngle, mass,
                   maxStrength, padding, predictiveContactDistance: cfloat;
                   maxNumHits: uint32; hitReductionCosMaxAngle,
                   penetrationRecoverySpeed: cfloat;
                   enhancedInternalEdgeRemoval: bool; backFaceMode: uint8;
                   maxCollisionIterations, maxConstraintIterations: uint32;
                   minTimeRemaining, collisionTolerance: cfloat;
                   userData: uint64; innerBodyShape: ptr Shape;
                   innerBodyLayer: ObjectLayer;
                   contactEventCapacity: uint32; canPushCharacter,
                   canReceiveImpulses, preventSliding: bool): ptr CharacterHandle
  {.importcpp: "joltnim_detail::CreateCharacter(@)", header: joltBindingsHeader.}
proc newCharacter*(system: ptr PhysicsSystem; shape: ptr Shape;
                   position: Vec3; rotation: Quat; layer: ObjectLayer;
                   centerOffset, supportingHeight, maxSlopeAngle, mass,
                   maxStrength, padding, predictiveContactDistance: cfloat;
                   maxNumHits: uint32; hitReductionCosMaxAngle,
                   penetrationRecoverySpeed: cfloat;
                   enhancedInternalEdgeRemoval: bool; backFaceMode: uint8;
                   maxCollisionIterations, maxConstraintIterations: uint32;
                   minTimeRemaining, collisionTolerance: cfloat;
                   userData: uint64; innerBodyShape: ptr Shape;
                   innerBodyLayer: ObjectLayer;
                   contactEventCapacity: uint32; canPushCharacter,
                   canReceiveImpulses, preventSliding: bool;
                   broadPhase: ptr CharacterBroadPhase): ptr CharacterHandle
  {.importcpp: "joltnim_detail::CreateCharacter(@)", header: joltBindingsHeader.}
proc delete*(self: ptr CharacterHandle)
  {.importcpp: "joltnim_detail::DestroyCharacter(@)", header: joltBindingsHeader.}
proc newCharacterBroadPhase*(cellSize: cfloat): ptr CharacterBroadPhase
  {.importcpp: "joltnim_detail::CreateCharacterBroadPhase(@)",
    header: joltBindingsHeader.}
proc delete*(self: ptr CharacterBroadPhase)
  {.importcpp: "joltnim_detail::DestroyCharacterBroadPhase(@)",
    header: joltBindingsHeader.}
proc stats*(self: ptr CharacterBroadPhase; characterCount,
            occupiedCellCount: ptr uint32; queryCount, candidateCount,
            narrowPhaseTestCount: ptr uint64)
  {.importcpp: "joltnim_detail::GetCharacterBroadPhaseStats(@)",
    noSideEffect, header: joltBindingsHeader.}
proc resetStats*(self: ptr CharacterBroadPhase)
  {.importcpp: "joltnim_detail::ResetCharacterBroadPhaseStats(@)",
    header: joltBindingsHeader.}
proc update*(self: ptr CharacterHandle; deltaTime: cfloat; gravity, stepUp,
             stepDown: Vec3; allocator: ptr TempAllocatorImpl)
  {.importcpp: "joltnim_detail::UpdateCharacter(@)", header: joltBindingsHeader.}
proc refreshContacts*(self: ptr CharacterHandle; allocator: ptr TempAllocatorImpl)
  {.importcpp: "joltnim_detail::RefreshCharacterContacts(@)", header: joltBindingsHeader.}
proc position*(self: ptr CharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetCharacterPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc setPosition*(self: ptr CharacterHandle; position: Vec3)
  {.importcpp: "joltnim_detail::SetCharacterPosition(@)", header: joltBindingsHeader.}
proc rotation*(self: ptr CharacterHandle): Quat
  {.importcpp: "joltnim_detail::GetCharacterRotation(@)", noSideEffect, header: joltBindingsHeader.}
proc setRotation*(self: ptr CharacterHandle; rotation: Quat)
  {.importcpp: "joltnim_detail::SetCharacterRotation(@)", header: joltBindingsHeader.}
proc linearVelocity*(self: ptr CharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetCharacterVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc setLinearVelocity*(self: ptr CharacterHandle; velocity: Vec3)
  {.importcpp: "joltnim_detail::SetCharacterVelocity(@)", header: joltBindingsHeader.}
proc groundState*(self: ptr CharacterHandle): uint8
  {.importcpp: "joltnim_detail::GetCharacterGroundState(@)", noSideEffect, header: joltBindingsHeader.}
proc isSupported*(self: ptr CharacterHandle): bool
  {.importcpp: "joltnim_detail::IsCharacterSupported(@)", noSideEffect, header: joltBindingsHeader.}
proc groundPosition*(self: ptr CharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetCharacterGroundPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc groundNormal*(self: ptr CharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetCharacterGroundNormal(@)", noSideEffect, header: joltBindingsHeader.}
proc groundVelocity*(self: ptr CharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetCharacterGroundVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc groundBodyID*(self: ptr CharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetCharacterGroundBodyID(@)", noSideEffect, header: joltBindingsHeader.}
proc activeContactCount*(self: ptr CharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetCharacterActiveContactCount(@)", noSideEffect, header: joltBindingsHeader.}
proc updateGroundVelocity*(self: ptr CharacterHandle)
  {.importcpp: "joltnim_detail::UpdateCharacterGroundVelocity(@)", header: joltBindingsHeader.}
proc maxNumHits*(self: ptr CharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetCharacterMaxNumHits(@)", noSideEffect, header: joltBindingsHeader.}
proc setMaxNumHits*(self: ptr CharacterHandle; value: uint32)
  {.importcpp: "joltnim_detail::SetCharacterMaxNumHits(@)", header: joltBindingsHeader.}
proc hitReductionCosMaxAngle*(self: ptr CharacterHandle): cfloat
  {.importcpp: "joltnim_detail::GetCharacterHitReductionCosMaxAngle(@)", noSideEffect, header: joltBindingsHeader.}
proc setHitReductionCosMaxAngle*(self: ptr CharacterHandle; value: cfloat)
  {.importcpp: "joltnim_detail::SetCharacterHitReductionCosMaxAngle(@)", header: joltBindingsHeader.}
proc penetrationRecoverySpeed*(self: ptr CharacterHandle): cfloat
  {.importcpp: "joltnim_detail::GetCharacterPenetrationRecoverySpeed(@)", noSideEffect, header: joltBindingsHeader.}
proc setPenetrationRecoverySpeed*(self: ptr CharacterHandle; value: cfloat)
  {.importcpp: "joltnim_detail::SetCharacterPenetrationRecoverySpeed(@)", header: joltBindingsHeader.}
proc maxHitsExceeded*(self: ptr CharacterHandle): bool
  {.importcpp: "joltnim_detail::GetCharacterMaxHitsExceeded(@)", noSideEffect, header: joltBindingsHeader.}
proc addPeer*(self, peer: ptr CharacterHandle)
  {.importcpp: "joltnim_detail::AddCharacterPeer(@)", header: joltBindingsHeader.}
proc removePeer*(self, peer: ptr CharacterHandle)
  {.importcpp: "joltnim_detail::RemoveCharacterPeer(@)", header: joltBindingsHeader.}
proc characterID*(self: ptr CharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetCharacterID(@)", noSideEffect, header: joltBindingsHeader.}
proc innerBodyID*(self: ptr CharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetCharacterInnerBodyID(@)", noSideEffect, header: joltBindingsHeader.}
proc hasCollidedWithBody*(self: ptr CharacterHandle; bodyId: uint32): bool
  {.importcpp: "joltnim_detail::CharacterHasCollidedWithBody(@)", noSideEffect, header: joltBindingsHeader.}
proc hasCollidedWithCharacter*(self, other: ptr CharacterHandle): bool
  {.importcpp: "joltnim_detail::CharacterHasCollidedWithCharacter(@)", noSideEffect, header: joltBindingsHeader.}
proc contactCount*(self: ptr CharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetCharacterContactCount(@)", noSideEffect, header: joltBindingsHeader.}
proc contact*(self: ptr CharacterHandle; index: uint32;
              bodyId, characterId, subShapeId: ptr uint32;
              position, linearVelocity, contactNormal,
              surfaceNormal: ptr Vec3; distance, fraction: ptr cfloat;
              motionType: ptr uint8; isSensor: ptr bool;
              userData: ptr uint64; hadCollision, wasDiscarded,
              canPushCharacter, isBackFacing: ptr bool)
  {.importcpp: "joltnim_detail::GetCharacterContact(@)", noSideEffect, header: joltBindingsHeader.}
proc cancelVelocityTowardsSteepSlopes*(self: ptr CharacterHandle;
                                       velocity: Vec3): Vec3
  {.importcpp: "joltnim_detail::CancelCharacterVelocityTowardsSteepSlopes(@)", noSideEffect, header: joltBindingsHeader.}
proc canWalkStairs*(self: ptr CharacterHandle; velocity: Vec3): bool
  {.importcpp: "joltnim_detail::CanCharacterWalkStairs(@)", noSideEffect, header: joltBindingsHeader.}
proc walkStairs*(self: ptr CharacterHandle; deltaTime: cfloat;
                 stepUp, stepForward, stepForwardTest,
                 stepDownExtra: Vec3; allocator: ptr TempAllocatorImpl): bool
  {.importcpp: "joltnim_detail::WalkCharacterStairs(@)", header: joltBindingsHeader.}
proc stickToFloor*(self: ptr CharacterHandle; stepDown: Vec3;
                   allocator: ptr TempAllocatorImpl): bool
  {.importcpp: "joltnim_detail::StickCharacterToFloor(@)", header: joltBindingsHeader.}
proc setShape*(self: ptr CharacterHandle; shape: ptr Shape;
               maxPenetrationDepth: cfloat;
               allocator: ptr TempAllocatorImpl): bool
  {.importcpp: "joltnim_detail::SetCharacterShape(@)", header: joltBindingsHeader.}
proc setInnerBodyShape*(self: ptr CharacterHandle; shape: ptr Shape)
  {.importcpp: "joltnim_detail::SetCharacterInnerBodyShape(@)", header: joltBindingsHeader.}
proc setShapeOffset*(self: ptr CharacterHandle; offset: Vec3)
  {.importcpp: "joltnim_detail::SetCharacterShapeOffset(@)", header: joltBindingsHeader.}
proc mass*(self: ptr CharacterHandle): cfloat
  {.importcpp: "joltnim_detail::GetCharacterMass(@)", noSideEffect, header: joltBindingsHeader.}
proc setMass*(self: ptr CharacterHandle; value: cfloat)
  {.importcpp: "joltnim_detail::SetCharacterMass(@)", header: joltBindingsHeader.}
proc maxStrength*(self: ptr CharacterHandle): cfloat
  {.importcpp: "joltnim_detail::GetCharacterMaxStrength(@)", noSideEffect, header: joltBindingsHeader.}
proc setMaxStrength*(self: ptr CharacterHandle; value: cfloat)
  {.importcpp: "joltnim_detail::SetCharacterMaxStrength(@)", header: joltBindingsHeader.}
proc enhancedInternalEdgeRemoval*(self: ptr CharacterHandle): bool
  {.importcpp: "joltnim_detail::GetCharacterEnhancedInternalEdgeRemoval(@)", noSideEffect, header: joltBindingsHeader.}
proc setEnhancedInternalEdgeRemoval*(self: ptr CharacterHandle; value: bool)
  {.importcpp: "joltnim_detail::SetCharacterEnhancedInternalEdgeRemoval(@)", header: joltBindingsHeader.}
proc userData*(self: ptr CharacterHandle): uint64
  {.importcpp: "joltnim_detail::GetCharacterUserData(@)", noSideEffect, header: joltBindingsHeader.}
proc setUserData*(self: ptr CharacterHandle; value: uint64)
  {.importcpp: "joltnim_detail::SetCharacterUserData(@)", header: joltBindingsHeader.}
proc pendingContactEventCount*(self: ptr CharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetPendingCharacterContactEventCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc popContactEvent*(self: ptr CharacterHandle; kind: ptr uint8;
                      bodyId, characterId, subShapeId: ptr uint32;
                      position, normal, contactVelocity, characterVelocity,
                      resultingVelocity: ptr Vec3; userData: ptr uint64;
                      isSensor, canPushCharacter,
                      canReceiveImpulses: ptr bool): bool
  {.importcpp: "joltnim_detail::PopCharacterContactEvent(@)",
    header: joltBindingsHeader.}
proc droppedContactEventCount*(self: ptr CharacterHandle;
                               reset: bool): uint64
  {.importcpp: "joltnim_detail::GetDroppedCharacterContactEventCount(@)",
    header: joltBindingsHeader.}
proc setContactResponse*(self: ptr CharacterHandle;
                         contactEventCapacity: uint32;
                         canPushCharacter, canReceiveImpulses,
                         preventSliding: bool)
  {.importcpp: "joltnim_detail::SetCharacterContactResponse(@)",
    header: joltBindingsHeader.}

proc newRigidCharacter*(system: ptr PhysicsSystem; shape: ptr Shape;
                        position: Vec3; rotation: Quat; layer: ObjectLayer;
                        up: Vec3; supportingHeight, maxSlopeAngle, mass,
                        friction, gravityFactor: cfloat; allowedDOFs: uint8;
                        enhancedInternalEdgeRemoval: bool; userData: uint64;
                        maxSeparationDistance: cfloat;
                        activate: bool): ptr RigidCharacterHandle
  {.importcpp: "joltnim_detail::CreateRigidCharacter(@)", header: joltBindingsHeader.}
proc delete*(self: ptr RigidCharacterHandle)
  {.importcpp: "joltnim_detail::DestroyRigidCharacter(@)", header: joltBindingsHeader.}
proc postSimulation*(self: ptr RigidCharacterHandle)
  {.importcpp: "joltnim_detail::PostSimulateRigidCharacter(@)", header: joltBindingsHeader.}
proc refreshGround*(self: ptr RigidCharacterHandle;
                    maxSeparationDistance: cfloat)
  {.importcpp: "joltnim_detail::RefreshRigidCharacter(@)", header: joltBindingsHeader.}
proc bodyID*(self: ptr RigidCharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetRigidCharacterBodyID(@)", noSideEffect, header: joltBindingsHeader.}
proc position*(self: ptr RigidCharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetRigidCharacterPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc setPosition*(self: ptr RigidCharacterHandle; position: Vec3;
                  activate: bool)
  {.importcpp: "joltnim_detail::SetRigidCharacterPosition(@)", header: joltBindingsHeader.}
proc rotation*(self: ptr RigidCharacterHandle): Quat
  {.importcpp: "joltnim_detail::GetRigidCharacterRotation(@)", noSideEffect, header: joltBindingsHeader.}
proc setRotation*(self: ptr RigidCharacterHandle; rotation: Quat;
                  activate: bool)
  {.importcpp: "joltnim_detail::SetRigidCharacterRotation(@)", header: joltBindingsHeader.}
proc centerOfMassPosition*(self: ptr RigidCharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetRigidCharacterCenterOfMassPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc linearVelocity*(self: ptr RigidCharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetRigidCharacterLinearVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc setLinearVelocity*(self: ptr RigidCharacterHandle; velocity: Vec3)
  {.importcpp: "joltnim_detail::SetRigidCharacterLinearVelocity(@)", header: joltBindingsHeader.}
proc addLinearVelocity*(self: ptr RigidCharacterHandle; velocity: Vec3)
  {.importcpp: "joltnim_detail::AddRigidCharacterLinearVelocity(@)", header: joltBindingsHeader.}
proc addImpulse*(self: ptr RigidCharacterHandle; impulse: Vec3)
  {.importcpp: "joltnim_detail::AddRigidCharacterImpulse(@)", header: joltBindingsHeader.}
proc activate*(self: ptr RigidCharacterHandle)
  {.importcpp: "joltnim_detail::ActivateRigidCharacter(@)", header: joltBindingsHeader.}
proc collisionLayer*(self: ptr RigidCharacterHandle): ObjectLayer
  {.importcpp: "joltnim_detail::GetRigidCharacterLayer(@)", noSideEffect, header: joltBindingsHeader.}
proc setCollisionLayer*(self: ptr RigidCharacterHandle; layer: ObjectLayer)
  {.importcpp: "joltnim_detail::SetRigidCharacterLayer(@)", header: joltBindingsHeader.}
proc setShape*(self: ptr RigidCharacterHandle; shape: ptr Shape;
               maxPenetrationDepth: cfloat): bool
  {.importcpp: "joltnim_detail::SetRigidCharacterShape(@)", header: joltBindingsHeader.}
proc groundState*(self: ptr RigidCharacterHandle): uint8
  {.importcpp: "joltnim_detail::GetRigidCharacterGroundState(@)", noSideEffect, header: joltBindingsHeader.}
proc isSupported*(self: ptr RigidCharacterHandle): bool
  {.importcpp: "joltnim_detail::IsRigidCharacterSupported(@)", noSideEffect, header: joltBindingsHeader.}
proc groundPosition*(self: ptr RigidCharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetRigidCharacterGroundPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc groundNormal*(self: ptr RigidCharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetRigidCharacterGroundNormal(@)", noSideEffect, header: joltBindingsHeader.}
proc groundVelocity*(self: ptr RigidCharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetRigidCharacterGroundVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc groundBodyID*(self: ptr RigidCharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetRigidCharacterGroundBodyID(@)", noSideEffect, header: joltBindingsHeader.}
proc groundSubShapeID*(self: ptr RigidCharacterHandle): uint32
  {.importcpp: "joltnim_detail::GetRigidCharacterGroundSubShapeID(@)", noSideEffect, header: joltBindingsHeader.}
proc groundUserData*(self: ptr RigidCharacterHandle): uint64
  {.importcpp: "joltnim_detail::GetRigidCharacterGroundUserData(@)", noSideEffect, header: joltBindingsHeader.}
proc maxSlopeAngle*(self: ptr RigidCharacterHandle): cfloat
  {.importcpp: "joltnim_detail::GetRigidCharacterMaxSlopeAngle(@)", noSideEffect, header: joltBindingsHeader.}
proc setMaxSlopeAngle*(self: ptr RigidCharacterHandle; angle: cfloat)
  {.importcpp: "joltnim_detail::SetRigidCharacterMaxSlopeAngle(@)", header: joltBindingsHeader.}
proc up*(self: ptr RigidCharacterHandle): Vec3
  {.importcpp: "joltnim_detail::GetRigidCharacterUp(@)", noSideEffect, header: joltBindingsHeader.}
proc setUp*(self: ptr RigidCharacterHandle; up: Vec3)
  {.importcpp: "joltnim_detail::SetRigidCharacterUp(@)", header: joltBindingsHeader.}
proc supportingHeight*(self: ptr RigidCharacterHandle): cfloat
  {.importcpp: "joltnim_detail::GetRigidCharacterSupportingHeight(@)", noSideEffect, header: joltBindingsHeader.}
proc setSupportingHeight*(self: ptr RigidCharacterHandle; height: cfloat)
  {.importcpp: "joltnim_detail::SetRigidCharacterSupportingHeight(@)", header: joltBindingsHeader.}
proc isSlopeTooSteep*(self: ptr RigidCharacterHandle; normal: Vec3): bool
  {.importcpp: "joltnim_detail::IsRigidCharacterSlopeTooSteep(@)", noSideEffect, header: joltBindingsHeader.}

proc vehicleWheelConfigData*(position, suspensionForcePoint,
                             suspensionDirection, steeringAxis,
                             wheelUp, wheelForward: Vec3;
                             suspensionMinLength, suspensionMaxLength,
                             suspensionPreloadLength, suspensionFrequency,
                             suspensionDamping, radius, width: cfloat;
                             enableSuspensionForcePoint: bool;
                             inertia, angularDamping, maxSteerAngle,
                             maxBrakeTorque, maxHandBrakeTorque,
                             longitudinalImpulseMultiplier,
                             lateralImpulseMultiplier: cfloat;
                             longitudinalFrictionSlips,
                             longitudinalFrictionValues: ptr cfloat;
                             longitudinalFrictionCount: uint32;
                             lateralFrictionSlips,
                             lateralFrictionValues: ptr cfloat;
                             lateralFrictionCount: uint32): VehicleWheelConfigData
  {.importcpp: "joltnim_detail::MakeVehicleWheelConfigData(@)", header: joltBindingsHeader.}
proc vehicleDifferentialConfigData*(leftWheel, rightWheel: int32;
                                    differentialRatio, leftRightSplit,
                                    limitedSlipRatio,
                                    engineTorqueRatio: cfloat): VehicleDifferentialConfigData
  {.importcpp: "joltnim_detail::MakeVehicleDifferentialConfigData(@)", header: joltBindingsHeader.}
proc vehicleAntiRollBarConfigData*(leftWheel, rightWheel: int32;
                                   stiffness: cfloat): VehicleAntiRollBarConfigData
  {.importcpp: "joltnim_detail::MakeVehicleAntiRollBarConfigData(@)", header: joltBindingsHeader.}
proc trackedVehicleWheelConfigData*(
    position, suspensionForcePoint, suspensionDirection, steeringAxis,
    wheelUp, wheelForward: Vec3;
    suspensionMinLength, suspensionMaxLength, suspensionPreloadLength,
    suspensionFrequency, suspensionDamping, radius, width: cfloat;
    enableSuspensionForcePoint: bool;
    longitudinalFriction,
    lateralFriction: cfloat): TrackedVehicleWheelConfigData
  {.importcpp: "joltnim_detail::MakeTrackedVehicleWheelConfigData(@)", header: joltBindingsHeader.}

proc newVehicle*(system: ptr PhysicsSystem; bodyId: BodyID;
                 halfWidth, halfHeight, halfLength, wheelRadius, wheelWidth,
                 suspensionMinLength, suspensionMaxLength,
                 suspensionFrequency, suspensionDamping, maxSteerAngle,
                 maxPitchRollAngle, engineMaxTorque: cfloat;
                 engineMinRPM, engineMaxRPM, engineInertia,
                 engineAngularDamping: cfloat;
                 torqueCurveRPMFractions,
                 torqueCurveFractions: ptr cfloat;
                 torqueCurveCount: uint32;
                 transmissionMode: uint8;
                 gearRatios: ptr cfloat; gearRatioCount: uint32;
                 reverseGearRatios: ptr cfloat;
                 reverseGearRatioCount: uint32;
                 transmissionSwitchTime, clutchReleaseTime,
                 transmissionSwitchLatency, shiftUpRPM, shiftDownRPM,
                 clutchStrength: cfloat;
                 fourWheelDrive, frontWheelDrive: bool;
                 frontTorqueRatio, differentialRatio,
                 differentialLeftRightSplit, differentialLimitedSlipRatio,
                 centerDifferentialLimitedSlipRatio, wheelTrack, frontAxleOffset,
                 rearAxleOffset, suspensionAttachmentHeightRatio,
                 rearMaxSteerAngle, frontBrakeTorque, rearBrakeTorque,
                 rearHandBrakeTorque, antiRollBarStiffness: cfloat;
                 wheelInertia, wheelAngularDamping,
                 tireLongitudinalImpulseMultiplier,
                 tireLateralImpulseMultiplier: cfloat;
                 customWheels: ptr VehicleWheelConfigData;
                 customWheelCount: uint32;
                 customDifferentials: ptr VehicleDifferentialConfigData;
                 customDifferentialCount: uint32;
                 customAntiRollBars: ptr VehicleAntiRollBarConfigData;
                 customAntiRollBarCount: uint32;
                 wheelCollisionMode: uint8;
                 wheelCollisionUp: Vec3;
                 wheelCollisionMaxSlopeAngle, wheelSphereCastRadius,
                 wheelCylinderConvexRadiusFraction: cfloat;
                 wheelCollisionLayer: ObjectLayer; controllerKind: uint8;
                 maxLeanAngle, leanSpringConstant, leanSpringDamping,
                 leanSpringIntegrationCoefficient,
                 leanSpringIntegrationCoefficientDecay,
                 leanSmoothingFactor: cfloat;
                 enableLeanController,
                 enableLeanSteeringLimit: bool): ptr VehicleHandle
  {.importcpp: "joltnim_detail::CreateVehicle(@)", header: joltBindingsHeader.}
proc newTrackedVehicle*(
    system: ptr PhysicsSystem; bodyId: BodyID;
    maxPitchRollAngle, engineMaxTorque, engineMinRPM, engineMaxRPM,
    engineInertia, engineAngularDamping: cfloat;
    torqueCurveRPMFractions, torqueCurveFractions: ptr cfloat;
    torqueCurveCount: uint32;
    transmissionMode: uint8;
    gearRatios: ptr cfloat; gearRatioCount: uint32;
    reverseGearRatios: ptr cfloat; reverseGearRatioCount: uint32;
    transmissionSwitchTime, clutchReleaseTime, transmissionSwitchLatency,
    shiftUpRPM, shiftDownRPM, clutchStrength: cfloat;
    wheels: ptr TrackedVehicleWheelConfigData; wheelCount: uint32;
    leftWheels: ptr uint32; leftWheelCount, leftDrivenWheel: uint32;
    leftInertia, leftAngularDamping, leftMaxBrakeTorque,
    leftDifferentialRatio: cfloat;
    rightWheels: ptr uint32; rightWheelCount, rightDrivenWheel: uint32;
    rightInertia, rightAngularDamping, rightMaxBrakeTorque,
    rightDifferentialRatio: cfloat;
    wheelCollisionMode: uint8; wheelCollisionUp: Vec3;
    wheelCollisionMaxSlopeAngle, wheelSphereCastRadius,
    wheelCylinderConvexRadiusFraction: cfloat;
    wheelCollisionLayer: ObjectLayer): ptr VehicleHandle
  {.importcpp: "joltnim_detail::CreateTrackedVehicle(@)", header: joltBindingsHeader.}
proc delete*(self: ptr VehicleHandle)
  {.importcpp: "joltnim_detail::DestroyVehicle(@)", header: joltBindingsHeader.}
proc setInput*(self: ptr VehicleHandle; forward, right, brake,
               handBrake: cfloat)
  {.importcpp: "joltnim_detail::SetVehicleInput(@)", header: joltBindingsHeader.}
proc motorcycleControllerState*(self: ptr VehicleHandle;
    wheelBase: ptr cfloat; leanControllerEnabled,
    leanSteeringLimitEnabled: ptr bool; leanSpringConstant,
    leanSpringDamping, leanSpringIntegrationCoefficient,
    leanSpringIntegrationCoefficientDecay,
    leanSmoothingFactor: ptr cfloat)
  {.importcpp: "joltnim_detail::GetMotorcycleControllerState(@)",
    noSideEffect, header: joltBindingsHeader.}
proc configureMotorcycleController*(self: ptr VehicleHandle;
    enableLeanController, enableLeanSteeringLimit: bool;
    leanSpringConstant, leanSpringDamping,
    leanSpringIntegrationCoefficient,
    leanSpringIntegrationCoefficientDecay,
    leanSmoothingFactor: cfloat)
  {.importcpp: "joltnim_detail::ConfigureMotorcycleController(@)",
    header: joltBindingsHeader.}
proc setTrackedInput*(self: ptr VehicleHandle; forward, leftRatio,
                      rightRatio, brake: cfloat)
  {.importcpp: "joltnim_detail::SetTrackedVehicleInput(@)", header: joltBindingsHeader.}
proc trackedPowertrainState*(self: ptr VehicleHandle;
                             engineRPM: ptr cfloat;
                             currentGear: ptr int32;
                             clutchFriction: ptr cfloat;
                             switchingGear: ptr bool;
                             transmissionRatio: ptr cfloat)
  {.importcpp: "joltnim_detail::GetTrackedVehiclePowertrainState(@)", noSideEffect, header: joltBindingsHeader.}
proc setTrackedEngineRPM*(self: ptr VehicleHandle; rpm: cfloat)
  {.importcpp: "joltnim_detail::SetTrackedVehicleEngineRPM(@)", header: joltBindingsHeader.}
proc setTrackedTransmission*(self: ptr VehicleHandle; gear: int32;
                             clutchFriction: cfloat)
  {.importcpp: "joltnim_detail::SetTrackedVehicleTransmission(@)", header: joltBindingsHeader.}
proc trackedTrackState*(self: ptr VehicleHandle; track: uint32;
                        drivenWheel: ptr uint32;
                        inertia, angularDamping, maxBrakeTorque,
                        differentialRatio,
                        angularVelocity: ptr cfloat)
  {.importcpp: "joltnim_detail::GetTrackedVehicleTrackState(@)", noSideEffect, header: joltBindingsHeader.}
proc trackedWheelDynamics*(self: ptr VehicleHandle; wheel: uint32;
                           combinedLongitudinalFriction,
                           combinedLateralFriction: ptr cfloat)
  {.importcpp: "joltnim_detail::GetTrackedVehicleWheelDynamics(@)", noSideEffect, header: joltBindingsHeader.}
proc powertrainState*(self: ptr VehicleHandle; engineRPM: ptr cfloat;
                      currentGear: ptr int32; clutchFriction: ptr cfloat;
                      switchingGear: ptr bool;
                      transmissionRatio,
                      wheelSpeedAtClutch: ptr cfloat)
  {.importcpp: "joltnim_detail::GetVehiclePowertrainState(@)", noSideEffect, header: joltBindingsHeader.}
proc setEngineRPM*(self: ptr VehicleHandle; rpm: cfloat)
  {.importcpp: "joltnim_detail::SetVehicleEngineRPM(@)", header: joltBindingsHeader.}
proc setTransmission*(self: ptr VehicleHandle; gear: int32;
                      clutchFriction: cfloat)
  {.importcpp: "joltnim_detail::SetVehicleTransmission(@)", header: joltBindingsHeader.}
proc differentialCount*(self: ptr VehicleHandle): uint32
  {.importcpp: "joltnim_detail::GetVehicleDifferentialCount(@)", noSideEffect, header: joltBindingsHeader.}
proc differentialState*(self: ptr VehicleHandle; differential: uint32;
                        leftWheel, rightWheel: ptr int32;
                        differentialRatio, leftRightSplit,
                        limitedSlipRatio,
                        engineTorqueRatio: ptr cfloat)
  {.importcpp: "joltnim_detail::GetVehicleDifferentialState(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelDynamics*(self: ptr VehicleHandle; wheel: uint32;
                    longitudinalSlip, lateralSlip,
                    combinedLongitudinalFriction,
                    combinedLateralFriction: ptr cfloat)
  {.importcpp: "joltnim_detail::GetVehicleWheelDynamics(@)", noSideEffect, header: joltBindingsHeader.}
proc antiRollBarCount*(self: ptr VehicleHandle): uint32
  {.importcpp: "joltnim_detail::GetVehicleAntiRollBarCount(@)", noSideEffect, header: joltBindingsHeader.}
proc antiRollBarState*(self: ptr VehicleHandle; antiRollBar: uint32;
                       leftWheel, rightWheel: ptr int32;
                       stiffness: ptr cfloat)
  {.importcpp: "joltnim_detail::GetVehicleAntiRollBarState(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelCount*(self: ptr VehicleHandle): uint32
  {.importcpp: "joltnim_detail::GetVehicleWheelCount(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelHasContact*(self: ptr VehicleHandle; wheel: uint32): bool
  {.importcpp: "joltnim_detail::VehicleWheelHasContact(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelSuspensionLength*(self: ptr VehicleHandle; wheel: uint32): cfloat
  {.importcpp: "joltnim_detail::GetVehicleWheelSuspensionLength(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelAngularVelocity*(self: ptr VehicleHandle; wheel: uint32): cfloat
  {.importcpp: "joltnim_detail::GetVehicleWheelAngularVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelSteerAngle*(self: ptr VehicleHandle; wheel: uint32): cfloat
  {.importcpp: "joltnim_detail::GetVehicleWheelSteerAngle(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelPosition*(self: ptr VehicleHandle; wheel: uint32): Vec3
  {.importcpp: "joltnim_detail::GetVehicleWheelPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelRotation*(self: ptr VehicleHandle; wheel: uint32): Quat
  {.importcpp: "joltnim_detail::GetVehicleWheelRotation(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelContactPosition*(self: ptr VehicleHandle; wheel: uint32): Vec3
  {.importcpp: "joltnim_detail::GetVehicleWheelContactPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelContactNormal*(self: ptr VehicleHandle; wheel: uint32): Vec3
  {.importcpp: "joltnim_detail::GetVehicleWheelContactNormal(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelContactBodyID*(self: ptr VehicleHandle; wheel: uint32): uint32
  {.importcpp: "joltnim_detail::GetVehicleWheelContactBodyID(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelConstraintState*(self: ptr VehicleHandle; wheel: uint32;
                           hitHardPoint: ptr bool;
                           suspensionImpulse, longitudinalImpulse,
                           lateralImpulse: ptr cfloat)
  {.importcpp: "joltnim_detail::GetVehicleWheelConstraintState(@)", noSideEffect, header: joltBindingsHeader.}
proc wheelContactDetails*(self: ptr VehicleHandle; wheel: uint32;
                          subShapeId: ptr uint32;
                          pointVelocity, longitudinal,
                          lateral: ptr Vec3)
  {.importcpp: "joltnim_detail::GetVehicleWheelContactDetails(@)", noSideEffect, header: joltBindingsHeader.}

proc bodyCreationSettings*(shape: ptr Shape; position: Vec3; rotation: Quat; motionType: EMotionType; objectLayer: ObjectLayer): BodyCreationSettings {.importcpp: "JPH::BodyCreationSettings(@)", constructor, header: joltBindingsHeader.}
proc setSensor*(self: var BodyCreationSettings; sensor: bool)
  {.importcpp: "joltnim_detail::SetBodyCreationSettingsSensor(@)", header: joltBindingsHeader.}
proc configure*(self: var BodyCreationSettings; allowedDOFs, motionQuality: uint8;
                mass, inertiaMultiplier: cfloat;
                linearVelocity, angularVelocity: Vec3; userData: uint64;
                allowSleeping, collideKinematicVsNonDynamic,
                useManifoldReduction, applyGyroscopicForce,
                enhancedInternalEdgeRemoval: bool;
                friction, restitution, linearDamping, angularDamping,
                maxLinearVelocity, maxAngularVelocity, gravityFactor: cfloat;
                numVelocityStepsOverride,
                numPositionStepsOverride: uint32;
                hasCustomMassProperties: bool;
                customMass: cfloat;
                customInertiaDiagonal: Vec3;
                customInertiaRotation: Quat)
  {.importcpp: "joltnim_detail::ConfigureBodyCreationSettings(@)", header: joltBindingsHeader.}
proc createBody*(self: ptr BodyInterface;
                 settings: BodyCreationSettings): ptr Body
  {.importcpp: "#->CreateBody(@)", header: joltBindingsHeader.}
proc id*(self: ptr Body): BodyID
  {.importcpp: "#->GetID()", noSideEffect, header: joltBindingsHeader.}
proc addBodies*(self: ptr BodyInterface; ids: ptr BodyID; count: uint32;
                activation: EActivation)
  {.importcpp: "joltnim_detail::AddBodies(@)", header: joltBindingsHeader.}
proc destroyBodies*(self: ptr BodyInterface; ids: ptr BodyID; count: uint32)
  {.importcpp: "joltnim_detail::DestroyBodies(@)", header: joltBindingsHeader.}
proc removeAndDestroyBodies*(self: ptr BodyInterface; ids: ptr BodyID;
                             count: uint32)
  {.importcpp: "joltnim_detail::RemoveAndDestroyBodies(@)",
    header: joltBindingsHeader.}
proc createAndAddBody*(self: ptr BodyInterface; settings: BodyCreationSettings; activation: EActivation): BodyID {.importcpp: "#->CreateAndAddBody(@)", header: joltBindingsHeader.}
proc createSoftBody*(self: ptr PhysicsSystem; positions, velocities: ptr Vec3;
                     inverseMasses: ptr cfloat; vertexCount: uint32;
                     attributeEdgeCompliances, attributeShearCompliances,
                     attributeBendCompliances: ptr cfloat;
                     attributeLRATypes: ptr uint8;
                     attributeLRAMultipliers: ptr cfloat;
                     vertexAttributeCount: uint32;
                     faceVertices, faceMaterialIndices: ptr uint32;
                     faceCount: uint32;
                     edgeVertices: ptr uint32; edgeCompliances: ptr cfloat;
                     edgeCount: uint32; dihedralVertices: ptr uint32;
                     dihedralCompliances: ptr cfloat; dihedralCount: uint32;
                     lraVertices: ptr uint32; lraMaxDistances: ptr cfloat;
                     lraCount: uint32;
                     volumeVertices: ptr uint32;
                     volumeCompliances: ptr cfloat; volumeCount: uint32;
                     rodVertices: ptr uint32; rodCompliances: ptr cfloat;
                     rodCount: uint32; rodPairs: ptr uint32;
                     rodPairCompliances: ptr cfloat; rodPairCount: uint32;
                     rodRemap, rodPairRemap: ptr uint32;
                     skinBindPositions: ptr Vec3;
                     skinBindRotations: ptr Quat; skinJointCount: uint32;
                     skinVertices, skinJointIndices: ptr uint32;
                     skinWeights, skinMaxDistances,
                     skinBackStopDistances, skinBackStopRadii: ptr cfloat;
                     skinConstraintCount: uint32;
                     position: Vec3; rotation: Quat; layer: ObjectLayer;
                     userData: uint64;
                     bendType, lraType: uint8;
                     lraMaxDistanceMultiplier, edgeCompliance, shearCompliance,
                     bendCompliance, angleTolerance: cfloat;
                     numIterations: uint32; linearDamping,
                     maxLinearVelocity, restitution, friction, pressure,
                     gravityFactor, vertexRadius: cfloat;
                     updatePosition, makeRotationIdentity, allowSleeping,
                     facesDoubleSided, enableSkinConstraints: bool;
                     skinnedMaxDistanceMultiplier: cfloat;
                     allocator: ptr TempAllocatorImpl;
                     materials: ptr ptr PhysicsMaterial;
                     materialCount: uint32): BodyID
  {.importcpp: "joltnim_detail::CreateSoftBody(@)", header: joltBindingsHeader.}
proc softBodyVertexCount*(self: ptr PhysicsSystem; id: BodyID): uint32
  {.importcpp: "joltnim_detail::GetSoftBodyVertexCount(@)", noSideEffect,
    header: joltBindingsHeader.}
proc softBodyVertexState*(self: ptr PhysicsSystem; id: BodyID; vertex: uint32;
                          position, velocity: ptr Vec3;
                          inverseMass: ptr cfloat): bool
  {.importcpp: "joltnim_detail::GetSoftBodyVertexState(@)", noSideEffect,
    header: joltBindingsHeader.}
proc setSoftBodyVertexVelocity*(self: ptr PhysicsSystem; id: BodyID;
                                vertex: uint32; velocity: Vec3): bool
  {.importcpp: "joltnim_detail::SetSoftBodyVertexVelocity(@)",
    header: joltBindingsHeader.}
proc setSoftBodyVertexInverseMass*(self: ptr PhysicsSystem; id: BodyID;
                                   vertex: uint32;
                                   inverseMass: cfloat): bool
  {.importcpp: "joltnim_detail::SetSoftBodyVertexInverseMass(@)",
    header: joltBindingsHeader.}
proc softBodyRuntimeSettings*(self: ptr PhysicsSystem; id: BodyID;
                              numIterations: ptr uint32;
                              pressure, vertexRadius, volume: ptr cfloat;
                              updatePosition,
                              facesDoubleSided,
                              enableSkinConstraints: ptr bool;
                              skinnedMaxDistanceMultiplier: ptr cfloat): bool
  {.importcpp: "joltnim_detail::GetSoftBodyRuntimeSettings(@)", noSideEffect,
    header: joltBindingsHeader.}
proc softBodyConstraintCounts*(self: ptr PhysicsSystem; id: BodyID;
                               edges, dihedralBends, volumes,
                               longRangeAttachments, rods, rodBendTwists,
                               skinned: ptr uint32): bool
  {.importcpp: "joltnim_detail::GetSoftBodyConstraintCounts(@)", noSideEffect,
    header: joltBindingsHeader.}
proc softBodyRodState*(self: ptr PhysicsSystem; id: BodyID; rod: uint32;
                       rotation: ptr Quat; angularVelocity: ptr Vec3): bool
  {.importcpp: "joltnim_detail::GetSoftBodyRodState(@)", noSideEffect,
    header: joltBindingsHeader.}
proc softBodyLocalBounds*(self: ptr PhysicsSystem; id: BodyID;
                          minimum, maximum: ptr Vec3): bool
  {.importcpp: "joltnim_detail::GetSoftBodyLocalBounds(@)", noSideEffect,
    header: joltBindingsHeader.}
proc customUpdateSoftBody*(self: ptr PhysicsSystem; id: BodyID;
                           deltaTime: cfloat): bool
  {.importcpp: "joltnim_detail::CustomUpdateSoftBody(@)",
    header: joltBindingsHeader.}
proc setSoftBodyRuntimeSettings*(self: ptr PhysicsSystem; id: BodyID;
                                 numIterations: uint32; pressure,
                                 vertexRadius: cfloat; updatePosition,
                                 facesDoubleSided,
                                 enableSkinConstraints: bool;
                                 skinnedMaxDistanceMultiplier: cfloat): bool
  {.importcpp: "joltnim_detail::SetSoftBodyRuntimeSettings(@)",
    header: joltBindingsHeader.}
proc skinSoftBodyVertices*(self: ptr PhysicsSystem; id: BodyID;
                           jointPositions: ptr Vec3;
                           jointRotations: ptr Quat; jointCount: uint32;
                           hardSkinAll: bool;
                           allocator: ptr TempAllocatorImpl): bool
  {.importcpp: "joltnim_detail::SkinSoftBodyVertices(@)",
    header: joltBindingsHeader.}
proc removeAndDestroyBody*(self: ptr BodyInterface; id: BodyID) {.importcpp: "joltnim_detail::RemoveAndDestroyBody(@)", header: joltBindingsHeader.}
proc isActive*(self: ptr BodyInterface; id: BodyID): bool {.importcpp: "#->IsActive(@)", noSideEffect, header: joltBindingsHeader.}
proc objectLayer*(self: ptr BodyInterface; id: BodyID): ObjectLayer {.importcpp: "#->GetObjectLayer(@)", noSideEffect, header: joltBindingsHeader.}
proc setObjectLayer*(self: ptr BodyInterface; id: BodyID; layer: ObjectLayer) {.importcpp: "#->SetObjectLayer(@)", header: joltBindingsHeader.}
proc position*(self: ptr BodyInterface; id: BodyID): Vec3 {.importcpp: "#->GetPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc centerOfMassPosition*(self: ptr BodyInterface; id: BodyID): Vec3 {.importcpp: "#->GetCenterOfMassPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc rotation*(self: ptr BodyInterface; id: BodyID): Quat {.importcpp: "#->GetRotation(@)", noSideEffect, header: joltBindingsHeader.}
proc linearVelocity*(self: ptr BodyInterface; id: BodyID): Vec3 {.importcpp: "#->GetLinearVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc setLinearVelocity*(self: ptr BodyInterface; id: BodyID; velocity: Vec3) {.importcpp: "#->SetLinearVelocity(@)", header: joltBindingsHeader.}
proc angularVelocity*(self: ptr BodyInterface; id: BodyID): Vec3 {.importcpp: "#->GetAngularVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc setAngularVelocity*(self: ptr BodyInterface; id: BodyID; velocity: Vec3) {.importcpp: "#->SetAngularVelocity(@)", header: joltBindingsHeader.}
proc linearAndAngularVelocity*(self: ptr BodyInterface; id: BodyID;
                               linearVelocity,
                               angularVelocity: ptr Vec3)
  {.importcpp: "joltnim_detail::GetBodyLinearAndAngularVelocity(@)",
    noSideEffect, header: joltBindingsHeader.}
proc setLinearAndAngularVelocity*(self: ptr BodyInterface; id: BodyID;
                                  linearVelocity, angularVelocity: Vec3)
  {.importcpp: "#->SetLinearAndAngularVelocity(@)",
    header: joltBindingsHeader.}
proc addLinearAndAngularVelocity*(self: ptr BodyInterface; id: BodyID;
                                  linearVelocity, angularVelocity: Vec3)
  {.importcpp: "#->AddLinearAndAngularVelocity(@)",
    header: joltBindingsHeader.}
proc pointVelocity*(self: ptr BodyInterface; id: BodyID; point: Vec3): Vec3 {.importcpp: "#->GetPointVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc setPositionAndRotation*(self: ptr BodyInterface; id: BodyID; position: Vec3; rotation: Quat; activation: EActivation) {.importcpp: "#->SetPositionAndRotation(@)", header: joltBindingsHeader.}
proc setPositionAndRotationWhenChanged*(self: ptr BodyInterface; id: BodyID;
                                        position: Vec3; rotation: Quat;
                                        activation: EActivation)
  {.importcpp: "#->SetPositionAndRotationWhenChanged(@)",
    header: joltBindingsHeader.}
proc setPositionRotationAndVelocity*(self: ptr BodyInterface; id: BodyID;
                                     position: Vec3; rotation: Quat;
                                     linearVelocity, angularVelocity: Vec3)
  {.importcpp: "#->SetPositionRotationAndVelocity(@)",
    header: joltBindingsHeader.}
proc moveKinematic*(self: ptr BodyInterface; id: BodyID; position: Vec3; rotation: Quat; deltaTime: cfloat) {.importcpp: "#->MoveKinematic(@)", header: joltBindingsHeader.}
proc addForce*(self: ptr BodyInterface; id: BodyID; force: Vec3; activation: EActivation) {.importcpp: "#->AddForce(@)", header: joltBindingsHeader.}
proc addForce*(self: ptr BodyInterface; id: BodyID; force, point: Vec3; activation: EActivation) {.importcpp: "#->AddForce(@)", header: joltBindingsHeader.}
proc addTorque*(self: ptr BodyInterface; id: BodyID; torque: Vec3; activation: EActivation) {.importcpp: "#->AddTorque(@)", header: joltBindingsHeader.}
proc addImpulse*(self: ptr BodyInterface; id: BodyID; impulse: Vec3) {.importcpp: "#->AddImpulse(@)", header: joltBindingsHeader.}
proc addImpulse*(self: ptr BodyInterface; id: BodyID; impulse, point: Vec3) {.importcpp: "#->AddImpulse(@)", header: joltBindingsHeader.}
proc addAngularImpulse*(self: ptr BodyInterface; id: BodyID; impulse: Vec3) {.importcpp: "#->AddAngularImpulse(@)", header: joltBindingsHeader.}
proc activate*(self: ptr BodyInterface; id: BodyID) {.importcpp: "#->ActivateBody(@)", header: joltBindingsHeader.}
proc deactivate*(self: ptr BodyInterface; id: BodyID) {.importcpp: "#->DeactivateBody(@)", header: joltBindingsHeader.}
proc resetSleepTimer*(self: ptr BodyInterface; id: BodyID) {.importcpp: "#->ResetSleepTimer(@)", header: joltBindingsHeader.}
proc motionQuality*(self: ptr BodyInterface; id: BodyID): uint8 {.importcpp: "joltnim_detail::GetBodyMotionQuality(@)", noSideEffect, header: joltBindingsHeader.}
proc setMotionQuality*(self: ptr BodyInterface; id: BodyID; quality: uint8) {.importcpp: "joltnim_detail::SetBodyMotionQuality(@)", header: joltBindingsHeader.}
proc friction*(self: ptr BodyInterface; id: BodyID): cfloat {.importcpp: "#->GetFriction(@)", noSideEffect, header: joltBindingsHeader.}
proc setFriction*(self: ptr BodyInterface; id: BodyID; friction: cfloat) {.importcpp: "#->SetFriction(@)", header: joltBindingsHeader.}
proc restitution*(self: ptr BodyInterface; id: BodyID): cfloat {.importcpp: "#->GetRestitution(@)", noSideEffect, header: joltBindingsHeader.}
proc setRestitution*(self: ptr BodyInterface; id: BodyID; restitution: cfloat) {.importcpp: "#->SetRestitution(@)", header: joltBindingsHeader.}
proc gravityFactor*(self: ptr BodyInterface; id: BodyID): cfloat {.importcpp: "#->GetGravityFactor(@)", noSideEffect, header: joltBindingsHeader.}
proc setGravityFactor*(self: ptr BodyInterface; id: BodyID; factor: cfloat) {.importcpp: "#->SetGravityFactor(@)", header: joltBindingsHeader.}
proc maxLinearVelocity*(self: ptr BodyInterface; id: BodyID): cfloat {.importcpp: "#->GetMaxLinearVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc setMaxLinearVelocity*(self: ptr BodyInterface; id: BodyID; velocity: cfloat) {.importcpp: "#->SetMaxLinearVelocity(@)", header: joltBindingsHeader.}
proc maxAngularVelocity*(self: ptr BodyInterface; id: BodyID): cfloat {.importcpp: "#->GetMaxAngularVelocity(@)", noSideEffect, header: joltBindingsHeader.}
proc setMaxAngularVelocity*(self: ptr BodyInterface; id: BodyID; velocity: cfloat) {.importcpp: "#->SetMaxAngularVelocity(@)", header: joltBindingsHeader.}
proc useManifoldReduction*(self: ptr BodyInterface; id: BodyID): bool {.importcpp: "#->GetUseManifoldReduction(@)", noSideEffect, header: joltBindingsHeader.}
proc setUseManifoldReduction*(self: ptr BodyInterface; id: BodyID; enabled: bool) {.importcpp: "#->SetUseManifoldReduction(@)", header: joltBindingsHeader.}
proc userData*(self: ptr BodyInterface; id: BodyID): uint64 {.importcpp: "#->GetUserData(@)", noSideEffect, header: joltBindingsHeader.}
proc setUserData*(self: ptr BodyInterface; id: BodyID; value: uint64) {.importcpp: "#->SetUserData(@)", header: joltBindingsHeader.}
proc invalidateContactCache*(self: ptr BodyInterface; id: BodyID) {.importcpp: "#->InvalidateContactCache(@)", header: joltBindingsHeader.}
proc setSensor*(self: ptr BodyInterface; id: BodyID; sensor: bool)
  {.importcpp: "#->SetIsSensor(@)", header: joltBindingsHeader.}
proc setShape*(self: ptr BodyInterface; id: BodyID; shape: ptr Shape;
               updateMassProperties: bool; activation: EActivation)
  {.importcpp: "#->SetShape(@)", header: joltBindingsHeader.}
proc applyBuoyancyImpulse*(self: ptr BodyInterface; id: BodyID;
                           surfacePosition, surfaceNormal: Vec3;
                           buoyancy, linearDrag, angularDrag: cfloat;
                           fluidVelocity, gravity: Vec3;
                           deltaTime: cfloat): bool
  {.importcpp: "#->ApplyBuoyancyImpulse(@)", header: joltBindingsHeader.}
proc bodyMass*(self: ptr PhysicsSystem; id: BodyID; mass: ptr cfloat): bool
  {.importcpp: "joltnim_detail::GetBodyMass(@)", noSideEffect, header: joltBindingsHeader.}
proc bodyMassProperties*(self: ptr PhysicsSystem; id: BodyID;
                         mass: ptr cfloat; inertiaDiagonal: ptr Vec3;
                         inertiaRotation: ptr Quat): bool
  {.importcpp: "joltnim_detail::GetBodyMassProperties(@)", noSideEffect,
    header: joltBindingsHeader.}
proc setBodyMassProperties*(self: ptr PhysicsSystem; id: BodyID;
                            mass: cfloat; inertiaDiagonal: Vec3;
                            inertiaRotation: Quat): bool
  {.importcpp: "joltnim_detail::SetBodyMassProperties(@)",
    header: joltBindingsHeader.}
proc readBodySnapshots*(self: ptr PhysicsSystem; bodyIDs: ptr uint32;
                        count: uint32; snapshots: ptr BodySnapshotData)
  {.importcpp: "joltnim_detail::ReadBodySnapshots(@)", noSideEffect,
    header: joltBindingsHeader.}
proc bodyAllowedDOFs*(self: ptr PhysicsSystem; id: BodyID;
                      allowedDOFs: ptr uint8): bool
  {.importcpp: "joltnim_detail::GetBodyAllowedDOFs(@)", noSideEffect, header: joltBindingsHeader.}
proc bodyCreationFlags*(self: ptr PhysicsSystem; id: BodyID;
                        allowSleeping, collideKinematicVsNonDynamic,
                        applyGyroscopicForce,
                        enhancedInternalEdgeRemoval: ptr bool): bool
  {.importcpp: "joltnim_detail::GetBodyCreationFlags(@)", noSideEffect, header: joltBindingsHeader.}
proc bodySolverStepOverrides*(self: ptr PhysicsSystem; id: BodyID;
                              velocitySteps, positionSteps: ptr uint32): bool
  {.importcpp: "joltnim_detail::GetBodySolverStepOverrides(@)", noSideEffect, header: joltBindingsHeader.}
proc setBodyMass*(self: ptr PhysicsSystem; id: BodyID; mass: cfloat): bool
  {.importcpp: "joltnim_detail::SetBodyMass(@)", header: joltBindingsHeader.}
proc setBodyCreationFlag*(self: ptr PhysicsSystem; id: BodyID;
                          flag: uint8; enabled: bool): bool
  {.importcpp: "joltnim_detail::SetBodyCreationFlag(@)", header: joltBindingsHeader.}
proc setBodySolverStepOverrides*(self: ptr PhysicsSystem; id: BodyID;
                                 velocitySteps, positionSteps: uint32): bool
  {.importcpp: "joltnim_detail::SetBodySolverStepOverrides(@)", header: joltBindingsHeader.}
proc addMutableCompoundShape*(self: ptr PhysicsSystem; id: BodyID;
                              position: Vec3; rotation: Quat;
                              childShape: ptr Shape; activate: bool;
                              index: ptr uint32): bool
  {.importcpp: "joltnim_detail::AddMutableCompoundShape(@)", header: joltBindingsHeader.}
proc removeMutableCompoundShape*(self: ptr PhysicsSystem; id: BodyID;
                                 index: uint32; activate: bool): bool
  {.importcpp: "joltnim_detail::RemoveMutableCompoundShape(@)", header: joltBindingsHeader.}
proc modifyMutableCompoundShape*(self: ptr PhysicsSystem; id: BodyID;
                                 index: uint32; position: Vec3;
                                 rotation: Quat; childShape: ptr Shape;
                                 replaceShape, activate: bool): bool
  {.importcpp: "joltnim_detail::ModifyMutableCompoundShape(@)", header: joltBindingsHeader.}
proc modifyMutableCompoundShapes*(self: ptr PhysicsSystem; id: BodyID;
                                  startIndex: uint32; positions: ptr Vec3;
                                  rotations: ptr Quat; count: uint32;
                                  activate: bool): bool
  {.importcpp: "joltnim_detail::ModifyMutableCompoundShapes(@)", header: joltBindingsHeader.}
proc setDamping*(self: ptr PhysicsSystem; id: BodyID; linear, angular: cfloat): bool
  {.importcpp: "joltnim_detail::SetBodyDamping(@)", header: joltBindingsHeader.}
proc damping*(self: ptr PhysicsSystem; id: BodyID; linear, angular: ptr cfloat): bool
  {.importcpp: "joltnim_detail::GetBodyDamping(@)", noSideEffect, header: joltBindingsHeader.}

proc createPointConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                            point1, point2: Vec3): ptr Constraint
  {.importcpp: "joltnim_detail::CreatePointConstraint(@)", header: joltBindingsHeader.}
proc createDistanceConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                               point1, point2: Vec3; minDistance, maxDistance: cfloat): ptr Constraint
  {.importcpp: "joltnim_detail::CreateDistanceConstraint(@)", header: joltBindingsHeader.}
proc createFixedConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID): ptr Constraint
  {.importcpp: "joltnim_detail::CreateFixedConstraint(@)", header: joltBindingsHeader.}
proc createFixedConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                            point1, point2, axisX1, axisY1,
                            axisX2, axisY2: Vec3): ptr Constraint
  {.importcpp: "joltnim_detail::CreateFixedConstraint(@)",
    header: joltBindingsHeader.}
proc createHingeConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                            point1, point2, axis: Vec3;
                            minAngle, maxAngle: cfloat): ptr Constraint
  {.importcpp: "joltnim_detail::CreateHingeConstraint(@)", header: joltBindingsHeader.}
proc createSliderConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                             point1, point2, axis: Vec3;
                             minPosition, maxPosition: cfloat): ptr Constraint
  {.importcpp: "joltnim_detail::CreateSliderConstraint(@)", header: joltBindingsHeader.}
proc createConeConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                           point1, point2, twistAxis1, twistAxis2: Vec3;
                           halfConeAngle: cfloat): ptr Constraint
  {.importcpp: "joltnim_detail::CreateConeConstraint(@)", header: joltBindingsHeader.}
proc createSwingTwistConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                                 point1, point2, twistAxis, planeAxis: Vec3;
                                 normalHalfConeAngle, planeHalfConeAngle,
                                 twistMinAngle, twistMaxAngle: cfloat): ptr Constraint
  {.importcpp: "joltnim_detail::CreateSwingTwistConstraint(@)", header: joltBindingsHeader.}
proc createSixDOFConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                             point1, point2, axisX, axisY: Vec3;
                             swingType: uint8;
                             limitMin, limitMax: ptr cfloat): ptr Constraint
  {.importcpp: "joltnim_detail::CreateSixDOFConstraint(@)", header: joltBindingsHeader.}
proc createGearConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                           axis1, axis2: Vec3; ratio: cfloat;
                           hinge1, hinge2: ptr Constraint): ptr Constraint
  {.importcpp: "joltnim_detail::CreateGearConstraint(@)", header: joltBindingsHeader.}
proc createPulleyConstraint*(self: ptr PhysicsSystem; body1, body2: BodyID;
                             bodyPoint1, fixedPoint1,
                             bodyPoint2, fixedPoint2: Vec3;
                             ratio, minLength,
                             maxLength: cfloat): ptr Constraint
  {.importcpp: "joltnim_detail::CreatePulleyConstraint(@)", header: joltBindingsHeader.}
proc createRackAndPinionConstraint*(self: ptr PhysicsSystem;
                                    pinionBody, rackBody: BodyID;
                                    hingeAxis, sliderAxis: Vec3;
                                    ratio: cfloat;
                                    hinge, slider: ptr Constraint): ptr Constraint
  {.importcpp: "joltnim_detail::CreateRackAndPinionConstraint(@)", header: joltBindingsHeader.}
proc createPathConstraint*(self: ptr PhysicsSystem;
                           pathBody, movingBody: BodyID;
                           positions, tangents, normals: ptr Vec3;
                           pointCount: uint32; looping: bool;
                           pathPosition: Vec3; pathRotation: Quat;
                           pathFraction, maxFrictionForce: cfloat;
                           rotationConstraintType: uint8): ptr Constraint
  {.importcpp: "joltnim_detail::CreatePathConstraint(@)", header: joltBindingsHeader.}
proc enabled*(constraint: ptr Constraint): bool
  {.importcpp: "#->GetEnabled()", noSideEffect, header: joltBindingsHeader.}
proc setEnabled*(constraint: ptr Constraint; enabled: bool)
  {.importcpp: "#->SetEnabled(@)", header: joltBindingsHeader.}
proc priority*(constraint: ptr Constraint): uint32
  {.importcpp: "#->GetConstraintPriority()", noSideEffect,
    header: joltBindingsHeader.}
proc setPriority*(constraint: ptr Constraint; priority: uint32)
  {.importcpp: "#->SetConstraintPriority(@)", header: joltBindingsHeader.}
proc velocityStepsOverride*(constraint: ptr Constraint): uint32
  {.importcpp: "#->GetNumVelocityStepsOverride()", noSideEffect,
    header: joltBindingsHeader.}
proc positionStepsOverride*(constraint: ptr Constraint): uint32
  {.importcpp: "#->GetNumPositionStepsOverride()", noSideEffect,
    header: joltBindingsHeader.}
proc setVelocityStepsOverride*(constraint: ptr Constraint; steps: uint32)
  {.importcpp: "#->SetNumVelocityStepsOverride(@)", header: joltBindingsHeader.}
proc setPositionStepsOverride*(constraint: ptr Constraint; steps: uint32)
  {.importcpp: "#->SetNumPositionStepsOverride(@)", header: joltBindingsHeader.}
proc userData*(constraint: ptr Constraint): uint64
  {.importcpp: "#->GetUserData()", noSideEffect, header: joltBindingsHeader.}
proc setUserData*(constraint: ptr Constraint; value: uint64)
  {.importcpp: "#->SetUserData(@)", header: joltBindingsHeader.}
proc resetWarmStart*(constraint: ptr Constraint)
  {.importcpp: "#->ResetWarmStart()", header: joltBindingsHeader.}
proc solverImpulse*(constraint: ptr Constraint; position, rotation: ptr Vec3;
                    limit: ptr cfloat; motorTranslation,
                    motorRotation: ptr Vec3): bool
  {.importcpp: "joltnim_detail::GetConstraintSolverImpulse(@)",
    noSideEffect, header: joltBindingsHeader.}
proc friction*(constraint: ptr Constraint; axis: uint8;
               value: ptr cfloat): bool
  {.importcpp: "joltnim_detail::GetConstraintFriction(@)",
    noSideEffect, header: joltBindingsHeader.}
proc limitSpring*(constraint: ptr Constraint; axis: uint8;
                  mode: ptr uint8; value, damping: ptr cfloat): bool
  {.importcpp: "joltnim_detail::GetConstraintLimitSpring(@)",
    noSideEffect, header: joltBindingsHeader.}
proc motorSettings*(constraint: ptr Constraint; motorIndex: uint8;
                    springMode: ptr uint8; springValue,
                    springDamping, minForce, maxForce,
                    minTorque, maxTorque: ptr cfloat): bool
  {.importcpp: "joltnim_detail::GetConstraintMotorSettings(@)",
    noSideEffect, header: joltBindingsHeader.}
proc swingTwistMotorState*(constraint: ptr Constraint;
                           swingState, twistState: ptr uint8;
                           angularVelocity: ptr Vec3;
                           orientation: ptr Quat): bool
  {.importcpp: "joltnim_detail::GetSwingTwistMotorState(@)",
    noSideEffect, header: joltBindingsHeader.}
proc sixDOFMotorState*(constraint: ptr Constraint; axis: uint8;
                       state: ptr uint8; velocity,
                       angularVelocity, position: ptr Vec3;
                       orientation: ptr Quat): bool
  {.importcpp: "joltnim_detail::GetSixDOFMotorState(@)",
    noSideEffect, header: joltBindingsHeader.}
proc subType*(constraint: ptr Constraint): uint8
  {.importcpp: "joltnim_detail::GetConstraintSubTypeValue(@)",
    noSideEffect, header: joltBindingsHeader.}
proc twoBodyIDs*(constraint: ptr Constraint;
                 body1: ptr uint32; body1IsFixed: ptr bool;
                 body2: ptr uint32; body2IsFixed: ptr bool): bool
  {.importcpp: "joltnim_detail::GetTwoBodyConstraintBodyIDs(@)",
    noSideEffect, header: joltBindingsHeader.}
proc gearTotalLambda*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetGearTotalLambda(@)", noSideEffect, header: joltBindingsHeader.}
proc rackAndPinionTotalLambda*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetRackAndPinionTotalLambda(@)", noSideEffect, header: joltBindingsHeader.}
proc pulleyCurrentLength*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetPulleyCurrentLength(@)", noSideEffect, header: joltBindingsHeader.}
proc pulleyLengths*(constraint: ptr Constraint; minimum, maximum: ptr cfloat)
  {.importcpp: "joltnim_detail::GetPulleyLengths(@)", noSideEffect, header: joltBindingsHeader.}
proc setPulleyLengths*(constraint: ptr Constraint; minimum, maximum: cfloat)
  {.importcpp: "joltnim_detail::SetPulleyLengths(@)", header: joltBindingsHeader.}
proc pathFraction*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetPathFraction(@)", noSideEffect, header: joltBindingsHeader.}
proc pathMaxFraction*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetPathMaxFraction(@)", noSideEffect, header: joltBindingsHeader.}
proc setPathFriction*(constraint: ptr Constraint; maximumForce: cfloat)
  {.importcpp: "joltnim_detail::SetPathFriction(@)", header: joltBindingsHeader.}
proc configurePathMotor*(constraint: ptr Constraint; springMode: uint8;
                         springValue, damping, minForce, maxForce: cfloat)
  {.importcpp: "joltnim_detail::ConfigurePathMotor(@)", header: joltBindingsHeader.}
proc setPathMotorState*(constraint: ptr Constraint; state: uint8)
  {.importcpp: "joltnim_detail::SetPathMotorState(@)", header: joltBindingsHeader.}
proc setPathMotorTargets*(constraint: ptr Constraint; velocity,
                          fraction: cfloat)
  {.importcpp: "joltnim_detail::SetPathMotorTargets(@)", header: joltBindingsHeader.}
proc pathMotor*(constraint: ptr Constraint; state: ptr uint8;
                velocity, fraction: ptr cfloat)
  {.importcpp: "joltnim_detail::GetPathMotor(@)", noSideEffect, header: joltBindingsHeader.}
proc hingeAngle*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetHingeAngle(@)", noSideEffect, header: joltBindingsHeader.}
proc sliderPosition*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetSliderPosition(@)", noSideEffect, header: joltBindingsHeader.}
proc setHingeLimits*(constraint: ptr Constraint; minimum, maximum: cfloat)
  {.importcpp: "joltnim_detail::SetHingeLimits(@)", header: joltBindingsHeader.}
proc setSliderLimits*(constraint: ptr Constraint; minimum, maximum: cfloat)
  {.importcpp: "joltnim_detail::SetSliderLimits(@)", header: joltBindingsHeader.}
proc setHingeFriction*(constraint: ptr Constraint; maximumTorque: cfloat)
  {.importcpp: "joltnim_detail::SetHingeFriction(@)", header: joltBindingsHeader.}
proc setSliderFriction*(constraint: ptr Constraint; maximumForce: cfloat)
  {.importcpp: "joltnim_detail::SetSliderFriction(@)", header: joltBindingsHeader.}
proc configureHingeMotor*(constraint: ptr Constraint; springMode: uint8;
                          springValue, damping, minTorque,
                          maxTorque: cfloat)
  {.importcpp: "joltnim_detail::ConfigureHingeMotor(@)", header: joltBindingsHeader.}
proc configureSliderMotor*(constraint: ptr Constraint; springMode: uint8;
                           springValue, damping, minForce,
                           maxForce: cfloat)
  {.importcpp: "joltnim_detail::ConfigureSliderMotor(@)", header: joltBindingsHeader.}
proc setHingeMotorState*(constraint: ptr Constraint; state: uint8)
  {.importcpp: "joltnim_detail::SetHingeMotorState(@)", header: joltBindingsHeader.}
proc setSliderMotorState*(constraint: ptr Constraint; state: uint8)
  {.importcpp: "joltnim_detail::SetSliderMotorState(@)", header: joltBindingsHeader.}
proc setHingeMotorTarget*(constraint: ptr Constraint; velocity,
                          position: cfloat)
  {.importcpp: "joltnim_detail::SetHingeMotorTarget(@)", header: joltBindingsHeader.}
proc setSliderMotorTarget*(constraint: ptr Constraint; velocity,
                           position: cfloat)
  {.importcpp: "joltnim_detail::SetSliderMotorTarget(@)", header: joltBindingsHeader.}
proc hingeMotor*(constraint: ptr Constraint; state: ptr uint8;
                 velocity, position: ptr cfloat)
  {.importcpp: "joltnim_detail::GetHingeMotor(@)", noSideEffect, header: joltBindingsHeader.}
proc sliderMotor*(constraint: ptr Constraint; state: ptr uint8;
                  velocity, position: ptr cfloat)
  {.importcpp: "joltnim_detail::GetSliderMotor(@)", noSideEffect, header: joltBindingsHeader.}
proc distanceLimits*(constraint: ptr Constraint;
                     minimum, maximum: ptr cfloat)
  {.importcpp: "joltnim_detail::GetDistanceLimits(@)",
    noSideEffect, header: joltBindingsHeader.}
proc setDistanceLimits*(constraint: ptr Constraint;
                        minimum, maximum: cfloat)
  {.importcpp: "joltnim_detail::SetDistanceLimits(@)",
    header: joltBindingsHeader.}
proc setDistanceLimitSpring*(constraint: ptr Constraint; mode: uint8;
                             value, damping: cfloat)
  {.importcpp: "joltnim_detail::SetDistanceLimitSpring(@)",
    header: joltBindingsHeader.}
proc setHingeLimitSpring*(constraint: ptr Constraint; mode: uint8;
                          value, damping: cfloat)
  {.importcpp: "joltnim_detail::SetHingeLimitSpring(@)", header: joltBindingsHeader.}
proc setSliderLimitSpring*(constraint: ptr Constraint; mode: uint8;
                           value, damping: cfloat)
  {.importcpp: "joltnim_detail::SetSliderLimitSpring(@)", header: joltBindingsHeader.}
proc coneHalfAngle*(constraint: ptr Constraint): cfloat
  {.importcpp: "joltnim_detail::GetConeHalfAngle(@)", noSideEffect, header: joltBindingsHeader.}
proc setConeHalfAngle*(constraint: ptr Constraint; angle: cfloat)
  {.importcpp: "joltnim_detail::SetConeHalfAngle(@)", header: joltBindingsHeader.}
proc setSwingTwistLimits*(constraint: ptr Constraint; normalHalfConeAngle,
                          planeHalfConeAngle, twistMinAngle,
                          twistMaxAngle: cfloat)
  {.importcpp: "joltnim_detail::SetSwingTwistLimits(@)", header: joltBindingsHeader.}
proc swingTwistRotation*(constraint: ptr Constraint): Quat
  {.importcpp: "joltnim_detail::GetSwingTwistRotation(@)", noSideEffect, header: joltBindingsHeader.}
proc setSwingTwistFriction*(constraint: ptr Constraint; torque: cfloat)
  {.importcpp: "joltnim_detail::SetSwingTwistFriction(@)", header: joltBindingsHeader.}
proc configureSwingTwistMotor*(constraint: ptr Constraint; swing: bool;
                               springMode: uint8; springValue, damping,
                               minTorque, maxTorque: cfloat)
  {.importcpp: "joltnim_detail::ConfigureSwingTwistMotor(@)", header: joltBindingsHeader.}
proc setSwingTwistMotorState*(constraint: ptr Constraint; swing: bool;
                              state: uint8)
  {.importcpp: "joltnim_detail::SetSwingTwistMotorState(@)", header: joltBindingsHeader.}
proc setSwingTwistMotorTargets*(constraint: ptr Constraint;
                                angularVelocity: Vec3; orientation: Quat)
  {.importcpp: "joltnim_detail::SetSwingTwistMotorTargets(@)", header: joltBindingsHeader.}
proc sixDOFAxisLimit*(constraint: ptr Constraint; axis: uint8;
                      minimum, maximum: ptr cfloat; mode: ptr uint8)
  {.importcpp: "joltnim_detail::GetSixDOFAxisLimit(@)", noSideEffect, header: joltBindingsHeader.}
proc sixDOFSwingType*(constraint: ptr Constraint): uint8
  {.importcpp: "joltnim_detail::GetSixDOFSwingType(@)", noSideEffect,
    header: joltBindingsHeader.}
proc setSixDOFAxisLimit*(constraint: ptr Constraint; axis, mode: uint8;
                         minimum, maximum: cfloat)
  {.importcpp: "joltnim_detail::SetSixDOFAxisLimit(@)", header: joltBindingsHeader.}
proc setSixDOFFriction*(constraint: ptr Constraint; axis: uint8;
                        friction: cfloat)
  {.importcpp: "joltnim_detail::SetSixDOFFriction(@)", header: joltBindingsHeader.}
proc setSixDOFLimitSpring*(constraint: ptr Constraint; axis,
                           springMode: uint8; springValue,
                           damping: cfloat)
  {.importcpp: "joltnim_detail::SetSixDOFLimitSpring(@)",
    header: joltBindingsHeader.}
proc configureSixDOFMotor*(constraint: ptr Constraint; axis,
                           springMode: uint8; springValue, damping,
                           minimum, maximum: cfloat)
  {.importcpp: "joltnim_detail::ConfigureSixDOFMotor(@)", header: joltBindingsHeader.}
proc setSixDOFMotorState*(constraint: ptr Constraint; axis, state: uint8)
  {.importcpp: "joltnim_detail::SetSixDOFMotorState(@)", header: joltBindingsHeader.}
proc setSixDOFMotorTargets*(constraint: ptr Constraint; velocity,
                            angularVelocity, position: Vec3;
                            orientation: Quat)
  {.importcpp: "joltnim_detail::SetSixDOFMotorTargets(@)", header: joltBindingsHeader.}
proc removeConstraint*(self: ptr PhysicsSystem; constraint: ptr Constraint)
  {.importcpp: "joltnim_detail::RemoveConstraint(@)", header: joltBindingsHeader.}

proc collidePoint*(self: ptr PhysicsSystem; point: Vec3;
                   bodyIds: ptr uint32; capacity: uint32;
                   objectLayers: ptr ObjectLayer;
                   objectLayerCount: uint32): uint32
  {.importcpp: "joltnim_detail::CollidePoint(@)", noSideEffect, header: joltBindingsHeader.}
proc broadPhaseCollideAABox*(self: ptr PhysicsSystem; minimum, maximum: Vec3;
                            bodyIds: ptr uint32; capacity: uint32;
                            objectLayers: ptr ObjectLayer;
                            objectLayerCount: uint32;
                            filterBodyIds: ptr uint32 = nil;
                            filterBodyIdCount: uint32 = 0;
                            includeBodies: bool = false): uint32
  {.importcpp: "joltnim_detail::BroadPhaseCollideAABox(@)", noSideEffect, header: joltBindingsHeader.}
proc broadPhaseCollideSphere*(self: ptr PhysicsSystem; center: Vec3;
                             radius: cfloat; bodyIds: ptr uint32;
                             capacity: uint32;
                             objectLayers: ptr ObjectLayer;
                             objectLayerCount: uint32;
                             filterBodyIds: ptr uint32 = nil;
                             filterBodyIdCount: uint32 = 0;
                             includeBodies: bool = false): uint32
  {.importcpp: "joltnim_detail::BroadPhaseCollideSphere(@)", noSideEffect, header: joltBindingsHeader.}
proc broadPhaseCollidePoint*(self: ptr PhysicsSystem; point: Vec3;
                            bodyIds: ptr uint32; capacity: uint32;
                            objectLayers: ptr ObjectLayer;
                            objectLayerCount: uint32;
                            filterBodyIds: ptr uint32 = nil;
                            filterBodyIdCount: uint32 = 0;
                            includeBodies: bool = false): uint32
  {.importcpp: "joltnim_detail::BroadPhaseCollidePoint(@)", noSideEffect, header: joltBindingsHeader.}
proc broadPhaseCollideOrientedBox*(self: ptr PhysicsSystem; center: Vec3;
                                  rotation: Quat; halfExtent: Vec3;
                                  bodyIds: ptr uint32; capacity: uint32;
                                  objectLayers: ptr ObjectLayer;
                                  objectLayerCount: uint32;
                                  filterBodyIds: ptr uint32 = nil;
                                  filterBodyIdCount: uint32 = 0;
                                  includeBodies: bool = false): uint32
  {.importcpp: "joltnim_detail::BroadPhaseCollideOrientedBox(@)", noSideEffect, header: joltBindingsHeader.}
proc broadPhaseCastRay*(self: ptr PhysicsSystem; origin,
                        directionAndLength: Vec3; bodyIds: ptr uint32;
                        fractions: ptr cfloat; capacity: uint32;
                        objectLayers: ptr ObjectLayer;
                        objectLayerCount: uint32;
                        filterBodyIds: ptr uint32 = nil;
                        filterBodyIdCount: uint32 = 0;
                        includeBodies: bool = false): uint32
  {.importcpp: "joltnim_detail::BroadPhaseCastRay(@)", noSideEffect, header: joltBindingsHeader.}
proc broadPhaseCastAABox*(self: ptr PhysicsSystem; center, halfExtent,
                          directionAndLength: Vec3; bodyIds: ptr uint32;
                          fractions: ptr cfloat; capacity: uint32;
                          objectLayers: ptr ObjectLayer;
                          objectLayerCount: uint32;
                          filterBodyIds: ptr uint32 = nil;
                          filterBodyIdCount: uint32 = 0;
                          includeBodies: bool = false): uint32
  {.importcpp: "joltnim_detail::BroadPhaseCastAABox(@)", noSideEffect, header: joltBindingsHeader.}
proc castRay*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
              bodyId: ptr uint32; fraction: ptr cfloat;
              subShapeId: ptr uint32; objectLayers: ptr ObjectLayer;
              objectLayerCount: uint32): bool
  {.importcpp: "joltnim_detail::CastRay(@)", noSideEffect, header: joltBindingsHeader.}
proc castRayAll*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
                 bodyIds: ptr uint32; fractions: ptr cfloat;
                 subShapeIds: ptr uint32; capacity: uint32;
                 objectLayers: ptr ObjectLayer;
                 objectLayerCount: uint32): uint32
  {.importcpp: "joltnim_detail::CastRayAll(@)", noSideEffect, header: joltBindingsHeader.}
proc castSphere*(self: ptr PhysicsSystem; radius: cfloat;
                 origin, directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3;
                 subShapeId: ptr uint32; objectLayers: ptr ObjectLayer;
                 objectLayerCount: uint32): bool
  {.importcpp: "joltnim_detail::CastSphere(@)", noSideEffect, header: joltBindingsHeader.}
proc castSphereAll*(self: ptr PhysicsSystem; radius: cfloat;
                    origin, directionAndLength: Vec3;
                    bodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer;
                    objectLayerCount: uint32): uint32
  {.importcpp: "joltnim_detail::CastSphereAll(@)", noSideEffect, header: joltBindingsHeader.}
proc overlapSphere*(self: ptr PhysicsSystem; center: Vec3; radius: cfloat;
                    bodyIds: ptr uint32; penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer;
                    objectLayerCount: uint32): uint32
  {.importcpp: "joltnim_detail::OverlapSphere(@)", noSideEffect, header: joltBindingsHeader.}
proc castConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                 origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3;
                 subShapeId: ptr uint32; objectLayers: ptr ObjectLayer;
                 objectLayerCount: uint32): bool
  {.importcpp: "joltnim_detail::CastConvex(@)", noSideEffect, header: joltBindingsHeader.}
proc castConvexAll*(self: ptr PhysicsSystem; shape: ptr Shape;
                    origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                    bodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer;
                    objectLayerCount: uint32): uint32
  {.importcpp: "joltnim_detail::CastConvexAll(@)", noSideEffect, header: joltBindingsHeader.}
proc overlapConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                    position: Vec3; rotation: Quat;
                    bodyIds: ptr uint32; penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer;
                    objectLayerCount: uint32): uint32
  {.importcpp: "joltnim_detail::OverlapConvex(@)", noSideEffect, header: joltBindingsHeader.}

proc collidePoint*(self: ptr PhysicsSystem; point: Vec3;
                   resultBodyIds: ptr uint32; capacity: uint32;
                   objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                   filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                   includeBodies: bool;
                   filterSubShapeBodyIds: ptr uint32 = nil;
                   filterSubShapeIds: ptr uint32 = nil;
                   filterSubShapeCount: uint32 = 0;
                   includeSubShapes: bool = false): uint32
  {.importcpp: "joltnim_detail::CollidePoint(@)", noSideEffect, header: joltBindingsHeader.}
proc castRay*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
              bodyId: ptr uint32; fraction: ptr cfloat;
              subShapeId: ptr uint32; objectLayers: ptr ObjectLayer;
              objectLayerCount: uint32; filterBodyIds: ptr uint32;
              filterBodyIdCount: uint32; includeBodies: bool;
              filterSubShapeBodyIds: ptr uint32 = nil;
              filterSubShapeIds: ptr uint32 = nil;
              filterSubShapeCount: uint32 = 0;
              includeSubShapes: bool = false): bool
  {.importcpp: "joltnim_detail::CastRay(@)", noSideEffect, header: joltBindingsHeader.}
proc castRayAll*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
                 resultBodyIds: ptr uint32; fractions: ptr cfloat;
                 subShapeIds: ptr uint32; capacity: uint32;
                 objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                 filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                 includeBodies: bool;
                 filterSubShapeBodyIds: ptr uint32 = nil;
                 filterSubShapeIds: ptr uint32 = nil;
                 filterSubShapeCount: uint32 = 0;
                 includeSubShapes: bool = false): uint32
  {.importcpp: "joltnim_detail::CastRayAll(@)", noSideEffect, header: joltBindingsHeader.}
proc castSphere*(self: ptr PhysicsSystem; radius: cfloat;
                 origin, directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3; subShapeId: ptr uint32;
                 objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                 filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                 includeBodies: bool;
                 filterSubShapeBodyIds: ptr uint32 = nil;
                 filterSubShapeIds: ptr uint32 = nil;
                 filterSubShapeCount: uint32 = 0;
                 includeSubShapes: bool = false): bool
  {.importcpp: "joltnim_detail::CastSphere(@)", noSideEffect, header: joltBindingsHeader.}
proc castSphereAll*(self: ptr PhysicsSystem; radius: cfloat;
                    origin, directionAndLength: Vec3;
                    resultBodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool;
                    filterSubShapeBodyIds: ptr uint32 = nil;
                    filterSubShapeIds: ptr uint32 = nil;
                    filterSubShapeCount: uint32 = 0;
                    includeSubShapes: bool = false): uint32
  {.importcpp: "joltnim_detail::CastSphereAll(@)", noSideEffect, header: joltBindingsHeader.}
proc overlapSphere*(self: ptr PhysicsSystem; center: Vec3; radius: cfloat;
                    resultBodyIds: ptr uint32;
                    penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool;
                    filterSubShapeBodyIds: ptr uint32 = nil;
                    filterSubShapeIds: ptr uint32 = nil;
                    filterSubShapeCount: uint32 = 0;
                    includeSubShapes: bool = false): uint32
  {.importcpp: "joltnim_detail::OverlapSphere(@)", noSideEffect, header: joltBindingsHeader.}
proc castConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                 origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3; subShapeId: ptr uint32;
                 objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                 filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                 includeBodies: bool;
                 filterSubShapeBodyIds: ptr uint32 = nil;
                 filterSubShapeIds: ptr uint32 = nil;
                 filterSubShapeCount: uint32 = 0;
                 includeSubShapes: bool = false): bool
  {.importcpp: "joltnim_detail::CastConvex(@)", noSideEffect, header: joltBindingsHeader.}
proc castConvexAll*(self: ptr PhysicsSystem; shape: ptr Shape;
                    origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                    resultBodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool;
                    filterSubShapeBodyIds: ptr uint32 = nil;
                    filterSubShapeIds: ptr uint32 = nil;
                    filterSubShapeCount: uint32 = 0;
                    includeSubShapes: bool = false): uint32
  {.importcpp: "joltnim_detail::CastConvexAll(@)", noSideEffect, header: joltBindingsHeader.}
proc overlapConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                    position: Vec3; rotation: Quat;
                    resultBodyIds: ptr uint32;
                    penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity: uint32;
                    objectLayers: ptr ObjectLayer; objectLayerCount: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool;
                    filterSubShapeBodyIds: ptr uint32 = nil;
                    filterSubShapeIds: ptr uint32 = nil;
                    filterSubShapeCount: uint32 = 0;
                    includeSubShapes: bool = false): uint32
  {.importcpp: "joltnim_detail::OverlapConvex(@)", noSideEffect, header: joltBindingsHeader.}

proc collidePoint*(self: ptr PhysicsSystem; point: Vec3;
                   bodyIds: ptr uint32; capacity,
                   objectLayer: uint32): uint32 =
  if objectLayer == high(uint32):
    return self.collidePoint(point, bodyIds, capacity, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.collidePoint(point, bodyIds, capacity, addr layer, 1)
proc broadPhaseCollideAABox*(self: ptr PhysicsSystem; minimum, maximum: Vec3;
                            bodyIds: ptr uint32; capacity,
                            objectLayer: uint32; filterBodyIds: ptr uint32 = nil;
                            filterBodyIdCount: uint32 = 0;
                            includeBodies: bool = false): uint32 =
  if objectLayer == high(uint32):
    return self.broadPhaseCollideAABox(
      minimum, maximum, bodyIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.broadPhaseCollideAABox(
    minimum, maximum, bodyIds, capacity, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies)
proc broadPhaseCollideSphere*(self: ptr PhysicsSystem; center: Vec3;
                             radius: cfloat; bodyIds: ptr uint32;
                             capacity, objectLayer: uint32;
                             filterBodyIds: ptr uint32 = nil;
                             filterBodyIdCount: uint32 = 0;
                             includeBodies: bool = false): uint32 =
  if objectLayer == high(uint32):
    return self.broadPhaseCollideSphere(
      center, radius, bodyIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.broadPhaseCollideSphere(
    center, radius, bodyIds, capacity, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies)
proc broadPhaseCollidePoint*(self: ptr PhysicsSystem; point: Vec3;
                            bodyIds: ptr uint32; capacity,
                            objectLayer: uint32; filterBodyIds: ptr uint32 = nil;
                            filterBodyIdCount: uint32 = 0;
                            includeBodies: bool = false): uint32 =
  if objectLayer == high(uint32):
    return self.broadPhaseCollidePoint(
      point, bodyIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.broadPhaseCollidePoint(
    point, bodyIds, capacity, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies)
proc broadPhaseCollideOrientedBox*(self: ptr PhysicsSystem; center: Vec3;
                                  rotation: Quat; halfExtent: Vec3;
                                  bodyIds: ptr uint32; capacity,
                                  objectLayer: uint32;
                                  filterBodyIds: ptr uint32 = nil;
                                  filterBodyIdCount: uint32 = 0;
                                  includeBodies: bool = false): uint32 =
  if objectLayer == high(uint32):
    return self.broadPhaseCollideOrientedBox(
      center, rotation, halfExtent, bodyIds, capacity, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.broadPhaseCollideOrientedBox(
    center, rotation, halfExtent, bodyIds, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies)
proc broadPhaseCastRay*(self: ptr PhysicsSystem; origin,
                        directionAndLength: Vec3; bodyIds: ptr uint32;
                        fractions: ptr cfloat; capacity,
                        objectLayer: uint32; filterBodyIds: ptr uint32 = nil;
                        filterBodyIdCount: uint32 = 0;
                        includeBodies: bool = false): uint32 =
  if objectLayer == high(uint32):
    return self.broadPhaseCastRay(
      origin, directionAndLength, bodyIds, fractions, capacity, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.broadPhaseCastRay(
    origin, directionAndLength, bodyIds, fractions, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies)
proc broadPhaseCastAABox*(self: ptr PhysicsSystem; center, halfExtent,
                          directionAndLength: Vec3; bodyIds: ptr uint32;
                          fractions: ptr cfloat; capacity,
                          objectLayer: uint32;
                          filterBodyIds: ptr uint32 = nil;
                          filterBodyIdCount: uint32 = 0;
                          includeBodies: bool = false): uint32 =
  if objectLayer == high(uint32):
    return self.broadPhaseCastAABox(
      center, halfExtent, directionAndLength, bodyIds, fractions,
      capacity, nil, 0, filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.broadPhaseCastAABox(
    center, halfExtent, directionAndLength, bodyIds, fractions,
    capacity, addr layer, 1, filterBodyIds, filterBodyIdCount, includeBodies)
proc broadPhaseBounds*(self: ptr PhysicsSystem; minimum, maximum: ptr Vec3)
  {.importcpp: "joltnim_detail::GetBroadPhaseBounds(@)", noSideEffect, header: joltBindingsHeader.}
proc castRay*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
              bodyId: ptr uint32; fraction: ptr cfloat;
              subShapeId: ptr uint32;
              objectLayer: uint32): bool =
  if objectLayer == high(uint32):
    return self.castRay(
      origin, directionAndLength, bodyId, fraction, subShapeId, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.castRay(
    origin, directionAndLength, bodyId, fraction, subShapeId, addr layer, 1)
proc castRayAll*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
                 bodyIds: ptr uint32; fractions: ptr cfloat;
                 subShapeIds: ptr uint32;
                 capacity, objectLayer: uint32): uint32 =
  if objectLayer == high(uint32):
    return self.castRayAll(
      origin, directionAndLength, bodyIds, fractions, subShapeIds,
      capacity, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.castRayAll(
    origin, directionAndLength, bodyIds, fractions, subShapeIds,
    capacity, addr layer, 1)
proc castSphere*(self: ptr PhysicsSystem; radius: cfloat;
                 origin, directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3;
                 subShapeId: ptr uint32;
                 objectLayer: uint32): bool =
  if objectLayer == high(uint32):
    return self.castSphere(
      radius, origin, directionAndLength, bodyId, fraction,
      contactPoint, normal, subShapeId, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.castSphere(
    radius, origin, directionAndLength, bodyId, fraction,
    contactPoint, normal, subShapeId, addr layer, 1)
proc castSphereAll*(self: ptr PhysicsSystem; radius: cfloat;
                    origin, directionAndLength: Vec3;
                    bodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32;
                    capacity, objectLayer: uint32): uint32 =
  if objectLayer == high(uint32):
    return self.castSphereAll(
      radius, origin, directionAndLength, bodyIds, fractions,
      contactPoints, normals, subShapeIds, capacity, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.castSphereAll(
    radius, origin, directionAndLength, bodyIds, fractions,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1)
proc overlapSphere*(self: ptr PhysicsSystem; center: Vec3; radius: cfloat;
                    bodyIds: ptr uint32; penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32;
                    capacity, objectLayer: uint32): uint32 =
  if objectLayer == high(uint32):
    return self.overlapSphere(
      center, radius, bodyIds, penetrationDepths, contactPoints,
      normals, subShapeIds, capacity, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.overlapSphere(
    center, radius, bodyIds, penetrationDepths, contactPoints,
    normals, subShapeIds, capacity, addr layer, 1)
proc castConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                 origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3;
                 subShapeId: ptr uint32;
                 objectLayer: uint32): bool =
  if objectLayer == high(uint32):
    return self.castConvex(
      shape, origin, rotation, directionAndLength, bodyId, fraction,
      contactPoint, normal, subShapeId, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.castConvex(
    shape, origin, rotation, directionAndLength, bodyId, fraction,
    contactPoint, normal, subShapeId, addr layer, 1)
proc castConvexAll*(self: ptr PhysicsSystem; shape: ptr Shape;
                    origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                    bodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32;
                    capacity, objectLayer: uint32): uint32 =
  if objectLayer == high(uint32):
    return self.castConvexAll(
      shape, origin, rotation, directionAndLength, bodyIds, fractions,
      contactPoints, normals, subShapeIds, capacity, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.castConvexAll(
    shape, origin, rotation, directionAndLength, bodyIds, fractions,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1)
proc overlapConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                    position: Vec3; rotation: Quat;
                    bodyIds: ptr uint32; penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32;
                    capacity, objectLayer: uint32): uint32 =
  if objectLayer == high(uint32):
    return self.overlapConvex(
      shape, position, rotation, bodyIds, penetrationDepths,
      contactPoints, normals, subShapeIds, capacity, nil, 0)
  var layer = ObjectLayer(objectLayer)
  self.overlapConvex(
    shape, position, rotation, bodyIds, penetrationDepths,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1)

proc collidePoint*(self: ptr PhysicsSystem; point: Vec3;
                   resultBodyIds: ptr uint32; capacity,
                   objectLayer: uint32; filterBodyIds: ptr uint32;
                   filterBodyIdCount: uint32;
                   includeBodies: bool): uint32 =
  if objectLayer == high(uint32):
    return self.collidePoint(
      point, resultBodyIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.collidePoint(
    point, resultBodyIds, capacity, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies)

proc castRay*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
              bodyId: ptr uint32; fraction: ptr cfloat;
              subShapeId: ptr uint32; objectLayer: uint32;
              filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
              includeBodies: bool): bool =
  if objectLayer == high(uint32):
    return self.castRay(
      origin, directionAndLength, bodyId, fraction, subShapeId, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.castRay(
    origin, directionAndLength, bodyId, fraction, subShapeId, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies)

proc castRayAll*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
                 resultBodyIds: ptr uint32; fractions: ptr cfloat;
                 subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                 filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                 includeBodies: bool): uint32 =
  if objectLayer == high(uint32):
    return self.castRayAll(
      origin, directionAndLength, resultBodyIds, fractions, subShapeIds,
      capacity, nil, 0, filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.castRayAll(
    origin, directionAndLength, resultBodyIds, fractions, subShapeIds,
    capacity, addr layer, 1, filterBodyIds, filterBodyIdCount, includeBodies)

proc castSphere*(self: ptr PhysicsSystem; radius: cfloat;
                 origin, directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3; subShapeId: ptr uint32;
                 objectLayer: uint32; filterBodyIds: ptr uint32;
                 filterBodyIdCount: uint32; includeBodies: bool): bool =
  if objectLayer == high(uint32):
    return self.castSphere(
      radius, origin, directionAndLength, bodyId, fraction, contactPoint,
      normal, subShapeId, nil, 0, filterBodyIds, filterBodyIdCount,
      includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.castSphere(
    radius, origin, directionAndLength, bodyId, fraction, contactPoint,
    normal, subShapeId, addr layer, 1, filterBodyIds, filterBodyIdCount,
    includeBodies)

proc castSphereAll*(self: ptr PhysicsSystem; radius: cfloat;
                    origin, directionAndLength: Vec3;
                    resultBodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool): uint32 =
  if objectLayer == high(uint32):
    return self.castSphereAll(
      radius, origin, directionAndLength, resultBodyIds, fractions,
      contactPoints, normals, subShapeIds, capacity, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.castSphereAll(
    radius, origin, directionAndLength, resultBodyIds, fractions,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies)

proc overlapSphere*(self: ptr PhysicsSystem; center: Vec3; radius: cfloat;
                    resultBodyIds: ptr uint32;
                    penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool): uint32 =
  if objectLayer == high(uint32):
    return self.overlapSphere(
      center, radius, resultBodyIds, penetrationDepths, contactPoints,
      normals, subShapeIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.overlapSphere(
    center, radius, resultBodyIds, penetrationDepths, contactPoints,
    normals, subShapeIds, capacity, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies)

proc castConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                 origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3; subShapeId: ptr uint32;
                 objectLayer: uint32; filterBodyIds: ptr uint32;
                 filterBodyIdCount: uint32; includeBodies: bool): bool =
  if objectLayer == high(uint32):
    return self.castConvex(
      shape, origin, rotation, directionAndLength, bodyId, fraction,
      contactPoint, normal, subShapeId, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.castConvex(
    shape, origin, rotation, directionAndLength, bodyId, fraction,
    contactPoint, normal, subShapeId, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies)

proc castConvexAll*(self: ptr PhysicsSystem; shape: ptr Shape;
                    origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                    resultBodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool): uint32 =
  if objectLayer == high(uint32):
    return self.castConvexAll(
      shape, origin, rotation, directionAndLength, resultBodyIds,
      fractions, contactPoints, normals, subShapeIds, capacity, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.castConvexAll(
    shape, origin, rotation, directionAndLength, resultBodyIds, fractions,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies)

proc overlapConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                    position: Vec3; rotation: Quat;
                    resultBodyIds: ptr uint32;
                    penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool): uint32 =
  if objectLayer == high(uint32):
    return self.overlapConvex(
      shape, position, rotation, resultBodyIds, penetrationDepths,
      contactPoints, normals, subShapeIds, capacity, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies)
  var layer = ObjectLayer(objectLayer)
  self.overlapConvex(
    shape, position, rotation, resultBodyIds, penetrationDepths,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies)

proc collidePoint*(self: ptr PhysicsSystem; point: Vec3;
                   resultBodyIds: ptr uint32; capacity, objectLayer: uint32;
                   filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                   includeBodies: bool; filterSubShapeBodyIds,
                   filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
                   includeSubShapes: bool): uint32 =
  if objectLayer == high(uint32):
    return self.collidePoint(
      point, resultBodyIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
      filterSubShapeIds, filterSubShapeCount, includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.collidePoint(
    point, resultBodyIds, capacity, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
    filterSubShapeIds, filterSubShapeCount, includeSubShapes)

proc castRay*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
              bodyId: ptr uint32; fraction: ptr cfloat;
              subShapeId: ptr uint32; objectLayer: uint32;
              filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
              includeBodies: bool; filterSubShapeBodyIds,
              filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
              includeSubShapes: bool): bool =
  if objectLayer == high(uint32):
    return self.castRay(
      origin, directionAndLength, bodyId, fraction, subShapeId, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies,
      filterSubShapeBodyIds, filterSubShapeIds, filterSubShapeCount,
      includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.castRay(
    origin, directionAndLength, bodyId, fraction, subShapeId, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
    filterSubShapeIds, filterSubShapeCount, includeSubShapes)

proc castRayAll*(self: ptr PhysicsSystem; origin, directionAndLength: Vec3;
                 resultBodyIds: ptr uint32; fractions: ptr cfloat;
                 subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                 filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                 includeBodies: bool; filterSubShapeBodyIds,
                 filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
                 includeSubShapes: bool): uint32 =
  if objectLayer == high(uint32):
    return self.castRayAll(
      origin, directionAndLength, resultBodyIds, fractions, subShapeIds,
      capacity, nil, 0, filterBodyIds, filterBodyIdCount, includeBodies,
      filterSubShapeBodyIds, filterSubShapeIds, filterSubShapeCount,
      includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.castRayAll(
    origin, directionAndLength, resultBodyIds, fractions, subShapeIds,
    capacity, addr layer, 1, filterBodyIds, filterBodyIdCount, includeBodies,
    filterSubShapeBodyIds, filterSubShapeIds, filterSubShapeCount,
    includeSubShapes)

proc castSphere*(self: ptr PhysicsSystem; radius: cfloat;
                 origin, directionAndLength: Vec3; bodyId: ptr uint32;
                 fraction: ptr cfloat; contactPoint, normal: ptr Vec3;
                 subShapeId: ptr uint32; objectLayer: uint32;
                 filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                 includeBodies: bool; filterSubShapeBodyIds,
                 filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
                 includeSubShapes: bool): bool =
  if objectLayer == high(uint32):
    return self.castSphere(
      radius, origin, directionAndLength, bodyId, fraction, contactPoint,
      normal, subShapeId, nil, 0, filterBodyIds, filterBodyIdCount,
      includeBodies, filterSubShapeBodyIds, filterSubShapeIds,
      filterSubShapeCount, includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.castSphere(
    radius, origin, directionAndLength, bodyId, fraction, contactPoint,
    normal, subShapeId, addr layer, 1, filterBodyIds, filterBodyIdCount,
    includeBodies, filterSubShapeBodyIds, filterSubShapeIds,
    filterSubShapeCount, includeSubShapes)

proc castSphereAll*(self: ptr PhysicsSystem; radius: cfloat;
                    origin, directionAndLength: Vec3;
                    resultBodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool; filterSubShapeBodyIds,
                    filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
                    includeSubShapes: bool): uint32 =
  if objectLayer == high(uint32):
    return self.castSphereAll(
      radius, origin, directionAndLength, resultBodyIds, fractions,
      contactPoints, normals, subShapeIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
      filterSubShapeIds, filterSubShapeCount, includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.castSphereAll(
    radius, origin, directionAndLength, resultBodyIds, fractions,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
    filterSubShapeIds, filterSubShapeCount, includeSubShapes)

proc overlapSphere*(self: ptr PhysicsSystem; center: Vec3; radius: cfloat;
                    resultBodyIds: ptr uint32; penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool; filterSubShapeBodyIds,
                    filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
                    includeSubShapes: bool): uint32 =
  if objectLayer == high(uint32):
    return self.overlapSphere(
      center, radius, resultBodyIds, penetrationDepths, contactPoints,
      normals, subShapeIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
      filterSubShapeIds, filterSubShapeCount, includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.overlapSphere(
    center, radius, resultBodyIds, penetrationDepths, contactPoints,
    normals, subShapeIds, capacity, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
    filterSubShapeIds, filterSubShapeCount, includeSubShapes)

proc castConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                 origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                 bodyId: ptr uint32; fraction: ptr cfloat;
                 contactPoint, normal: ptr Vec3; subShapeId: ptr uint32;
                 objectLayer: uint32; filterBodyIds: ptr uint32;
                 filterBodyIdCount: uint32; includeBodies: bool;
                 filterSubShapeBodyIds, filterSubShapeIds: ptr uint32;
                 filterSubShapeCount: uint32; includeSubShapes: bool): bool =
  if objectLayer == high(uint32):
    return self.castConvex(
      shape, origin, rotation, directionAndLength, bodyId, fraction,
      contactPoint, normal, subShapeId, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
      filterSubShapeIds, filterSubShapeCount, includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.castConvex(
    shape, origin, rotation, directionAndLength, bodyId, fraction,
    contactPoint, normal, subShapeId, addr layer, 1, filterBodyIds,
    filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
    filterSubShapeIds, filterSubShapeCount, includeSubShapes)

proc castConvexAll*(self: ptr PhysicsSystem; shape: ptr Shape;
                    origin: Vec3; rotation: Quat; directionAndLength: Vec3;
                    resultBodyIds: ptr uint32; fractions: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool; filterSubShapeBodyIds,
                    filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
                    includeSubShapes: bool): uint32 =
  if objectLayer == high(uint32):
    return self.castConvexAll(
      shape, origin, rotation, directionAndLength, resultBodyIds,
      fractions, contactPoints, normals, subShapeIds, capacity, nil, 0,
      filterBodyIds, filterBodyIdCount, includeBodies,
      filterSubShapeBodyIds, filterSubShapeIds, filterSubShapeCount,
      includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.castConvexAll(
    shape, origin, rotation, directionAndLength, resultBodyIds, fractions,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies,
    filterSubShapeBodyIds, filterSubShapeIds, filterSubShapeCount,
    includeSubShapes)

proc overlapConvex*(self: ptr PhysicsSystem; shape: ptr Shape;
                    position: Vec3; rotation: Quat;
                    resultBodyIds: ptr uint32; penetrationDepths: ptr cfloat;
                    contactPoints, normals: ptr Vec3;
                    subShapeIds: ptr uint32; capacity, objectLayer: uint32;
                    filterBodyIds: ptr uint32; filterBodyIdCount: uint32;
                    includeBodies: bool; filterSubShapeBodyIds,
                    filterSubShapeIds: ptr uint32; filterSubShapeCount: uint32;
                    includeSubShapes: bool): uint32 =
  if objectLayer == high(uint32):
    return self.overlapConvex(
      shape, position, rotation, resultBodyIds, penetrationDepths,
      contactPoints, normals, subShapeIds, capacity, nil, 0, filterBodyIds,
      filterBodyIdCount, includeBodies, filterSubShapeBodyIds,
      filterSubShapeIds, filterSubShapeCount, includeSubShapes)
  var layer = ObjectLayer(objectLayer)
  self.overlapConvex(
    shape, position, rotation, resultBodyIds, penetrationDepths,
    contactPoints, normals, subShapeIds, capacity, addr layer, 1,
    filterBodyIds, filterBodyIdCount, includeBodies,
    filterSubShapeBodyIds, filterSubShapeIds, filterSubShapeCount,
    includeSubShapes)

proc newEventBridge*(self: ptr PhysicsSystem; capacity: uint32;
                     layer1, layer2: ptr ObjectLayer;
                     responses: ptr uint8;
                     frictions, restitutions, inverseMassScales1,
                     inverseInertiaScales1, inverseMassScales2,
                     inverseInertiaScales2: ptr cfloat;
                     linearSurfaceVelocities,
                     angularSurfaceVelocities: ptr Vec3;
                     policyCount: uint32): ptr EventBridge
  {.importcpp: "joltnim_detail::CreateEventBridge(@)", header: joltBindingsHeader.}
proc deleteEventBridge*(self: ptr PhysicsSystem; bridge: ptr EventBridge)
  {.importcpp: "joltnim_detail::DestroyEventBridge(@)", header: joltBindingsHeader.}
proc popEvent*(bridge: ptr EventBridge; kind: ptr uint8;
               body1, body2, subShape1, subShape2: ptr uint32;
               point, normal: ptr Vec3): bool
  {.importcpp: "joltnim_detail::PopEvent(@)", header: joltBindingsHeader.}
proc pendingEventCount*(bridge: ptr EventBridge): uint32
  {.importcpp: "joltnim_detail::PendingEventCount(@)", noSideEffect, header: joltBindingsHeader.}
proc popSoftBodyContactEvent*(bridge: ptr EventBridge;
                              softBody, otherBody, vertex: ptr uint32;
                              point, normal: ptr Vec3;
                              isSensor: ptr bool): bool
  {.importcpp: "joltnim_detail::PopSoftBodyContactEvent(@)",
    header: joltBindingsHeader.}
proc pendingSoftBodyContactEventCount*(bridge: ptr EventBridge): uint32
  {.importcpp: "joltnim_detail::PendingSoftBodyContactEventCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc droppedEventCount*(bridge: ptr EventBridge; reset: bool): uint64
  {.importcpp: "joltnim_detail::DroppedEventCount(@)", header: joltBindingsHeader.}
proc setBodyPairContactPolicy*(self: ptr PhysicsSystem;
                               bridge: ptr EventBridge;
                               body1, body2: uint32;
                               response: uint8;
                               friction, restitution,
                               inverseMassScale1, inverseInertiaScale1,
                               inverseMassScale2,
                               inverseInertiaScale2: cfloat;
                               linearSurfaceVelocity,
                               angularSurfaceVelocity: Vec3)
  {.importcpp: "joltnim_detail::SetBodyPairContactPolicy(@)",
    header: joltBindingsHeader.}
proc removeBodyPairContactPolicy*(self: ptr PhysicsSystem;
                                  bridge: ptr EventBridge;
                                  body1, body2: uint32): bool
  {.importcpp: "joltnim_detail::RemoveBodyPairContactPolicy(@)",
    header: joltBindingsHeader.}
proc hasBodyPairContactPolicy*(bridge: ptr EventBridge;
                               body1, body2: uint32): bool
  {.importcpp: "joltnim_detail::HasBodyPairContactPolicy(@)",
    noSideEffect, header: joltBindingsHeader.}
proc bodyPairContactPolicyCount*(bridge: ptr EventBridge): uint32
  {.importcpp: "joltnim_detail::BodyPairContactPolicyCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc setSubShapePairContactPolicy*(self: ptr PhysicsSystem;
                                   bridge: ptr EventBridge;
                                   body1, subShape1,
                                   body2, subShape2: uint32;
                                   response: uint8;
                                   friction, restitution,
                                   inverseMassScale1, inverseInertiaScale1,
                                   inverseMassScale2,
                                   inverseInertiaScale2: cfloat;
                                   linearSurfaceVelocity,
                                   angularSurfaceVelocity: Vec3)
  {.importcpp: "joltnim_detail::SetSubShapePairContactPolicy(@)",
    header: joltBindingsHeader.}
proc removeSubShapePairContactPolicy*(self: ptr PhysicsSystem;
                                      bridge: ptr EventBridge;
                                      body1, subShape1,
                                      body2, subShape2: uint32): bool
  {.importcpp: "joltnim_detail::RemoveSubShapePairContactPolicy(@)",
    header: joltBindingsHeader.}
proc hasSubShapePairContactPolicy*(bridge: ptr EventBridge;
                                   body1, subShape1,
                                   body2, subShape2: uint32): bool
  {.importcpp: "joltnim_detail::HasSubShapePairContactPolicy(@)",
    noSideEffect, header: joltBindingsHeader.}
proc subShapePairContactPolicyCount*(bridge: ptr EventBridge): uint32
  {.importcpp: "joltnim_detail::SubShapePairContactPolicyCount(@)",
    noSideEffect, header: joltBindingsHeader.}
proc removeBodySubShapeContactPolicies*(bridge: ptr EventBridge; body: uint32)
  {.importcpp: "joltnim_detail::RemoveBodySubShapeContactPolicies(@)",
    header: joltBindingsHeader.}
proc removeBodyContactPolicies*(bridge: ptr EventBridge; body: uint32)
  {.importcpp: "joltnim_detail::RemoveBodyContactPolicies(@)",
    header: joltBindingsHeader.}
proc saveWorldState*(self: ptr PhysicsSystem;
                     characters: ptr ptr CharacterHandle;
                     characterCount: uint32): ptr WorldStateHandle
  {.importcpp: "joltnim_detail::SaveWorldState(@)", header: joltBindingsHeader.}
proc restoreWorldState*(self: ptr PhysicsSystem; state: ptr WorldStateHandle;
                        characters: ptr ptr CharacterHandle;
                        characterCount: uint32;
                        eventBridge: ptr EventBridge): bool
  {.importcpp: "joltnim_detail::RestoreWorldState(@)", header: joltBindingsHeader.}
proc byteSize*(state: ptr WorldStateHandle): csize_t
  {.importcpp: "joltnim_detail::WorldStateSize(@)", noSideEffect,
    header: joltBindingsHeader.}
proc delete*(state: ptr WorldStateHandle)
  {.importcpp: "delete #", header: joltBindingsHeader.}
