## Ownership-aware core API for Jolt Physics.

import std/[math, options, sets]
import jolt/bridge as raw

type
  JoltError* = object of CatchableError

  Vec3* = object
    x*, y*, z*: float32

  Quat* = object
    x*, y*, z*, w*: float32

  MaterialColor* = object
    r*, g*, b*, a*: uint8

  PhysicsMaterial* = object
    name*: string
    debugColor*: MaterialColor

  ShapeKind* = enum
    Box
    Sphere
    Capsule
    Cylinder
    TaperedCapsule
    TaperedCylinder
    Triangle
    Plane
    Empty
    ConvexHull
    TriangleMesh
    HeightField
    StaticCompound
    MutableCompound
    Scaled
    RotatedTranslated
    OffsetCenterOfMass

  Shape* = object
    kind*: ShapeKind
    material*: Option[PhysicsMaterial]
    materials*: seq[PhysicsMaterial]
    materialIndices*: seq[uint32]
    halfExtent*: Vec3
    convexRadius*: float32
    radius*: float32
    topRadius*: float32
    bottomRadius*: float32
    halfHeight*: float32
    planeNormal*: Vec3
    planeConstant*: float32
    planeHalfExtent*: float32
    centerOfMass*: Vec3
    points*: seq[Vec3]
    vertices*: seq[Vec3]
    triangleIndices*: seq[uint32]
    heightSamples*: seq[float32]
    sampleCount*: uint32
    heightOffset*: Vec3
    heightScale*: Vec3
    blockSize*: uint32
    bitsPerSample*: uint32
    children*: seq[CompoundChild]
    innerShapes*: seq[Shape]
    shapeScale*: Vec3
    shapePosition*: Vec3
    shapeRotation*: Quat
    centerOfMassOffset*: Vec3

  CompoundChild* = object
    shape*: Shape
    position*: Vec3
    rotation*: Quat

  CompoundChildTransform* = object
    position*: Vec3
    rotation*: Quat

  MotionType* = enum
    Static
    Kinematic
    Dynamic

  MotionQuality* = enum
    Discrete
    LinearCast

  AllowedDOF* = enum
    TranslationXAxis
    TranslationYAxis
    TranslationZAxis
    RotationXAxis
    RotationYAxis
    RotationZAxis

  UpdateError* = enum
    ManifoldCacheFull
    BodyPairCacheFull
    ContactConstraintsFull

  ConstraintKind* = enum
    Point
    Distance
    Fixed
    Hinge
    Slider
    Cone
    SwingTwist
    SixDOF
    Gear
    Pulley
    RackAndPinion
    Path

  PathRotationConstraintType* = enum
    PathRotationFree
    PathRotationAroundTangent
    PathRotationAroundNormal
    PathRotationAroundBinormal
    PathRotationToPath
    PathRotationFullyConstrained

  PathPoint* = object
    position*: Vec3
    tangent*: Vec3
    normal*: Vec3

  MotorState* = enum
    Disabled
    Velocity
    Position
    PositionAndVelocity

  SpringMode* = enum
    FrequencyAndDamping
    StiffnessAndDamping
    MassNormalizedStiffnessAndDamping

  SpringSettings* = object
    mode*: SpringMode
    value*: float32
    damping*: float32

  MotorSettings* = object
    spring*: SpringSettings
    minForce*: float32
    maxForce*: float32
    minTorque*: float32
    maxTorque*: float32

  SixDOFAxis* = enum
    TranslationX
    TranslationY
    TranslationZ
    RotationX
    RotationY
    RotationZ

  SixDOFAxisMode* = enum
    AxisFree
    AxisFixed
    AxisLimited

  SixDOFSwingType* = enum
    SwingCone
    SwingPyramid

  SixDOFAxisLimit* = object
    mode*: SixDOFAxisMode
    minimum*: float32
    maximum*: float32

  SixDOFConfig* = object
    axisX*: Vec3
    axisY*: Vec3
    swingType*: SixDOFSwingType
    limits*: array[SixDOFAxis, SixDOFAxisLimit]

  RagdollScalarMotorPreset* = object
    settings*: MotorSettings
    state*: MotorState
    targetVelocity*: float32
    targetPosition*: float32

  RagdollSwingTwistMotorPreset* = object
    swingSettings*: MotorSettings
    twistSettings*: MotorSettings
    swingState*: MotorState
    twistState*: MotorState
    targetAngularVelocity*: Vec3
    targetOrientation*: Quat

  RagdollSixDOFMotorPreset* = object
    settings*: array[SixDOFAxis, MotorSettings]
    states*: array[SixDOFAxis, MotorState]
    targetVelocity*: Vec3
    targetAngularVelocity*: Vec3
    targetPosition*: Vec3
    targetOrientation*: Quat

  CharacterGroundState* = enum
    OnGround
    OnSteepGround
    NotSupported
    InAir

  CharacterBackFaceMode* = enum
    CollideWithBackFaces
    IgnoreBackFaces

  CharacterConfig* = object
    maxSlopeAngle*: float32
    mass*: float32
    maxStrength*: float32
    padding*: float32
    predictiveContactDistance*: float32
    maxNumHits*: uint32
    hitReductionCosMaxAngle*: float32
    penetrationRecoverySpeed*: float32
    stepUp*: float32
    stepDown*: float32
    enhancedInternalEdgeRemoval*: bool
    backFaceMode*: CharacterBackFaceMode
    maxCollisionIterations*: uint32
    maxConstraintIterations*: uint32
    minTimeRemaining*: float32
    collisionTolerance*: float32
    userData*: uint64
    innerBodyShape*: Option[Shape]
    innerBodyLayer*: CollisionLayer
    maxQueuedContactEvents*: uint32
    canPushCharacter*: bool
    canReceiveImpulses*: bool
    preventSliding*: bool

  CharacterContactInfo* = object
    bodyId*: Option[BodyId]
    characterId*: Option[uint32]
    subShapeId*: uint32
    position*: Vec3
    linearVelocity*: Vec3
    contactNormal*: Vec3
    surfaceNormal*: Vec3
    distance*: float32
    fraction*: float32
    motionType*: MotionType
    isSensor*: bool
    userData*: uint64
    hadCollision*: bool
    wasDiscarded*: bool
    canPushCharacter*: bool
    isBackFacing*: bool

  CharacterContactEventKind* = enum
    BodyContactAdded
    BodyContactPersisted
    BodyContactRemoved
    BodyContactSolved
    VirtualContactAdded
    VirtualContactPersisted
    VirtualContactRemoved
    VirtualContactSolved

  CharacterContactEvent* = object
    kind*: CharacterContactEventKind
    bodyId*: Option[BodyId]
    characterId*: Option[uint32]
    subShapeId*: uint32
    position*: Vec3
    normal*: Vec3
    contactVelocity*: Vec3
    characterVelocity*: Vec3
    resultingVelocity*: Vec3
    userData*: uint64
    isSensor*: bool
    canPushCharacter*: bool
    canReceiveImpulses*: bool

  RigidCharacterConfig* = object
    maxSlopeAngle*: float32
    up*: Vec3
    supportingHeight*: float32
    mass*: float32
    friction*: float32
    gravityFactor*: float32
    allowedDOFs*: set[AllowedDOF]
    enhancedInternalEdgeRemoval*: bool
    userData*: uint64
    maxSeparationDistance*: float32

  VehicleTransmissionMode* = enum
    Automatic
    Manual

  VehicleControllerKind* = enum
    Wheeled
    Motorcycle

  VehicleWheelCollisionMode* = enum
    Ray
    SphereCast
    CylinderCast

  TrackedVehicleSide* = enum
    LeftTrack
    RightTrack

  VehicleTorquePoint* = object
    rpmFraction*: float32
    torqueFraction*: float32

  VehicleTireFrictionPoint* = object
    slip*: float32
    friction*: float32

  VehicleWheelConfig* = object
    position*: Vec3
    suspensionForcePoint*: Vec3
    suspensionDirection*: Vec3
    steeringAxis*: Vec3
    wheelUp*: Vec3
    wheelForward*: Vec3
    suspensionMinLength*: float32
    suspensionMaxLength*: float32
    suspensionPreloadLength*: float32
    suspensionFrequency*: float32
    suspensionDamping*: float32
    radius*: float32
    width*: float32
    enableSuspensionForcePoint*: bool
    inertia*: float32
    angularDamping*: float32
    maxSteerAngle*: float32
    maxBrakeTorque*: float32
    maxHandBrakeTorque*: float32
    longitudinalImpulseMultiplier*: float32
    lateralImpulseMultiplier*: float32
    longitudinalFrictionCurve*: seq[VehicleTireFrictionPoint]
    lateralFrictionCurve*: seq[VehicleTireFrictionPoint]

  VehicleDifferentialConfig* = object
    leftWheel*: int
    rightWheel*: int
    differentialRatio*: float32
    leftRightSplit*: float32
    limitedSlipRatio*: float32
    engineTorqueRatio*: float32

  VehicleAntiRollBarConfig* = object
    leftWheel*: int
    rightWheel*: int
    stiffness*: float32

  VehicleConfig* = object
    controllerKind*: VehicleControllerKind
    wheelRadius*: float32
    wheelWidth*: float32
    suspensionMinLength*: float32
    suspensionMaxLength*: float32
    suspensionFrequency*: float32
    suspensionDamping*: float32
    maxSteerAngle*: float32
    maxPitchRollAngle*: float32
    engineMaxTorque*: float32
    engineMinRPM*: float32
    engineMaxRPM*: float32
    engineInertia*: float32
    engineAngularDamping*: float32
    engineTorqueCurve*: seq[VehicleTorquePoint]
    transmissionMode*: VehicleTransmissionMode
    gearRatios*: seq[float32]
    reverseGearRatios*: seq[float32]
    transmissionSwitchTime*: float32
    clutchReleaseTime*: float32
    transmissionSwitchLatency*: float32
    shiftUpRPM*: float32
    shiftDownRPM*: float32
    clutchStrength*: float32
    fourWheelDrive*: bool
    frontWheelDrive*: bool
    frontTorqueRatio*: float32
    differentialRatio*: float32
    differentialLeftRightSplit*: float32
    differentialLimitedSlipRatio*: float32
    centerDifferentialLimitedSlipRatio*: float32
    wheelTrack*: float32
    frontAxleOffset*: float32
    rearAxleOffset*: float32
    suspensionAttachmentHeightRatio*: float32
    rearMaxSteerAngle*: float32
    frontBrakeTorque*: float32
    rearBrakeTorque*: float32
    rearHandBrakeTorque*: float32
    antiRollBarStiffness*: float32
    wheelInertia*: float32
    wheelAngularDamping*: float32
    tireLongitudinalImpulseMultiplier*: float32
    tireLateralImpulseMultiplier*: float32
    wheelCollisionMode*: VehicleWheelCollisionMode
    wheelCollisionUp*: Vec3
    wheelCollisionMaxSlopeAngle*: float32
    wheelSphereCastRadius*: float32
    wheelCylinderConvexRadiusFraction*: float32
    wheels*: seq[VehicleWheelConfig]
    differentials*: seq[VehicleDifferentialConfig]
    antiRollBars*: seq[VehicleAntiRollBarConfig]

    # MotorcycleController settings. Ignored for a wheeled controller.
    maxLeanAngle*: float32
    leanSpringConstant*: float32
    leanSpringDamping*: float32
    leanSpringIntegrationCoefficient*: float32
    leanSpringIntegrationCoefficientDecay*: float32
    leanSmoothingFactor*: float32
    enableLeanController*: bool
    enableLeanSteeringLimit*: bool

  VehiclePowertrainState* = object
    engineRPM*: float32
    currentGear*: int
    clutchFriction*: float32
    switchingGear*: bool
    transmissionRatio*: float32
    wheelSpeedAtClutch*: float32

  MotorcycleControllerState* = object
    wheelBase*: float32
    leanControllerEnabled*: bool
    leanSteeringLimitEnabled*: bool
    leanSpringConstant*: float32
    leanSpringDamping*: float32
    leanSpringIntegrationCoefficient*: float32
    leanSpringIntegrationCoefficientDecay*: float32
    leanSmoothingFactor*: float32

  TrackedVehicleWheelConfig* = object
    position*: Vec3
    suspensionForcePoint*: Vec3
    suspensionDirection*: Vec3
    steeringAxis*: Vec3
    wheelUp*: Vec3
    wheelForward*: Vec3
    suspensionMinLength*: float32
    suspensionMaxLength*: float32
    suspensionPreloadLength*: float32
    suspensionFrequency*: float32
    suspensionDamping*: float32
    radius*: float32
    width*: float32
    enableSuspensionForcePoint*: bool
    longitudinalFriction*: float32
    lateralFriction*: float32

  TrackedVehicleTrackConfig* = object
    wheelIndices*: seq[int]
    drivenWheel*: int
    inertia*: float32
    angularDamping*: float32
    maxBrakeTorque*: float32
    differentialRatio*: float32

  TrackedVehicleConfig* = object
    maxPitchRollAngle*: float32
    engineMaxTorque*: float32
    engineMinRPM*: float32
    engineMaxRPM*: float32
    engineInertia*: float32
    engineAngularDamping*: float32
    engineTorqueCurve*: seq[VehicleTorquePoint]
    transmissionMode*: VehicleTransmissionMode
    gearRatios*: seq[float32]
    reverseGearRatios*: seq[float32]
    transmissionSwitchTime*: float32
    clutchReleaseTime*: float32
    transmissionSwitchLatency*: float32
    shiftUpRPM*: float32
    shiftDownRPM*: float32
    clutchStrength*: float32
    wheels*: seq[TrackedVehicleWheelConfig]
    tracks*: array[TrackedVehicleSide, TrackedVehicleTrackConfig]
    wheelCollisionMode*: VehicleWheelCollisionMode
    wheelCollisionUp*: Vec3
    wheelCollisionMaxSlopeAngle*: float32
    wheelSphereCastRadius*: float32
    wheelCylinderConvexRadiusFraction*: float32

  TrackedVehiclePowertrainState* = object
    engineRPM*: float32
    currentGear*: int
    clutchFriction*: float32
    switchingGear*: bool
    transmissionRatio*: float32

  TrackedVehicleTrackState* = object
    wheelIndices*: seq[int]
    drivenWheel*: int
    inertia*: float32
    angularDamping*: float32
    maxBrakeTorque*: float32
    differentialRatio*: float32
    angularVelocity*: float32

  BodyConfig* = object
    allowedDOFs*: set[AllowedDOF]
    motionQuality*: MotionQuality
    mass*: float32
    inertiaMultiplier*: float32
    linearVelocity*: Vec3
    angularVelocity*: Vec3
    userData*: uint64
    allowSleeping*: bool
    collideKinematicVsNonDynamic*: bool
    useManifoldReduction*: bool
    applyGyroscopicForce*: bool
    enhancedInternalEdgeRemoval*: bool
    friction*: float32
    restitution*: float32
    linearDamping*: float32
    angularDamping*: float32
    maxLinearVelocity*: float32
    maxAngularVelocity*: float32
    gravityFactor*: float32
    numVelocityStepsOverride*: uint32
    numPositionStepsOverride*: uint32
    massProperties*: Option[BodyMassProperties]

  BodyMassProperties* = object
    ## Principal moments and their orientation in body-local space.
    mass*: float32
    inertiaDiagonal*: Vec3
    inertiaRotation*: Quat

  BodyMotionSnapshot* = object
    ## Motion-only state captured while holding the body's read lock.
    motionQuality*: MotionQuality
    allowedDOFs*: set[AllowedDOF]
    linearDamping*: float32
    angularDamping*: float32
    maxLinearVelocity*: float32
    maxAngularVelocity*: float32
    gravityFactor*: float32
    allowSleeping*: bool
    collideKinematicVsNonDynamic*: bool
    applyGyroscopicForce*: bool
    enhancedInternalEdgeRemoval*: bool
    numVelocityStepsOverride*: uint32
    numPositionStepsOverride*: uint32
    mass*: Option[float32]
    massProperties*: Option[BodyMassProperties]

  BodySnapshot* = object
    ## A detached, internally consistent rigid-body state value.
    bodyId*: BodyId
    motionType*: MotionType
    collisionLayer*: CollisionLayer
    position*: Vec3
    centerOfMassPosition*: Vec3
    rotation*: Quat
    linearVelocity*: Vec3
    angularVelocity*: Vec3
    active*: bool
    sensor*: bool
    inBroadPhase*: bool
    collisionCacheInvalid*: bool
    useManifoldReduction*: bool
    friction*: float32
    restitution*: float32
    userData*: uint64
    motion*: Option[BodyMotionSnapshot]

  BodySpec* = object
    ## Complete description of one rigid body for `addBodies`.
    shape*: Shape
    position*: Vec3
    rotation*: Quat
    motionType*: MotionType
    layer*: CollisionLayer
    sensor*: bool
    config*: BodyConfig

  AuthoredConstraintConfig* = object
    enabled*: bool
    priority*: uint32
    velocityStepsOverride*: uint32
    positionStepsOverride*: uint32
    drawSize*: float32
    userData*: uint64

  SoftBodyBendType* = enum
    NoBend
    DistanceBend
    DihedralBend

  SoftBodyLRAType* = enum
    NoLRA
    EuclideanLRA
    GeodesicLRA

  SoftBodyVertex* = object
    position*: Vec3
    velocity*: Vec3
    inverseMass*: float32

  SoftBodyVertexAttributes* = object
    edgeCompliance*: float32
    shearCompliance*: float32
    bendCompliance*: float32
    lraType*: SoftBodyLRAType
    lraMaxDistanceMultiplier*: float32

  SoftBodyFace* = object
    vertices*: array[3, uint32]
    materialIndex*: uint32

  SoftBodyEdgeConstraint* = object
    vertices*: array[2, uint32]
    compliance*: float32

  SoftBodyDihedralBendConstraint* = object
    vertices*: array[4, uint32]
    compliance*: float32

  SoftBodyLongRangeConstraint* = object
    vertices*: array[2, uint32]
    maxDistance*: float32

  SoftBodyVolumeConstraint* = object
    vertices*: array[4, uint32]
    compliance*: float32

  SoftBodyRodConstraint* = object
    vertices*: array[2, uint32]
    compliance*: float32

  SoftBodyRodBendTwistConstraint* = object
    rods*: array[2, uint32]
    compliance*: float32

  SoftBodyRodOptimizationRemap* = object
    ## Maps authored rod constraint indices to Jolt's optimized native order.
    stretchShear*: seq[uint32]
    bendTwist*: seq[uint32]

  SoftBodyJointTransform* = object
    position*: Vec3
    rotation*: Quat

  SoftBodySkinWeight* = object
    joint*: uint32
    weight*: float32

  SoftBodySkinConstraint* = object
    vertex*: uint32
    weights*: seq[SoftBodySkinWeight]
    maxDistance*: float32
    backStopDistance*: float32
    backStopRadius*: float32

  SoftBodyMesh* = object
    vertices*: seq[SoftBodyVertex]
    vertexAttributes*: seq[SoftBodyVertexAttributes]
    faces*: seq[SoftBodyFace]
    materials*: seq[PhysicsMaterial]
    edgeConstraints*: seq[SoftBodyEdgeConstraint]
    dihedralBendConstraints*: seq[SoftBodyDihedralBendConstraint]
    longRangeConstraints*: seq[SoftBodyLongRangeConstraint]
    volumeConstraints*: seq[SoftBodyVolumeConstraint]
    rods*: seq[SoftBodyRodConstraint]
    rodBendTwistConstraints*: seq[SoftBodyRodBendTwistConstraint]
    skinBindPose*: seq[SoftBodyJointTransform]
    skinConstraints*: seq[SoftBodySkinConstraint]

  SoftBodyConfig* = object
    bendType*: SoftBodyBendType
    lraType*: SoftBodyLRAType
    lraMaxDistanceMultiplier*: float32
    edgeCompliance*: float32
    shearCompliance*: float32
    bendCompliance*: float32
    angleTolerance*: float32
    numIterations*: uint32
    linearDamping*: float32
    maxLinearVelocity*: float32
    restitution*: float32
    friction*: float32
    pressure*: float32
    gravityFactor*: float32
    vertexRadius*: float32
    updatePosition*: bool
    makeRotationIdentity*: bool
    allowSleeping*: bool
    userData*: uint64
    facesDoubleSided*: bool
    enableSkinConstraints*: bool
    skinnedMaxDistanceMultiplier*: float32
    material*: Option[PhysicsMaterial]

  SoftBodyVertexState* = object
    position*: Vec3
    velocity*: Vec3
    inverseMass*: float32

  SoftBodyRuntimeState* = object
    numIterations*: uint32
    pressure*: float32
    vertexRadius*: float32
    volume*: float32
    updatePosition*: bool
    facesDoubleSided*: bool
    skinConstraintsEnabled*: bool
    skinnedMaxDistanceMultiplier*: float32

  SoftBodyConstraintCounts* = object
    edges*: int
    dihedralBends*: int
    volumes*: int
    longRangeAttachments*: int
    rods*: int
    rodBendTwists*: int
    skinned*: int

  SoftBodyRodState* = object
    rotation*: Quat
    angularVelocity*: Vec3

  SoftBodyBounds* = object
    minimum*: Vec3
    maximum*: Vec3

  CollisionLayer* = raw.ObjectLayer

  RagdollJointKind* = enum
    RagdollSwingTwist
    RagdollHinge
    RagdollPoint
    RagdollFixed
    RagdollCone
    RagdollSlider
    RagdollSixDOF

  RagdollJointConfig* = object
    parent*: int
    kind*: RagdollJointKind
    position*: Vec3
    twistAxis*: Vec3
    planeAxis*: Vec3
    normalHalfConeAngle*: float32
    planeHalfConeAngle*: float32
    twistMinAngle*: float32
    twistMaxAngle*: float32
    maxFrictionTorque*: float32
    motorFrequency*: float32
    motorDamping*: float32
    maxMotorTorque*: float32
    sixDOF*: SixDOFConfig
    linearFriction*: Vec3
    angularFriction*: Vec3

  RagdollDistanceConstraintConfig* = object
    part1*: int
    part2*: int
    point1*: Vec3
    point2*: Vec3
    minDistance*: float32
    maxDistance*: float32

  RagdollPointConstraintConfig* = object
    part1*: int
    part2*: int
    point*: Vec3

  RagdollFixedConstraintConfig* = object
    part1*: int
    part2*: int

  RagdollHingeConstraintConfig* = object
    part1*: int
    part2*: int
    point*: Vec3
    hingeAxis*: Vec3
    normalAxis*: Vec3
    minAngle*: float32
    maxAngle*: float32
    maxFrictionTorque*: float32
    motor*: Option[RagdollScalarMotorPreset]

  RagdollSliderConstraintConfig* = object
    part1*: int
    part2*: int
    point*: Vec3
    sliderAxis*: Vec3
    normalAxis*: Vec3
    minPosition*: float32
    maxPosition*: float32
    maxFrictionForce*: float32
    motor*: Option[RagdollScalarMotorPreset]

  RagdollSwingTwistConstraintConfig* = object
    part1*: int
    part2*: int
    point*: Vec3
    twistAxis*: Vec3
    planeAxis*: Vec3
    normalHalfConeAngle*: float32
    planeHalfConeAngle*: float32
    twistMinAngle*: float32
    twistMaxAngle*: float32
    maxFrictionTorque*: float32
    motor*: Option[RagdollSwingTwistMotorPreset]

  RagdollSixDOFConstraintConfig* = object
    part1*: int
    part2*: int
    point*: Vec3
    config*: SixDOFConfig
    linearFriction*: Vec3
    angularFriction*: Vec3
    motor*: Option[RagdollSixDOFMotorPreset]

  RagdollConeConstraintConfig* = object
    part1*: int
    part2*: int
    point*: Vec3
    twistAxis1*: Vec3
    twistAxis2*: Vec3
    halfConeAngle*: float32

  RagdollPartConfig* = object
    name*: string
    shape*: Shape
    position*: Vec3
    rotation*: Quat
    motionType*: MotionType
    layer*: CollisionLayer
    body*: BodyConfig
    joint*: RagdollJointConfig

  RagdollConfig* = object
    parts*: seq[RagdollPartConfig]
    distanceConstraints*: seq[RagdollDistanceConstraintConfig]
    pointConstraints*: seq[RagdollPointConstraintConfig]
    fixedConstraints*: seq[RagdollFixedConstraintConfig]
    hingeConstraints*: seq[RagdollHingeConstraintConfig]
    sliderConstraints*: seq[RagdollSliderConstraintConfig]
    swingTwistConstraints*: seq[RagdollSwingTwistConstraintConfig]
    sixDOFConstraints*: seq[RagdollSixDOFConstraintConfig]
    coneConstraints*: seq[RagdollConeConstraintConfig]
    groupId*: uint32
    disableParentChildCollisions*: bool
    stabilize*: bool
    calculateConstraintPriorities*: bool
    activate*: bool

  RagdollTransform* = object
    position*: Vec3
    rotation*: Quat

  SkeletonTransform* = object
    position*: Vec3
    rotation*: Quat

  SkeletonJoint* = object
    name*: string
    parent*: int
    neutralTransform*: SkeletonTransform

  SkeletonDefinition* = object
    joints*: seq[SkeletonJoint]

  SkeletalAnimationKeyframe* = object
    time*: float32
    transform*: SkeletonTransform

  SkeletalAnimationTrack* = object
    jointName*: string
    keyframes*: seq[SkeletalAnimationKeyframe]

  QueryLayerSet* = object
    layers: seq[CollisionLayer]

  CollisionLayerConfig* = object
    broadPhaseLayer*: uint8

  CollisionPair* = object
    layer1*: CollisionLayer
    layer2*: CollisionLayer

  ContactPolicyResponse* = enum
    ContactPolicyCollide
    ContactPolicySensor
    ContactPolicyReject

  ContactPolicy* = object
    layer1*: CollisionLayer
    layer2*: CollisionLayer
    response*: ContactPolicyResponse
    friction*: Option[float32]
    restitution*: Option[float32]
    inverseMassScale1*: float32
    inverseInertiaScale1*: float32
    inverseMassScale2*: float32
    inverseInertiaScale2*: float32
    linearSurfaceVelocity*: Vec3
    angularSurfaceVelocity*: Vec3

  BodyPairContactPolicy* = object
    response*: ContactPolicyResponse
    friction*: Option[float32]
    restitution*: Option[float32]
    inverseMassScale1*: float32
    inverseInertiaScale1*: float32
    inverseMassScale2*: float32
    inverseInertiaScale2*: float32
    linearSurfaceVelocity*: Vec3
    angularSurfaceVelocity*: Vec3

  ConstraintSolverImpulse* = object
    ## Accumulated impulses from the most recently solved simulation step.
    ## Distance, pulley and rack-and-pinion use position.x; gear uses
    ## rotation.x. Hinge/slider/path pack their native two-axis values into
    ## x/y. SwingTwist rotation uses twist/swingY/swingZ.
    position*: Vec3
    rotation*: Vec3
    limit*: float32
    motorTranslation*: Vec3
    motorRotation*: Vec3

  SimulationSettings* = object
    ## Advanced solver, contact-cache and sleeping controls corresponding to
    ## Jolt's PhysicsSettings. Distances ending in `Squared` use square units.
    maxInFlightBodyPairs*: int32
    stepListenersBatchSize*: int32
    stepListenerBatchesPerJob*: int32
    baumgarte*: float32
    speculativeContactDistance*: float32
    penetrationSlop*: float32
    linearCastThreshold*: float32
    linearCastMaxPenetration*: float32
    manifoldTolerance*: float32
    maxPenetrationDistance*: float32
    bodyPairCacheMaxDeltaPositionSquared*: float32
    bodyPairCacheCosMaxDeltaRotationDiv2*: float32
    contactNormalCosMaxDeltaRotation*: float32
    contactPointPreserveLambdaMaxDistanceSquared*: float32
    internalEdgeRemovalVertexToleranceSquared*: float32
    numVelocitySteps*: uint32
    numPositionSteps*: uint32
    minVelocityForRestitution*: float32
    timeBeforeSleep*: float32
    pointVelocitySleepThreshold*: float32
    deterministicSimulation*: bool
    constraintWarmStart*: bool
    useBodyPairContactCache*: bool
    useManifoldReduction*: bool
    useLargeIslandSplitter*: bool
    allowSleeping*: bool
    checkActiveEdges*: bool

  WorldConfig* = object
    maxBodies*: uint
    numBodyMutexes*: uint
    maxBodyPairs*: uint
    maxContactConstraints*: uint
    tempAllocatorBytes*: uint
    maxJobs*: uint
    maxBarriers*: uint
    numThreads*: int32
    maxQueuedEvents*: uint
    characterBroadPhaseCellSize*: float32
    collisionLayers*: seq[CollisionLayerConfig]
    collisionPairs*: seq[CollisionPair]
    contactPolicies*: seq[ContactPolicy]

  CharacterBroadPhaseStats* = object
    registeredCharacters*: uint32
    occupiedCells*: uint32
    queryCount*: uint64
    candidateCount*: uint64
    narrowPhaseTestCount*: uint64

  World* = ref WorldObj
  WorldState* = ref WorldStateObj
  Body* = ref BodyObj
  SoftBody* = ref SoftBodyObj
  Constraint* = ref ConstraintObj
  Character* = ref CharacterObj
  RigidCharacter* = ref RigidCharacterObj
  Vehicle* = ref VehicleObj
  TrackedVehicle* = ref TrackedVehicleObj
  Ragdoll* = ref RagdollObj
  SkeletonMapper* = ref SkeletonMapperObj
  SkeletalAnimation* = ref SkeletalAnimationObj
  PhysicsScene* = ref PhysicsSceneObj
  PhysicsSceneInstance* = ref PhysicsSceneInstanceObj

  PhysicsSceneObjectStreamFormat* = enum
    PhysicsSceneStreamText
    PhysicsSceneStreamBinary
  CollisionGroupFilter* = ref CollisionGroupFilterObj

  BodyCollisionGroup* = object
    filter*: CollisionGroupFilter
    groupId*: uint32
    subgroupId*: uint32

  BodyId* = distinct uint32

  QueryBodyInfo* = object
    ## Detached body properties captured under one native multi-body read lock.
    bodyId*: BodyId
    motionType*: MotionType
    collisionLayer*: CollisionLayer
    position*: Vec3
    active*: bool
    sensor*: bool
    softBody*: bool
    inBroadPhase*: bool
    userData*: uint64

  BodyQueryCriteria* = object
    ## Empty motion/layer sets and `none` boolean/user-data values match any.
    motionTypes*: set[MotionType]
    layers*: seq[CollisionLayer]
    active*: Option[bool]
    sensor*: Option[bool]
    softBody*: Option[bool]
    inBroadPhase*: Option[bool]
    userData*: Option[uint64]

  QueryBodyPredicate* = proc(info: QueryBodyInfo): bool {.closure.}

  QueryBodyFilterMode* = enum
    IncludeOnly
    Exclude

  QueryBodyFilter* = object
    bodyIds: seq[uint32]
    mode: QueryBodyFilterMode
    enabled: bool

  DebugShapeColorMode* = enum
    DebugInstanceColor
    DebugShapeTypeColor
    DebugMotionTypeColor
    DebugSleepColor
    DebugIslandColor
    DebugMaterialColor

  DebugSoftBodyConstraintColorMode* = enum
    DebugConstraintTypeColor
    DebugConstraintGroupColor
    DebugConstraintOrderColor

  DebugColor* = object
    r*, g*, b*, a*: uint8

  DebugLine* = object
    fromPosition*, toPosition*: Vec3
    color*: DebugColor

  DebugTriangle* = object
    v1*, v2*, v3*: Vec3
    color*: DebugColor
    castsShadow*: bool

  DebugText* = object
    position*: Vec3
    text*: string
    color*: DebugColor
    height*: float32

  DebugBodyDrawSettings* = object
    drawGetSupportFunction*: bool
    drawSupportDirection*: bool
    drawGetSupportingFace*: bool
    drawShape*: bool
    drawShapeWireframe*: bool
    shapeColor*: DebugShapeColorMode
    drawBoundingBox*: bool
    drawCenterOfMassTransform*: bool
    drawWorldTransform*: bool
    drawVelocity*: bool
    drawMassAndInertia*: bool
    drawSleepStats*: bool
    drawSoftBodyVertices*: bool
    drawSoftBodyVertexVelocities*: bool
    drawSoftBodyEdgeConstraints*: bool
    drawSoftBodyBendConstraints*: bool
    drawSoftBodyVolumeConstraints*: bool
    drawSoftBodySkinConstraints*: bool
    drawSoftBodyLRAConstraints*: bool
    drawSoftBodyRods*: bool
    drawSoftBodyRodStates*: bool
    drawSoftBodyRodBendTwistConstraints*: bool
    drawSoftBodyPredictedBounds*: bool
    softBodyConstraintColor*: DebugSoftBodyConstraintColorMode

  DebugDrawLimits* = object
    maxLines*: uint
    maxTriangles*: uint
    maxTexts*: uint
    maxTextBytes*: uint

  DebugDrawOptions* = object
    cameraPosition*: Vec3
    bodySettings*: DebugBodyDrawSettings
    bodyFilter*: QueryBodyFilter
    drawBodies*: bool
    drawConstraints*: bool
    drawConstraintLimits*: bool
    drawConstraintReferenceFrames*: bool
    limits*: DebugDrawLimits

  DebugDrawFrame* = object
    ## Detached primitives ready for raylib, naylib, SDL3 or another renderer.
    lines*: seq[DebugLine]
    triangles*: seq[DebugTriangle]
    texts*: seq[DebugText]
    droppedLines*: uint64
    droppedTriangles*: uint64
    droppedTexts*: uint64

  QuerySubShape* = object
    bodyId*: BodyId
    subShapeId*: uint32

  QuerySubShapeFilterMode* = enum
    IncludeOnlySubShapes
    ExcludeSubShapes

  QuerySubShapeFilter* = object
    bodyIds: seq[uint32]
    subShapeIds: seq[uint32]
    mode: QuerySubShapeFilterMode

  RayHit* = object
    bodyId*: BodyId
    subShapeId*: uint32
    fraction*: float32
    distance*: float32
    position*: Vec3

  ShapeCastHit* = object
    bodyId*: BodyId
    subShapeId*: uint32
    fraction*: float32
    distance*: float32
    position*: Vec3
    contactPoint*: Vec3
    normal*: Vec3

  OverlapHit* = object
    bodyId*: BodyId
    subShapeId*: uint32
    penetrationDepth*: float32
    contactPoint*: Vec3
    normal*: Vec3

  BroadPhaseCastHit* = object
    bodyId*: BodyId
    fraction*: float32
    distance*: float32

  BroadPhaseBounds* = object
    minimum*: Vec3
    maximum*: Vec3

  VehicleWheelState* = object
    position*: Vec3
    rotation*: Quat
    hasContact*: bool
    contactBodyId*: Option[BodyId]
    contactSubShapeId*: Option[uint32]
    contactPosition*: Vec3
    contactNormal*: Vec3
    contactPointVelocity*: Vec3
    contactLongitudinal*: Vec3
    contactLateral*: Vec3
    suspensionLength*: float32
    hitHardPoint*: bool
    suspensionImpulse*: float32
    longitudinalImpulse*: float32
    lateralImpulse*: float32
    angularVelocity*: float32
    steerAngle*: float32
    longitudinalSlip*: float32
    lateralSlip*: float32
    combinedLongitudinalFriction*: float32
    combinedLateralFriction*: float32

  VehicleDifferentialState* = object
    leftWheel*: int
    rightWheel*: int
    differentialRatio*: float32
    leftRightSplit*: float32
    limitedSlipRatio*: float32
    engineTorqueRatio*: float32

  PhysicsEventKind* = enum
    ContactAdded
    ContactPersisted
    ContactRemoved
    BodyActivated
    BodyDeactivated

  PhysicsEvent* = object
    kind*: PhysicsEventKind
    body1*: BodyId
    body2*: Option[BodyId]
    subShapeId1*: Option[uint32]
    subShapeId2*: Option[uint32]
    contactPoint*: Vec3
    contactNormal*: Vec3
    hasManifold*: bool

  SoftBodyContactEvent* = object
    softBody*: BodyId
    otherBody*: BodyId
    vertex*: Option[uint32]
    contactPoint*: Vec3
    contactNormal*: Vec3
    isSensor*: bool

  WorldObj = object
    physics: ptr raw.PhysicsSystem
    allocator: ptr raw.TempAllocatorImpl
    jobs: ptr raw.JobSystemThreadPool
    objectPairs: ptr raw.ObjectLayerPairFilterTable
    broadPhases: ptr raw.BroadPhaseLayerInterfaceTable
    objectVsBroadPhase: ptr raw.ObjectVsBroadPhaseLayerFilterTable
    eventBridge: ptr raw.EventBridge
    characterBroadPhase: ptr raw.CharacterBroadPhase
    bodyIds: seq[uint32]
    constraints: seq[ptr raw.Constraint]
    characters: seq[ptr raw.CharacterHandle]
    characterBodyIds: seq[uint32]
    rigidCharacters: seq[ptr raw.RigidCharacterHandle]
    rigidCharacterBodyIds: seq[uint32]
    vehicles: seq[ptr raw.VehicleHandle]
    ragdolls: seq[ptr raw.RagdollHandle]
    sceneInstances: seq[ptr raw.PhysicsSceneInstanceHandle]
    ragdollBodyIds: seq[uint32]
    layerCount: uint32
    acquiredJolt: bool
    closing: bool
    closed: bool

  WorldStateObj = object
    owner: World
    native: ptr raw.WorldStateHandle
    bodyIds: seq[uint32]
    constraints: seq[ptr raw.Constraint]
    characters: seq[ptr raw.CharacterHandle]
    rigidCharacters: seq[ptr raw.RigidCharacterHandle]
    vehicles: seq[ptr raw.VehicleHandle]
    ragdolls: seq[ptr raw.RagdollHandle]
    alive: bool

  BodyObj = object
    owner: World
    rawId: uint32
    shapeDesc: Shape
    motion: MotionType
    constraintCount: int
    sensor: bool
    group: Option[BodyCollisionGroup]
    alive: bool

  SoftBodyObj = object
    owner: World
    rawId: uint32
    meshDesc: SoftBodyMesh
    rodRemap: seq[uint32]
    rodPairRemap: seq[uint32]
    config: SoftBodyConfig
    layer: CollisionLayer
    group: Option[BodyCollisionGroup]
    alive: bool

  CollisionGroupFilterObj = object
    native: ptr raw.GroupFilterTable
    subgroupCount: uint32
    alive: bool

  ConstraintObj = object
    owner: World
    native: ptr raw.Constraint
    body1: Body
    body2: Body
    dependencies: seq[Constraint]
    pathMaxFraction: float32
    pathLooping: bool
    constraintKind: ConstraintKind
    alive: bool

  CharacterObj = object
    owner: World
    native: ptr raw.CharacterHandle
    shapeDesc: Shape
    config: CharacterConfig
    layer: CollisionLayer
    innerBodyIdValue: Option[uint32]
    alive: bool

  RigidCharacterObj = object
    owner: World
    native: ptr raw.RigidCharacterHandle
    shapeDesc: Shape
    config: RigidCharacterConfig
    layer: CollisionLayer
    rawId: uint32
    alive: bool

  VehicleObj = object
    owner: World
    native: ptr raw.VehicleHandle
    chassisBody: Body
    config: VehicleConfig
    wheelLayer: CollisionLayer
    alive: bool

  TrackedVehicleObj = object
    owner: World
    native: ptr raw.VehicleHandle
    chassisBody: Body
    config: TrackedVehicleConfig
    wheelLayer: CollisionLayer
    alive: bool

  RagdollObj = object
    owner: World
    native: ptr raw.RagdollHandle
    config: RagdollConfig
    bodyIds: seq[uint32]
    alive: bool

  SkeletonMapperObj = object
    native: ptr raw.SkeletonMapperHandle
    source: SkeletonDefinition
    target: SkeletonDefinition
    acquiredJolt: bool
    alive: bool

  SkeletalAnimationObj = object
    native: ptr raw.SkeletalAnimationHandle
    skeleton: SkeletonDefinition
    tracks: seq[SkeletalAnimationTrack]
    acquiredJolt: bool
    alive: bool

  PhysicsSceneObj = object
    native: ptr raw.PhysicsSceneHandle
    acquiredJolt: bool
    alive: bool

  PhysicsSceneInstanceObj = object
    owner: World
    native: ptr raw.PhysicsSceneInstanceHandle
    bodyIds: seq[uint32]
    constraints: seq[ptr raw.Constraint]
    alive: bool

const
  nonMovingLayer* = CollisionLayer(0)
  movingLayer* = CollisionLayer(1)
  fixedWorldBodyIndex* = -1
  maxFiniteFloat32 = cast[float32](0x7f7fffff'u32)
  maxSceneSerializedBytes = 512 * 1024 * 1024
  maxAnimationSerializedBytes = 256 * 1024 * 1024
  physicsSceneMagic = [
    0x4a'u8, 0x4e'u8, 0x53'u8, 0x43'u8,
    0x45'u8, 0x4e'u8, 0x45'u8, 0x01'u8]
  physicsSceneFormatVersion = 1'u32
  physicsSceneHeaderSize = 28
  skeletalAnimationMagic = [
    0x4a'u8, 0x4e'u8, 0x41'u8, 0x4e'u8,
    0x49'u8, 0x4d'u8, 0x00'u8, 0x01'u8]
  skeletalAnimationFormatVersion = 1'u32
  skeletalAnimationHeaderSize = 28
  fnvOffsetBasis64 = 14695981039346656037'u64
  fnvPrime64 = 1099511628211'u64
  heightFieldNoCollision* = maxFiniteFloat32
  softBodyDisabledCompliance* = maxFiniteFloat32

proc closeBody(body: var BodyObj) {.raises: [].}
proc closeSoftBody(body: var SoftBodyObj) {.raises: [].}
proc closeConstraint(constraint: var ConstraintObj) {.raises: [].}
proc closeCharacter(character: var CharacterObj) {.raises: [].}
proc closeRigidCharacter(character: var RigidCharacterObj) {.raises: [].}
proc closeVehicle(vehicle: var VehicleObj) {.raises: [].}
proc closeTrackedVehicle(vehicle: var TrackedVehicleObj) {.raises: [].}
proc closeRagdoll(ragdoll: var RagdollObj) {.raises: [].}
proc closeSkeletonMapper(mapper: var SkeletonMapperObj) {.raises: [].}
proc closeSkeletalAnimation(animation: var SkeletalAnimationObj) {.raises: [].}
proc closePhysicsScene(scene: var PhysicsSceneObj) {.raises: [].}
proc closePhysicsSceneInstance(
  instance: var PhysicsSceneInstanceObj) {.raises: [].}
proc closeCollisionGroupFilter(filter: var CollisionGroupFilterObj) {.raises: [].}
proc closeWorld(world: var WorldObj) {.raises: [].}
proc closeWorldState(state: var WorldStateObj) {.raises: [].}
proc requireMotionProperties(body: Body; operation: string)

proc `=destroy`(body: var BodyObj) =
  closeBody(body)

proc `=destroy`(body: var SoftBodyObj) =
  closeSoftBody(body)

proc `=destroy`(constraint: var ConstraintObj) =
  closeConstraint(constraint)

proc `=destroy`(character: var CharacterObj) =
  closeCharacter(character)

proc `=destroy`(character: var RigidCharacterObj) =
  closeRigidCharacter(character)

proc `=destroy`(vehicle: var VehicleObj) =
  closeVehicle(vehicle)

proc `=destroy`(vehicle: var TrackedVehicleObj) =
  closeTrackedVehicle(vehicle)

proc `=destroy`(ragdoll: var RagdollObj) =
  closeRagdoll(ragdoll)

proc `=destroy`(mapper: var SkeletonMapperObj) =
  closeSkeletonMapper(mapper)

proc `=destroy`(animation: var SkeletalAnimationObj) =
  closeSkeletalAnimation(animation)

proc `=destroy`(scene: var PhysicsSceneObj) =
  closePhysicsScene(scene)

proc `=destroy`(instance: var PhysicsSceneInstanceObj) =
  closePhysicsSceneInstance(instance)

proc `=destroy`(filter: var CollisionGroupFilterObj) =
  closeCollisionGroupFilter(filter)

proc `=destroy`(world: var WorldObj) =
  closeWorld(world)

proc `=destroy`(state: var WorldStateObj) =
  closeWorldState(state)

func vec3*[X, Y, Z: SomeNumber](x: X; y: Y; z: Z): Vec3 =
  Vec3(x: float32(x), y: float32(y), z: float32(z))

proc materialColor*[R, G, B: SomeInteger](r: R; g: G; b: B;
                                          a = 255): MaterialColor =
  for value in [int64(r), int64(g), int64(b), int64(a)]:
    if value < 0 or value > 255:
      raise newException(ValueError, "material color channels must be in [0, 255]")
  MaterialColor(r: uint8(r), g: uint8(g), b: uint8(b), a: uint8(a))

proc physicsMaterial*(name: string;
                      debugColor = materialColor(128, 128, 128)): PhysicsMaterial =
  if name.len == 0:
    raise newException(ValueError, "physics material name must not be empty")
  if '\0' in name:
    raise newException(ValueError, "physics material name must not contain NUL")
  PhysicsMaterial(name: name, debugColor: debugColor)

func pathPoint*(position, tangent, normal: Vec3): PathPoint =
  PathPoint(position: position, tangent: tangent, normal: normal)

func vehicleTorquePoint*[X, Y: SomeNumber](
    rpmFraction: X; torqueFraction: Y): VehicleTorquePoint =
  VehicleTorquePoint(
    rpmFraction: float32(rpmFraction),
    torqueFraction: float32(torqueFraction))

func vehicleTireFrictionPoint*[X, Y: SomeNumber](
    slip: X; friction: Y): VehicleTireFrictionPoint =
  VehicleTireFrictionPoint(
    slip: float32(slip),
    friction: float32(friction))

func defaultVehicleWheelConfig*(position: Vec3): VehicleWheelConfig =
  VehicleWheelConfig(
    position: position,
    suspensionForcePoint: position,
    suspensionDirection: vec3(0, -1, 0),
    steeringAxis: vec3(0, 1, 0),
    wheelUp: vec3(0, 1, 0),
    wheelForward: vec3(0, 0, 1),
    suspensionMinLength: 0.2,
    suspensionMaxLength: 0.45,
    suspensionPreloadLength: 0,
    suspensionFrequency: 1.5,
    suspensionDamping: 0.5,
    radius: 0.35,
    width: 0.2,
    enableSuspensionForcePoint: false,
    inertia: 0.9,
    angularDamping: 0.2,
    maxSteerAngle: 0,
    maxBrakeTorque: 1_500,
    maxHandBrakeTorque: 0,
    longitudinalImpulseMultiplier: 10,
    lateralImpulseMultiplier: 1)

func defaultTrackedVehicleWheelConfig*(
    position: Vec3): TrackedVehicleWheelConfig =
  TrackedVehicleWheelConfig(
    position: position,
    suspensionForcePoint: position,
    suspensionDirection: vec3(0, -1, 0),
    steeringAxis: vec3(0, 1, 0),
    wheelUp: vec3(0, 1, 0),
    wheelForward: vec3(0, 0, 1),
    suspensionMinLength: 0.3,
    suspensionMaxLength: 0.5,
    suspensionPreloadLength: 0,
    suspensionFrequency: 1,
    suspensionDamping: 0.5,
    radius: 0.3,
    width: 0.1,
    enableSuspensionForcePoint: false,
    longitudinalFriction: 4,
    lateralFriction: 2)

proc trackedVehicleTrack*(wheelIndices: openArray[int]; drivenWheel: int;
                          inertia = 10.0'f32;
                          angularDamping = 0.5'f32;
                          maxBrakeTorque = 15_000.0'f32;
                          differentialRatio = 6.0'f32):
    TrackedVehicleTrackConfig =
  result.wheelIndices = @wheelIndices
  result.drivenWheel = drivenWheel
  result.inertia = inertia
  result.angularDamping = angularDamping
  result.maxBrakeTorque = maxBrakeTorque
  result.differentialRatio = differentialRatio

func vehicleDifferential*(leftWheel, rightWheel: int;
                          engineTorqueRatio = 1.0'f32;
                          differentialRatio = 3.42'f32;
                          leftRightSplit = 0.5'f32;
                          limitedSlipRatio = 1.4'f32): VehicleDifferentialConfig =
  VehicleDifferentialConfig(
    leftWheel: leftWheel,
    rightWheel: rightWheel,
    differentialRatio: differentialRatio,
    leftRightSplit: leftRightSplit,
    limitedSlipRatio: limitedSlipRatio,
    engineTorqueRatio: engineTorqueRatio)

func vehicleAntiRollBar*(leftWheel, rightWheel: int;
                         stiffness = 1_000.0'f32): VehicleAntiRollBarConfig =
  VehicleAntiRollBarConfig(
    leftWheel: leftWheel,
    rightWheel: rightWheel,
    stiffness: stiffness)

func quatIdentity*(): Quat =
  Quat(w: 1)

func collisionLayerConfig*[T: SomeInteger](broadPhaseLayer: T): CollisionLayerConfig =
  if broadPhaseLayer < 0 or uint64(broadPhaseLayer) > uint64(high(uint8)):
    raise newException(
      ValueError,
      "broadPhaseLayer must fit in an unsigned 8-bit integer"
    )
  CollisionLayerConfig(broadPhaseLayer: uint8(broadPhaseLayer))

func collisionPair*(layer1, layer2: CollisionLayer): CollisionPair =
  CollisionPair(layer1: layer1, layer2: layer2)

func contactPolicy*(layer1, layer2: CollisionLayer;
                    response = ContactPolicyCollide;
                    friction = none(float32);
                    restitution = none(float32);
                    inverseMassScale1 = 1.0'f32;
                    inverseInertiaScale1 = 1.0'f32;
                    inverseMassScale2 = 1.0'f32;
                    inverseInertiaScale2 = 1.0'f32;
                    linearSurfaceVelocity = vec3(0, 0, 0);
                    angularSurfaceVelocity = vec3(0, 0, 0)):
                    ContactPolicy =
  ## Defines worker-safe contact behavior for an enabled layer pair.
  ## Reject, sensor and directional inverse-mass settings also apply to
  ## soft-body versus rigid-body contacts. Friction, restitution, surface
  ## velocity and the soft body's inertia scale are rigid-contact-only because
  ## Jolt does not expose those settings for soft contacts.
  ## Surface velocities use layer2 minus layer1 orientation.
  ContactPolicy(
    layer1: layer1,
    layer2: layer2,
    response: response,
    friction: friction,
    restitution: restitution,
    inverseMassScale1: inverseMassScale1,
    inverseInertiaScale1: inverseInertiaScale1,
    inverseMassScale2: inverseMassScale2,
    inverseInertiaScale2: inverseInertiaScale2,
    linearSurfaceVelocity: linearSurfaceVelocity,
    angularSurfaceVelocity: angularSurfaceVelocity)

func bodyPairContactPolicy*(response = ContactPolicyCollide;
                            friction = none(float32);
                            restitution = none(float32);
                            inverseMassScale1 = 1.0'f32;
                            inverseInertiaScale1 = 1.0'f32;
                            inverseMassScale2 = 1.0'f32;
                            inverseInertiaScale2 = 1.0'f32;
                            linearSurfaceVelocity = vec3(0, 0, 0);
                            angularSurfaceVelocity = vec3(0, 0, 0)):
                            BodyPairContactPolicy =
  ## Defines mutable contact behavior for one exact pair of bodies. Directional
  ## settings use the body1/body2 order supplied when the policy is installed.
  BodyPairContactPolicy(
    response: response,
    friction: friction,
    restitution: restitution,
    inverseMassScale1: inverseMassScale1,
    inverseInertiaScale1: inverseInertiaScale1,
    inverseMassScale2: inverseMassScale2,
    inverseInertiaScale2: inverseInertiaScale2,
    linearSurfaceVelocity: linearSurfaceVelocity,
    angularSurfaceVelocity: angularSurfaceVelocity)

func springSettings*[V, D: SomeNumber](
    value: V; damping: D;
    mode = SpringMode.FrequencyAndDamping): SpringSettings =
  SpringSettings(
    mode: mode,
    value: float32(value),
    damping: float32(damping)
  )

func defaultMotorSettings*(): MotorSettings =
  MotorSettings(
    spring: springSettings(2, 1),
    minForce: -maxFiniteFloat32,
    maxForce: maxFiniteFloat32,
    minTorque: -maxFiniteFloat32,
    maxTorque: maxFiniteFloat32
  )

func freeAxis*(): SixDOFAxisLimit =
  SixDOFAxisLimit(mode: SixDOFAxisMode.AxisFree)

func fixedAxis*(): SixDOFAxisLimit =
  SixDOFAxisLimit(mode: SixDOFAxisMode.AxisFixed)

func limitedAxis*[A, B: SomeNumber](
    minimum: A; maximum: B): SixDOFAxisLimit =
  SixDOFAxisLimit(
    mode: SixDOFAxisMode.AxisLimited,
    minimum: float32(minimum),
    maximum: float32(maximum)
  )

func defaultSixDOFConfig*(): SixDOFConfig =
  result.axisX = vec3(1, 0, 0)
  result.axisY = vec3(0, 1, 0)
  result.swingType = SixDOFSwingType.SwingPyramid
  for axis in SixDOFAxis:
    result.limits[axis] = fixedAxis()

func ragdollScalarMotorPreset*(state: MotorState;
                               targetVelocity = 0'f32;
                               targetPosition = 0'f32;
                               settings = defaultMotorSettings()):
                               RagdollScalarMotorPreset =
  RagdollScalarMotorPreset(
    settings: settings, state: state,
    targetVelocity: targetVelocity, targetPosition: targetPosition)

func ragdollSwingTwistMotorPreset*(swingState, twistState: MotorState;
                                   targetAngularVelocity = vec3(0, 0, 0);
                                   targetOrientation = quatIdentity();
                                   swingSettings = defaultMotorSettings();
                                   twistSettings = defaultMotorSettings()):
                                   RagdollSwingTwistMotorPreset =
  RagdollSwingTwistMotorPreset(
    swingSettings: swingSettings, twistSettings: twistSettings,
    swingState: swingState, twistState: twistState,
    targetAngularVelocity: targetAngularVelocity,
    targetOrientation: targetOrientation)

func defaultRagdollSixDOFMotorPreset*(): RagdollSixDOFMotorPreset =
  result.targetOrientation = quatIdentity()
  for axis in SixDOFAxis:
    result.settings[axis] = defaultMotorSettings()
    result.states[axis] = MotorState.Disabled

func defaultAuthoredConstraintConfig*(): AuthoredConstraintConfig =
  ## Common Jolt ConstraintSettings values stored in PhysicsScene streams.
  AuthoredConstraintConfig(enabled: true, drawSize: 1)

func `==`*(left, right: BodyId): bool =
  uint32(left) == uint32(right)

func isFinite(value: float32): bool =
  classify(value) in {fcNormal, fcSubnormal, fcZero, fcNegZero}

func isFinite(value: Vec3): bool =
  value.x.isFinite and value.y.isFinite and value.z.isFinite

func isUnitVector(value: Vec3): bool =
  if not value.isFinite:
    return false
  let lengthSquared =
    value.x * value.x + value.y * value.y + value.z * value.z
  abs(lengthSquared - 1.0'f32) <= 1.0e-3'f32

func isFinite(value: Quat): bool =
  value.x.isFinite and value.y.isFinite and value.z.isFinite and value.w.isFinite

proc validate(settings: SpringSettings) =
  if not settings.value.isFinite or settings.value < 0 or
      not settings.damping.isFinite or settings.damping < 0:
    raise newException(
      ValueError,
      "spring value and damping must be finite and non-negative"
    )

proc validate(settings: MotorSettings) =
  settings.spring.validate()
  if not settings.minForce.isFinite or not settings.maxForce.isFinite or
      settings.minForce > settings.maxForce:
    raise newException(ValueError, "motor force limits must be finite and ordered")
  if not settings.minTorque.isFinite or not settings.maxTorque.isFinite or
      settings.minTorque > settings.maxTorque:
    raise newException(ValueError, "motor torque limits must be finite and ordered")

proc decodeMotorState(value: uint8; name: string): MotorState =
  if value > uint8(ord(high(MotorState))):
    raise newException(JoltError, "Jolt returned an invalid " & name & " state")
  MotorState(value)

proc validate(limit: SixDOFAxisLimit; axis: SixDOFAxis) =
  if limit.mode != SixDOFAxisMode.AxisLimited:
    return
  if not limit.minimum.isFinite or not limit.maximum.isFinite or
      limit.minimum > limit.maximum:
    raise newException(ValueError, "SixDOF limited axes must be finite and ordered")
  if axis >= SixDOFAxis.RotationX and
      (limit.minimum < -PI.float32 or limit.maximum > PI.float32):
    raise newException(ValueError, "SixDOF rotation limits must be within [-PI, PI]")

proc validate(limit: SixDOFAxisLimit; axis: SixDOFAxis;
              swingType: SixDOFSwingType) =
  limit.validate(axis)
  if swingType == SixDOFSwingType.SwingCone and
      axis in {SixDOFAxis.RotationY, SixDOFAxis.RotationZ} and
      limit.mode == SixDOFAxisMode.AxisLimited and
      (limit.maximum < 0 or
       abs(limit.minimum + limit.maximum) > 1.0e-5'f32):
    raise newException(
      ValueError,
      "SixDOF cone swing limits must be non-negative and symmetric")

proc validateLimits(config: SixDOFConfig) =
  for axis in SixDOFAxis:
    config.limits[axis].validate(axis, config.swingType)

proc softBodyVertex*(position: Vec3; velocity = vec3(0, 0, 0);
                     inverseMass = 1.0'f32): SoftBodyVertex =
  if not position.isFinite or not velocity.isFinite:
    raise newException(
      ValueError, "soft body vertex position and velocity must be finite")
  if not inverseMass.isFinite or inverseMass < 0:
    raise newException(
      ValueError, "soft body vertex inverse mass must be finite and non-negative")
  SoftBodyVertex(
    position: position, velocity: velocity, inverseMass: inverseMass)

proc softBodyVertexAttributes*(edgeCompliance = 0.0'f32;
    shearCompliance = 0.0'f32; bendCompliance = softBodyDisabledCompliance;
    lraType = SoftBodyLRAType.NoLRA;
    lraMaxDistanceMultiplier = 1.0'f32): SoftBodyVertexAttributes =
  for value in [edgeCompliance, shearCompliance, bendCompliance]:
    if not value.isFinite or value < 0:
      raise newException(
        ValueError, "soft body vertex compliance must be non-negative")
  if not lraMaxDistanceMultiplier.isFinite or
      lraMaxDistanceMultiplier <= 0:
    raise newException(
      ValueError, "soft body vertex LRA multiplier must be positive")
  SoftBodyVertexAttributes(
    edgeCompliance: edgeCompliance,
    shearCompliance: shearCompliance,
    bendCompliance: bendCompliance,
    lraType: lraType,
    lraMaxDistanceMultiplier: lraMaxDistanceMultiplier)

proc softBodyFace*(vertex1, vertex2, vertex3: SomeInteger;
                   materialIndex: SomeInteger = 0): SoftBodyFace =
  for vertex in [int64(vertex1), int64(vertex2), int64(vertex3)]:
    if vertex < 0 or uint64(vertex) > uint64(high(uint32)):
      raise newException(ValueError, "soft body face index must fit uint32")
  if vertex1 == vertex2 or vertex1 == vertex3 or vertex2 == vertex3:
    raise newException(ValueError, "soft body face is degenerate")
  let materialValue = int64(materialIndex)
  if materialValue < 0 or uint64(materialValue) > uint64(high(uint32)):
    raise newException(ValueError, "soft body material index must fit uint32")
  SoftBodyFace(
    vertices: [uint32(vertex1), uint32(vertex2), uint32(vertex3)],
    materialIndex: uint32(materialIndex))

proc withMaterials*(mesh: SoftBodyMesh;
                    materials: openArray[PhysicsMaterial]): SoftBodyMesh =
  if materials.len == 0:
    raise newException(ValueError, "at least one soft body material is required")
  if uint64(materials.len) > uint64(high(uint32)):
    raise newException(ValueError, "soft body material count must fit uint32")
  for material in materials:
    discard physicsMaterial(material.name, material.debugColor)
  for face in mesh.faces:
    if uint64(face.materialIndex) >= uint64(materials.len):
      raise newException(ValueError, "soft body material index is out of bounds")
  result = mesh
  result.materials = @materials

proc softBodyEdgeConstraint*(vertex1, vertex2: SomeInteger;
                             compliance = 0.0'f32): SoftBodyEdgeConstraint =
  for vertex in [int64(vertex1), int64(vertex2)]:
    if vertex < 0 or uint64(vertex) > uint64(high(uint32)):
      raise newException(ValueError, "soft body edge index must fit uint32")
  if vertex1 == vertex2:
    raise newException(ValueError, "soft body edge needs two different vertices")
  if not compliance.isFinite or compliance < 0:
    raise newException(
      ValueError, "soft body edge compliance must be finite and non-negative")
  SoftBodyEdgeConstraint(
    vertices: [uint32(vertex1), uint32(vertex2)], compliance: compliance)

proc softBodyDihedralBendConstraint*(
    edgeVertex1, edgeVertex2, triangleVertex1,
    triangleVertex2: SomeInteger;
    compliance = 0.0'f32): SoftBodyDihedralBendConstraint =
  let vertices = [int64(edgeVertex1), int64(edgeVertex2),
    int64(triangleVertex1), int64(triangleVertex2)]
  for vertex in vertices:
    if vertex < 0 or uint64(vertex) > uint64(high(uint32)):
      raise newException(ValueError, "soft body dihedral index must fit uint32")
  for first in 0 ..< vertices.len:
    for second in first + 1 ..< vertices.len:
      if vertices[first] == vertices[second]:
        raise newException(
          ValueError, "soft body dihedral bend needs four vertices")
  if not compliance.isFinite or compliance < 0:
    raise newException(
      ValueError, "soft body dihedral compliance must be finite and non-negative")
  SoftBodyDihedralBendConstraint(
    vertices: [uint32(edgeVertex1), uint32(edgeVertex2),
      uint32(triangleVertex1), uint32(triangleVertex2)],
    compliance: compliance)

proc softBodyLongRangeConstraint*(kinematicVertex,
    dynamicVertex: SomeInteger; maxDistance: float32): SoftBodyLongRangeConstraint =
  for vertex in [int64(kinematicVertex), int64(dynamicVertex)]:
    if vertex < 0 or uint64(vertex) > uint64(high(uint32)):
      raise newException(ValueError, "soft body LRA index must fit uint32")
  if kinematicVertex == dynamicVertex:
    raise newException(ValueError, "soft body LRA needs two different vertices")
  if not maxDistance.isFinite or maxDistance < 0:
    raise newException(
      ValueError, "soft body LRA maximum distance must be non-negative")
  SoftBodyLongRangeConstraint(
    vertices: [uint32(kinematicVertex), uint32(dynamicVertex)],
    maxDistance: maxDistance)

proc softBodyVolumeConstraint*(vertex1, vertex2, vertex3, vertex4: SomeInteger;
                               compliance = 0.0'f32): SoftBodyVolumeConstraint =
  let vertices = [int64(vertex1), int64(vertex2), int64(vertex3), int64(vertex4)]
  for vertex in vertices:
    if vertex < 0 or uint64(vertex) > uint64(high(uint32)):
      raise newException(ValueError, "soft body volume index must fit uint32")
  for first in 0 ..< vertices.len:
    for second in first + 1 ..< vertices.len:
      if vertices[first] == vertices[second]:
        raise newException(ValueError, "soft body volume needs four vertices")
  if not compliance.isFinite or compliance < 0:
    raise newException(
      ValueError, "soft body volume compliance must be finite and non-negative")
  SoftBodyVolumeConstraint(
    vertices: [uint32(vertex1), uint32(vertex2), uint32(vertex3),
      uint32(vertex4)],
    compliance: compliance)

proc softBodyRodConstraint*(vertex1, vertex2: SomeInteger;
                            compliance = 0.0'f32): SoftBodyRodConstraint =
  for vertex in [int64(vertex1), int64(vertex2)]:
    if vertex < 0 or uint64(vertex) > uint64(high(uint32)):
      raise newException(ValueError, "soft body rod vertex index must fit uint32")
  if vertex1 == vertex2:
    raise newException(ValueError, "soft body rod needs two different vertices")
  if not compliance.isFinite or compliance < 0:
    raise newException(
      ValueError, "soft body rod compliance must be finite and non-negative")
  SoftBodyRodConstraint(
    vertices: [uint32(vertex1), uint32(vertex2)], compliance: compliance)

proc softBodyRodBendTwistConstraint*(rod1, rod2: SomeInteger;
    compliance = 0.0'f32): SoftBodyRodBendTwistConstraint =
  for rod in [int64(rod1), int64(rod2)]:
    if rod < 0 or uint64(rod) > uint64(high(uint32)):
      raise newException(ValueError, "soft body rod index must fit uint32")
  if rod1 == rod2:
    raise newException(ValueError, "rod bend-twist needs two different rods")
  if not compliance.isFinite or compliance < 0:
    raise newException(
      ValueError, "rod bend-twist compliance must be finite and non-negative")
  SoftBodyRodBendTwistConstraint(
    rods: [uint32(rod1), uint32(rod2)], compliance: compliance)

proc softBodyJointTransform*(position: Vec3;
                             rotation = quatIdentity()): SoftBodyJointTransform =
  if not position.isFinite or not rotation.isFinite:
    raise newException(ValueError, "soft body joint transform must be finite")
  let lengthSquared = rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "soft body joint rotation must be non-zero")
  let inverseLength = 1.0'f32 / sqrt(lengthSquared)
  SoftBodyJointTransform(
    position: position,
    rotation: Quat(
      x: rotation.x * inverseLength, y: rotation.y * inverseLength,
      z: rotation.z * inverseLength, w: rotation.w * inverseLength))

proc softBodySkinWeight*(joint: SomeInteger;
                         weight: SomeNumber): SoftBodySkinWeight =
  let jointValue = int64(joint)
  let weightValue = float32(weight)
  if jointValue < 0 or uint64(jointValue) > uint64(high(uint32)):
    raise newException(ValueError, "soft body skin joint index must fit uint32")
  if not weightValue.isFinite or weightValue <= 0:
    raise newException(ValueError, "soft body skin weight must be positive")
  SoftBodySkinWeight(joint: uint32(jointValue), weight: weightValue)

proc softBodySkinConstraint*(vertex: SomeInteger;
    weights: openArray[SoftBodySkinWeight];
    maxDistance = maxFiniteFloat32;
    backStopDistance = maxFiniteFloat32;
    backStopRadius = 40.0'f32): SoftBodySkinConstraint =
  let vertexValue = int64(vertex)
  if vertexValue < 0 or uint64(vertexValue) > uint64(high(uint32)):
    raise newException(ValueError, "soft body skinned vertex must fit uint32")
  if weights.len < 1 or weights.len > 4:
    raise newException(ValueError, "soft body skin constraint needs 1 to 4 weights")
  if not maxDistance.isFinite or maxDistance < 0 or
      not backStopDistance.isFinite or backStopDistance < 0 or
      not backStopRadius.isFinite or backStopRadius <= 0:
    raise newException(ValueError, "soft body skin distances are invalid")
  var total = 0.0'f32
  for weight in weights:
    if not weight.weight.isFinite or weight.weight <= 0:
      raise newException(ValueError, "soft body skin weight must be positive")
    for existing in result.weights:
      if existing.joint == weight.joint:
        raise newException(ValueError, "soft body skin joints must be unique")
    result.weights.add(weight)
    total += weight.weight
  if not total.isFinite or total <= 0:
    raise newException(ValueError, "soft body skin weights have invalid total")
  for weight in result.weights.mitems:
    weight.weight /= total
  result.vertex = uint32(vertexValue)
  result.maxDistance = maxDistance
  result.backStopDistance = backStopDistance
  result.backStopRadius = backStopRadius

proc clothSoftBodyMesh*(columns, rows: int; spacing = 0.5'f32;
                        fixedVertices: openArray[int] = []): SoftBodyMesh =
  if columns < 2 or rows < 2:
    raise newException(ValueError, "soft body cloth requires at least 2 x 2 vertices")
  if not spacing.isFinite or spacing <= 0:
    raise newException(ValueError, "soft body cloth spacing must be positive")
  if uint64(columns) * uint64(rows) > uint64(high(uint32)):
    raise newException(ValueError, "soft body cloth exceeds Jolt index limits")
  var fixed = newSeq[bool](columns * rows)
  for index in fixedVertices:
    if index < 0 or index >= fixed.len:
      raise newException(IndexDefect, "fixed cloth vertex is out of bounds")
    fixed[index] = true
  let offsetX = -0.5'f32 * spacing * float32(columns - 1)
  let offsetZ = -0.5'f32 * spacing * float32(rows - 1)
  for row in 0 ..< rows:
    for column in 0 ..< columns:
      let index = column + row * columns
      result.vertices.add(softBodyVertex(
        Vec3(x: offsetX + float32(column) * spacing, y: 0,
          z: offsetZ + float32(row) * spacing),
        Vec3(),
        if fixed[index]: 0.0'f32 else: 1.0'f32))
  for row in 0 ..< rows - 1:
    for column in 0 ..< columns - 1:
      let topLeft = column + row * columns
      let bottomLeft = topLeft + columns
      result.faces.add(softBodyFace(topLeft, bottomLeft, bottomLeft + 1))
      result.faces.add(softBodyFace(topLeft, bottomLeft + 1, topLeft + 1))

proc rodSoftBodyMesh*(points: openArray[Vec3]; fixedFirst = true;
                      inverseMass = 1.0'f32;
                      rodCompliance = 0.0'f32;
                      bendTwistCompliance = 0.0'f32): SoftBodyMesh =
  if points.len < 3:
    raise newException(ValueError, "soft body rod requires at least three points")
  if uint64(points.len) > uint64(high(uint32)):
    raise newException(ValueError, "soft body rod exceeds Jolt index limits")
  if not inverseMass.isFinite or inverseMass <= 0:
    raise newException(ValueError, "soft body rod inverse mass must be positive")
  if not rodCompliance.isFinite or rodCompliance < 0 or
      not bendTwistCompliance.isFinite or bendTwistCompliance < 0:
    raise newException(ValueError, "soft body rod compliance is invalid")
  for index, point in points:
    result.vertices.add(softBodyVertex(
      point,
      inverseMass = if fixedFirst and index == 0: 0.0'f32 else: inverseMass))
    if index > 0:
      result.rods.add(softBodyRodConstraint(
        index - 1, index, rodCompliance))
    if index > 1:
      result.rodBendTwistConstraints.add(
        softBodyRodBendTwistConstraint(
          index - 2, index - 1, bendTwistCompliance))

proc normalized*(value: Quat): Quat =
  if not value.isFinite:
    raise newException(ValueError, "quaternion components must be finite")
  let lengthSquared =
    value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "quaternion must have non-zero length")
  let inverseLength = 1.0'f32 / sqrt(lengthSquared)
  Quat(
    x: value.x * inverseLength,
    y: value.y * inverseLength,
    z: value.z * inverseLength,
    w: value.w * inverseLength
  )

proc quatFromAxisAngle*(axis: Vec3; angleRadians: SomeNumber): Quat =
  if not axis.isFinite or not float32(angleRadians).isFinite:
    raise newException(ValueError, "axis and angle must be finite")
  let axisLengthSquared = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z
  if axisLengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "rotation axis must have non-zero length")
  let inverseAxisLength = 1.0'f32 / sqrt(axisLengthSquared)
  let halfAngle = float32(angleRadians) * 0.5'f32
  let scale = sin(halfAngle) * inverseAxisLength
  Quat(
    x: axis.x * scale,
    y: axis.y * scale,
    z: axis.z * scale,
    w: cos(halfAngle)
  )

proc boxShape*(halfExtent: Vec3; convexRadius = 0.05'f32): Shape =
  if not halfExtent.isFinite or halfExtent.x <= 0 or
      halfExtent.y <= 0 or halfExtent.z <= 0:
    raise newException(ValueError, "box half extents must be finite and positive")
  if not convexRadius.isFinite or convexRadius < 0:
    raise newException(ValueError, "box convex radius must be finite and non-negative")
  let minExtent = min(halfExtent.x, min(halfExtent.y, halfExtent.z))
  Shape(
    kind: ShapeKind.Box,
    halfExtent: halfExtent,
    convexRadius: min(convexRadius, minExtent)
  )

proc sphereShape*(radius: SomeNumber): Shape =
  let nativeRadius = float32(radius)
  if not nativeRadius.isFinite or nativeRadius <= 0:
    raise newException(ValueError, "sphere radius must be finite and positive")
  Shape(kind: ShapeKind.Sphere, radius: nativeRadius)

proc capsuleShape*[H, R: SomeNumber](halfHeight: H; radius: R): Shape =
  let nativeHalfHeight = float32(halfHeight)
  let nativeRadius = float32(radius)
  if not nativeHalfHeight.isFinite or nativeHalfHeight <= 0:
    raise newException(ValueError, "capsule half height must be finite and positive")
  if not nativeRadius.isFinite or nativeRadius <= 0:
    raise newException(ValueError, "capsule radius must be finite and positive")
  Shape(
    kind: ShapeKind.Capsule,
    radius: nativeRadius,
    halfHeight: nativeHalfHeight
  )

proc cylinderShape*[H, R: SomeNumber](halfHeight: H; radius: R;
                                      convexRadius = 0.05'f32): Shape =
  let nativeHalfHeight = float32(halfHeight)
  let nativeRadius = float32(radius)
  if not nativeHalfHeight.isFinite or nativeHalfHeight <= 0:
    raise newException(ValueError, "cylinder half height must be finite and positive")
  if not nativeRadius.isFinite or nativeRadius <= 0:
    raise newException(ValueError, "cylinder radius must be finite and positive")
  if not convexRadius.isFinite or convexRadius < 0:
    raise newException(ValueError, "cylinder convex radius must be finite and non-negative")
  Shape(
    kind: ShapeKind.Cylinder,
    radius: nativeRadius,
    halfHeight: nativeHalfHeight,
    convexRadius: min(convexRadius, min(nativeHalfHeight, nativeRadius))
  )

proc taperedCapsuleShape*[H, T, B: SomeNumber](
    halfHeight: H; topRadius: T; bottomRadius: B): Shape =
  let nativeHalfHeight = float32(halfHeight)
  let nativeTopRadius = float32(topRadius)
  let nativeBottomRadius = float32(bottomRadius)
  if not nativeHalfHeight.isFinite or nativeHalfHeight < 0:
    raise newException(
      ValueError, "tapered capsule half height must be finite and non-negative")
  if not nativeTopRadius.isFinite or nativeTopRadius <= 0 or
      not nativeBottomRadius.isFinite or nativeBottomRadius <= 0:
    raise newException(
      ValueError, "tapered capsule radii must be finite and positive")
  Shape(
    kind: ShapeKind.TaperedCapsule,
    halfHeight: nativeHalfHeight,
    topRadius: nativeTopRadius,
    bottomRadius: nativeBottomRadius)

proc taperedCylinderShape*[H, T, B: SomeNumber](
    halfHeight: H; topRadius: T; bottomRadius: B;
    convexRadius = 0.05'f32): Shape =
  let nativeHalfHeight = float32(halfHeight)
  let nativeTopRadius = float32(topRadius)
  let nativeBottomRadius = float32(bottomRadius)
  if not nativeHalfHeight.isFinite or nativeHalfHeight <= 0:
    raise newException(
      ValueError, "tapered cylinder half height must be finite and positive")
  if not nativeTopRadius.isFinite or nativeTopRadius <= 0 or
      not nativeBottomRadius.isFinite or nativeBottomRadius <= 0:
    raise newException(
      ValueError, "tapered cylinder radii must be finite and positive")
  if not convexRadius.isFinite or convexRadius < 0:
    raise newException(
      ValueError, "tapered cylinder convex radius must be finite and non-negative")
  Shape(
    kind: ShapeKind.TaperedCylinder,
    halfHeight: nativeHalfHeight,
    topRadius: nativeTopRadius,
    bottomRadius: nativeBottomRadius,
    convexRadius: min(
      convexRadius,
      min(nativeHalfHeight, min(nativeTopRadius, nativeBottomRadius))))

proc triangleShape*(v1, v2, v3: Vec3;
                    convexRadius = 0.0'f32): Shape =
  if not v1.isFinite or not v2.isFinite or not v3.isFinite:
    raise newException(ValueError, "triangle vertices must be finite")
  if not convexRadius.isFinite or convexRadius < 0:
    raise newException(
      ValueError, "triangle convex radius must be finite and non-negative")
  let ab = vec3(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z)
  let ac = vec3(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z)
  let cross = vec3(
    ab.y * ac.z - ab.z * ac.y,
    ab.z * ac.x - ab.x * ac.z,
    ab.x * ac.y - ab.y * ac.x)
  if cross.x * cross.x + cross.y * cross.y + cross.z * cross.z <= 1.0e-12'f32:
    raise newException(ValueError, "triangle vertices must not be collinear")
  Shape(
    kind: ShapeKind.Triangle,
    points: @[v1, v2, v3],
    convexRadius: convexRadius)

proc planeShape*(normal: Vec3; constant = 0.0'f32;
                 halfExtent = 1_000.0'f32): Shape =
  if not normal.isFinite or not constant.isFinite or
      not halfExtent.isFinite:
    raise newException(ValueError, "plane settings must be finite")
  let lengthSquared =
    normal.x * normal.x + normal.y * normal.y + normal.z * normal.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "plane normal must have non-zero length")
  if halfExtent <= 0:
    raise newException(ValueError, "plane half extent must be positive")
  let inverseLength = 1.0'f32 / sqrt(lengthSquared)
  Shape(
    kind: ShapeKind.Plane,
    planeNormal: Vec3(
      x: normal.x * inverseLength,
      y: normal.y * inverseLength,
      z: normal.z * inverseLength),
    planeConstant: constant * inverseLength,
    planeHalfExtent: halfExtent)

proc emptyShape*(centerOfMass = vec3(0, 0, 0)): Shape =
  if not centerOfMass.isFinite:
    raise newException(ValueError, "empty shape center of mass must be finite")
  Shape(kind: ShapeKind.Empty, centerOfMass: centerOfMass)

proc convexHullShape*(points: openArray[Vec3];
                      maxConvexRadius = 0.05'f32): Shape =
  if points.len < 4:
    raise newException(ValueError, "a convex hull requires at least four points")
  if uint64(points.len) > uint64(high(uint32)):
    raise newException(ValueError, "convex hull point count must fit in uint32")
  if not maxConvexRadius.isFinite or maxConvexRadius < 0:
    raise newException(
      ValueError,
      "convex hull radius must be finite and non-negative"
    )
  for point in points:
    if not point.isFinite:
      raise newException(ValueError, "convex hull points must be finite")
  Shape(
    kind: ShapeKind.ConvexHull,
    convexRadius: maxConvexRadius,
    points: @points
  )

proc triangleMeshShape*(vertices: openArray[Vec3];
                        triangleIndices: openArray[uint32]): Shape =
  if vertices.len < 3:
    raise newException(ValueError, "a triangle mesh requires at least three vertices")
  if triangleIndices.len < 3 or triangleIndices.len mod 3 != 0:
    raise newException(
      ValueError, "triangle mesh indices must contain complete triangles")
  if uint64(vertices.len) > uint64(high(uint32)) or
      uint64(triangleIndices.len div 3) > uint64(high(uint32)):
    raise newException(ValueError, "triangle mesh sizes must fit in uint32")
  for vertex in vertices:
    if not vertex.isFinite:
      raise newException(ValueError, "triangle mesh vertices must be finite")
  for index in triangleIndices:
    if uint64(index) >= uint64(vertices.len):
      raise newException(ValueError, "triangle mesh index is out of bounds")
  Shape(
    kind: ShapeKind.TriangleMesh,
    vertices: @vertices,
    triangleIndices: @triangleIndices)

proc heightFieldShape*[T: SomeNumber](heightSamples: openArray[T];
                                      sampleCount: int;
                                      offset = vec3(0, 0, 0);
                                      scale = vec3(1, 1, 1);
                                      blockSize = 2'u32;
                                      bitsPerSample = 8'u32): Shape =
  if blockSize < 2 or blockSize > 8:
    raise newException(ValueError, "height field blockSize must be in [2, 8]")
  if bitsPerSample < 1 or bitsPerSample > 16:
    raise newException(
      ValueError, "height field bitsPerSample must be in [1, 16]")
  if sampleCount < int(blockSize * 2):
    raise newException(
      ValueError, "height field sampleCount must span at least two blocks")
  if uint64(sampleCount) > uint64(high(uint32)):
    raise newException(ValueError, "height field sampleCount must fit in uint32")
  let requiredSamples = uint64(sampleCount) * uint64(sampleCount)
  if uint64(heightSamples.len) != requiredSamples:
    raise newException(
      ValueError, "height field requires sampleCount squared height samples")
  if not offset.isFinite:
    raise newException(ValueError, "height field offset must be finite")
  if not scale.isFinite or scale.x <= 0 or scale.y <= 0 or scale.z <= 0:
    raise newException(
      ValueError, "height field scale must be finite and positive")

  var nativeSamples = newSeq[float32](heightSamples.len)
  for index, sample in heightSamples:
    nativeSamples[index] = float32(sample)
    if not nativeSamples[index].isFinite:
      raise newException(ValueError, "height field samples must be finite")
  Shape(
    kind: ShapeKind.HeightField,
    heightSamples: nativeSamples,
    sampleCount: uint32(sampleCount),
    heightOffset: offset,
    heightScale: scale,
    blockSize: blockSize,
    bitsPerSample: bitsPerSample)

proc compoundChild*(shape: Shape; position = vec3(0, 0, 0);
                    rotation = quatIdentity()): CompoundChild =
  if not position.isFinite:
    raise newException(ValueError, "compound child position must be finite")
  CompoundChild(
    shape: shape,
    position: position,
    rotation: rotation.normalized)

proc compoundChildTransform*(position: Vec3;
                             rotation = quatIdentity()): CompoundChildTransform =
  if not position.isFinite:
    raise newException(ValueError, "compound child position must be finite")
  CompoundChildTransform(position: position, rotation: rotation.normalized)

proc staticCompoundShape*(children: openArray[CompoundChild]): Shape =
  if children.len < 2:
    raise newException(ValueError, "a compound requires at least two children")
  if uint64(children.len) > uint64(high(uint32)):
    raise newException(ValueError, "compound child count must fit in uint32")
  var validatedChildren = newSeq[CompoundChild](children.len)
  for index, child in children:
    validatedChildren[index] = compoundChild(
      child.shape, child.position, child.rotation)
  Shape(kind: ShapeKind.StaticCompound, children: validatedChildren)

proc mutableCompoundShape*(children: openArray[CompoundChild]): Shape =
  if children.len < 1:
    raise newException(ValueError, "a mutable compound requires at least one child")
  if uint64(children.len) > uint64(high(uint32)):
    raise newException(ValueError, "compound child count must fit in uint32")
  var validatedChildren = newSeq[CompoundChild](children.len)
  for index, child in children:
    validatedChildren[index] = compoundChild(
      child.shape, child.position, child.rotation)
  Shape(kind: ShapeKind.MutableCompound, children: validatedChildren)

proc scaledShape*(innerShape: Shape; scale: Vec3): Shape =
  if not scale.isFinite or abs(scale.x) <= 1.0e-6'f32 or
      abs(scale.y) <= 1.0e-6'f32 or abs(scale.z) <= 1.0e-6'f32:
    raise newException(
      ValueError, "shape scale must be finite and have no zero component")
  Shape(
    kind: ShapeKind.Scaled,
    innerShapes: @[innerShape],
    shapeScale: scale)

proc rotatedTranslatedShape*(innerShape: Shape; position: Vec3;
                             rotation = quatIdentity()): Shape =
  if not position.isFinite:
    raise newException(ValueError, "decorated shape position must be finite")
  Shape(
    kind: ShapeKind.RotatedTranslated,
    innerShapes: @[innerShape],
    shapePosition: position,
    shapeRotation: rotation.normalized)

proc offsetCenterOfMassShape*(innerShape: Shape; offset: Vec3): Shape =
  if not offset.isFinite:
    raise newException(ValueError, "center-of-mass offset must be finite")
  Shape(
    kind: ShapeKind.OffsetCenterOfMass,
    innerShapes: @[innerShape],
    centerOfMassOffset: offset)

proc innerShape*(shape: Shape): Shape =
  if shape.kind notin {
      ShapeKind.Scaled,
      ShapeKind.RotatedTranslated,
      ShapeKind.OffsetCenterOfMass
    } or shape.innerShapes.len != 1:
    raise newException(ValueError, "shape is not a valid decorated shape")
  shape.innerShapes[0]

proc withMaterials*(shape: Shape; materials: openArray[PhysicsMaterial];
                    materialIndices: openArray[uint32]): Shape =
  if shape.kind notin {ShapeKind.TriangleMesh, ShapeKind.HeightField}:
    raise newException(
      ValueError, "per-sub-shape materials require a mesh or height field")
  if materials.len == 0:
    raise newException(ValueError, "at least one physics material is required")
  if uint64(materials.len) > uint64(high(uint32)):
    raise newException(ValueError, "physics material count must fit in uint32")
  if shape.kind == ShapeKind.HeightField and materials.len > 256:
    raise newException(ValueError, "height fields support at most 256 materials")
  for material in materials:
    discard physicsMaterial(material.name, material.debugColor)
  let expectedIndices = if shape.kind == ShapeKind.TriangleMesh:
      shape.triangleIndices.len div 3
    else:
      (int(shape.sampleCount) - 1) * (int(shape.sampleCount) - 1)
  if materialIndices.len != expectedIndices:
    raise newException(
      ValueError, "material index count must match mesh triangles or height cells")
  for index in materialIndices:
    if uint64(index) >= uint64(materials.len):
      raise newException(ValueError, "physics material index is out of bounds")
  result = shape
  result.material = none(PhysicsMaterial)
  result.materials = @materials
  result.materialIndices = @materialIndices

proc withMaterial*(shape: Shape; material: PhysicsMaterial): Shape =
  discard physicsMaterial(material.name, material.debugColor)
  case shape.kind
  of ShapeKind.Empty:
    raise newException(ValueError, "empty shapes cannot have a physics material")
  of ShapeKind.TriangleMesh:
    result = shape.withMaterials(
      [material], newSeq[uint32](shape.triangleIndices.len div 3))
  of ShapeKind.HeightField:
    let cellCount = (int(shape.sampleCount) - 1) * (int(shape.sampleCount) - 1)
    result = shape.withMaterials([material], newSeq[uint32](cellCount))
  of ShapeKind.StaticCompound, ShapeKind.MutableCompound:
    result = shape
    for index in 0 ..< result.children.len:
      result.children[index].shape =
        result.children[index].shape.withMaterial(material)
  of ShapeKind.Scaled, ShapeKind.RotatedTranslated,
      ShapeKind.OffsetCenterOfMass:
    result = shape
    if result.innerShapes.len != 1:
      raise newException(ValueError, "decorated shape has no valid inner shape")
    result.innerShapes[0] = result.innerShapes[0].withMaterial(material)
  else:
    result = shape
    result.material = some(material)

func defaultCharacterConfig*(): CharacterConfig =
  CharacterConfig(
    maxSlopeAngle: PI.float32 * 50.0'f32 / 180.0'f32,
    mass: 70,
    maxStrength: 100,
    padding: 0.02,
    predictiveContactDistance: 0.1,
    maxNumHits: 256,
    hitReductionCosMaxAngle: 0.999,
    penetrationRecoverySpeed: 1,
    stepUp: 0.4,
    stepDown: 0.5,
    enhancedInternalEdgeRemoval: true,
    backFaceMode: CharacterBackFaceMode.CollideWithBackFaces,
    maxCollisionIterations: 5,
    maxConstraintIterations: 15,
    minTimeRemaining: 1.0e-4,
    collisionTolerance: 1.0e-3,
    userData: 0,
    innerBodyShape: none(Shape),
    innerBodyLayer: movingLayer,
    maxQueuedContactEvents: 1024,
    canPushCharacter: true,
    canReceiveImpulses: true,
    preventSliding: false
  )

func defaultRigidCharacterConfig*(): RigidCharacterConfig =
  RigidCharacterConfig(
    maxSlopeAngle: PI.float32 * 50.0'f32 / 180.0'f32,
    up: vec3(0, 1, 0),
    supportingHeight: 0.5,
    mass: 80,
    friction: 0.2,
    gravityFactor: 1,
    allowedDOFs: {
      AllowedDOF.TranslationXAxis,
      AllowedDOF.TranslationYAxis,
      AllowedDOF.TranslationZAxis
    },
    enhancedInternalEdgeRemoval: true,
    userData: 0,
    maxSeparationDistance: 0.05
  )

func defaultVehicleConfig*(): VehicleConfig =
  VehicleConfig(
    controllerKind: VehicleControllerKind.Wheeled,
    wheelRadius: 0.35,
    wheelWidth: 0.2,
    suspensionMinLength: 0.2,
    suspensionMaxLength: 0.45,
    suspensionFrequency: 1.5,
    suspensionDamping: 0.5,
    maxSteerAngle: PI.float32 / 6.0'f32,
    maxPitchRollAngle: PI.float32 / 3.0'f32,
    engineMaxTorque: 800,
    engineMinRPM: 1_000,
    engineMaxRPM: 6_000,
    engineInertia: 0.5,
    engineAngularDamping: 0.2,
    engineTorqueCurve: @[
      vehicleTorquePoint(0, 0.8),
      vehicleTorquePoint(0.66, 1),
      vehicleTorquePoint(1, 0.8)
    ],
    transmissionMode: VehicleTransmissionMode.Automatic,
    gearRatios: @[2.66'f32, 1.78, 1.3, 1, 0.74],
    reverseGearRatios: @[-2.9'f32],
    transmissionSwitchTime: 0.5,
    clutchReleaseTime: 0.3,
    transmissionSwitchLatency: 0.5,
    shiftUpRPM: 4_000,
    shiftDownRPM: 2_000,
    clutchStrength: 10,
    fourWheelDrive: true,
    frontWheelDrive: true,
    frontTorqueRatio: 0.5,
    differentialRatio: 3.42,
    differentialLeftRightSplit: 0.5,
    differentialLimitedSlipRatio: 1.4,
    centerDifferentialLimitedSlipRatio: 1.4,
    wheelTrack: 0,
    frontAxleOffset: 0,
    rearAxleOffset: 0,
    suspensionAttachmentHeightRatio: -0.9,
    rearMaxSteerAngle: 0,
    frontBrakeTorque: 1_500,
    rearBrakeTorque: 1_500,
    rearHandBrakeTorque: 4_000,
    antiRollBarStiffness: 1_000,
    wheelInertia: 0.9,
    wheelAngularDamping: 0.2,
    tireLongitudinalImpulseMultiplier: 10,
    tireLateralImpulseMultiplier: 1,
    wheelCollisionMode: VehicleWheelCollisionMode.Ray,
    wheelCollisionUp: vec3(0, 1, 0),
    wheelCollisionMaxSlopeAngle: PI.float32 * 80.0'f32 / 180.0'f32,
    wheelSphereCastRadius: 0.05,
    wheelCylinderConvexRadiusFraction: 0.1,
    wheels: @[],
    differentials: @[],
    antiRollBars: @[],
    maxLeanAngle: PI.float32 / 4,
    leanSpringConstant: 5_000,
    leanSpringDamping: 1_000,
    leanSpringIntegrationCoefficient: 0,
    leanSpringIntegrationCoefficientDecay: 4,
    leanSmoothingFactor: 0.8,
    enableLeanController: true,
    enableLeanSteeringLimit: true
  )

func defaultMotorcycleConfig*(): VehicleConfig =
  ## A two-wheel configuration for Jolt's MotorcycleController.
  result = defaultVehicleConfig()
  result.controllerKind = VehicleControllerKind.Motorcycle
  result.maxPitchRollAngle = PI.float32 / 3
  result.engineMaxTorque = 150
  result.engineMinRPM = 1_000
  result.engineMaxRPM = 10_000
  result.engineTorqueCurve = @[
    vehicleTorquePoint(0, 0.8),
    vehicleTorquePoint(0.65, 1),
    vehicleTorquePoint(1, 0.75)]
  result.gearRatios = @[2.27'f32, 1.63, 1.3, 1.09, 0.96, 0.88]
  result.reverseGearRatios = @[-4.0'f32]
  result.shiftDownRPM = 2_000
  result.shiftUpRPM = 8_000
  result.clutchStrength = 2
  result.wheelCollisionMode = VehicleWheelCollisionMode.CylinderCast
  result.wheelCylinderConvexRadiusFraction = 1
  var front = defaultVehicleWheelConfig(vec3(0, -0.27, 0.75))
  front.suspensionDirection = vec3(0, -0.8660254, 0.5)
  front.steeringAxis = vec3(0, 0.8660254, -0.5)
  front.wheelUp = vec3(0, 0.8660254, -0.5)
  front.suspensionMinLength = 0.3
  front.suspensionMaxLength = 0.5
  front.suspensionFrequency = 1.5
  front.radius = 0.31
  front.width = 0.05
  front.maxSteerAngle = PI.float32 / 6
  front.maxBrakeTorque = 500
  var rear = defaultVehicleWheelConfig(vec3(0, -0.27, -0.75))
  rear.suspensionMinLength = 0.3
  rear.suspensionMaxLength = 0.5
  rear.suspensionFrequency = 2
  rear.radius = 0.31
  rear.width = 0.05
  rear.maxBrakeTorque = 250
  result.wheels = @[front, rear]
  result.differentials = @[
    vehicleDifferential(
      -1, 1, differentialRatio = 4.825,
      limitedSlipRatio = 1.4)]
  result.antiRollBars = @[]

func defaultTrackedVehicleConfig*(): TrackedVehicleConfig =
  result.maxPitchRollAngle = PI.float32 / 3
  result.engineMaxTorque = 500
  result.engineMinRPM = 500
  result.engineMaxRPM = 4_000
  result.engineInertia = 0.5
  result.engineAngularDamping = 0.2
  result.engineTorqueCurve = @[
    vehicleTorquePoint(0, 0.8),
    vehicleTorquePoint(0.66, 1),
    vehicleTorquePoint(1, 0.8)
  ]
  result.transmissionMode = VehicleTransmissionMode.Automatic
  result.gearRatios = @[4.0'f32, 3, 2, 1]
  result.reverseGearRatios = @[-4.0'f32, -3]
  result.transmissionSwitchTime = 0.5
  result.clutchReleaseTime = 0.3
  result.transmissionSwitchLatency = 0.5
  result.shiftUpRPM = 3_500
  result.shiftDownRPM = 1_000
  result.clutchStrength = 10
  result.wheelCollisionMode = VehicleWheelCollisionMode.Ray
  result.wheelCollisionUp = vec3(0, 1, 0)
  result.wheelCollisionMaxSlopeAngle = PI.float32 * 80 / 180
  result.wheelSphereCastRadius = 0.05
  result.wheelCylinderConvexRadiusFraction = 0.1

  let longitudinalPositions = [
    2.95'f32, 2.1, 1.4, 0.7, 0, -0.7, -1.4, -2.1, -2.75]
  var leftIndices: seq[int]
  var rightIndices: seq[int]
  for side in TrackedVehicleSide:
    let x = if side == TrackedVehicleSide.LeftTrack: 1.5'f32 else: -1.5'f32
    for index, z in longitudinalPositions:
      let y = if index == 0 or index == longitudinalPositions.high:
          0.0'f32
        else:
          -0.3'f32
      var wheel = defaultTrackedVehicleWheelConfig(Vec3(x: x, y: y, z: z))
      if index == 0 or index == longitudinalPositions.high:
        wheel.suspensionMaxLength = wheel.suspensionMinLength
      let wheelIndex = result.wheels.len
      result.wheels.add(wheel)
      if side == TrackedVehicleSide.LeftTrack:
        leftIndices.add(wheelIndex)
      else:
        rightIndices.add(wheelIndex)
  result.tracks[TrackedVehicleSide.LeftTrack] = trackedVehicleTrack(
    leftIndices, leftIndices[^1])
  result.tracks[TrackedVehicleSide.RightTrack] = trackedVehicleTrack(
    rightIndices, rightIndices[^1])

func allAllowedDOFs*(): set[AllowedDOF] =
  {low(AllowedDOF) .. high(AllowedDOF)}

func plane2DAllowedDOFs*(): set[AllowedDOF] =
  {
    AllowedDOF.TranslationXAxis,
    AllowedDOF.TranslationYAxis,
    AllowedDOF.RotationZAxis
  }

func defaultBodyConfig*(): BodyConfig =
  BodyConfig(
    allowedDOFs: allAllowedDOFs(),
    motionQuality: MotionQuality.Discrete,
    mass: 0,
    inertiaMultiplier: 1,
    allowSleeping: true,
    useManifoldReduction: true,
    friction: 0.2,
    restitution: 0,
    linearDamping: 0.05,
    angularDamping: 0.05,
    maxLinearVelocity: 500,
    maxAngularVelocity: PI.float32 * 15,
    gravityFactor: 1
  )

func bodyMassProperties*[M: SomeNumber](
    mass: M; inertiaDiagonal: Vec3;
    inertiaRotation = quatIdentity()): BodyMassProperties =
  BodyMassProperties(
    mass: float32(mass),
    inertiaDiagonal: inertiaDiagonal,
    inertiaRotation: inertiaRotation)

func bodySpec*(shape: Shape; position: Vec3; motionType: MotionType;
               layer: CollisionLayer; rotation = quatIdentity();
               sensor = false; config = defaultBodyConfig()): BodySpec =
  BodySpec(
    shape: shape,
    position: position,
    rotation: rotation,
    motionType: motionType,
    layer: layer,
    sensor: sensor,
    config: config)

func staticBodySpec*(shape: Shape; position: Vec3;
                     rotation = quatIdentity(); sensor = false;
                     layer = nonMovingLayer;
                     config = defaultBodyConfig()): BodySpec =
  bodySpec(shape, position, MotionType.Static, layer,
           rotation, sensor, config)

func dynamicBodySpec*(shape: Shape; position: Vec3;
                      rotation = quatIdentity(); sensor = false;
                      layer = movingLayer;
                      config = defaultBodyConfig()): BodySpec =
  bodySpec(shape, position, MotionType.Dynamic, layer,
           rotation, sensor, config)

func kinematicBodySpec*(shape: Shape; position: Vec3;
                        rotation = quatIdentity(); sensor = false;
                        layer = movingLayer;
                        config = defaultBodyConfig()): BodySpec =
  bodySpec(shape, position, MotionType.Kinematic, layer,
           rotation, sensor, config)

func ragdollJoint*(parent: int; position: Vec3;
                   twistAxis = vec3(1, 0, 0);
                   planeAxis = vec3(0, 0, 1);
                   normalHalfConeAngle = PI.float32 / 4;
                   planeHalfConeAngle = PI.float32 / 4;
                   twistMinAngle = -PI.float32 / 4;
                   twistMaxAngle = PI.float32 / 4;
                   maxFrictionTorque = 0.0'f32;
                   motorFrequency = 2.0'f32;
                   motorDamping = 1.0'f32;
                   maxMotorTorque = maxFiniteFloat32): RagdollJointConfig =
  RagdollJointConfig(
    parent: parent,
    kind: RagdollSwingTwist,
    position: position,
    twistAxis: twistAxis,
    planeAxis: planeAxis,
    normalHalfConeAngle: normalHalfConeAngle,
    planeHalfConeAngle: planeHalfConeAngle,
    twistMinAngle: twistMinAngle,
    twistMaxAngle: twistMaxAngle,
    maxFrictionTorque: maxFrictionTorque,
    motorFrequency: motorFrequency,
    motorDamping: motorDamping,
    maxMotorTorque: maxMotorTorque)

func ragdollHingeJoint*(parent: int; position: Vec3;
                        hingeAxis = vec3(1, 0, 0);
                        normalAxis = vec3(0, 0, 1);
                        minAngle = -PI.float32;
                        maxAngle = PI.float32;
                        maxFrictionTorque = 0.0'f32;
                        motorFrequency = 2.0'f32;
                        motorDamping = 1.0'f32;
                        maxMotorTorque = maxFiniteFloat32):
                        RagdollJointConfig =
  ## Creates a single-axis parent joint. Angles are in radians.
  RagdollJointConfig(
    parent: parent,
    kind: RagdollHinge,
    position: position,
    twistAxis: hingeAxis,
    planeAxis: normalAxis,
    twistMinAngle: minAngle,
    twistMaxAngle: maxAngle,
    maxFrictionTorque: maxFrictionTorque,
    motorFrequency: motorFrequency,
    motorDamping: motorDamping,
    maxMotorTorque: maxMotorTorque)

func ragdollPointJoint*(parent: int; position: Vec3): RagdollJointConfig =
  ## Keeps the parent and child joined at one world-space point.
  RagdollJointConfig(
    parent: parent, kind: RagdollPoint, position: position)

func ragdollFixedJoint*(parent: int; position: Vec3): RagdollJointConfig =
  ## Welds the child to its parent at a world-space point.
  RagdollJointConfig(
    parent: parent, kind: RagdollFixed, position: position)

func ragdollConeJoint*(parent: int; position: Vec3;
                       twistAxis = vec3(1, 0, 0);
                       halfConeAngle = PI.float32 / 4):
                       RagdollJointConfig =
  ## Joins two parts at a point and limits swing to a cone.
  RagdollJointConfig(
    parent: parent,
    kind: RagdollCone,
    position: position,
    twistAxis: twistAxis,
    normalHalfConeAngle: halfConeAngle)

func ragdollSliderJoint*(parent: int; position: Vec3;
                         sliderAxis = vec3(1, 0, 0);
                         normalAxis = vec3(0, 1, 0);
                         minPosition = -maxFiniteFloat32;
                         maxPosition = maxFiniteFloat32;
                         maxFrictionForce = 0.0'f32):
                         RagdollJointConfig =
  ## Allows translation along one axis while locking relative rotation.
  RagdollJointConfig(
    parent: parent,
    kind: RagdollSlider,
    position: position,
    twistAxis: sliderAxis,
    planeAxis: normalAxis,
    twistMinAngle: minPosition,
    twistMaxAngle: maxPosition,
    maxFrictionTorque: maxFrictionForce)

func ragdollSixDOFJoint*(parent: int; position: Vec3;
                         config = defaultSixDOFConfig();
                         linearFriction = vec3(0, 0, 0);
                         angularFriction = vec3(0, 0, 0)):
                         RagdollJointConfig =
  ## Configures each translation and rotation axis independently.
  RagdollJointConfig(
    parent: parent,
    kind: RagdollSixDOF,
    position: position,
    twistAxis: config.axisX,
    planeAxis: config.axisY,
    sixDOF: config,
    linearFriction: linearFriction,
    angularFriction: angularFriction)

func ragdollDistanceConstraint*(part1, part2: int;
                                point1, point2: Vec3;
                                minDistance, maxDistance: float32):
                                RagdollDistanceConstraintConfig =
  ## Adds a non-hierarchical distance link between two ragdoll parts.
  RagdollDistanceConstraintConfig(
    part1: part1, part2: part2, point1: point1, point2: point2,
    minDistance: minDistance, maxDistance: maxDistance)

func ragdollPointConstraint*(part1, part2: int;
                             point: Vec3): RagdollPointConstraintConfig =
  ## Adds a non-hierarchical point link between two ragdoll parts.
  RagdollPointConstraintConfig(
    part1: part1, part2: part2, point: point)

func ragdollFixedConstraint*(part1, part2: int):
                             RagdollFixedConstraintConfig =
  ## Welds two non-hierarchical parts in their current relative transform.
  RagdollFixedConstraintConfig(part1: part1, part2: part2)

func ragdollHingeConstraint*(part1, part2: int; point,
                             hingeAxis, normalAxis: Vec3;
                             minAngle = -PI.float32;
                             maxAngle = PI.float32;
                             maxFrictionTorque = 0'f32;
                             motor = none(RagdollScalarMotorPreset)):
                             RagdollHingeConstraintConfig =
  ## Adds a non-hierarchical hinge link between two ragdoll parts.
  RagdollHingeConstraintConfig(
    part1: part1, part2: part2, point: point,
    hingeAxis: hingeAxis, normalAxis: normalAxis,
    minAngle: minAngle, maxAngle: maxAngle,
    maxFrictionTorque: maxFrictionTorque, motor: motor)

func ragdollSliderConstraint*(part1, part2: int; point,
                              sliderAxis, normalAxis: Vec3;
                              minPosition = -maxFiniteFloat32;
                              maxPosition = maxFiniteFloat32;
                              maxFrictionForce = 0'f32;
                              motor = none(RagdollScalarMotorPreset)):
                              RagdollSliderConstraintConfig =
  ## Adds a non-hierarchical slider link between two ragdoll parts.
  RagdollSliderConstraintConfig(
    part1: part1, part2: part2, point: point,
    sliderAxis: sliderAxis, normalAxis: normalAxis,
    minPosition: minPosition, maxPosition: maxPosition,
    maxFrictionForce: maxFrictionForce, motor: motor)

func ragdollSwingTwistConstraint*(part1, part2: int; point: Vec3;
                                  twistAxis = vec3(1, 0, 0);
                                  planeAxis = vec3(0, 0, 1);
                                  normalHalfConeAngle = PI.float32 / 4;
                                  planeHalfConeAngle = PI.float32 / 4;
                                  twistMinAngle = -PI.float32 / 4;
                                  twistMaxAngle = PI.float32 / 4;
                                  maxFrictionTorque = 0'f32;
                                  motor = none(RagdollSwingTwistMotorPreset)):
                                  RagdollSwingTwistConstraintConfig =
  ## Adds a non-hierarchical humanoid-style angular link.
  RagdollSwingTwistConstraintConfig(
    part1: part1, part2: part2, point: point,
    twistAxis: twistAxis, planeAxis: planeAxis,
    normalHalfConeAngle: normalHalfConeAngle,
    planeHalfConeAngle: planeHalfConeAngle,
    twistMinAngle: twistMinAngle, twistMaxAngle: twistMaxAngle,
    maxFrictionTorque: maxFrictionTorque, motor: motor)

func ragdollSixDOFConstraint*(part1, part2: int; point: Vec3;
                              config = defaultSixDOFConfig();
                              linearFriction = vec3(0, 0, 0);
                              angularFriction = vec3(0, 0, 0);
                              motor = none(RagdollSixDOFMotorPreset)):
                              RagdollSixDOFConstraintConfig =
  ## Adds a non-hierarchical link with independent control of all six axes.
  RagdollSixDOFConstraintConfig(
    part1: part1, part2: part2, point: point, config: config,
    linearFriction: linearFriction, angularFriction: angularFriction,
    motor: motor)

func ragdollConeConstraint*(part1, part2: int; point,
                            twistAxis1, twistAxis2: Vec3;
                            halfConeAngle: float32):
                            RagdollConeConstraintConfig =
  ## Adds a non-hierarchical point link with a bounded swing angle.
  RagdollConeConstraintConfig(
    part1: part1, part2: part2, point: point,
    twistAxis1: twistAxis1, twistAxis2: twistAxis2,
    halfConeAngle: halfConeAngle)

func ragdollRootJoint*(): RagdollJointConfig =
  RagdollJointConfig(
    parent: -1,
    kind: RagdollSwingTwist,
    twistAxis: vec3(1, 0, 0),
    planeAxis: vec3(0, 0, 1),
    motorFrequency: 2,
    motorDamping: 1,
    maxMotorTorque: maxFiniteFloat32)

func ragdollPart*(name: string; shape: Shape; position: Vec3;
                  joint: RagdollJointConfig;
                  rotation = quatIdentity();
                  motionType = MotionType.Dynamic;
                  layer = movingLayer;
                  body = defaultBodyConfig()): RagdollPartConfig =
  RagdollPartConfig(
    name: name, shape: shape, position: position, rotation: rotation,
    motionType: motionType, layer: layer, body: body, joint: joint)

func ragdollConfig*(parts: seq[RagdollPartConfig]; groupId = 1'u32;
                    distanceConstraints: seq[RagdollDistanceConstraintConfig] = @[];
                    pointConstraints: seq[RagdollPointConstraintConfig] = @[];
                    fixedConstraints: seq[RagdollFixedConstraintConfig] = @[];
                    hingeConstraints: seq[RagdollHingeConstraintConfig] = @[];
                    sliderConstraints: seq[RagdollSliderConstraintConfig] = @[];
                    swingTwistConstraints:
                      seq[RagdollSwingTwistConstraintConfig] = @[];
                    sixDOFConstraints: seq[RagdollSixDOFConstraintConfig] = @[];
                    coneConstraints: seq[RagdollConeConstraintConfig] = @[];
                    disableParentChildCollisions = true;
                    stabilize = true;
                    calculateConstraintPriorities = true;
                    activate = true): RagdollConfig =
  RagdollConfig(
    parts: parts,
    distanceConstraints: distanceConstraints,
    pointConstraints: pointConstraints,
    fixedConstraints: fixedConstraints,
    hingeConstraints: hingeConstraints,
    sliderConstraints: sliderConstraints,
    swingTwistConstraints: swingTwistConstraints,
    sixDOFConstraints: sixDOFConstraints,
    coneConstraints: coneConstraints,
    groupId: groupId,
    disableParentChildCollisions: disableParentChildCollisions,
    stabilize: stabilize,
    calculateConstraintPriorities: calculateConstraintPriorities,
    activate: activate)

func skeletonTransform*(position: Vec3;
                        rotation = quatIdentity()): SkeletonTransform =
  SkeletonTransform(position: position, rotation: rotation)

func skeletonJoint*(name: string; parent: int;
                    neutralTransform: SkeletonTransform): SkeletonJoint =
  SkeletonJoint(
    name: name, parent: parent, neutralTransform: neutralTransform)

func skeletonDefinition*(joints: seq[SkeletonJoint]): SkeletonDefinition =
  SkeletonDefinition(joints: joints)

func skeletalAnimationKeyframe*(time: SomeNumber; position: Vec3;
                                rotation = quatIdentity()):
                                SkeletalAnimationKeyframe =
  SkeletalAnimationKeyframe(
    time: float32(time),
    transform: skeletonTransform(position, rotation))

func skeletalAnimationTrack*(jointName: string;
                             keyframes: seq[SkeletalAnimationKeyframe]):
                             SkeletalAnimationTrack =
  SkeletalAnimationTrack(jointName: jointName, keyframes: keyframes)

func defaultSoftBodyConfig*(): SoftBodyConfig =
  SoftBodyConfig(
    bendType: SoftBodyBendType.DistanceBend,
    lraType: SoftBodyLRAType.NoLRA,
    lraMaxDistanceMultiplier: 1,
    edgeCompliance: 1.0e-5,
    shearCompliance: 1.0e-5,
    bendCompliance: 1.0e-5,
    angleTolerance: PI.float32 * 8 / 180,
    numIterations: 5,
    linearDamping: 0.1,
    maxLinearVelocity: 500,
    restitution: 0,
    friction: 0.2,
    gravityFactor: 1,
    updatePosition: true,
    makeRotationIdentity: true,
    allowSleeping: true,
    enableSkinConstraints: true,
    skinnedMaxDistanceMultiplier: 1
  )

func defaultSimulationSettings*(): SimulationSettings =
  ## Jolt 5.6.0 defaults, exposed as a value so callers can change selected
  ## fields and apply the complete validated setting set atomically.
  SimulationSettings(
    maxInFlightBodyPairs: 16_384,
    stepListenersBatchSize: 8,
    stepListenerBatchesPerJob: 1,
    baumgarte: 0.2,
    speculativeContactDistance: 0.02,
    penetrationSlop: 0.02,
    linearCastThreshold: 0.75,
    linearCastMaxPenetration: 0.25,
    manifoldTolerance: 1.0e-3,
    maxPenetrationDistance: 0.2,
    # Exact float32 result of Jolt's Square(0.001f).
    bodyPairCacheMaxDeltaPositionSquared: cast[float32](0x358637be'u32),
    bodyPairCacheCosMaxDeltaRotationDiv2: 0.9998477,
    contactNormalCosMaxDeltaRotation: 0.9961947,
    contactPointPreserveLambdaMaxDistanceSquared: 1.0e-4,
    internalEdgeRemovalVertexToleranceSquared: 1.0e-8,
    numVelocitySteps: 10,
    numPositionSteps: 2,
    minVelocityForRestitution: 1,
    timeBeforeSleep: 0.5,
    pointVelocitySleepThreshold: 0.03,
    deterministicSimulation: true,
    constraintWarmStart: true,
    useBodyPairContactCache: true,
    useManifoldReduction: true,
    useLargeIslandSplitter: true,
    allowSleeping: true,
    checkActiveEdges: true
  )

func defaultWorldConfig*(): WorldConfig =
  WorldConfig(
    maxBodies: 65_536,
    numBodyMutexes: 0,
    maxBodyPairs: 65_536,
    maxContactConstraints: 10_240,
    tempAllocatorBytes: 10 * 1024 * 1024,
    maxJobs: 2_048,
    maxBarriers: 8,
    numThreads: -1,
    maxQueuedEvents: 16_384,
    characterBroadPhaseCellSize: 4,
    collisionLayers: @[
      collisionLayerConfig(0),
      collisionLayerConfig(1)
    ],
    collisionPairs: @[
      collisionPair(nonMovingLayer, movingLayer),
      collisionPair(movingLayer, movingLayer)
    ],
    contactPolicies: @[]
  )

func debugRendererEnabled*(): bool =
  ## True when this module was compiled with `-d:joltDebugRenderer`.
  when defined(joltDebugRenderer):
    true
  else:
    false

func defaultDebugBodyDrawSettings*(): DebugBodyDrawSettings =
  ## Matches Jolt's `BodyManager::DrawSettings` defaults.
  DebugBodyDrawSettings(
    drawShape: true,
    shapeColor: DebugMotionTypeColor,
    softBodyConstraintColor: DebugConstraintTypeColor)

func defaultDebugDrawLimits*(): DebugDrawLimits =
  ## Bounds memory use while retaining enough primitives for large scenes.
  DebugDrawLimits(
    maxLines: 262_144,
    maxTriangles: 262_144,
    maxTexts: 4_096,
    maxTextBytes: 1_048_576)

func defaultDebugDrawOptions*(): DebugDrawOptions =
  DebugDrawOptions(
    bodySettings: defaultDebugBodyDrawSettings(),
    drawBodies: true,
    limits: defaultDebugDrawLimits())

func truncated*(frame: DebugDrawFrame): bool =
  frame.droppedLines != 0 or frame.droppedTriangles != 0 or
    frame.droppedTexts != 0

func toRaw(value: Vec3): raw.Vec3 =
  raw.vec3(value.x, value.y, value.z)

func fromRaw(value: raw.Vec3): Vec3 =
  Vec3(x: value.x, y: value.y, z: value.z)

func toRaw(value: Quat): raw.Quat =
  raw.quat(value.x, value.y, value.z, value.w)

func fromRaw(value: raw.Quat): Quat =
  Quat(x: value.x, y: value.y, z: value.z, w: value.w)

proc toRaw(settings: SimulationSettings): raw.PhysicsSettings =
  result = raw.constructPhysicsSettings()
  result.mMaxInFlightBodyPairs = cint(settings.maxInFlightBodyPairs)
  result.mStepListenersBatchSize = cint(settings.stepListenersBatchSize)
  result.mStepListenerBatchesPerJob = cint(settings.stepListenerBatchesPerJob)
  result.mBaumgarte = settings.baumgarte
  result.mSpeculativeContactDistance = settings.speculativeContactDistance
  result.mPenetrationSlop = settings.penetrationSlop
  result.mLinearCastThreshold = settings.linearCastThreshold
  result.mLinearCastMaxPenetration = settings.linearCastMaxPenetration
  result.mManifoldTolerance = settings.manifoldTolerance
  result.mMaxPenetrationDistance = settings.maxPenetrationDistance
  result.mBodyPairCacheMaxDeltaPositionSq =
    settings.bodyPairCacheMaxDeltaPositionSquared
  result.mBodyPairCacheCosMaxDeltaRotationDiv2 =
    settings.bodyPairCacheCosMaxDeltaRotationDiv2
  result.mContactNormalCosMaxDeltaRotation =
    settings.contactNormalCosMaxDeltaRotation
  result.mContactPointPreserveLambdaMaxDistSq =
    settings.contactPointPreserveLambdaMaxDistanceSquared
  result.mInternalEdgeRemovalVertexToleranceSq =
    settings.internalEdgeRemovalVertexToleranceSquared
  result.mNumVelocitySteps = uint(settings.numVelocitySteps)
  result.mNumPositionSteps = uint(settings.numPositionSteps)
  result.mMinVelocityForRestitution = settings.minVelocityForRestitution
  result.mTimeBeforeSleep = settings.timeBeforeSleep
  result.mPointVelocitySleepThreshold = settings.pointVelocitySleepThreshold
  result.mDeterministicSimulation = settings.deterministicSimulation
  result.mConstraintWarmStart = settings.constraintWarmStart
  result.mUseBodyPairContactCache = settings.useBodyPairContactCache
  result.mUseManifoldReduction = settings.useManifoldReduction
  result.mUseLargeIslandSplitter = settings.useLargeIslandSplitter
  result.mAllowSleeping = settings.allowSleeping
  result.mCheckActiveEdges = settings.checkActiveEdges

func fromRaw(settings: raw.PhysicsSettings): SimulationSettings =
  SimulationSettings(
    maxInFlightBodyPairs: int32(settings.mMaxInFlightBodyPairs),
    stepListenersBatchSize: int32(settings.mStepListenersBatchSize),
    stepListenerBatchesPerJob: int32(settings.mStepListenerBatchesPerJob),
    baumgarte: settings.mBaumgarte,
    speculativeContactDistance: settings.mSpeculativeContactDistance,
    penetrationSlop: settings.mPenetrationSlop,
    linearCastThreshold: settings.mLinearCastThreshold,
    linearCastMaxPenetration: settings.mLinearCastMaxPenetration,
    manifoldTolerance: settings.mManifoldTolerance,
    maxPenetrationDistance: settings.mMaxPenetrationDistance,
    bodyPairCacheMaxDeltaPositionSquared:
      settings.mBodyPairCacheMaxDeltaPositionSq,
    bodyPairCacheCosMaxDeltaRotationDiv2:
      settings.mBodyPairCacheCosMaxDeltaRotationDiv2,
    contactNormalCosMaxDeltaRotation:
      settings.mContactNormalCosMaxDeltaRotation,
    contactPointPreserveLambdaMaxDistanceSquared:
      settings.mContactPointPreserveLambdaMaxDistSq,
    internalEdgeRemovalVertexToleranceSquared:
      settings.mInternalEdgeRemovalVertexToleranceSq,
    numVelocitySteps: uint32(settings.mNumVelocitySteps),
    numPositionSteps: uint32(settings.mNumPositionSteps),
    minVelocityForRestitution: settings.mMinVelocityForRestitution,
    timeBeforeSleep: settings.mTimeBeforeSleep,
    pointVelocitySleepThreshold: settings.mPointVelocitySleepThreshold,
    deterministicSimulation: settings.mDeterministicSimulation,
    constraintWarmStart: settings.mConstraintWarmStart,
    useBodyPairContactCache: settings.mUseBodyPairContactCache,
    useManifoldReduction: settings.mUseManifoldReduction,
    useLargeIslandSplitter: settings.mUseLargeIslandSplitter,
    allowSleeping: settings.mAllowSleeping,
    checkActiveEdges: settings.mCheckActiveEdges
  )

func allowedDOFMask(dofs: set[AllowedDOF]): uint8 =
  for dof in dofs:
    result = result or (1'u8 shl ord(dof))

func allowedDOFsFromMask(mask: uint8): set[AllowedDOF] =
  for dof in AllowedDOF:
    if (mask and (1'u8 shl ord(dof))) != 0:
      result.incl(dof)

proc validate(properties: BodyMassProperties) =
  if not properties.mass.isFinite or properties.mass <= 0:
    raise newException(
      ValueError, "custom body mass must be finite and positive")
  if not properties.inertiaDiagonal.isFinite or
      properties.inertiaDiagonal.x <= 0 or
      properties.inertiaDiagonal.y <= 0 or
      properties.inertiaDiagonal.z <= 0:
    raise newException(
      ValueError,
      "custom principal inertia values must be finite and positive")
  discard properties.inertiaRotation.normalized

proc validate(config: BodyConfig; motionType: MotionType) =
  if motionType != MotionType.Static and config.allowedDOFs == {}:
    raise newException(ValueError, "moving bodies require at least one allowed DOF")
  if not config.mass.isFinite or config.mass < 0:
    raise newException(ValueError, "body mass must be finite and non-negative")
  if config.mass > 0 and motionType != MotionType.Dynamic:
    raise newException(ValueError, "explicit body mass requires a dynamic body")
  if not config.inertiaMultiplier.isFinite or config.inertiaMultiplier <= 0:
    raise newException(
      ValueError, "body inertiaMultiplier must be finite and positive")
  if not config.linearVelocity.isFinite or not config.angularVelocity.isFinite:
    raise newException(ValueError, "initial body velocities must be finite")
  if not config.friction.isFinite or config.friction < 0:
    raise newException(ValueError, "body friction must be finite and non-negative")
  if not config.restitution.isFinite or
      config.restitution < 0 or config.restitution > 1:
    raise newException(ValueError, "body restitution must be in [0, 1]")
  if not config.linearDamping.isFinite or config.linearDamping < 0 or
      not config.angularDamping.isFinite or config.angularDamping < 0:
    raise newException(ValueError, "body damping must be finite and non-negative")
  if not config.maxLinearVelocity.isFinite or config.maxLinearVelocity <= 0 or
      not config.maxAngularVelocity.isFinite or config.maxAngularVelocity <= 0:
    raise newException(ValueError, "body velocity limits must be finite and positive")
  let linearSpeedSquared =
    config.linearVelocity.x * config.linearVelocity.x +
    config.linearVelocity.y * config.linearVelocity.y +
    config.linearVelocity.z * config.linearVelocity.z
  let angularSpeedSquared =
    config.angularVelocity.x * config.angularVelocity.x +
    config.angularVelocity.y * config.angularVelocity.y +
    config.angularVelocity.z * config.angularVelocity.z
  if linearSpeedSquared > config.maxLinearVelocity * config.maxLinearVelocity:
    raise newException(ValueError, "initial linear velocity exceeds its limit")
  if angularSpeedSquared > config.maxAngularVelocity * config.maxAngularVelocity:
    raise newException(ValueError, "initial angular velocity exceeds its limit")
  if not config.gravityFactor.isFinite:
    raise newException(ValueError, "body gravityFactor must be finite")
  if config.numVelocityStepsOverride >= 256 or
      config.numPositionStepsOverride >= 256:
    raise newException(ValueError, "body solver step overrides must be below 256")
  if config.massProperties.isSome:
    let properties = config.massProperties.get
    if motionType != MotionType.Dynamic:
      raise newException(
        ValueError, "custom mass properties require a dynamic body")
    if config.mass != 0:
      raise newException(
        ValueError, "custom mass properties cannot be combined with mass")
    if config.inertiaMultiplier != 1:
      raise newException(
        ValueError,
        "custom mass properties cannot be combined with inertiaMultiplier")
    properties.validate()

func toRaw(value: MotionType): raw.EMotionType =
  case value
  of MotionType.Static: raw.EMotionType.Static
  of MotionType.Kinematic: raw.EMotionType.Kinematic
  of MotionType.Dynamic: raw.EMotionType.Dynamic

proc requireOpen(world: World) =
  if world.isNil or world.closed or world.closing:
    raise newException(JoltError, "Jolt world is closed")

proc requireFinite(value: Vec3; name: string) =
  if not value.isFinite:
    raise newException(ValueError, name & " must contain only finite values")

proc validate(settings: SimulationSettings) =
  if settings.maxInFlightBodyPairs <= 0 or
      settings.stepListenersBatchSize <= 0 or
      settings.stepListenerBatchesPerJob <= 0:
    raise newException(
      ValueError, "simulation work batch sizes must be positive")
  for value in [
      settings.baumgarte,
      settings.linearCastThreshold,
      settings.linearCastMaxPenetration]:
    if not value.isFinite or value < 0 or value > 1:
      raise newException(
        ValueError, "simulation fractions must be finite and in [0, 1]")
  for value in [
      settings.speculativeContactDistance,
      settings.penetrationSlop,
      settings.manifoldTolerance,
      settings.maxPenetrationDistance,
      settings.bodyPairCacheMaxDeltaPositionSquared,
      settings.contactPointPreserveLambdaMaxDistanceSquared,
      settings.internalEdgeRemovalVertexToleranceSquared,
      settings.timeBeforeSleep]:
    if not value.isFinite or value < 0:
      raise newException(
        ValueError, "simulation distances and times must be finite and non-negative")
  for value in [
      settings.bodyPairCacheCosMaxDeltaRotationDiv2,
      settings.contactNormalCosMaxDeltaRotation]:
    if not value.isFinite or value < -1 or value > 1:
      raise newException(
        ValueError, "simulation cosine thresholds must be finite and in [-1, 1]")
  if settings.numVelocitySteps < 2:
    raise newException(
      ValueError, "simulation velocity steps must be at least 2")
  if settings.numPositionSteps == 0:
    raise newException(
      ValueError, "simulation position steps must be positive")
  if not settings.minVelocityForRestitution.isFinite or
      settings.minVelocityForRestitution <= 0 or
      not settings.pointVelocitySleepThreshold.isFinite or
      settings.pointVelocitySleepThreshold <= 0:
    raise newException(
      ValueError, "simulation velocity thresholds must be finite and positive")

proc validate(policy: BodyPairContactPolicy) =
  if policy.friction.isSome and
      (not policy.friction.get.isFinite or policy.friction.get < 0):
    raise newException(
      ValueError, "body-pair contact friction must be finite and non-negative")
  if policy.restitution.isSome and
      (not policy.restitution.get.isFinite or
        policy.restitution.get < 0 or policy.restitution.get > 1):
    raise newException(
      ValueError, "body-pair contact restitution must be in [0, 1]")
  for scale in [
      policy.inverseMassScale1, policy.inverseInertiaScale1,
      policy.inverseMassScale2, policy.inverseInertiaScale2]:
    if not scale.isFinite or scale < 0:
      raise newException(
        ValueError,
        "body-pair contact inverse scales must be finite and non-negative")
  if not policy.linearSurfaceVelocity.isFinite or
      not policy.angularSurfaceVelocity.isFinite:
    raise newException(
      ValueError, "body-pair contact surface velocities must be finite")

proc validate(config: WorldConfig) =
  if config.maxBodies == 0:
    raise newException(ValueError, "maxBodies must be positive")
  if config.maxBodyPairs == 0:
    raise newException(ValueError, "maxBodyPairs must be positive")
  if config.maxContactConstraints == 0:
    raise newException(ValueError, "maxContactConstraints must be positive")
  if config.tempAllocatorBytes == 0:
    raise newException(ValueError, "tempAllocatorBytes must be positive")
  if config.maxJobs == 0:
    raise newException(ValueError, "maxJobs must be positive")
  if config.maxBarriers == 0:
    raise newException(ValueError, "maxBarriers must be positive")
  if config.numThreads < -1:
    raise newException(ValueError, "numThreads must be -1 or non-negative")
  if config.maxQueuedEvents == 0 or
      config.maxQueuedEvents > uint(high(uint32)):
    raise newException(ValueError, "maxQueuedEvents must fit in a positive uint32")
  if not config.characterBroadPhaseCellSize.isFinite or
      config.characterBroadPhaseCellSize <= 0:
    raise newException(
      ValueError, "characterBroadPhaseCellSize must be finite and positive")
  if config.collisionLayers.len < 2:
    raise newException(ValueError, "collisionLayers must define at least two layers")
  if uint64(config.collisionLayers.len) > uint64(high(raw.ObjectLayer)) + 1'u64:
    raise newException(ValueError, "collisionLayers exceed Jolt's ObjectLayer range")
  for pair in config.collisionPairs:
    if uint32(pair.layer1) >= uint32(config.collisionLayers.len) or
        uint32(pair.layer2) >= uint32(config.collisionLayers.len):
      raise newException(ValueError, "collision pair refers to an undefined layer")
  for index, policy in config.contactPolicies:
    if uint32(policy.layer1) >= uint32(config.collisionLayers.len) or
        uint32(policy.layer2) >= uint32(config.collisionLayers.len):
      raise newException(
        ValueError, "contact policy refers to an undefined layer")
    var pairEnabled = false
    for pair in config.collisionPairs:
      if (pair.layer1 == policy.layer1 and pair.layer2 == policy.layer2) or
          (pair.layer1 == policy.layer2 and pair.layer2 == policy.layer1):
        pairEnabled = true
        break
    if not pairEnabled:
      raise newException(
        ValueError, "contact policy requires an enabled collision pair")
    for previous in 0 ..< index:
      let other = config.contactPolicies[previous]
      if (other.layer1 == policy.layer1 and other.layer2 == policy.layer2) or
          (other.layer1 == policy.layer2 and other.layer2 == policy.layer1):
        raise newException(
          ValueError, "contact policies must use unique layer pairs")
    if policy.friction.isSome and
        (not policy.friction.get.isFinite or policy.friction.get < 0):
      raise newException(
        ValueError, "contact policy friction must be finite and non-negative")
    if policy.restitution.isSome and
        (not policy.restitution.get.isFinite or
          policy.restitution.get < 0 or policy.restitution.get > 1):
      raise newException(
        ValueError, "contact policy restitution must be in [0, 1]")
    for scale in [
        policy.inverseMassScale1, policy.inverseInertiaScale1,
        policy.inverseMassScale2, policy.inverseInertiaScale2]:
      if not scale.isFinite or scale < 0:
        raise newException(
          ValueError, "contact policy inverse scales must be finite and non-negative")
    if not policy.linearSurfaceVelocity.isFinite or
        not policy.angularSurfaceVelocity.isFinite:
      raise newException(
        ValueError, "contact policy surface velocities must be finite")

proc validate(config: SoftBodyConfig) =
  if config.numIterations == 0:
    raise newException(ValueError, "soft body solver iterations must be positive")
  if not config.lraMaxDistanceMultiplier.isFinite or
      config.lraMaxDistanceMultiplier <= 0:
    raise newException(
      ValueError, "soft body LRA distance multiplier must be positive")
  if not config.skinnedMaxDistanceMultiplier.isFinite or
      config.skinnedMaxDistanceMultiplier < 0:
    raise newException(
      ValueError, "soft body skin distance multiplier must be non-negative")
  for value in [config.edgeCompliance, config.shearCompliance,
      config.bendCompliance]:
    if not value.isFinite or value < 0:
      raise newException(
        ValueError, "soft body compliance must be finite and non-negative")
  if not config.angleTolerance.isFinite or config.angleTolerance < 0 or
      config.angleTolerance > PI.float32:
    raise newException(
      ValueError, "soft body angle tolerance must be in [0, PI]")
  if not config.linearDamping.isFinite or config.linearDamping < 0 or
      not config.maxLinearVelocity.isFinite or config.maxLinearVelocity <= 0:
    raise newException(
      ValueError, "soft body damping and velocity limit are invalid")
  if not config.restitution.isFinite or config.restitution < 0 or
      not config.friction.isFinite or config.friction < 0 or
      not config.pressure.isFinite or config.pressure < 0 or
      not config.gravityFactor.isFinite or
      not config.vertexRadius.isFinite or config.vertexRadius < 0:
    raise newException(ValueError, "soft body physical settings are invalid")

proc validate(mesh: SoftBodyMesh) =
  if mesh.vertices.len < 3:
    raise newException(ValueError, "soft body mesh requires at least three vertices")
  if mesh.faces.len == 0 and mesh.edgeConstraints.len == 0 and
      mesh.longRangeConstraints.len == 0 and mesh.rods.len == 0:
    raise newException(
      ValueError, "soft body mesh requires faces or particle constraints")
  if uint64(mesh.vertices.len) > uint64(high(uint32)) or
      uint64(mesh.vertexAttributes.len) > uint64(high(uint32)) or
      uint64(mesh.faces.len) > uint64(high(uint32)) or
      uint64(mesh.materials.len) > uint64(high(uint32)) or
      uint64(mesh.edgeConstraints.len) > uint64(high(uint32)) or
      uint64(mesh.dihedralBendConstraints.len) > uint64(high(uint32)) or
      uint64(mesh.longRangeConstraints.len) > uint64(high(uint32)) or
      uint64(mesh.volumeConstraints.len) > uint64(high(uint32)) or
      uint64(mesh.rods.len) > uint64(high(uint32)) or
      uint64(mesh.rodBendTwistConstraints.len) > uint64(high(uint32)) or
      uint64(mesh.skinBindPose.len) > uint64(high(uint32)) or
      uint64(mesh.skinConstraints.len) > uint64(high(uint32)):
    raise newException(ValueError, "soft body mesh exceeds Jolt index limits")
  if mesh.vertexAttributes.len > mesh.vertices.len:
    raise newException(
      ValueError, "soft body vertex attributes exceed the vertex count")
  if mesh.skinBindPose.len == 0 and mesh.skinConstraints.len > 0:
    raise newException(ValueError, "soft body skin constraints need a bind pose")
  if mesh.skinBindPose.len > 0 and mesh.skinConstraints.len == 0:
    raise newException(ValueError, "soft body skin bind pose needs constraints")
  for material in mesh.materials:
    discard physicsMaterial(material.name, material.debugColor)
  for vertex in mesh.vertices:
    if not vertex.position.isFinite or not vertex.velocity.isFinite or
        not vertex.inverseMass.isFinite or vertex.inverseMass < 0:
      raise newException(ValueError, "soft body vertex data is invalid")
  for attributes in mesh.vertexAttributes:
    for value in [attributes.edgeCompliance, attributes.shearCompliance,
        attributes.bendCompliance]:
      if not value.isFinite or value < 0:
        raise newException(
          ValueError, "soft body vertex compliance is invalid")
    if not attributes.lraMaxDistanceMultiplier.isFinite or
        attributes.lraMaxDistanceMultiplier <= 0:
      raise newException(
        ValueError, "soft body vertex LRA multiplier is invalid")
  for face in mesh.faces:
    let a = int(face.vertices[0])
    let b = int(face.vertices[1])
    let c = int(face.vertices[2])
    if a >= mesh.vertices.len or b >= mesh.vertices.len or
        c >= mesh.vertices.len:
      raise newException(ValueError, "soft body face index is out of bounds")
    if mesh.materials.len == 0:
      if face.materialIndex != 0:
        raise newException(
          ValueError, "soft body material index requires mesh materials")
    elif uint64(face.materialIndex) >= uint64(mesh.materials.len):
      raise newException(ValueError, "soft body material index is out of bounds")
    if a == b or a == c or b == c:
      raise newException(ValueError, "soft body face is degenerate")
    let p1 = mesh.vertices[a].position
    let p2 = mesh.vertices[b].position
    let p3 = mesh.vertices[c].position
    let e1 = Vec3(x: p2.x - p1.x, y: p2.y - p1.y, z: p2.z - p1.z)
    let e2 = Vec3(x: p3.x - p1.x, y: p3.y - p1.y, z: p3.z - p1.z)
    let cross = Vec3(
      x: e1.y * e2.z - e1.z * e2.y,
      y: e1.z * e2.x - e1.x * e2.z,
      z: e1.x * e2.y - e1.y * e2.x)
    if cross.x * cross.x + cross.y * cross.y + cross.z * cross.z <= 1.0e-12:
      raise newException(ValueError, "soft body face has zero area")
  for edge in mesh.edgeConstraints:
    let first = int(edge.vertices[0])
    let second = int(edge.vertices[1])
    if first >= mesh.vertices.len or second >= mesh.vertices.len:
      raise newException(ValueError, "soft body edge index is out of bounds")
    if first == second:
      raise newException(ValueError, "soft body edge needs two vertices")
    if not edge.compliance.isFinite or edge.compliance < 0:
      raise newException(ValueError, "soft body edge compliance is invalid")
    let delta = Vec3(
      x: mesh.vertices[second].position.x - mesh.vertices[first].position.x,
      y: mesh.vertices[second].position.y - mesh.vertices[first].position.y,
      z: mesh.vertices[second].position.z - mesh.vertices[first].position.z)
    if delta.x * delta.x + delta.y * delta.y + delta.z * delta.z <=
        1.0e-12'f32:
      raise newException(ValueError, "soft body edge has zero length")
  for bend in mesh.dihedralBendConstraints:
    var seen: array[4, int]
    for corner in 0 ..< 4:
      seen[corner] = int(bend.vertices[corner])
      if seen[corner] >= mesh.vertices.len:
        raise newException(
          ValueError, "soft body dihedral index is out of bounds")
      for previous in 0 ..< corner:
        if seen[previous] == seen[corner]:
          raise newException(
            ValueError, "soft body dihedral bend needs four vertices")
    if not bend.compliance.isFinite or bend.compliance < 0:
      raise newException(ValueError, "soft body dihedral compliance is invalid")
  for attachment in mesh.longRangeConstraints:
    let fixed = int(attachment.vertices[0])
    let dynamic = int(attachment.vertices[1])
    if fixed >= mesh.vertices.len or dynamic >= mesh.vertices.len:
      raise newException(ValueError, "soft body LRA index is out of bounds")
    if fixed == dynamic:
      raise newException(ValueError, "soft body LRA needs two vertices")
    if mesh.vertices[fixed].inverseMass != 0 or
        mesh.vertices[dynamic].inverseMass <= 0:
      raise newException(
        ValueError, "soft body LRA must connect a fixed vertex to a dynamic vertex")
    if not attachment.maxDistance.isFinite or attachment.maxDistance < 0:
      raise newException(ValueError, "soft body LRA maximum distance is invalid")
  for volume in mesh.volumeConstraints:
    var indices: array[4, int]
    for corner in 0 ..< 4:
      indices[corner] = int(volume.vertices[corner])
      if indices[corner] >= mesh.vertices.len:
        raise newException(ValueError, "soft body volume index is out of bounds")
      for previous in 0 ..< corner:
        if indices[previous] == indices[corner]:
          raise newException(ValueError, "soft body volume needs four vertices")
    if not volume.compliance.isFinite or volume.compliance < 0:
      raise newException(ValueError, "soft body volume compliance is invalid")
    let p1 = mesh.vertices[indices[0]].position
    let p2 = mesh.vertices[indices[1]].position
    let p3 = mesh.vertices[indices[2]].position
    let p4 = mesh.vertices[indices[3]].position
    let e1 = Vec3(x: p2.x - p1.x, y: p2.y - p1.y, z: p2.z - p1.z)
    let e2 = Vec3(x: p3.x - p1.x, y: p3.y - p1.y, z: p3.z - p1.z)
    let e3 = Vec3(x: p4.x - p1.x, y: p4.y - p1.y, z: p4.z - p1.z)
    let sixVolume = abs(
      (e1.y * e2.z - e1.z * e2.y) * e3.x +
      (e1.z * e2.x - e1.x * e2.z) * e3.y +
      (e1.x * e2.y - e1.y * e2.x) * e3.z)
    if sixVolume <= 1.0e-9'f32:
      raise newException(ValueError, "soft body volume has zero volume")
  var constrainedRods = newSeq[bool](mesh.rods.len)
  for rod in mesh.rods:
    let first = int(rod.vertices[0])
    let second = int(rod.vertices[1])
    if first >= mesh.vertices.len or second >= mesh.vertices.len:
      raise newException(ValueError, "soft body rod vertex is out of bounds")
    if first == second:
      raise newException(ValueError, "soft body rod needs two vertices")
    if not rod.compliance.isFinite or rod.compliance < 0:
      raise newException(ValueError, "soft body rod compliance is invalid")
    let p1 = mesh.vertices[first].position
    let p2 = mesh.vertices[second].position
    let dx = p2.x - p1.x
    let dy = p2.y - p1.y
    let dz = p2.z - p1.z
    if dx * dx + dy * dy + dz * dz <= 1.0e-12'f32:
      raise newException(ValueError, "soft body rod has zero length")
  for constraint in mesh.rodBendTwistConstraints:
    let first = int(constraint.rods[0])
    let second = int(constraint.rods[1])
    if first >= mesh.rods.len or second >= mesh.rods.len:
      raise newException(ValueError, "rod bend-twist index is out of bounds")
    if first == second:
      raise newException(ValueError, "rod bend-twist needs two rods")
    if not constraint.compliance.isFinite or constraint.compliance < 0:
      raise newException(ValueError, "rod bend-twist compliance is invalid")
    let firstRod = mesh.rods[first].vertices
    let secondRod = mesh.rods[second].vertices
    var connected = false
    for firstVertex in firstRod:
      for secondVertex in secondRod:
        if firstVertex == secondVertex:
          connected = true
    if not connected:
      raise newException(ValueError, "rod bend-twist rods must share a vertex")
    constrainedRods[first] = true
    constrainedRods[second] = true
  for constrained in constrainedRods:
    if not constrained:
      raise newException(ValueError, "each soft body rod needs a bend-twist constraint")
  for joint in mesh.skinBindPose:
    if not joint.position.isFinite or not joint.rotation.isFinite:
      raise newException(ValueError, "soft body skin bind pose is invalid")
    let lengthSquared = joint.rotation.x * joint.rotation.x +
      joint.rotation.y * joint.rotation.y + joint.rotation.z * joint.rotation.z +
      joint.rotation.w * joint.rotation.w
    if lengthSquared <= 1.0e-12'f32:
      raise newException(ValueError, "soft body skin bind rotation is invalid")
  var skinnedVertices = newSeq[bool](mesh.vertices.len)
  for constraint in mesh.skinConstraints:
    let vertex = int(constraint.vertex)
    if vertex >= mesh.vertices.len:
      raise newException(ValueError, "soft body skinned vertex is out of bounds")
    if skinnedVertices[vertex]:
      raise newException(ValueError, "soft body vertex has multiple skin constraints")
    skinnedVertices[vertex] = true
    if constraint.weights.len < 1 or constraint.weights.len > 4:
      raise newException(ValueError, "soft body skin constraint needs 1 to 4 weights")
    if not constraint.maxDistance.isFinite or constraint.maxDistance < 0 or
        not constraint.backStopDistance.isFinite or
        constraint.backStopDistance < 0 or
        not constraint.backStopRadius.isFinite or constraint.backStopRadius <= 0:
      raise newException(ValueError, "soft body skin distances are invalid")
    var total = 0.0'f32
    var joints: seq[uint32]
    for weight in constraint.weights:
      if int(weight.joint) >= mesh.skinBindPose.len:
        raise newException(ValueError, "soft body skin joint is out of bounds")
      if weight.joint in joints:
        raise newException(ValueError, "soft body skin joints must be unique")
      joints.add(weight.joint)
      if not weight.weight.isFinite or weight.weight <= 0:
        raise newException(ValueError, "soft body skin weight must be positive")
      total += weight.weight
    if not total.isFinite or abs(total - 1.0'f32) > 1.0e-3'f32:
      raise newException(ValueError, "soft body skin weights must add up to 1")

proc requireLayer(world: World; layer: CollisionLayer) =
  world.requireOpen()
  if uint32(layer) >= world.layerCount:
    raise newException(ValueError, "collision layer is not defined by this world")

proc queryLayerValue(world: World; layer: Option[CollisionLayer]): uint32 =
  if layer.isNone:
    return high(uint32)
  world.requireLayer(layer.get)
  uint32(layer.get)

proc queryLayerSet*(layers: openArray[CollisionLayer]): QueryLayerSet =
  if layers.len == 0:
    raise newException(ValueError, "a query layer set must not be empty")
  for layer in layers:
    var found = false
    for existing in result.layers:
      if existing == layer:
        found = true
        break
    if not found:
      result.layers.add(layer)

func len*(layers: QueryLayerSet): int =
  layers.layers.len

func `[]`*(layers: QueryLayerSet; index: int): CollisionLayer =
  layers.layers[index]

proc requireLayers(world: World; layers: QueryLayerSet) =
  if layers.layers.len == 0:
    raise newException(ValueError, "a query layer set must not be empty")
  for layer in layers.layers:
    world.requireLayer(layer)

proc bodyQueryCriteria*(motionTypes: set[MotionType] = {};
                        layers: seq[CollisionLayer] = @[];
                        active = none(bool);
                        sensor = none(bool);
                        softBody = none(bool);
                        inBroadPhase = none(bool);
                        userData = none(uint64)): BodyQueryCriteria =
  ## Builds reusable declarative criteria for `queryBodies` and
  ## `queryBodyFilter`. Empty sets and absent values are unconstrained.
  BodyQueryCriteria(
    motionTypes: motionTypes,
    layers: layers,
    active: active,
    sensor: sensor,
    softBody: softBody,
    inBroadPhase: inBroadPhase,
    userData: userData)

proc queryBodyFilter*(bodyIds: openArray[BodyId];
                      mode: QueryBodyFilterMode): QueryBodyFilter =
  if bodyIds.len == 0:
    raise newException(ValueError, "a query body filter must not be empty")
  result.mode = mode
  result.enabled = true
  for bodyId in bodyIds:
    let value = uint32(bodyId)
    if value == high(uint32):
      raise newException(ValueError, "a query body filter contains an invalid body ID")
    if value notin result.bodyIds:
      result.bodyIds.add(value)

proc includeBodies*(bodyIds: openArray[BodyId]): QueryBodyFilter =
  queryBodyFilter(bodyIds, QueryBodyFilterMode.IncludeOnly)

proc excludeBodies*(bodyIds: openArray[BodyId]): QueryBodyFilter =
  queryBodyFilter(bodyIds, QueryBodyFilterMode.Exclude)

func len*(filter: QueryBodyFilter): int =
  filter.bodyIds.len

func filterMode*(filter: QueryBodyFilter): QueryBodyFilterMode =
  filter.mode

func isEnabled*(filter: QueryBodyFilter): bool =
  filter.enabled

proc requireBodyFilter(world: World; filter: QueryBodyFilter) =
  if not filter.enabled:
    return
  for filterId in filter.bodyIds:
    if filterId notin world.bodyIds and filterId notin world.characterBodyIds and
        filterId notin world.ragdollBodyIds and
        filterId notin world.rigidCharacterBodyIds:
      raise newException(
        ValueError, "query body filter contains a body outside this world")

proc nativeBodyIds(filter: QueryBodyFilter): ptr uint32 =
  if filter.bodyIds.len == 0:
    nil
  else:
    unsafeAddr filter.bodyIds[0]

func includesOnly(filter: QueryBodyFilter): bool =
  filter.enabled and filter.mode == QueryBodyFilterMode.IncludeOnly

proc trackedBodyIds(world: World): seq[uint32] =
  var seen = initHashSet[uint32]()
  template appendUnique(source: untyped) =
    for bodyId in source:
      if bodyId notin seen:
        seen.incl(bodyId)
        result.add(bodyId)
  appendUnique(world.bodyIds)
  appendUnique(world.characterBodyIds)
  appendUnique(world.rigidCharacterBodyIds)
  appendUnique(world.ragdollBodyIds)

proc queryBodies*(world: World): seq[QueryBodyInfo] =
  ## Captures every body currently owned by the world under one native
  ## multi-body read lock. Returned values are detached from Jolt.
  world.requireOpen()
  let ids = world.trackedBodyIds()
  if ids.len == 0:
    return @[]
  if uint64(ids.len) > uint64(high(uint32)):
    raise newException(ValueError, "too many bodies to query")

  var native = newSeq[raw.BodySnapshotData](ids.len)
  world.physics.readBodySnapshots(
    unsafeAddr ids[0], uint32(ids.len), addr native[0])
  result = newSeqOfCap[QueryBodyInfo](ids.len)
  for index, state in native:
    if not state.mSucceeded:
      raise newException(JoltError, "Jolt could not lock a body for query")
    if state.mMotionType > uint8(ord(high(MotionType))):
      raise newException(JoltError, "Jolt returned an unknown body motion type")
    if uint32(state.mObjectLayer) >= world.layerCount:
      raise newException(JoltError, "Jolt returned an unknown body collision layer")
    result.add(QueryBodyInfo(
      bodyId: BodyId(ids[index]),
      motionType: MotionType(state.mMotionType),
      collisionLayer: CollisionLayer(state.mObjectLayer),
      position: state.mPosition.fromRaw,
      active: state.mActive,
      sensor: state.mSensor,
      softBody: state.mSoftBody,
      inBroadPhase: state.mInBroadPhase,
      userData: state.mUserData))

func matches(info: QueryBodyInfo; criteria: BodyQueryCriteria): bool =
  if criteria.motionTypes != {} and info.motionType notin criteria.motionTypes:
    return false
  if criteria.layers.len > 0 and info.collisionLayer notin criteria.layers:
    return false
  if criteria.active.isSome and info.active != criteria.active.get:
    return false
  if criteria.sensor.isSome and info.sensor != criteria.sensor.get:
    return false
  if criteria.softBody.isSome and info.softBody != criteria.softBody.get:
    return false
  if criteria.inBroadPhase.isSome and
      info.inBroadPhase != criteria.inBroadPhase.get:
    return false
  if criteria.userData.isSome and info.userData != criteria.userData.get:
    return false
  true

proc queryBodies*(world: World;
                  criteria: BodyQueryCriteria): seq[QueryBodyInfo] =
  ## Applies declarative criteria to a detached, internally consistent body
  ## snapshot. No user code runs while Jolt body locks are held.
  world.requireOpen()
  for layer in criteria.layers:
    world.requireLayer(layer)
  for info in world.queryBodies():
    if info.matches(criteria):
      result.add(info)

proc queryBodies*(world: World;
                  predicate: QueryBodyPredicate): seq[QueryBodyInfo] =
  ## Runs an arbitrary Nim predicate on the caller thread after all native
  ## body locks have been released.
  world.requireOpen()
  if predicate.isNil:
    raise newException(ValueError, "query body predicate must not be nil")
  let snapshot = world.queryBodies()
  for info in snapshot:
    if predicate(info):
      result.add(info)

proc queryBodyFilter*(world: World;
                      criteria: BodyQueryCriteria): QueryBodyFilter =
  ## Resolves body properties to a reusable native ID filter. The selection is
  ## a snapshot; call this again after relevant body properties change.
  result.enabled = true
  result.mode = QueryBodyFilterMode.IncludeOnly
  for info in world.queryBodies(criteria):
    result.bodyIds.add(uint32(info.bodyId))

proc queryBodyFilter*(world: World;
                      predicate: QueryBodyPredicate): QueryBodyFilter =
  ## Resolves a caller-thread Nim predicate to a reusable native ID filter.
  ## The predicate is never invoked by Jolt or from a Jolt worker thread.
  result.enabled = true
  result.mode = QueryBodyFilterMode.IncludeOnly
  for info in world.queryBodies(predicate):
    result.bodyIds.add(uint32(info.bodyId))

proc validate(limits: DebugDrawLimits) =
  for value in [limits.maxLines, limits.maxTriangles, limits.maxTexts]:
    if uint64(value) > uint64(high(uint32)) or value > uint(high(int)):
      raise newException(ValueError, "debug draw limits exceed this platform")
  if limits.maxTextBytes > uint(high(int)):
    raise newException(ValueError, "debug text byte limit exceeds this platform")

when defined(joltDebugRenderer):
  func fromRaw(value: raw.DebugPointData): Vec3 =
    Vec3(x: value.mX, y: value.mY, z: value.mZ)

  func fromRaw(value: raw.DebugColorData): DebugColor =
    DebugColor(r: value.mR, g: value.mG, b: value.mB, a: value.mA)

  func toRaw(settings: DebugBodyDrawSettings): raw.DebugBodyDrawSettingsData =
    result.mDrawGetSupportFunction = settings.drawGetSupportFunction
    result.mDrawSupportDirection = settings.drawSupportDirection
    result.mDrawGetSupportingFace = settings.drawGetSupportingFace
    result.mDrawShape = settings.drawShape
    result.mDrawShapeWireframe = settings.drawShapeWireframe
    result.mDrawShapeColor = uint8(ord(settings.shapeColor))
    result.mDrawBoundingBox = settings.drawBoundingBox
    result.mDrawCenterOfMassTransform = settings.drawCenterOfMassTransform
    result.mDrawWorldTransform = settings.drawWorldTransform
    result.mDrawVelocity = settings.drawVelocity
    result.mDrawMassAndInertia = settings.drawMassAndInertia
    result.mDrawSleepStats = settings.drawSleepStats
    result.mDrawSoftBodyVertices = settings.drawSoftBodyVertices
    result.mDrawSoftBodyVertexVelocities = settings.drawSoftBodyVertexVelocities
    result.mDrawSoftBodyEdgeConstraints = settings.drawSoftBodyEdgeConstraints
    result.mDrawSoftBodyBendConstraints = settings.drawSoftBodyBendConstraints
    result.mDrawSoftBodyVolumeConstraints = settings.drawSoftBodyVolumeConstraints
    result.mDrawSoftBodySkinConstraints = settings.drawSoftBodySkinConstraints
    result.mDrawSoftBodyLRAConstraints = settings.drawSoftBodyLRAConstraints
    result.mDrawSoftBodyRods = settings.drawSoftBodyRods
    result.mDrawSoftBodyRodStates = settings.drawSoftBodyRodStates
    result.mDrawSoftBodyRodBendTwistConstraints =
      settings.drawSoftBodyRodBendTwistConstraints
    result.mDrawSoftBodyPredictedBounds = settings.drawSoftBodyPredictedBounds
    result.mDrawSoftBodyConstraintColor =
      uint8(ord(settings.softBodyConstraintColor))

  proc captureDebugDraw*(world: World;
                         options = defaultDebugDrawOptions()): DebugDrawFrame =
    ## Captures Jolt's body and constraint diagnostics as detached primitives.
    ## No Nim callback is retained or invoked by Jolt.
    world.requireOpen()
    options.cameraPosition.requireFinite("debug camera position")
    options.limits.validate()
    world.requireBodyFilter(options.bodyFilter)

    let collector = world.physics.captureDebugDraw(
      options.cameraPosition.toRaw,
      options.bodySettings.toRaw,
      options.bodyFilter.nativeBodyIds,
      uint32(options.bodyFilter.len),
      options.bodyFilter.enabled,
      options.bodyFilter.includesOnly,
      options.drawBodies,
      options.drawConstraints,
      options.drawConstraintLimits,
      options.drawConstraintReferenceFrames,
      uint32(options.limits.maxLines),
      uint32(options.limits.maxTriangles),
      uint32(options.limits.maxTexts),
      csize_t(options.limits.maxTextBytes))
    if collector.isNil:
      raise newException(JoltError, "Jolt could not capture debug drawing")
    defer: raw.delete(collector)

    result.droppedLines = collector.droppedDebugLineCount()
    result.droppedTriangles = collector.droppedDebugTriangleCount()
    result.droppedTexts = collector.droppedDebugTextCount()

    result.lines = newSeq[DebugLine](int(collector.debugLineCount()))
    for index in 0 ..< result.lines.len:
      var value: raw.DebugLineData
      if not collector.getDebugLine(uint32(index), addr value):
        raise newException(JoltError, "Jolt returned an invalid debug line")
      result.lines[index] = DebugLine(
        fromPosition: value.mFrom.fromRaw,
        toPosition: value.mTo.fromRaw,
        color: value.mColor.fromRaw)

    result.triangles =
      newSeq[DebugTriangle](int(collector.debugTriangleCount()))
    for index in 0 ..< result.triangles.len:
      var value: raw.DebugTriangleData
      if not collector.getDebugTriangle(uint32(index), addr value):
        raise newException(JoltError, "Jolt returned an invalid debug triangle")
      result.triangles[index] = DebugTriangle(
        v1: value.mV1.fromRaw,
        v2: value.mV2.fromRaw,
        v3: value.mV3.fromRaw,
        color: value.mColor.fromRaw,
        castsShadow: value.mCastsShadow)

    result.texts = newSeq[DebugText](int(collector.debugTextCount()))
    for index in 0 ..< result.texts.len:
      var value: raw.DebugTextData
      if not collector.getDebugText(uint32(index), addr value):
        raise newException(JoltError, "Jolt returned invalid debug text")
      if value.mTextLength > 0 and value.mText.isNil:
        raise newException(JoltError, "Jolt returned missing debug text")
      var text = newString(int(value.mTextLength))
      if text.len > 0:
        copyMem(addr text[0], value.mText, text.len)
      result.texts[index] = DebugText(
        position: value.mPosition.fromRaw,
        text: text,
        color: value.mColor.fromRaw,
        height: value.mHeight)
else:
  proc captureDebugDraw*(world: World;
                         options = defaultDebugDrawOptions()): DebugDrawFrame =
    ## Enable this API with `-d:joltDebugRenderer` and a matching Jolt build.
    discard world
    discard options
    raise newException(
      JoltError,
      "debug drawing requires -d:joltDebugRenderer and a matching Jolt build")

func querySubShape*(bodyId: BodyId; subShapeId: uint32): QuerySubShape =
  QuerySubShape(bodyId: bodyId, subShapeId: subShapeId)

func querySubShape*(hit: RayHit): QuerySubShape =
  querySubShape(hit.bodyId, hit.subShapeId)

func querySubShape*(hit: ShapeCastHit): QuerySubShape =
  querySubShape(hit.bodyId, hit.subShapeId)

func querySubShape*(hit: OverlapHit): QuerySubShape =
  querySubShape(hit.bodyId, hit.subShapeId)

proc querySubShapeFilter*(subShapes: openArray[QuerySubShape];
                          mode: QuerySubShapeFilterMode): QuerySubShapeFilter =
  if subShapes.len == 0:
    raise newException(ValueError, "a query sub-shape filter must not be empty")
  result.mode = mode
  for subShape in subShapes:
    let bodyId = uint32(subShape.bodyId)
    if bodyId == high(uint32):
      raise newException(
        ValueError, "a query sub-shape filter contains an invalid body ID")
    var found = false
    for index in 0 ..< result.bodyIds.len:
      if result.bodyIds[index] == bodyId and
          result.subShapeIds[index] == subShape.subShapeId:
        found = true
        break
    if not found:
      result.bodyIds.add(bodyId)
      result.subShapeIds.add(subShape.subShapeId)

proc includeSubShapes*(subShapes: openArray[QuerySubShape]): QuerySubShapeFilter =
  querySubShapeFilter(subShapes, QuerySubShapeFilterMode.IncludeOnlySubShapes)

proc excludeSubShapes*(subShapes: openArray[QuerySubShape]): QuerySubShapeFilter =
  querySubShapeFilter(subShapes, QuerySubShapeFilterMode.ExcludeSubShapes)

func len*(filter: QuerySubShapeFilter): int =
  filter.bodyIds.len

func filterMode*(filter: QuerySubShapeFilter): QuerySubShapeFilterMode =
  filter.mode

proc requireSubShapeFilter(world: World; filter: QuerySubShapeFilter) =
  if filter.bodyIds.len == 0:
    return
  for bodyId in filter.bodyIds:
    if bodyId notin world.bodyIds and bodyId notin world.characterBodyIds and
        bodyId notin world.ragdollBodyIds and
        bodyId notin world.rigidCharacterBodyIds:
      raise newException(
        ValueError, "query sub-shape filter contains a body outside this world")

proc nativeSubShapeBodyIds(filter: QuerySubShapeFilter): ptr uint32 =
  if filter.bodyIds.len == 0: nil else: unsafeAddr filter.bodyIds[0]

proc nativeSubShapeIds(filter: QuerySubShapeFilter): ptr uint32 =
  if filter.subShapeIds.len == 0: nil else: unsafeAddr filter.subShapeIds[0]

func includesOnly(filter: QuerySubShapeFilter): bool =
  filter.bodyIds.len > 0 and
    filter.mode == QuerySubShapeFilterMode.IncludeOnlySubShapes

proc removeTrackedId(world: World; id: uint32) =
  for index, candidate in world.bodyIds:
    if candidate == id:
      world.bodyIds.delete(index)
      return

proc removeTrackedConstraint(world: World; constraint: ptr raw.Constraint) =
  for index, candidate in world.constraints:
    if candidate == constraint:
      world.constraints.delete(index)
      return

proc removeTrackedCharacter(world: World; character: ptr raw.CharacterHandle) =
  for index, candidate in world.characters:
    if candidate == character:
      world.characters.delete(index)
      return

proc removeTrackedCharacterBodyId(world: World; bodyId: uint32) =
  for index, candidate in world.characterBodyIds:
    if candidate == bodyId:
      world.characterBodyIds.delete(index)
      return

proc removeTrackedRigidCharacter(
    world: World; character: ptr raw.RigidCharacterHandle; bodyId: uint32) =
  for index, candidate in world.rigidCharacters:
    if candidate == character:
      world.rigidCharacters.delete(index)
      break
  for index, candidate in world.rigidCharacterBodyIds:
    if candidate == bodyId:
      world.rigidCharacterBodyIds.delete(index)
      break

proc removeTrackedVehicle(world: World; vehicle: ptr raw.VehicleHandle) =
  for index, candidate in world.vehicles:
    if candidate == vehicle:
      world.vehicles.delete(index)
      return

proc removeTrackedRagdoll(world: World; ragdoll: ptr raw.RagdollHandle) =
  for index, candidate in world.ragdolls:
    if candidate == ragdoll:
      world.ragdolls.delete(index)
      return

proc removeTrackedSceneInstance(
    world: World; instance: ptr raw.PhysicsSceneInstanceHandle) =
  for index, candidate in world.sceneInstances:
    if candidate == instance:
      world.sceneInstances.delete(index)
      return

proc closeBody(body: var BodyObj) {.raises: [].} =
  if not body.alive:
    return

  let world = body.owner
  if not world.isNil and not world.closed and not world.closing:
    world.eventBridge.removeBodyContactPolicies(body.rawId)
    let bodies = world.physics.bodyInterface()
    bodies.removeAndDestroyBody(raw.bodyID(body.rawId))
    world.removeTrackedId(body.rawId)

  body.alive = false

proc closeSoftBody(body: var SoftBodyObj) {.raises: [].} =
  if not body.alive:
    return
  let world = body.owner
  if not world.isNil and not world.closed and not world.closing:
    world.eventBridge.removeBodyContactPolicies(body.rawId)
    world.physics.bodyInterface().removeAndDestroyBody(raw.bodyID(body.rawId))
    world.removeTrackedId(body.rawId)
  body.alive = false

proc closeConstraint(constraint: var ConstraintObj) {.raises: [].} =
  if not constraint.alive:
    return

  let world = constraint.owner
  if not world.isNil and not world.closed and not world.closing and
      not constraint.native.isNil:
    world.physics.removeConstraint(constraint.native)
    world.removeTrackedConstraint(constraint.native)

  if not constraint.body1.isNil:
    dec constraint.body1.constraintCount
  if not constraint.body2.isNil:
    dec constraint.body2.constraintCount
  constraint.dependencies.setLen(0)
  constraint.native = nil
  constraint.alive = false

proc closeCharacter(character: var CharacterObj) {.raises: [].} =
  if not character.alive:
    return

  let world = character.owner
  if not world.isNil and not world.closed and not world.closing and
      not character.native.isNil:
    raw.delete(character.native)
    world.removeTrackedCharacter(character.native)
    if character.innerBodyIdValue.isSome:
      world.removeTrackedCharacterBodyId(character.innerBodyIdValue.get)

  character.native = nil
  character.innerBodyIdValue = none(uint32)
  character.alive = false

proc closeRigidCharacter(character: var RigidCharacterObj) {.raises: [].} =
  if not character.alive:
    return

  let world = character.owner
  if not world.isNil and not world.closed and not world.closing and
      not character.native.isNil:
    raw.delete(character.native)
    world.removeTrackedRigidCharacter(character.native, character.rawId)

  character.native = nil
  character.alive = false

proc closeVehicle(vehicle: var VehicleObj) {.raises: [].} =
  if not vehicle.alive:
    return

  let world = vehicle.owner
  if not world.isNil and not world.closed and not world.closing and
      not vehicle.native.isNil:
    raw.delete(vehicle.native)
    world.removeTrackedVehicle(vehicle.native)

  if not vehicle.chassisBody.isNil:
    dec vehicle.chassisBody.constraintCount
  vehicle.native = nil
  vehicle.alive = false

proc closeTrackedVehicle(vehicle: var TrackedVehicleObj) {.raises: [].} =
  if not vehicle.alive:
    return

  let world = vehicle.owner
  if not world.isNil and not world.closed and not world.closing and
      not vehicle.native.isNil:
    raw.delete(vehicle.native)
    world.removeTrackedVehicle(vehicle.native)

  if not vehicle.chassisBody.isNil:
    dec vehicle.chassisBody.constraintCount
  vehicle.native = nil
  vehicle.alive = false

proc closeRagdoll(ragdoll: var RagdollObj) {.raises: [].} =
  if not ragdoll.alive:
    return
  let world = ragdoll.owner
  if not world.isNil and not world.closed and not world.closing and
      not ragdoll.native.isNil:
    raw.delete(ragdoll.native)
    world.removeTrackedRagdoll(ragdoll.native)
    for id in ragdoll.bodyIds:
      for index, candidate in world.ragdollBodyIds:
        if candidate == id:
          world.ragdollBodyIds.delete(index)
          break
  ragdoll.native = nil
  ragdoll.bodyIds.setLen(0)
  ragdoll.alive = false

proc closeSkeletonMapper(mapper: var SkeletonMapperObj) {.raises: [].} =
  if not mapper.alive and mapper.native.isNil and not mapper.acquiredJolt:
    return
  if not mapper.native.isNil:
    raw.delete(mapper.native)
  mapper.native = nil
  if mapper.acquiredJolt:
    raw.releaseJolt()
    mapper.acquiredJolt = false
  mapper.alive = false

proc closeSkeletalAnimation(
    animation: var SkeletalAnimationObj) {.raises: [].} =
  if not animation.alive and animation.native.isNil and
      not animation.acquiredJolt:
    return
  if not animation.native.isNil:
    raw.delete(animation.native)
  animation.native = nil
  if animation.acquiredJolt:
    raw.releaseJolt()
    animation.acquiredJolt = false
  animation.alive = false

proc closePhysicsScene(scene: var PhysicsSceneObj) {.raises: [].} =
  if not scene.alive and scene.native.isNil and not scene.acquiredJolt:
    return
  if not scene.native.isNil:
    raw.delete(scene.native)
  scene.native = nil
  if scene.acquiredJolt:
    raw.releaseJolt()
    scene.acquiredJolt = false
  scene.alive = false

proc closePhysicsSceneInstance(
    instance: var PhysicsSceneInstanceObj) {.raises: [].} =
  if not instance.alive:
    return
  let world = instance.owner
  if not world.isNil and not world.closed and not world.closing and
      not instance.native.isNil:
    raw.delete(instance.native)
    for constraint in instance.constraints:
      world.removeTrackedConstraint(constraint)
    for id in instance.bodyIds:
      world.removeTrackedId(id)
    world.removeTrackedSceneInstance(instance.native)
  instance.native = nil
  instance.bodyIds.setLen(0)
  instance.constraints.setLen(0)
  instance.alive = false

proc closeCollisionGroupFilter(
    filter: var CollisionGroupFilterObj) {.raises: [].} =
  if not filter.alive:
    return
  if not filter.native.isNil:
    raw.release(filter.native)
  filter.native = nil
  filter.alive = false

proc closeWorldState(state: var WorldStateObj) {.raises: [].} =
  if not state.alive:
    return
  if not state.native.isNil:
    raw.delete(state.native)
  state.native = nil
  state.bodyIds.setLen(0)
  state.constraints.setLen(0)
  state.characters.setLen(0)
  state.rigidCharacters.setLen(0)
  state.vehicles.setLen(0)
  state.ragdolls.setLen(0)
  state.alive = false

proc closeWorld(world: var WorldObj) {.raises: [].} =
  if world.closed or world.closing:
    return

  world.closing = true

  if not world.physics.isNil:
    for instance in world.sceneInstances:
      raw.abandon(instance)
    world.sceneInstances.setLen(0)

    for ragdoll in world.ragdolls:
      raw.delete(ragdoll)
    world.ragdolls.setLen(0)
    world.ragdollBodyIds.setLen(0)

    for vehicle in world.vehicles:
      raw.delete(vehicle)
    world.vehicles.setLen(0)

    for character in world.characters:
      raw.delete(character)
    world.characters.setLen(0)
    world.characterBodyIds.setLen(0)

    for character in world.rigidCharacters:
      raw.delete(character)
    world.rigidCharacters.setLen(0)
    world.rigidCharacterBodyIds.setLen(0)

    if not world.eventBridge.isNil:
      world.physics.deleteEventBridge(world.eventBridge)
      world.eventBridge = nil

    for constraint in world.constraints:
      world.physics.removeConstraint(constraint)
    world.constraints.setLen(0)

    let bodies = world.physics.bodyInterface()
    for id in world.bodyIds:
      bodies.removeAndDestroyBody(raw.bodyID(id))
    world.bodyIds.setLen(0)

    raw.delete(world.physics)
    world.physics = nil

  if not world.characterBroadPhase.isNil:
    raw.delete(world.characterBroadPhase)
    world.characterBroadPhase = nil

  if not world.objectVsBroadPhase.isNil:
    raw.delete(world.objectVsBroadPhase)
    world.objectVsBroadPhase = nil
  if not world.broadPhases.isNil:
    raw.delete(world.broadPhases)
    world.broadPhases = nil
  if not world.objectPairs.isNil:
    raw.delete(world.objectPairs)
    world.objectPairs = nil
  if not world.jobs.isNil:
    raw.delete(world.jobs)
    world.jobs = nil
  if not world.allocator.isNil:
    raw.delete(world.allocator)
    world.allocator = nil

  if world.acquiredJolt:
    raw.releaseJolt()
    world.acquiredJolt = false

  world.closed = true
  world.closing = false

proc newWorld*(config: WorldConfig): World =
  config.validate()
  new(result)

  if not raw.acquireJolt():
    raise newException(JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true

  let objectLayerCount = uint(config.collisionLayers.len)
  var broadPhaseLayerCount = 0'u
  for layer in config.collisionLayers:
    broadPhaseLayerCount = max(broadPhaseLayerCount, uint(layer.broadPhaseLayer) + 1'u)

  result.layerCount = uint32(config.collisionLayers.len)
  result.objectPairs = raw.newObjectLayerPairFilterTable(objectLayerCount)
  for pair in config.collisionPairs:
    result.objectPairs.enableCollision(pair.layer1, pair.layer2)

  result.broadPhases = raw.newBroadPhaseLayerInterfaceTable(
    objectLayerCount,
    broadPhaseLayerCount
  )
  for index, layer in config.collisionLayers:
    result.broadPhases.mapObjectToBroadPhaseLayer(
      CollisionLayer(index),
      raw.broadPhaseLayer(layer.broadPhaseLayer)
    )

  result.objectVsBroadPhase = raw.newObjectVsBroadPhaseLayerFilterTable(
    result.broadPhases,
    broadPhaseLayerCount,
    result.objectPairs,
    objectLayerCount
  )

  result.physics = raw.newPhysicsSystem()
  result.physics.init(
    config.maxBodies,
    config.numBodyMutexes,
    config.maxBodyPairs,
    config.maxContactConstraints,
    result.broadPhases,
    result.objectVsBroadPhase,
    result.objectPairs
  )
  result.characterBroadPhase = raw.newCharacterBroadPhase(
    config.characterBroadPhaseCellSize)
  let policyCount = config.contactPolicies.len
  var policyLayers1 = newSeq[raw.ObjectLayer](policyCount)
  var policyLayers2 = newSeq[raw.ObjectLayer](policyCount)
  var policyResponses = newSeq[uint8](policyCount)
  var policyFrictions = newSeq[cfloat](policyCount)
  var policyRestitutions = newSeq[cfloat](policyCount)
  var policyMassScales1 = newSeq[cfloat](policyCount)
  var policyInertiaScales1 = newSeq[cfloat](policyCount)
  var policyMassScales2 = newSeq[cfloat](policyCount)
  var policyInertiaScales2 = newSeq[cfloat](policyCount)
  var policyLinearVelocities = newSeq[raw.Vec3](policyCount)
  var policyAngularVelocities = newSeq[raw.Vec3](policyCount)
  for index, policy in config.contactPolicies:
    policyLayers1[index] = policy.layer1
    policyLayers2[index] = policy.layer2
    policyResponses[index] = uint8(ord(policy.response))
    policyFrictions[index] =
      if policy.friction.isSome: policy.friction.get else: -1
    policyRestitutions[index] =
      if policy.restitution.isSome: policy.restitution.get else: -1
    policyMassScales1[index] = policy.inverseMassScale1
    policyInertiaScales1[index] = policy.inverseInertiaScale1
    policyMassScales2[index] = policy.inverseMassScale2
    policyInertiaScales2[index] = policy.inverseInertiaScale2
    policyLinearVelocities[index] = policy.linearSurfaceVelocity.toRaw
    policyAngularVelocities[index] = policy.angularSurfaceVelocity.toRaw
  var policyLayers1Ptr, policyLayers2Ptr: ptr raw.ObjectLayer
  var policyResponsesPtr: ptr uint8
  var policyFrictionsPtr, policyRestitutionsPtr,
    policyMassScales1Ptr, policyInertiaScales1Ptr,
    policyMassScales2Ptr, policyInertiaScales2Ptr: ptr cfloat
  var policyLinearVelocitiesPtr, policyAngularVelocitiesPtr: ptr raw.Vec3
  if policyCount > 0:
    policyLayers1Ptr = addr policyLayers1[0]
    policyLayers2Ptr = addr policyLayers2[0]
    policyResponsesPtr = addr policyResponses[0]
    policyFrictionsPtr = addr policyFrictions[0]
    policyRestitutionsPtr = addr policyRestitutions[0]
    policyMassScales1Ptr = addr policyMassScales1[0]
    policyInertiaScales1Ptr = addr policyInertiaScales1[0]
    policyMassScales2Ptr = addr policyMassScales2[0]
    policyInertiaScales2Ptr = addr policyInertiaScales2[0]
    policyLinearVelocitiesPtr = addr policyLinearVelocities[0]
    policyAngularVelocitiesPtr = addr policyAngularVelocities[0]
  result.eventBridge = result.physics.newEventBridge(
    uint32(config.maxQueuedEvents),
    policyLayers1Ptr, policyLayers2Ptr, policyResponsesPtr,
    policyFrictionsPtr, policyRestitutionsPtr,
    policyMassScales1Ptr, policyInertiaScales1Ptr,
    policyMassScales2Ptr, policyInertiaScales2Ptr,
    policyLinearVelocitiesPtr, policyAngularVelocitiesPtr,
    uint32(policyCount))

  result.allocator = raw.newTempAllocator(csize_t(config.tempAllocatorBytes))
  result.jobs = raw.newJobSystemThreadPool(
    config.maxJobs,
    config.maxBarriers,
    cint(config.numThreads)
  )

proc newWorld*(): World =
  newWorld(defaultWorldConfig())

proc simulationSettings*(world: World): SimulationSettings =
  ## Returns a detached snapshot; changing it does not affect the world until
  ## `setSimulationSettings` is called.
  world.requireOpen()
  let settings = world.physics.GetPhysicsSettings()
  if settings.isNil:
    raise newException(JoltError, "Jolt returned no simulation settings")
  result = settings[].fromRaw

proc setSimulationSettings*(world: World; settings: SimulationSettings) =
  ## Validates and atomically replaces Jolt's complete PhysicsSettings value.
  world.requireOpen()
  settings.validate()
  world.physics.SetPhysicsSettings(settings.toRaw)

proc newWorld*(config: WorldConfig; settings: SimulationSettings): World =
  settings.validate()
  result = newWorld(config)
  try:
    result.setSimulationSettings(settings)
  except:
    if not result.isNil:
      closeWorld(result[])
    raise

proc newWorld*(settings: SimulationSettings): World =
  newWorld(defaultWorldConfig(), settings)

proc close*(world: World) =
  if not world.isNil:
    closeWorld(world[])

proc close*(state: WorldState) =
  if not state.isNil:
    closeWorldState(state[])

proc close*(body: Body) =
  if not body.isNil:
    if body.constraintCount > 0 and not body.owner.isNil and
        not body.owner.closed and not body.owner.closing:
      raise newException(JoltError, "close the body's constraints before closing the body")
    closeBody(body[])

proc close*(body: SoftBody) =
  if not body.isNil:
    closeSoftBody(body[])

proc close*(constraint: Constraint) =
  if not constraint.isNil:
    closeConstraint(constraint[])

proc close*(character: Character) =
  if not character.isNil:
    closeCharacter(character[])

proc close*(character: RigidCharacter) =
  if not character.isNil:
    closeRigidCharacter(character[])

proc close*(vehicle: Vehicle) =
  if not vehicle.isNil:
    closeVehicle(vehicle[])

proc close*(vehicle: TrackedVehicle) =
  if not vehicle.isNil:
    closeTrackedVehicle(vehicle[])

proc close*(ragdoll: Ragdoll) =
  if not ragdoll.isNil:
    closeRagdoll(ragdoll[])

proc close*(mapper: SkeletonMapper) =
  if not mapper.isNil:
    closeSkeletonMapper(mapper[])

proc close*(animation: SkeletalAnimation) =
  if not animation.isNil:
    closeSkeletalAnimation(animation[])

proc close*(scene: PhysicsScene) =
  if not scene.isNil:
    closePhysicsScene(scene[])

proc close*(instance: PhysicsSceneInstance) =
  if not instance.isNil:
    closePhysicsSceneInstance(instance[])

proc close*(filter: CollisionGroupFilter) =
  if not filter.isNil:
    closeCollisionGroupFilter(filter[])

proc isOpen*(world: World): bool =
  not world.isNil and not world.closed and not world.closing

proc characterBroadPhaseStats*(world: World): CharacterBroadPhaseStats =
  ## Returns cumulative CharacterVirtual broad-phase work since creation or
  ## the most recent resetCharacterBroadPhaseStats call.
  world.requireOpen()
  world.characterBroadPhase.stats(
    addr result.registeredCharacters,
    addr result.occupiedCells,
    addr result.queryCount,
    addr result.candidateCount,
    addr result.narrowPhaseTestCount)

proc resetCharacterBroadPhaseStats*(world: World) =
  world.requireOpen()
  world.characterBroadPhase.resetStats()

proc checkedContactBodyId[T: Body | SoftBody](world: World; body: T): uint32 =
  world.requireOpen()
  if body.isNil or not body.alive:
    raise newException(JoltError, "contact-policy body is no longer alive")
  if body.owner != world:
    raise newException(
      JoltError, "contact-policy bodies must belong to the target world")
  body.rawId

proc setBodyPairContactPolicy*[A: Body | SoftBody; B: Body | SoftBody](
    world: World; body1: A; body2: B; policy: BodyPairContactPolicy) =
  ## Installs or replaces a worker-safe policy for one exact body pair.
  ## Call between simulation steps. Exact policies override layer policies.
  let id1 = world.checkedContactBodyId(body1)
  let id2 = world.checkedContactBodyId(body2)
  if id1 == id2:
    raise newException(ValueError, "a body-pair contact policy needs two bodies")
  policy.validate()
  world.physics.setBodyPairContactPolicy(
    world.eventBridge,
    id1, id2, uint8(ord(policy.response)),
    if policy.friction.isSome: policy.friction.get else: -1,
    if policy.restitution.isSome: policy.restitution.get else: -1,
    policy.inverseMassScale1, policy.inverseInertiaScale1,
    policy.inverseMassScale2, policy.inverseInertiaScale2,
    policy.linearSurfaceVelocity.toRaw,
    policy.angularSurfaceVelocity.toRaw)

proc removeBodyPairContactPolicy*[A: Body | SoftBody; B: Body | SoftBody](
    world: World; body1: A; body2: B): bool =
  ## Removes an exact policy and restores layer/default behavior next step.
  let id1 = world.checkedContactBodyId(body1)
  let id2 = world.checkedContactBodyId(body2)
  if id1 == id2:
    return false
  world.physics.removeBodyPairContactPolicy(world.eventBridge, id1, id2)

proc hasBodyPairContactPolicy*[A: Body | SoftBody; B: Body | SoftBody](
    world: World; body1: A; body2: B): bool =
  let id1 = world.checkedContactBodyId(body1)
  let id2 = world.checkedContactBodyId(body2)
  id1 != id2 and world.eventBridge.hasBodyPairContactPolicy(id1, id2)

proc bodyPairContactPolicyCount*(world: World): uint32 =
  world.requireOpen()
  world.eventBridge.bodyPairContactPolicyCount()

proc setSubShapePairContactPolicy*(world: World;
                                   body1: Body; subShapeId1: uint32;
                                   body2: Body; subShapeId2: uint32;
                                   policy: BodyPairContactPolicy) =
  ## Installs or replaces a rigid contact rule for one exact sub-shape pair.
  ## It overrides both exact-body and layer rules. For coplanar compound
  ## children, disable manifold reduction when preserving child identity in
  ## ContactSettings is required.
  let id1 = world.checkedContactBodyId(body1)
  let id2 = world.checkedContactBodyId(body2)
  if id1 == id2:
    raise newException(
      ValueError, "a sub-shape contact policy needs two bodies")
  policy.validate()
  world.physics.setSubShapePairContactPolicy(
    world.eventBridge,
    id1, subShapeId1, id2, subShapeId2,
    uint8(ord(policy.response)),
    if policy.friction.isSome: policy.friction.get else: -1,
    if policy.restitution.isSome: policy.restitution.get else: -1,
    policy.inverseMassScale1, policy.inverseInertiaScale1,
    policy.inverseMassScale2, policy.inverseInertiaScale2,
    policy.linearSurfaceVelocity.toRaw,
    policy.angularSurfaceVelocity.toRaw)

proc removeSubShapePairContactPolicy*(world: World;
                                      body1: Body; subShapeId1: uint32;
                                      body2: Body; subShapeId2: uint32): bool =
  let id1 = world.checkedContactBodyId(body1)
  let id2 = world.checkedContactBodyId(body2)
  if id1 == id2:
    return false
  world.physics.removeSubShapePairContactPolicy(
    world.eventBridge, id1, subShapeId1, id2, subShapeId2)

proc hasSubShapePairContactPolicy*(world: World;
                                   body1: Body; subShapeId1: uint32;
                                   body2: Body; subShapeId2: uint32): bool =
  let id1 = world.checkedContactBodyId(body1)
  let id2 = world.checkedContactBodyId(body2)
  id1 != id2 and world.eventBridge.hasSubShapePairContactPolicy(
    id1, subShapeId1, id2, subShapeId2)

proc subShapePairContactPolicyCount*(world: World): uint32 =
  world.requireOpen()
  world.eventBridge.subShapePairContactPolicyCount()

proc isAlive*(state: WorldState): bool =
  not state.isNil and state.alive and not state.native.isNil

proc nativeCharacters(world: World): ptr ptr raw.CharacterHandle =
  if world.characters.len == 0:
    nil
  else:
    unsafeAddr world.characters[0]

proc saveState*(world: World): WorldState =
  ## Captures simulation state for deterministic rollback in this world.
  ## Shapes, friction and other configuration are not copied by Jolt and must
  ## remain unchanged until restoration.
  world.requireOpen()
  new(result)
  result.owner = world
  result.bodyIds = world.bodyIds
  result.constraints = world.constraints
  result.characters = world.characters
  result.rigidCharacters = world.rigidCharacters
  result.vehicles = world.vehicles
  result.ragdolls = world.ragdolls
  result.native = world.physics.saveWorldState(
    world.nativeCharacters,
    uint32(world.characters.len))
  if result.native.isNil:
    raise newException(JoltError, "Jolt could not save the world state")
  result.alive = true

proc byteSize*(state: WorldState): int =
  if not state.isAlive:
    raise newException(JoltError, "Jolt world state is closed")
  int(state.native.byteSize)

proc restoreState*(world: World; state: WorldState) =
  ## Restores a snapshot while preserving all existing Nim object handles.
  world.requireOpen()
  if not state.isAlive:
    raise newException(JoltError, "Jolt world state is closed")
  if state.owner != world:
    raise newException(ValueError, "world state belongs to a different world")
  if state.bodyIds != world.bodyIds or
      state.constraints != world.constraints or
      state.characters != world.characters or
      state.rigidCharacters != world.rigidCharacters or
      state.vehicles != world.vehicles or
      state.ragdolls != world.ragdolls:
    raise newException(
      ValueError, "world topology changed after the state was saved")
  if not world.physics.restoreWorldState(
      state.native,
      world.nativeCharacters,
      uint32(world.characters.len),
      world.eventBridge):
    raise newException(JoltError, "Jolt could not restore the world state")
  for character in world.rigidCharacters:
    character.postSimulation()

proc isAlive*(body: Body): bool =
  not body.isNil and body.alive and body.owner.isOpen

proc isAlive*(body: SoftBody): bool =
  not body.isNil and body.alive and body.owner.isOpen

proc isAlive*(constraint: Constraint): bool =
  not constraint.isNil and constraint.alive and constraint.owner.isOpen

proc isAlive*(character: Character): bool =
  not character.isNil and character.alive and character.owner.isOpen

proc isAlive*(character: RigidCharacter): bool =
  not character.isNil and character.alive and character.owner.isOpen

proc isAlive*(vehicle: Vehicle): bool =
  not vehicle.isNil and vehicle.alive and vehicle.owner.isOpen and
    vehicle.chassisBody.isAlive

proc isAlive*(vehicle: TrackedVehicle): bool =
  not vehicle.isNil and vehicle.alive and vehicle.owner.isOpen and
    vehicle.chassisBody.isAlive

proc isAlive*(filter: CollisionGroupFilter): bool =
  not filter.isNil and filter.alive and not filter.native.isNil

proc newCollisionGroupFilter*(subgroupCount: SomeInteger): CollisionGroupFilter =
  if subgroupCount < 2 or uint64(subgroupCount) > uint64(high(uint32)):
    raise newException(
      ValueError, "collision group filter requires 2..uint32.high subgroups")
  new(result)
  result.subgroupCount = uint32(subgroupCount)
  result.native = raw.newGroupFilterTable(result.subgroupCount)
  if result.native.isNil:
    raise newException(JoltError, "Jolt could not create the collision group filter")
  result.alive = true

proc subgroupCount*(filter: CollisionGroupFilter): uint32 =
  if not filter.isAlive:
    raise newException(JoltError, "Jolt collision group filter is closed")
  filter.subgroupCount

proc validateSubgroup(filter: CollisionGroupFilter; subgroup: uint32) =
  if not filter.isAlive:
    raise newException(JoltError, "Jolt collision group filter is closed")
  if subgroup >= filter.subgroupCount:
    raise newException(IndexDefect, "collision subgroup index is out of bounds")

proc setCollisionEnabled*(filter: CollisionGroupFilter;
                          subgroup1, subgroup2: uint32; enabled: bool) =
  filter.validateSubgroup(subgroup1)
  filter.validateSubgroup(subgroup2)
  if subgroup1 == subgroup2:
    if enabled:
      raise newException(
        ValueError, "a collision subgroup never collides with itself")
    return
  if enabled:
    filter.native.enableCollision(subgroup1, subgroup2)
  else:
    filter.native.disableCollision(subgroup1, subgroup2)

proc collisionEnabled*(filter: CollisionGroupFilter;
                       subgroup1, subgroup2: uint32): bool =
  filter.validateSubgroup(subgroup1)
  filter.validateSubgroup(subgroup2)
  subgroup1 != subgroup2 and
    filter.native.isCollisionEnabled(subgroup1, subgroup2)

proc bodyCollisionGroup*(filter: CollisionGroupFilter;
                         groupId, subgroupId: uint32): BodyCollisionGroup =
  filter.validateSubgroup(subgroupId)
  BodyCollisionGroup(
    filter: filter, groupId: groupId, subgroupId: subgroupId)

type CookedShape = object
  native: ptr raw.Shape
  ownsNativeReference: bool

proc cookMaterial(material: PhysicsMaterial): ptr raw.PhysicsMaterial =
  discard physicsMaterial(material.name, material.debugColor)
  result = raw.newPhysicsMaterial(
    material.name.cstring,
    material.debugColor.r,
    material.debugColor.g,
    material.debugColor.b,
    material.debugColor.a)
  if result.isNil:
    raise newException(JoltError, "Jolt could not create a physics material")

proc releaseMaterials(materials: openArray[ptr raw.PhysicsMaterial]) =
  for material in materials:
    if not material.isNil:
      material.release()

proc cookMaterials(
    materials: openArray[PhysicsMaterial]): seq[ptr raw.PhysicsMaterial] =
  result = newSeq[ptr raw.PhysicsMaterial](materials.len)
  try:
    for index, material in materials:
      result[index] = cookMaterial(material)
  except:
    result.releaseMaterials()
    raise

proc release(shape: CookedShape) =
  if shape.ownsNativeReference and not shape.native.isNil:
    shape.native.release()

proc cookShape(shape: Shape; motionType: MotionType): CookedShape =
  var nativeMaterial: ptr raw.PhysicsMaterial
  if shape.material.isSome:
    nativeMaterial = cookMaterial(shape.material.get)
  defer:
    if not nativeMaterial.isNil:
      nativeMaterial.release()
  case shape.kind
  of ShapeKind.Box:
    discard boxShape(shape.halfExtent, shape.convexRadius)
    result.native = raw.newBoxShape(
      shape.halfExtent.toRaw, shape.convexRadius, nativeMaterial)
  of ShapeKind.Sphere:
    discard sphereShape(shape.radius)
    result.native = raw.newSphereShape(shape.radius, nativeMaterial)
  of ShapeKind.Capsule:
    discard capsuleShape(shape.halfHeight, shape.radius)
    result.native = raw.newCapsuleShape(
      shape.halfHeight, shape.radius, nativeMaterial)
  of ShapeKind.Cylinder:
    discard cylinderShape(shape.halfHeight, shape.radius, shape.convexRadius)
    result.native = raw.newCylinderShape(
      shape.halfHeight, shape.radius, shape.convexRadius, nativeMaterial)
  of ShapeKind.TaperedCapsule:
    discard taperedCapsuleShape(
      shape.halfHeight, shape.topRadius, shape.bottomRadius)
    result.native = raw.newTaperedCapsuleShape(
      shape.halfHeight, shape.topRadius, shape.bottomRadius, nativeMaterial)
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the tapered capsule")
    result.ownsNativeReference = true
  of ShapeKind.TaperedCylinder:
    discard taperedCylinderShape(
      shape.halfHeight, shape.topRadius, shape.bottomRadius,
      shape.convexRadius)
    result.native = raw.newTaperedCylinderShape(
      shape.halfHeight, shape.topRadius, shape.bottomRadius,
      shape.convexRadius, nativeMaterial)
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the tapered cylinder")
    result.ownsNativeReference = true
  of ShapeKind.Triangle:
    if shape.points.len != 3:
      raise newException(ValueError, "triangle shape must contain three vertices")
    discard triangleShape(
      shape.points[0], shape.points[1], shape.points[2], shape.convexRadius)
    result.native = raw.newTriangleShape(
      shape.points[0].toRaw,
      shape.points[1].toRaw,
      shape.points[2].toRaw,
      shape.convexRadius,
      nativeMaterial)
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the triangle")
    result.ownsNativeReference = true
  of ShapeKind.Plane:
    if motionType == MotionType.Dynamic:
      raise newException(ValueError, "plane bodies cannot be dynamic")
    let validated = planeShape(
      shape.planeNormal, shape.planeConstant, shape.planeHalfExtent)
    result.native = raw.newPlaneShape(
      validated.planeNormal.toRaw,
      validated.planeConstant,
      validated.planeHalfExtent,
      nativeMaterial)
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the plane")
    result.ownsNativeReference = true
  of ShapeKind.Empty:
    if motionType == MotionType.Dynamic:
      raise newException(ValueError, "empty shape bodies cannot be dynamic")
    let validated = emptyShape(shape.centerOfMass)
    result.native = raw.newEmptyShape(validated.centerOfMass.toRaw)
    result.ownsNativeReference = true
  of ShapeKind.ConvexHull:
    discard convexHullShape(shape.points, shape.convexRadius)
    var nativePoints = newSeq[raw.Vec3](shape.points.len)
    for index, point in shape.points:
      nativePoints[index] = point.toRaw
    result.native = raw.newConvexHullShape(
      addr nativePoints[0], uint32(nativePoints.len), shape.convexRadius,
      nativeMaterial)
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the convex hull")
    result.ownsNativeReference = true
  of ShapeKind.TriangleMesh:
    if motionType != MotionType.Static:
      raise newException(ValueError, "triangle mesh bodies must be static")
    discard triangleMeshShape(shape.vertices, shape.triangleIndices)
    var nativeVertices = newSeq[raw.Vec3](shape.vertices.len)
    for index, vertex in shape.vertices:
      nativeVertices[index] = vertex.toRaw
    var nativeMaterials = newSeq[ptr raw.PhysicsMaterial](shape.materials.len)
    defer:
      for material in nativeMaterials:
        if not material.isNil:
          material.release()
    for index, material in shape.materials:
      nativeMaterials[index] = cookMaterial(material)
    let materialsPointer = if nativeMaterials.len == 0:
        nil
      else:
        addr nativeMaterials[0]
    let materialIndicesPointer = if shape.materialIndices.len == 0:
        nil
      else:
        unsafeAddr shape.materialIndices[0]
    result.native = raw.newTriangleMeshShape(
      addr nativeVertices[0],
      uint32(nativeVertices.len),
      unsafeAddr shape.triangleIndices[0],
      uint32(shape.triangleIndices.len div 3),
      materialsPointer,
      uint32(nativeMaterials.len),
      materialIndicesPointer)
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the triangle mesh")
    result.ownsNativeReference = true
  of ShapeKind.HeightField:
    if motionType != MotionType.Static:
      raise newException(ValueError, "height field bodies must be static")
    discard heightFieldShape(
      shape.heightSamples,
      int(shape.sampleCount),
      shape.heightOffset,
      shape.heightScale,
      shape.blockSize,
      shape.bitsPerSample)
    var nativeMaterials = newSeq[ptr raw.PhysicsMaterial](shape.materials.len)
    defer:
      for material in nativeMaterials:
        if not material.isNil:
          material.release()
    for index, material in shape.materials:
      nativeMaterials[index] = cookMaterial(material)
    var nativeMaterialIndices = newSeq[uint8](shape.materialIndices.len)
    for index, materialIndex in shape.materialIndices:
      nativeMaterialIndices[index] = uint8(materialIndex)
    let materialsPointer = if nativeMaterials.len == 0:
        nil
      else:
        addr nativeMaterials[0]
    let materialIndicesPointer = if nativeMaterialIndices.len == 0:
        nil
      else:
        addr nativeMaterialIndices[0]
    result.native = raw.newHeightFieldShape(
      unsafeAddr shape.heightSamples[0],
      shape.sampleCount,
      shape.heightOffset.toRaw,
      shape.heightScale.toRaw,
      shape.blockSize,
      shape.bitsPerSample,
      materialIndicesPointer,
      materialsPointer,
      uint32(nativeMaterials.len))
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the height field")
    result.ownsNativeReference = true
  of ShapeKind.StaticCompound:
    let validated = staticCompoundShape(shape.children)
    var cookedChildren = newSeq[CookedShape](validated.children.len)
    defer:
      for child in cookedChildren:
        child.release()
    var nativeShapes = newSeq[ptr raw.Shape](validated.children.len)
    var nativePositions = newSeq[raw.Vec3](validated.children.len)
    var nativeRotations = newSeq[raw.Quat](validated.children.len)
    for index, child in validated.children:
      cookedChildren[index] = cookShape(child.shape, motionType)
      nativeShapes[index] = cookedChildren[index].native
      nativePositions[index] = child.position.toRaw
      nativeRotations[index] = child.rotation.toRaw
    result.native = raw.newStaticCompoundShape(
      addr nativeShapes[0],
      addr nativePositions[0],
      addr nativeRotations[0],
      uint32(validated.children.len))
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the compound shape")
    result.ownsNativeReference = true
  of ShapeKind.MutableCompound:
    let validated = mutableCompoundShape(shape.children)
    var cookedChildren = newSeq[CookedShape](validated.children.len)
    defer:
      for child in cookedChildren:
        child.release()
    var nativeShapes = newSeq[ptr raw.Shape](validated.children.len)
    var nativePositions = newSeq[raw.Vec3](validated.children.len)
    var nativeRotations = newSeq[raw.Quat](validated.children.len)
    for index, child in validated.children:
      cookedChildren[index] = cookShape(child.shape, motionType)
      nativeShapes[index] = cookedChildren[index].native
      nativePositions[index] = child.position.toRaw
      nativeRotations[index] = child.rotation.toRaw
    result.native = raw.newMutableCompoundShape(
      addr nativeShapes[0],
      addr nativePositions[0],
      addr nativeRotations[0],
      uint32(validated.children.len))
    if result.native.isNil:
      raise newException(JoltError, "Jolt could not build the mutable compound shape")
    result.ownsNativeReference = true
  of ShapeKind.Scaled:
    let validated = scaledShape(shape.innerShape, shape.shapeScale)
    let inner = cookShape(validated.innerShape, motionType)
    defer: inner.release()
    result.native = raw.newScaledShape(inner.native, validated.shapeScale.toRaw)
    if result.native.isNil:
      raise newException(JoltError, "Jolt rejected the decorated shape scale")
    result.ownsNativeReference = true
  of ShapeKind.RotatedTranslated:
    let validated = rotatedTranslatedShape(
      shape.innerShape, shape.shapePosition, shape.shapeRotation)
    let inner = cookShape(validated.innerShape, motionType)
    defer: inner.release()
    result.native = raw.newRotatedTranslatedShape(
      inner.native,
      validated.shapePosition.toRaw,
      validated.shapeRotation.toRaw)
    if result.native.isNil:
      raise newException(JoltError, "Jolt rejected the decorated shape transform")
    result.ownsNativeReference = true
  of ShapeKind.OffsetCenterOfMass:
    let validated = offsetCenterOfMassShape(
      shape.innerShape, shape.centerOfMassOffset)
    let inner = cookShape(validated.innerShape, motionType)
    defer: inner.release()
    result.native = raw.newOffsetCenterOfMassShape(
      inner.native, validated.centerOfMassOffset.toRaw)
    if result.native.isNil:
      raise newException(JoltError, "Jolt rejected the center-of-mass offset")
    result.ownsNativeReference = true

proc configureBodySettings(settings: var raw.BodyCreationSettings;
                           config: BodyConfig) =
  let hasCustomMassProperties = config.massProperties.isSome
  let customMass =
    if hasCustomMassProperties: config.massProperties.get.mass
    else: 0.0'f32
  let customInertiaDiagonal =
    if hasCustomMassProperties: config.massProperties.get.inertiaDiagonal
    else: vec3(1, 1, 1)
  let customInertiaRotation =
    if hasCustomMassProperties:
      config.massProperties.get.inertiaRotation.normalized
    else:
      quatIdentity()
  settings.configure(
    allowedDOFMask(config.allowedDOFs),
    uint8(ord(config.motionQuality)),
    config.mass,
    config.inertiaMultiplier,
    config.linearVelocity.toRaw,
    config.angularVelocity.toRaw,
    config.userData,
    config.allowSleeping,
    config.collideKinematicVsNonDynamic,
    config.useManifoldReduction,
    config.applyGyroscopicForce,
    config.enhancedInternalEdgeRemoval,
    config.friction,
    config.restitution,
    config.linearDamping,
    config.angularDamping,
    config.maxLinearVelocity,
    config.maxAngularVelocity,
    config.gravityFactor,
    config.numVelocityStepsOverride,
    config.numPositionStepsOverride,
    hasCustomMassProperties,
    customMass,
    customInertiaDiagonal.toRaw,
    customInertiaRotation.toRaw)

proc addBody*(world: World; shape: Shape; position: Vec3;
              motionType: MotionType; objectLayer: CollisionLayer;
              activate = true; rotation = quatIdentity();
              sensor = false; config = defaultBodyConfig()): Body =
  world.requireOpen()
  world.requireLayer(objectLayer)
  position.requireFinite("position")
  config.validate(motionType)
  let nativeRotation = rotation.normalized.toRaw
  let cookedShape = cookShape(shape, motionType)
  defer: cookedShape.release()

  var settings = raw.bodyCreationSettings(
    cookedShape.native,
    position.toRaw,
    nativeRotation,
    motionType.toRaw,
    objectLayer
  )
  settings.setSensor(sensor)
  settings.configureBodySettings(config)

  let activation = if activate: raw.EActivation.Activate else: raw.EActivation.DontActivate
  let id = world.physics.bodyInterface().createAndAddBody(settings, activation)
  if id.isInvalid:
    raise newException(JoltError, "Jolt could not allocate another body")

  new(result)
  result.owner = world
  result.rawId = id.value
  result.shapeDesc = shape
  result.motion = motionType
  result.sensor = sensor
  result.alive = true
  world.bodyIds.add(result.rawId)

proc addBodies*(world: World; specs: openArray[BodySpec];
                activate = true): seq[Body] =
  ## Creates all bodies first and inserts them through Jolt's native batch
  ## broad-phase path. If validation, cooking or allocation fails, no body from
  ## this call remains in the world.
  world.requireOpen()
  if specs.len == 0:
    return @[]
  if specs.len > int(high(cint)):
    raise newException(ValueError, "body batch is too large")

  var cookedShapes = newSeq[CookedShape](specs.len)
  defer:
    for cooked in cookedShapes:
      cooked.release()

  for index, spec in specs:
    world.requireLayer(spec.layer)
    spec.position.requireFinite("body position")
    spec.config.validate(spec.motionType)
    discard spec.rotation.normalized
    cookedShapes[index] = cookShape(spec.shape, spec.motionType)

  let bodies = world.physics.bodyInterface()
  var nativeIds = newSeq[raw.BodyID](specs.len)
  var createdCount = 0
  var added = false
  defer:
    if not added and createdCount > 0:
      bodies.destroyBodies(addr nativeIds[0], uint32(createdCount))

  result = newSeq[Body](specs.len)
  for index, spec in specs:
    var settings = raw.bodyCreationSettings(
      cookedShapes[index].native,
      spec.position.toRaw,
      spec.rotation.normalized.toRaw,
      spec.motionType.toRaw,
      spec.layer)
    settings.setSensor(spec.sensor)
    settings.configureBodySettings(spec.config)

    let nativeBody = bodies.createBody(settings)
    if nativeBody.isNil:
      raise newException(
        JoltError, "Jolt could not allocate every body in the batch")
    nativeIds[index] = nativeBody.id
    inc createdCount

    new(result[index])
    result[index].owner = world
    result[index].rawId = nativeIds[index].value
    result[index].shapeDesc = spec.shape
    result[index].motion = spec.motionType
    result[index].sensor = spec.sensor
    result[index].alive = true

  let activation =
    if activate: raw.EActivation.Activate
    else: raw.EActivation.DontActivate
  bodies.addBodies(addr nativeIds[0], uint32(nativeIds.len), activation)
  added = true
  for body in result:
    world.bodyIds.add(body.rawId)

proc closeBodies*(bodies: openArray[Body]) =
  ## Removes and destroys live bodies from one world in a single native batch.
  ## The entire input is validated before the world is changed.
  var liveBodies: seq[Body]
  var owner: World
  for body in bodies:
    if body.isNil or not body.alive:
      continue
    if owner.isNil:
      owner = body.owner
    elif body.owner != owner:
      raise newException(JoltError, "a body batch must belong to one world")
    if body.constraintCount > 0 and body.owner.isOpen:
      raise newException(
        JoltError, "close the bodies' constraints before closing the batch")
    for candidate in liveBodies:
      if candidate.rawId == body.rawId:
        raise newException(ValueError, "a body batch contains a duplicate body")
    liveBodies.add(body)

  if liveBodies.len == 0:
    return
  if not owner.isOpen:
    for body in liveBodies:
      body.alive = false
    return

  var nativeIds = newSeq[raw.BodyID](liveBodies.len)
  for index, body in liveBodies:
    owner.eventBridge.removeBodyContactPolicies(body.rawId)
    nativeIds[index] = raw.bodyID(body.rawId)
  owner.physics.bodyInterface().removeAndDestroyBodies(
    addr nativeIds[0], uint32(nativeIds.len))
  for body in liveBodies:
    owner.removeTrackedId(body.rawId)
    body.alive = false

proc addStaticBody*(world: World; shape: Shape; position: Vec3;
                    rotation = quatIdentity(); sensor = false;
                    layer = nonMovingLayer;
                    config = defaultBodyConfig()): Body =
  world.addBody(
    shape,
    position,
    MotionType.Static,
    layer,
    activate = false,
    rotation = rotation,
    sensor = sensor,
    config = config
  )

proc addDynamicBody*(world: World; shape: Shape; position: Vec3;
                     rotation = quatIdentity(); sensor = false;
                     layer = movingLayer;
                     config = defaultBodyConfig()): Body =
  world.addBody(
    shape,
    position,
    MotionType.Dynamic,
    layer,
    activate = true,
    rotation = rotation,
    sensor = sensor,
    config = config
  )

proc addKinematicBody*(world: World; shape: Shape; position: Vec3;
                       rotation = quatIdentity(); sensor = false;
                       layer = movingLayer;
                       config = defaultBodyConfig()): Body =
  world.addBody(
    shape,
    position,
    MotionType.Kinematic,
    layer,
    activate = true,
    rotation = rotation,
    sensor = sensor,
    config = config
  )

proc validateRagdollConstraintParts(part1, part2, partCount: int;
                                    name: string) =
  if part1 < 0 or part1 >= partCount or part2 < 0 or part2 >= partCount:
    raise newException(
      ValueError, name & " part index is out of bounds")
  if part1 == part2:
    raise newException(ValueError, name & " requires two distinct parts")

proc validateRagdollConstraintFrame(primaryAxis, normalAxis: Vec3;
                                    name: string) =
  primaryAxis.requireFinite(name & " primary axis")
  normalAxis.requireFinite(name & " normal axis")
  let primaryLengthSquared = primaryAxis.x * primaryAxis.x +
    primaryAxis.y * primaryAxis.y + primaryAxis.z * primaryAxis.z
  let normalLengthSquared = normalAxis.x * normalAxis.x +
    normalAxis.y * normalAxis.y + normalAxis.z * normalAxis.z
  if primaryLengthSquared <= 1.0e-12'f32 or
      normalLengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, name & " axes must be non-zero")
  let axisDot = primaryAxis.x * normalAxis.x +
    primaryAxis.y * normalAxis.y + primaryAxis.z * normalAxis.z
  if abs(axisDot) >
      1.0e-4'f32 * sqrt(primaryLengthSquared * normalLengthSquared):
    raise newException(ValueError, name & " axes must be perpendicular")

proc validateRagdoll(config: RagdollConfig; world: World) =
  if config.parts.len == 0:
    raise newException(ValueError, "ragdoll requires at least one part")
  if uint64(config.parts.len) > uint64(high(uint32)):
    raise newException(ValueError, "ragdoll has too many parts")
  for index, part in config.parts:
    if part.name.len == 0:
      raise newException(ValueError, "ragdoll part names must not be empty")
    for previous in 0 ..< index:
      if config.parts[previous].name == part.name:
        raise newException(ValueError, "ragdoll part names must be unique")
    world.requireLayer(part.layer)
    part.position.requireFinite("ragdoll part position")
    discard part.rotation.normalized
    part.body.validate(part.motionType)
    if index == 0:
      if part.joint.parent != -1:
        raise newException(ValueError, "the first ragdoll part must be the root")
    elif part.joint.parent < 0 or part.joint.parent >= index:
      raise newException(
        ValueError, "each ragdoll parent must precede its child")
    if index > 0:
      part.joint.position.requireFinite("ragdoll joint position")
      let joint = part.joint
      if joint.kind in {
          RagdollSwingTwist, RagdollHinge, RagdollCone,
          RagdollSlider, RagdollSixDOF}:
        let twist = if joint.kind == RagdollSixDOF:
          joint.sixDOF.axisX
        else:
          joint.twistAxis
        twist.requireFinite("ragdoll primary joint axis")
        let twistLengthSquared =
          twist.x * twist.x + twist.y * twist.y + twist.z * twist.z
        if twistLengthSquared <= 1.0e-12'f32:
          raise newException(ValueError, "ragdoll joint axes must be non-zero")
        if joint.kind in {
            RagdollSwingTwist, RagdollHinge,
            RagdollSlider, RagdollSixDOF}:
          let plane = if joint.kind == RagdollSixDOF:
            joint.sixDOF.axisY
          else:
            joint.planeAxis
          plane.requireFinite("ragdoll secondary joint axis")
          let planeLengthSquared =
            plane.x * plane.x + plane.y * plane.y + plane.z * plane.z
          if planeLengthSquared <= 1.0e-12'f32:
            raise newException(ValueError, "ragdoll joint axes must be non-zero")
          let axisDot =
            twist.x * plane.x + twist.y * plane.y + twist.z * plane.z
          if abs(axisDot) >
              1.0e-4'f32 * sqrt(twistLengthSquared * planeLengthSquared):
            raise newException(
              ValueError, "ragdoll joint axes must be perpendicular")
      if joint.kind in {RagdollSwingTwist, RagdollCone}:
        let coneAngles = if joint.kind == RagdollSwingTwist:
          [joint.normalHalfConeAngle, joint.planeHalfConeAngle]
        else:
          [joint.normalHalfConeAngle, joint.normalHalfConeAngle]
        for angle in coneAngles:
          if not angle.isFinite or angle < 0 or angle > PI.float32:
            raise newException(
              ValueError, "ragdoll half cone angles must be in [0, PI]")
      if joint.kind in {RagdollSwingTwist, RagdollHinge}:
        if not joint.twistMinAngle.isFinite or
            not joint.twistMaxAngle.isFinite or
            joint.twistMinAngle < -PI.float32 or
            joint.twistMaxAngle > PI.float32 or
            joint.twistMinAngle > joint.twistMaxAngle:
          raise newException(
            ValueError, "ragdoll angular limits must be ordered within [-PI, PI]")
      if joint.kind == RagdollSlider:
        if not joint.twistMinAngle.isFinite or
            not joint.twistMaxAngle.isFinite or
            joint.twistMinAngle > 0 or joint.twistMaxAngle < 0:
          raise newException(
            ValueError,
            "ragdoll slider limits must satisfy minimum <= 0 <= maximum")
      if joint.kind == RagdollSixDOF:
        joint.sixDOF.validateLimits()
        for axis in SixDOFAxis:
          let limit = joint.sixDOF.limits[axis]
          if limit.mode == SixDOFAxisMode.AxisLimited:
            if not limit.minimum.isFinite or not limit.maximum.isFinite or
                limit.minimum > limit.maximum:
              raise newException(
                ValueError, "ragdoll SixDOF limits must be finite and ordered")
            if axis >= SixDOFAxis.RotationX and
                (limit.minimum < -PI.float32 or
                 limit.maximum > PI.float32):
              raise newException(
                ValueError,
                "ragdoll SixDOF rotation limits must be within [-PI, PI]")
        for friction in [joint.linearFriction, joint.angularFriction]:
          friction.requireFinite("ragdoll SixDOF friction")
          if friction.x < 0 or friction.y < 0 or friction.z < 0:
            raise newException(
              ValueError, "ragdoll SixDOF friction must be non-negative")
      if not joint.maxFrictionTorque.isFinite or joint.maxFrictionTorque < 0:
        raise newException(
          ValueError, "ragdoll friction torque must be finite and non-negative")
      if not joint.motorFrequency.isFinite or joint.motorFrequency < 0 or
          not joint.motorDamping.isFinite or joint.motorDamping < 0 or
          not joint.maxMotorTorque.isFinite or joint.maxMotorTorque < 0:
        raise newException(
          ValueError, "ragdoll motor settings must be finite and non-negative")
  for constraint in config.distanceConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll distance constraint")
    constraint.point1.requireFinite("ragdoll distance constraint point1")
    constraint.point2.requireFinite("ragdoll distance constraint point2")
    if not constraint.minDistance.isFinite or
        not constraint.maxDistance.isFinite or
        constraint.minDistance < 0 or
        constraint.minDistance > constraint.maxDistance:
      raise newException(
        ValueError, "ragdoll distances must be finite, non-negative and ordered")
  for constraint in config.pointConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll point constraint")
    constraint.point.requireFinite("ragdoll point constraint point")
  for constraint in config.fixedConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll fixed constraint")
  for constraint in config.hingeConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll hinge constraint")
    constraint.point.requireFinite("ragdoll hinge constraint point")
    validateRagdollConstraintFrame(
      constraint.hingeAxis, constraint.normalAxis,
      "ragdoll hinge constraint")
    if not constraint.minAngle.isFinite or
        not constraint.maxAngle.isFinite or
        constraint.minAngle < -PI.float32 or constraint.minAngle > 0 or
        constraint.maxAngle < 0 or constraint.maxAngle > PI.float32:
      raise newException(
        ValueError,
        "ragdoll hinge limits must satisfy -PI <= minimum <= 0 <= maximum <= PI")
    if not constraint.maxFrictionTorque.isFinite or
        constraint.maxFrictionTorque < 0:
      raise newException(
        ValueError,
        "ragdoll hinge friction torque must be finite and non-negative")
    if constraint.motor.isSome:
      let motor = constraint.motor.get
      motor.settings.validate()
      if not motor.targetVelocity.isFinite or
          not motor.targetPosition.isFinite:
        raise newException(
          ValueError, "ragdoll hinge motor targets must be finite")
  for constraint in config.sliderConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll slider constraint")
    constraint.point.requireFinite("ragdoll slider constraint point")
    validateRagdollConstraintFrame(
      constraint.sliderAxis, constraint.normalAxis,
      "ragdoll slider constraint")
    if not constraint.minPosition.isFinite or
        not constraint.maxPosition.isFinite or
        constraint.minPosition > 0 or constraint.maxPosition < 0:
      raise newException(
        ValueError,
        "ragdoll slider limits must satisfy minimum <= 0 <= maximum")
    if not constraint.maxFrictionForce.isFinite or
        constraint.maxFrictionForce < 0:
      raise newException(
        ValueError,
        "ragdoll slider friction force must be finite and non-negative")
    if constraint.motor.isSome:
      let motor = constraint.motor.get
      motor.settings.validate()
      if not motor.targetVelocity.isFinite or
          not motor.targetPosition.isFinite:
        raise newException(
          ValueError, "ragdoll slider motor targets must be finite")
  for constraint in config.swingTwistConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll swing-twist constraint")
    constraint.point.requireFinite("ragdoll swing-twist constraint point")
    validateRagdollConstraintFrame(
      constraint.twistAxis, constraint.planeAxis,
      "ragdoll swing-twist constraint")
    for angle in [
        constraint.normalHalfConeAngle, constraint.planeHalfConeAngle]:
      if not angle.isFinite or angle < 0 or angle > PI.float32:
        raise newException(
          ValueError,
          "ragdoll swing-twist half cone angles must be within [0, PI]")
    if not constraint.twistMinAngle.isFinite or
        not constraint.twistMaxAngle.isFinite or
        constraint.twistMinAngle < -PI.float32 or
        constraint.twistMaxAngle > PI.float32 or
        constraint.twistMinAngle > constraint.twistMaxAngle:
      raise newException(
        ValueError,
        "ragdoll swing-twist limits must be ordered within [-PI, PI]")
    if not constraint.maxFrictionTorque.isFinite or
        constraint.maxFrictionTorque < 0:
      raise newException(
        ValueError,
        "ragdoll swing-twist friction must be finite and non-negative")
    if constraint.motor.isSome:
      let motor = constraint.motor.get
      motor.swingSettings.validate()
      motor.twistSettings.validate()
      motor.targetAngularVelocity.requireFinite(
        "ragdoll swing-twist motor target angular velocity")
      discard motor.targetOrientation.normalized
  for constraint in config.sixDOFConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll SixDOF constraint")
    constraint.point.requireFinite("ragdoll SixDOF constraint point")
    validateRagdollConstraintFrame(
      constraint.config.axisX, constraint.config.axisY,
      "ragdoll SixDOF constraint")
    constraint.config.validateLimits()
    for friction in [constraint.linearFriction, constraint.angularFriction]:
      friction.requireFinite("ragdoll SixDOF constraint friction")
      if friction.x < 0 or friction.y < 0 or friction.z < 0:
        raise newException(
          ValueError,
          "ragdoll SixDOF constraint friction must be non-negative")
    if constraint.motor.isSome:
      let motor = constraint.motor.get
      for settings in motor.settings:
        settings.validate()
      motor.targetVelocity.requireFinite(
        "ragdoll SixDOF motor target velocity")
      motor.targetAngularVelocity.requireFinite(
        "ragdoll SixDOF motor target angular velocity")
      motor.targetPosition.requireFinite(
        "ragdoll SixDOF motor target position")
      discard motor.targetOrientation.normalized
  for constraint in config.coneConstraints:
    validateRagdollConstraintParts(
      constraint.part1, constraint.part2, config.parts.len,
      "ragdoll cone constraint")
    constraint.point.requireFinite("ragdoll cone constraint point")
    for axis in [constraint.twistAxis1, constraint.twistAxis2]:
      axis.requireFinite("ragdoll cone constraint twist axis")
      if axis.x * axis.x + axis.y * axis.y + axis.z * axis.z <= 1.0e-12'f32:
        raise newException(
          ValueError, "ragdoll cone constraint axes must be non-zero")
    if not constraint.halfConeAngle.isFinite or
        constraint.halfConeAngle < 0 or
        constraint.halfConeAngle > PI.float32:
      raise newException(
        ValueError, "ragdoll cone half angle must be within [0, PI]")

proc addRagdoll*(world: World; config: RagdollConfig): Ragdoll =
  ## Creates the complete body hierarchy and its swing-twist constraints as
  ## one owned unit. Body IDs remain valid only while the ragdoll is alive.
  world.requireOpen()
  config.validateRagdoll(world)
  var cooked = newSeq[CookedShape](config.parts.len)
  defer:
    for shape in cooked:
      shape.release()
  var nativeParts = newSeq[raw.RagdollPartData](config.parts.len)
  for index, part in config.parts:
    cooked[index] = cookShape(part.shape, part.motionType)
    let body = part.body
    let primaryAxis = if part.joint.kind == RagdollSixDOF:
      part.joint.sixDOF.axisX
    else:
      part.joint.twistAxis
    let secondaryAxis = if part.joint.kind == RagdollSixDOF:
      part.joint.sixDOF.axisY
    else:
      part.joint.planeAxis
    var linearLimitMin: Vec3 = vec3(0, 0, 0)
    var linearLimitMax: Vec3 = vec3(0, 0, 0)
    var angularLimitMin: Vec3 = vec3(0, 0, 0)
    var angularLimitMax: Vec3 = vec3(0, 0, 0)
    var linearFriction: Vec3 = vec3(0, 0, 0)
    var angularFriction: Vec3 = vec3(0, 0, 0)
    if part.joint.kind == RagdollSlider:
      linearLimitMin.x = part.joint.twistMinAngle
      linearLimitMax.x = part.joint.twistMaxAngle
      linearFriction.x = part.joint.maxFrictionTorque
    elif part.joint.kind == RagdollSixDOF:
      var limitMin, limitMax: array[6, float32]
      for axis in SixDOFAxis:
        let limit = part.joint.sixDOF.limits[axis]
        case limit.mode
        of SixDOFAxisMode.AxisFree:
          limitMin[ord(axis)] = -maxFiniteFloat32
          limitMax[ord(axis)] = maxFiniteFloat32
        of SixDOFAxisMode.AxisFixed:
          limitMin[ord(axis)] = maxFiniteFloat32
          limitMax[ord(axis)] = -maxFiniteFloat32
        of SixDOFAxisMode.AxisLimited:
          limitMin[ord(axis)] = limit.minimum
          limitMax[ord(axis)] = limit.maximum
      linearLimitMin = Vec3(x: limitMin[0], y: limitMin[1], z: limitMin[2])
      linearLimitMax = Vec3(x: limitMax[0], y: limitMax[1], z: limitMax[2])
      angularLimitMin = Vec3(
        x: limitMin[3], y: limitMin[4], z: limitMin[5])
      angularLimitMax = Vec3(
        x: limitMax[3], y: limitMax[4], z: limitMax[5])
      linearFriction = part.joint.linearFriction
      angularFriction = part.joint.angularFriction
    nativeParts[index] = raw.ragdollPartData(
      cooked[index].native,
      part.position.toRaw,
      part.rotation.normalized.toRaw,
      int32(part.joint.parent),
      uint8(ord(part.joint.kind)),
      part.layer,
      uint8(ord(part.motionType)),
      part.joint.position.toRaw,
      primaryAxis.toRaw,
      secondaryAxis.toRaw,
      uint8(ord(part.joint.sixDOF.swingType)),
      part.joint.normalHalfConeAngle,
      part.joint.planeHalfConeAngle,
      part.joint.twistMinAngle,
      part.joint.twistMaxAngle,
      part.joint.maxFrictionTorque,
      part.joint.motorFrequency,
      part.joint.motorDamping,
      part.joint.maxMotorTorque,
      linearLimitMin.toRaw,
      linearLimitMax.toRaw,
      angularLimitMin.toRaw,
      angularLimitMax.toRaw,
      linearFriction.toRaw,
      angularFriction.toRaw,
      allowedDOFMask(body.allowedDOFs),
      uint8(ord(body.motionQuality)),
      body.mass,
      body.inertiaMultiplier,
      body.linearVelocity.toRaw,
      body.angularVelocity.toRaw,
      body.userData,
      body.allowSleeping,
      body.collideKinematicVsNonDynamic,
      body.useManifoldReduction,
      body.applyGyroscopicForce,
      body.enhancedInternalEdgeRemoval,
      body.friction,
      body.restitution,
      body.linearDamping,
      body.angularDamping,
      body.maxLinearVelocity,
      body.maxAngularVelocity,
      body.gravityFactor,
      body.numVelocityStepsOverride,
      body.numPositionStepsOverride)
  var nativeDistances =
    newSeq[raw.RagdollDistanceConstraintData](config.distanceConstraints.len)
  for index, constraint in config.distanceConstraints:
    nativeDistances[index] = raw.ragdollDistanceConstraintData(
      uint32(constraint.part1), uint32(constraint.part2),
      constraint.point1.toRaw, constraint.point2.toRaw,
      constraint.minDistance, constraint.maxDistance)
  var nativeDistancePtr: ptr raw.RagdollDistanceConstraintData
  if nativeDistances.len > 0:
    nativeDistancePtr = addr nativeDistances[0]
  var nativePoints =
    newSeq[raw.RagdollPointConstraintData](config.pointConstraints.len)
  for index, constraint in config.pointConstraints:
    nativePoints[index] = raw.ragdollPointConstraintData(
      uint32(constraint.part1), uint32(constraint.part2),
      constraint.point.toRaw)
  var nativePointPtr: ptr raw.RagdollPointConstraintData
  if nativePoints.len > 0:
    nativePointPtr = addr nativePoints[0]
  var nativeFixed =
    newSeq[raw.RagdollFixedConstraintData](config.fixedConstraints.len)
  for index, constraint in config.fixedConstraints:
    nativeFixed[index] = raw.ragdollFixedConstraintData(
      uint32(constraint.part1), uint32(constraint.part2))
  var nativeFixedPtr: ptr raw.RagdollFixedConstraintData
  if nativeFixed.len > 0:
    nativeFixedPtr = addr nativeFixed[0]
  var nativeHinges =
    newSeq[raw.RagdollHingeConstraintData](config.hingeConstraints.len)
  for index, constraint in config.hingeConstraints:
    nativeHinges[index] = raw.ragdollHingeConstraintData(
      uint32(constraint.part1), uint32(constraint.part2),
      constraint.point.toRaw, constraint.hingeAxis.toRaw,
      constraint.normalAxis.toRaw, constraint.minAngle,
      constraint.maxAngle, constraint.maxFrictionTorque)
  var nativeHingePtr: ptr raw.RagdollHingeConstraintData
  if nativeHinges.len > 0:
    nativeHingePtr = addr nativeHinges[0]
  var nativeSliders =
    newSeq[raw.RagdollSliderConstraintData](config.sliderConstraints.len)
  for index, constraint in config.sliderConstraints:
    nativeSliders[index] = raw.ragdollSliderConstraintData(
      uint32(constraint.part1), uint32(constraint.part2),
      constraint.point.toRaw, constraint.sliderAxis.toRaw,
      constraint.normalAxis.toRaw, constraint.minPosition,
      constraint.maxPosition, constraint.maxFrictionForce)
  var nativeSliderPtr: ptr raw.RagdollSliderConstraintData
  if nativeSliders.len > 0:
    nativeSliderPtr = addr nativeSliders[0]
  var nativeSwingTwists = newSeq[raw.RagdollSwingTwistConstraintData](
    config.swingTwistConstraints.len)
  for index, constraint in config.swingTwistConstraints:
    nativeSwingTwists[index] = raw.ragdollSwingTwistConstraintData(
      uint32(constraint.part1), uint32(constraint.part2),
      constraint.point.toRaw, constraint.twistAxis.toRaw,
      constraint.planeAxis.toRaw, constraint.normalHalfConeAngle,
      constraint.planeHalfConeAngle, constraint.twistMinAngle,
      constraint.twistMaxAngle, constraint.maxFrictionTorque)
  var nativeSwingTwistPtr: ptr raw.RagdollSwingTwistConstraintData
  if nativeSwingTwists.len > 0:
    nativeSwingTwistPtr = addr nativeSwingTwists[0]
  var nativeSixDOFs = newSeq[raw.RagdollSixDOFConstraintData](
    config.sixDOFConstraints.len)
  for index, constraint in config.sixDOFConstraints:
    var limitMin, limitMax: array[6, float32]
    for axis in SixDOFAxis:
      let limit = constraint.config.limits[axis]
      case limit.mode
      of SixDOFAxisMode.AxisFree:
        limitMin[ord(axis)] = -maxFiniteFloat32
        limitMax[ord(axis)] = maxFiniteFloat32
      of SixDOFAxisMode.AxisFixed:
        limitMin[ord(axis)] = maxFiniteFloat32
        limitMax[ord(axis)] = -maxFiniteFloat32
      of SixDOFAxisMode.AxisLimited:
        limitMin[ord(axis)] = limit.minimum
        limitMax[ord(axis)] = limit.maximum
    nativeSixDOFs[index] = raw.ragdollSixDOFConstraintData(
      uint32(constraint.part1), uint32(constraint.part2),
      constraint.point.toRaw, constraint.config.axisX.toRaw,
      constraint.config.axisY.toRaw, uint8(ord(constraint.config.swingType)),
      Vec3(x: limitMin[0], y: limitMin[1], z: limitMin[2]).toRaw,
      Vec3(x: limitMax[0], y: limitMax[1], z: limitMax[2]).toRaw,
      Vec3(x: limitMin[3], y: limitMin[4], z: limitMin[5]).toRaw,
      Vec3(x: limitMax[3], y: limitMax[4], z: limitMax[5]).toRaw,
      constraint.linearFriction.toRaw, constraint.angularFriction.toRaw)
  var nativeSixDOFPtr: ptr raw.RagdollSixDOFConstraintData
  if nativeSixDOFs.len > 0:
    nativeSixDOFPtr = addr nativeSixDOFs[0]
  var nativeCones =
    newSeq[raw.RagdollConeConstraintData](config.coneConstraints.len)
  for index, constraint in config.coneConstraints:
    nativeCones[index] = raw.ragdollConeConstraintData(
      uint32(constraint.part1), uint32(constraint.part2),
      constraint.point.toRaw, constraint.twistAxis1.toRaw,
      constraint.twistAxis2.toRaw, constraint.halfConeAngle)
  var nativeConePtr: ptr raw.RagdollConeConstraintData
  if nativeCones.len > 0:
    nativeConePtr = addr nativeCones[0]
  let native = world.physics.newRagdoll(
    addr nativeParts[0], uint32(nativeParts.len),
    nativeDistancePtr, uint32(nativeDistances.len),
    nativePointPtr, uint32(nativePoints.len),
    nativeFixedPtr, uint32(nativeFixed.len),
    nativeHingePtr, uint32(nativeHinges.len),
    nativeSliderPtr, uint32(nativeSliders.len),
    nativeSwingTwistPtr, uint32(nativeSwingTwists.len),
    nativeSixDOFPtr, uint32(nativeSixDOFs.len),
    nativeConePtr, uint32(nativeCones.len), config.groupId,
    config.disableParentChildCollisions, config.stabilize,
    config.calculateConstraintPriorities, config.activate)
  if native.isNil:
    raise newException(JoltError, "Jolt could not create the ragdoll")
  let additionalBase = config.parts.len - 1
  let hingeBase = additionalBase + config.distanceConstraints.len +
    config.pointConstraints.len + config.fixedConstraints.len
  for offset, hinge in config.hingeConstraints:
    if hinge.motor.isSome:
      let constraint = native.constraint(uint32(hingeBase + offset))
      if constraint.isNil:
        raw.delete(native)
        raise newException(JoltError, "Jolt could not configure a ragdoll hinge motor")
      let motor = hinge.motor.get
      constraint.configureHingeMotor(
        uint8(ord(motor.settings.spring.mode)), motor.settings.spring.value,
        motor.settings.spring.damping, motor.settings.minTorque,
        motor.settings.maxTorque)
      constraint.setHingeMotorTarget(
        motor.targetVelocity, motor.targetPosition)
      constraint.setHingeMotorState(uint8(ord(motor.state)))
  let sliderBase = hingeBase + config.hingeConstraints.len
  for offset, slider in config.sliderConstraints:
    if slider.motor.isSome:
      let constraint = native.constraint(uint32(sliderBase + offset))
      if constraint.isNil:
        raw.delete(native)
        raise newException(JoltError, "Jolt could not configure a ragdoll slider motor")
      let motor = slider.motor.get
      constraint.configureSliderMotor(
        uint8(ord(motor.settings.spring.mode)), motor.settings.spring.value,
        motor.settings.spring.damping, motor.settings.minForce,
        motor.settings.maxForce)
      constraint.setSliderMotorTarget(
        motor.targetVelocity, motor.targetPosition)
      constraint.setSliderMotorState(uint8(ord(motor.state)))
  let swingTwistBase = sliderBase + config.sliderConstraints.len
  for offset, swingTwist in config.swingTwistConstraints:
    if swingTwist.motor.isSome:
      let constraint = native.constraint(uint32(swingTwistBase + offset))
      if constraint.isNil:
        raw.delete(native)
        raise newException(
          JoltError, "Jolt could not configure a ragdoll swing-twist motor")
      let motor = swingTwist.motor.get
      constraint.configureSwingTwistMotor(
        true, uint8(ord(motor.swingSettings.spring.mode)),
        motor.swingSettings.spring.value,
        motor.swingSettings.spring.damping,
        motor.swingSettings.minTorque, motor.swingSettings.maxTorque)
      constraint.configureSwingTwistMotor(
        false, uint8(ord(motor.twistSettings.spring.mode)),
        motor.twistSettings.spring.value,
        motor.twistSettings.spring.damping,
        motor.twistSettings.minTorque, motor.twistSettings.maxTorque)
      constraint.setSwingTwistMotorTargets(
        motor.targetAngularVelocity.toRaw,
        motor.targetOrientation.normalized.toRaw)
      constraint.setSwingTwistMotorState(
        true, uint8(ord(motor.swingState)))
      constraint.setSwingTwistMotorState(
        false, uint8(ord(motor.twistState)))
  let sixDOFBase = swingTwistBase + config.swingTwistConstraints.len
  for offset, sixDOF in config.sixDOFConstraints:
    if sixDOF.motor.isSome:
      let constraint = native.constraint(uint32(sixDOFBase + offset))
      if constraint.isNil:
        raw.delete(native)
        raise newException(JoltError, "Jolt could not configure a ragdoll SixDOF motor")
      let motor = sixDOF.motor.get
      for axis in SixDOFAxis:
        let settings = motor.settings[axis]
        let minimum = if axis <= SixDOFAxis.TranslationZ:
            settings.minForce
          else:
            settings.minTorque
        let maximum = if axis <= SixDOFAxis.TranslationZ:
            settings.maxForce
          else:
            settings.maxTorque
        constraint.configureSixDOFMotor(
          uint8(ord(axis)), uint8(ord(settings.spring.mode)),
          settings.spring.value, settings.spring.damping, minimum, maximum)
      constraint.setSixDOFMotorTargets(
        motor.targetVelocity.toRaw, motor.targetAngularVelocity.toRaw,
        motor.targetPosition.toRaw, motor.targetOrientation.normalized.toRaw)
      for axis in SixDOFAxis:
        constraint.setSixDOFMotorState(
          uint8(ord(axis)), uint8(ord(motor.states[axis])))
  new(result)
  result.owner = world
  result.native = native
  result.config = config
  result.bodyIds = newSeq[uint32](config.parts.len)
  for index in 0 ..< config.parts.len:
    result.bodyIds[index] = native.bodyId(uint32(index))
    world.ragdollBodyIds.add(result.bodyIds[index])
  result.alive = true
  world.ragdolls.add(native)

proc isAlive*(ragdoll: Ragdoll): bool =
  not ragdoll.isNil and ragdoll.alive and not ragdoll.native.isNil and
    not ragdoll.owner.isNil and ragdoll.owner.isOpen

proc requireAlive(ragdoll: Ragdoll) =
  if not ragdoll.isAlive:
    raise newException(JoltError, "Jolt ragdoll is no longer alive")

proc requirePart(ragdoll: Ragdoll; index: int) =
  ragdoll.requireAlive()
  if index < 0 or index >= ragdoll.bodyIds.len:
    raise newException(IndexDefect, "ragdoll part index is out of bounds")

proc partCount*(ragdoll: Ragdoll): int =
  ragdoll.requireAlive()
  ragdoll.bodyIds.len

proc constraintCount*(ragdoll: Ragdoll): int =
  ragdoll.requireAlive()
  int(ragdoll.native.constraintCount)

proc requireConstraint(ragdoll: Ragdoll; index: int): ptr raw.Constraint =
  ragdoll.requireAlive()
  if index < 0 or index >= int(ragdoll.native.constraintCount):
    raise newException(IndexDefect, "ragdoll constraint index is out of bounds")
  result = ragdoll.native.constraint(uint32(index))
  if result.isNil:
    raise newException(JoltError, "Jolt could not inspect ragdoll constraint")

proc constraintKind*(ragdoll: Ragdoll; index: int): ConstraintKind =
  ## Returns the native kind without transferring ragdoll constraint ownership.
  case ragdoll.requireConstraint(index).subType
  of 0: ConstraintKind.Fixed
  of 1: ConstraintKind.Point
  of 2: ConstraintKind.Hinge
  of 3: ConstraintKind.Slider
  of 4: ConstraintKind.Distance
  of 5: ConstraintKind.Cone
  of 6: ConstraintKind.SwingTwist
  of 7: ConstraintKind.SixDOF
  of 8: ConstraintKind.Path
  of 10: ConstraintKind.RackAndPinion
  of 11: ConstraintKind.Gear
  of 12: ConstraintKind.Pulley
  else: raise newException(JoltError, "Jolt ragdoll constraint kind is unsupported")

proc constraintBodyParts*(ragdoll: Ragdoll; index: int):
    tuple[part1, part2: int] =
  ## Maps a native constraint back to its two ragdoll part indices.
  discard ragdoll.requireConstraint(index)
  var first, second: uint32
  if not ragdoll.native.constraintBodyIndices(
      uint32(index), addr first, addr second):
    raise newException(JoltError, "Jolt could not map ragdoll constraint bodies")
  (int(first), int(second))

proc sixDOFSwingType*(ragdoll: Ragdoll; index: int): SixDOFSwingType =
  ## Reads the native swing solver used by a ragdoll SixDOF constraint.
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  let value = constraint.sixDOFSwingType
  if value > uint8(ord(high(SixDOFSwingType))):
    raise newException(JoltError, "Jolt returned an invalid SixDOF swing type")
  SixDOFSwingType(value)

proc axisLimit*(ragdoll: Ragdoll; index: int;
                axis: SixDOFAxis): SixDOFAxisLimit =
  ## Reads one native SixDOF limit without escaping ragdoll ownership.
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  var minimum, maximum: cfloat
  var mode: uint8
  constraint.sixDOFAxisLimit(
    uint8(ord(axis)), addr minimum, addr maximum, addr mode)
  case mode
  of 0: freeAxis()
  of 1: fixedAxis()
  of 2: limitedAxis(minimum, maximum)
  else: raise newException(JoltError, "Jolt returned an unknown SixDOF axis mode")

proc wakeRagdollConstraints(ragdoll: Ragdoll) =
  ragdoll.native.activate()

proc constraintEnabled*(ragdoll: Ragdoll; index: int): bool =
  ragdoll.requireConstraint(index).enabled

proc setConstraintEnabled*(ragdoll: Ragdoll; index: int; enabled: bool) =
  ragdoll.requireConstraint(index).setEnabled(enabled)
  ragdoll.wakeRagdollConstraints()

proc constraintPriority*(ragdoll: Ragdoll; index: int): uint32 =
  ragdoll.requireConstraint(index).priority

proc setConstraintPriority*(ragdoll: Ragdoll; index: int; priority: uint32) =
  ragdoll.requireConstraint(index).setPriority(priority)

proc constraintSolverStepOverrides*(ragdoll: Ragdoll; index: int):
    tuple[velocity, position: uint32] =
  let constraint = ragdoll.requireConstraint(index)
  (constraint.velocityStepsOverride, constraint.positionStepsOverride)

proc setConstraintSolverStepOverrides*(ragdoll: Ragdoll; index: int;
                                       velocity, position: uint32) =
  let constraint = ragdoll.requireConstraint(index)
  if velocity >= 256 or position >= 256:
    raise newException(
      ValueError, "constraint solver step overrides must be below 256")
  constraint.setVelocityStepsOverride(velocity)
  constraint.setPositionStepsOverride(position)
  ragdoll.wakeRagdollConstraints()

proc constraintUserData*(ragdoll: Ragdoll; index: int): uint64 =
  ragdoll.requireConstraint(index).userData

proc setConstraintUserData*(ragdoll: Ragdoll; index: int; value: uint64) =
  ragdoll.requireConstraint(index).setUserData(value)

proc resetConstraintWarmStart*(ragdoll: Ragdoll; index: int) =
  ragdoll.requireConstraint(index).resetWarmStart()

proc constraintSolverImpulse*(ragdoll: Ragdoll;
                              index: int): ConstraintSolverImpulse =
  let constraint = ragdoll.requireConstraint(index)
  var position, rotation, motorTranslation, motorRotation: raw.Vec3
  var limit: cfloat
  if not constraint.solverImpulse(
      addr position, addr rotation, addr limit,
      addr motorTranslation, addr motorRotation):
    raise newException(JoltError, "Jolt could not inspect ragdoll constraint impulses")
  ConstraintSolverImpulse(
    position: position.fromRaw, rotation: rotation.fromRaw, limit: limit,
    motorTranslation: motorTranslation.fromRaw,
    motorRotation: motorRotation.fromRaw)

proc setAxisLimit*(ragdoll: Ragdoll; index: int; axis: SixDOFAxis;
                   limit: SixDOFAxisLimit) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  limit.validate(axis, ragdoll.sixDOFSwingType(index))
  constraint.setSixDOFAxisLimit(
    uint8(ord(axis)), uint8(ord(limit.mode)), limit.minimum, limit.maximum)
  ragdoll.wakeRagdollConstraints()

proc sixDOFConstraintFriction*(ragdoll: Ragdoll; index: int;
                               axis: SixDOFAxis): float32 =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  var value: cfloat
  if not constraint.friction(uint8(ord(axis)), addr value):
    raise newException(JoltError, "Jolt could not inspect ragdoll SixDOF friction")
  float32(value)

proc setAxisFriction*(ragdoll: Ragdoll; index: int; axis: SixDOFAxis;
                      maximum: float32) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  if not maximum.isFinite or maximum < 0:
    raise newException(ValueError, "axis friction must be finite and non-negative")
  constraint.setSixDOFFriction(uint8(ord(axis)), maximum)
  ragdoll.wakeRagdollConstraints()

proc sixDOFConstraintLimitSpring*(ragdoll: Ragdoll; index: int;
                                  axis: SixDOFAxis): SpringSettings =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  if axis > SixDOFAxis.TranslationZ:
    raise newException(
      ValueError, "SixDOF limit springs only support translation axes")
  var mode: uint8
  var value, damping: cfloat
  if not constraint.limitSpring(
      uint8(ord(axis)), addr mode, addr value, addr damping):
    raise newException(JoltError, "Jolt could not inspect ragdoll SixDOF spring")
  if mode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid spring mode")
  SpringSettings(
    mode: SpringMode(mode), value: float32(value), damping: float32(damping))

proc setAxisLimitSpring*(ragdoll: Ragdoll; index: int; axis: SixDOFAxis;
                         settings: SpringSettings) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  if axis > SixDOFAxis.TranslationZ:
    raise newException(
      ValueError, "SixDOF limit springs only support translation axes")
  settings.validate()
  constraint.setSixDOFLimitSpring(
    uint8(ord(axis)), uint8(ord(settings.mode)),
    settings.value, settings.damping)
  ragdoll.wakeRagdollConstraints()

proc sixDOFAxisMotorSettings*(ragdoll: Ragdoll; index: int;
                              axis: SixDOFAxis): MotorSettings =
  ## Reads force/torque and spring tuning from a ragdoll SixDOF motor.
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  var springMode: uint8
  var springValue, springDamping, minForce, maxForce,
    minTorque, maxTorque: cfloat
  if not constraint.motorSettings(
      uint8(ord(axis)), addr springMode, addr springValue,
      addr springDamping, addr minForce, addr maxForce,
      addr minTorque, addr maxTorque):
    raise newException(JoltError, "Jolt could not inspect ragdoll SixDOF motor")
  if springMode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid motor spring mode")
  MotorSettings(
    spring: SpringSettings(
      mode: SpringMode(springMode), value: float32(springValue),
      damping: float32(springDamping)),
    minForce: float32(minForce), maxForce: float32(maxForce),
    minTorque: float32(minTorque), maxTorque: float32(maxTorque))

proc configureAxisMotor*(ragdoll: Ragdoll; index: int; axis: SixDOFAxis;
                         settings: MotorSettings) =
  ## Changes one ragdoll SixDOF motor's tuning without escaping ownership.
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  settings.validate()
  let minimum = if axis <= SixDOFAxis.TranslationZ:
      settings.minForce
    else:
      settings.minTorque
  let maximum = if axis <= SixDOFAxis.TranslationZ:
      settings.maxForce
    else:
      settings.maxTorque
  constraint.configureSixDOFMotor(
    uint8(ord(axis)), uint8(ord(settings.spring.mode)),
    settings.spring.value, settings.spring.damping, minimum, maximum)

proc setAxisMotorState*(ragdoll: Ragdoll; index: int; axis: SixDOFAxis;
                        state: MotorState) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  constraint.setSixDOFMotorState(uint8(ord(axis)), uint8(ord(state)))
  ragdoll.wakeRagdollConstraints()

proc setSixDOFMotorTargets*(ragdoll: Ragdoll; index: int;
                            velocity, angularVelocity, position: Vec3;
                            orientation: Quat) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  velocity.requireFinite("ragdoll SixDOF target velocity")
  angularVelocity.requireFinite("ragdoll SixDOF target angular velocity")
  position.requireFinite("ragdoll SixDOF target position")
  constraint.setSixDOFMotorTargets(
    velocity.toRaw, angularVelocity.toRaw, position.toRaw,
    orientation.normalized.toRaw)
  ragdoll.wakeRagdollConstraints()

proc sixDOFMotor*(ragdoll: Ragdoll; index: int; axis: SixDOFAxis): tuple[
    state: MotorState; targetVelocity, targetAngularVelocity,
    targetPosition: Vec3; targetOrientation: Quat] =
  ## Reads one axis state and the shared SixDOF targets from native Jolt state.
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "ragdoll constraint is not SixDOF")
  var state: uint8
  var velocity, angularVelocity, position: raw.Vec3
  var orientation: raw.Quat
  if not constraint.sixDOFMotorState(
      uint8(ord(axis)), addr state, addr velocity, addr angularVelocity,
      addr position, addr orientation):
    raise newException(JoltError, "Jolt could not inspect ragdoll SixDOF motor state")
  (decodeMotorState(state, "ragdoll SixDOF motor"), velocity.fromRaw,
   angularVelocity.fromRaw, position.fromRaw, orientation.fromRaw)

proc readRagdollMotorSettings(ragdoll: Ragdoll; index: int;
                              motorIndex: uint8): MotorSettings =
  let constraint = ragdoll.requireConstraint(index)
  var springMode: uint8
  var springValue, springDamping, minForce, maxForce,
    minTorque, maxTorque: cfloat
  if not constraint.motorSettings(
      motorIndex, addr springMode, addr springValue,
      addr springDamping, addr minForce, addr maxForce,
      addr minTorque, addr maxTorque):
    raise newException(JoltError, "Jolt could not inspect ragdoll motor settings")
  if springMode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid motor spring mode")
  MotorSettings(
    spring: SpringSettings(
      mode: SpringMode(springMode), value: float32(springValue),
      damping: float32(springDamping)),
    minForce: float32(minForce), maxForce: float32(maxForce),
    minTorque: float32(minTorque), maxTorque: float32(maxTorque))

proc motorSettings*(ragdoll: Ragdoll; index: int): MotorSettings =
  ## Reads native tuning for a ragdoll hinge or slider motor.
  if ragdoll.constraintKind(index) notin
      {ConstraintKind.Hinge, ConstraintKind.Slider}:
    raise newException(ValueError, "ragdoll constraint is not a hinge or slider")
  ragdoll.readRagdollMotorSettings(index, 0)

proc configureMotor*(ragdoll: Ragdoll; index: int;
                     settings: MotorSettings) =
  ## Changes native tuning for a ragdoll hinge or slider motor.
  let constraint = ragdoll.requireConstraint(index)
  settings.validate()
  case ragdoll.constraintKind(index)
  of ConstraintKind.Hinge:
    constraint.configureHingeMotor(
      uint8(ord(settings.spring.mode)), settings.spring.value,
      settings.spring.damping, settings.minTorque, settings.maxTorque)
  of ConstraintKind.Slider:
    constraint.configureSliderMotor(
      uint8(ord(settings.spring.mode)), settings.spring.value,
      settings.spring.damping, settings.minForce, settings.maxForce)
  else:
    raise newException(ValueError, "ragdoll constraint is not a hinge or slider")
  ragdoll.wakeRagdollConstraints()

proc setMotorTarget*(ragdoll: Ragdoll; index: int;
                     velocity, position: float32) =
  let constraint = ragdoll.requireConstraint(index)
  if not velocity.isFinite or not position.isFinite:
    raise newException(ValueError, "ragdoll motor targets must be finite")
  case ragdoll.constraintKind(index)
  of ConstraintKind.Hinge:
    constraint.setHingeMotorTarget(velocity, position)
  of ConstraintKind.Slider:
    constraint.setSliderMotorTarget(velocity, position)
  else:
    raise newException(ValueError, "ragdoll constraint is not a hinge or slider")
  ragdoll.wakeRagdollConstraints()

proc setMotorState*(ragdoll: Ragdoll; index: int; state: MotorState) =
  let constraint = ragdoll.requireConstraint(index)
  case ragdoll.constraintKind(index)
  of ConstraintKind.Hinge:
    constraint.setHingeMotorState(uint8(ord(state)))
  of ConstraintKind.Slider:
    constraint.setSliderMotorState(uint8(ord(state)))
  else:
    raise newException(ValueError, "ragdoll constraint is not a hinge or slider")
  ragdoll.wakeRagdollConstraints()

proc motor*(ragdoll: Ragdoll; index: int): tuple[
    state: MotorState; targetVelocity, targetPosition: float32] =
  let constraint = ragdoll.requireConstraint(index)
  var state: uint8
  var velocity, position: cfloat
  case ragdoll.constraintKind(index)
  of ConstraintKind.Hinge:
    constraint.hingeMotor(addr state, addr velocity, addr position)
  of ConstraintKind.Slider:
    constraint.sliderMotor(addr state, addr velocity, addr position)
  else:
    raise newException(ValueError, "ragdoll constraint is not a hinge or slider")
  (decodeMotorState(state, "ragdoll scalar motor"),
   float32(velocity), float32(position))

proc setMotor*(ragdoll: Ragdoll; index: int; state: MotorState;
               targetVelocity, targetPosition: float32;
               settings = defaultMotorSettings()) =
  ragdoll.configureMotor(index, settings)
  ragdoll.setMotorTarget(index, targetVelocity, targetPosition)
  ragdoll.setMotorState(index, state)

proc swingMotorSettings*(ragdoll: Ragdoll; index: int): MotorSettings =
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  ragdoll.readRagdollMotorSettings(index, 0)

proc twistMotorSettings*(ragdoll: Ragdoll; index: int): MotorSettings =
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  ragdoll.readRagdollMotorSettings(index, 1)

proc configureSwingMotor*(ragdoll: Ragdoll; index: int;
                          settings: MotorSettings) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  settings.validate()
  constraint.configureSwingTwistMotor(
    true, uint8(ord(settings.spring.mode)), settings.spring.value,
    settings.spring.damping, settings.minTorque, settings.maxTorque)
  ragdoll.wakeRagdollConstraints()

proc configureTwistMotor*(ragdoll: Ragdoll; index: int;
                          settings: MotorSettings) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  settings.validate()
  constraint.configureSwingTwistMotor(
    false, uint8(ord(settings.spring.mode)), settings.spring.value,
    settings.spring.damping, settings.minTorque, settings.maxTorque)
  ragdoll.wakeRagdollConstraints()

proc setSwingMotorState*(ragdoll: Ragdoll; index: int;
                         state: MotorState) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  constraint.setSwingTwistMotorState(true, uint8(ord(state)))
  ragdoll.wakeRagdollConstraints()

proc setTwistMotorState*(ragdoll: Ragdoll; index: int;
                         state: MotorState) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  constraint.setSwingTwistMotorState(false, uint8(ord(state)))
  ragdoll.wakeRagdollConstraints()

proc setSwingTwistMotorTargets*(ragdoll: Ragdoll; index: int;
                                angularVelocity: Vec3;
                                orientation: Quat) =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  angularVelocity.requireFinite("ragdoll swing-twist target angular velocity")
  constraint.setSwingTwistMotorTargets(
    angularVelocity.toRaw, orientation.normalized.toRaw)
  ragdoll.wakeRagdollConstraints()

proc swingTwistMotor*(ragdoll: Ragdoll; index: int): tuple[
    swingState, twistState: MotorState; targetAngularVelocity: Vec3;
    targetOrientation: Quat] =
  let constraint = ragdoll.requireConstraint(index)
  if ragdoll.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "ragdoll constraint is not SwingTwist")
  var swingState, twistState: uint8
  var angularVelocity: raw.Vec3
  var orientation: raw.Quat
  if not constraint.swingTwistMotorState(
      addr swingState, addr twistState, addr angularVelocity,
      addr orientation):
    raise newException(JoltError, "Jolt could not inspect ragdoll SwingTwist motor")
  (decodeMotorState(swingState, "ragdoll swing motor"),
   decodeMotorState(twistState, "ragdoll twist motor"),
   angularVelocity.fromRaw, orientation.fromRaw)

proc partId*(ragdoll: Ragdoll; index: int): BodyId =
  ragdoll.requirePart(index)
  BodyId(ragdoll.bodyIds[index])

proc partName*(ragdoll: Ragdoll; index: int): string =
  ragdoll.requirePart(index)
  ragdoll.config.parts[index].name

proc partParent*(ragdoll: Ragdoll; index: int): Option[int] =
  ragdoll.requirePart(index)
  let parent = ragdoll.config.parts[index].joint.parent
  if parent < 0: none(int) else: some(parent)

proc partShape*(ragdoll: Ragdoll; index: int): Shape =
  ragdoll.requirePart(index)
  ragdoll.config.parts[index].shape

proc partPosition*(ragdoll: Ragdoll; index: int): Vec3 =
  ragdoll.requirePart(index)
  fromRaw(ragdoll.owner.physics.bodyInterface().position(
    raw.bodyID(ragdoll.bodyIds[index])))

proc partRotation*(ragdoll: Ragdoll; index: int): Quat =
  ragdoll.requirePart(index)
  fromRaw(ragdoll.owner.physics.bodyInterface().rotation(
    raw.bodyID(ragdoll.bodyIds[index])))

proc partLinearVelocity*(ragdoll: Ragdoll; index: int): Vec3 =
  ragdoll.requirePart(index)
  fromRaw(ragdoll.owner.physics.bodyInterface().linearVelocity(
    raw.bodyID(ragdoll.bodyIds[index])))

proc partAngularVelocity*(ragdoll: Ragdoll; index: int): Vec3 =
  ragdoll.requirePart(index)
  fromRaw(ragdoll.owner.physics.bodyInterface().angularVelocity(
    raw.bodyID(ragdoll.bodyIds[index])))

proc pose*(ragdoll: Ragdoll): seq[RagdollTransform] =
  ragdoll.requireAlive()
  var positions = newSeq[raw.Vec3](ragdoll.bodyIds.len)
  var rotations = newSeq[raw.Quat](ragdoll.bodyIds.len)
  ragdoll.native.pose(addr positions[0], addr rotations[0])
  result = newSeq[RagdollTransform](ragdoll.bodyIds.len)
  for index in 0 ..< result.len:
    result[index] = RagdollTransform(
      position: fromRaw(positions[index]),
      rotation: fromRaw(rotations[index]))

proc validatePose(ragdoll: Ragdoll; transforms: openArray[RagdollTransform]) =
  ragdoll.requireAlive()
  if transforms.len != ragdoll.bodyIds.len:
    raise newException(ValueError, "ragdoll pose must contain one transform per part")
  for transform in transforms:
    transform.position.requireFinite("ragdoll pose position")
    discard transform.rotation.normalized

proc nativePose(transforms: openArray[RagdollTransform];
                positions: var seq[raw.Vec3]; rotations: var seq[raw.Quat]) =
  positions = newSeq[raw.Vec3](transforms.len)
  rotations = newSeq[raw.Quat](transforms.len)
  for index, transform in transforms:
    positions[index] = transform.position.toRaw
    rotations[index] = transform.rotation.normalized.toRaw

proc setPose*(ragdoll: Ragdoll; transforms: openArray[RagdollTransform]) =
  ragdoll.validatePose(transforms)
  var positions: seq[raw.Vec3]
  var rotations: seq[raw.Quat]
  transforms.nativePose(positions, rotations)
  ragdoll.native.setPose(addr positions[0], addr rotations[0])
  ragdoll.native.resetWarmStart()

proc driveKinematic*(ragdoll: Ragdoll;
                     transforms: openArray[RagdollTransform];
                     deltaTime: float32) =
  ragdoll.validatePose(transforms)
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  for part in ragdoll.config.parts:
    if part.motionType != MotionType.Kinematic:
      raise newException(
        JoltError, "kinematic pose driving requires kinematic ragdoll parts")
  var positions: seq[raw.Vec3]
  var rotations: seq[raw.Quat]
  transforms.nativePose(positions, rotations)
  ragdoll.native.driveKinematic(
    addr positions[0], addr rotations[0], deltaTime)

proc driveMotors*(ragdoll: Ragdoll;
                  localPose: openArray[RagdollTransform]) =
  ## Drives dynamic child parts using their configured joint motors.
  ragdoll.validatePose(localPose)
  for part in ragdoll.config.parts:
    if part.motionType != MotionType.Dynamic:
      raise newException(
        JoltError, "motor pose driving requires dynamic ragdoll parts")
    if part.joint.parent >= 0 and
        part.joint.kind notin {RagdollSwingTwist, RagdollHinge}:
      raise newException(
        JoltError, "motor pose driving requires swing-twist or hinge joints")
  var translations: seq[raw.Vec3]
  var rotations: seq[raw.Quat]
  localPose.nativePose(translations, rotations)
  ragdoll.native.driveMotors(addr translations[0], addr rotations[0])

proc driveMotors*(ragdoll: Ragdoll;
                  previousLocalPose, localPose: openArray[RagdollTransform];
                  deltaTime: float32) =
  ## Drives position and target angular velocity from consecutive local poses.
  ragdoll.validatePose(previousLocalPose)
  ragdoll.validatePose(localPose)
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  for part in ragdoll.config.parts:
    if part.motionType != MotionType.Dynamic:
      raise newException(
        JoltError, "motor pose driving requires dynamic ragdoll parts")
    if part.joint.parent >= 0 and
        part.joint.kind notin {RagdollSwingTwist, RagdollHinge}:
      raise newException(
        JoltError, "motor pose driving requires swing-twist or hinge joints")
  var previousTranslations, translations: seq[raw.Vec3]
  var previousRotations, rotations: seq[raw.Quat]
  previousLocalPose.nativePose(previousTranslations, previousRotations)
  localPose.nativePose(translations, rotations)
  ragdoll.native.driveMotorsWithVelocity(
    addr previousTranslations[0], addr previousRotations[0],
    addr translations[0], addr rotations[0], deltaTime)

proc rootTransform*(ragdoll: Ragdoll): RagdollTransform =
  ragdoll.requireAlive()
  var position: raw.Vec3
  var rotation: raw.Quat
  ragdoll.native.rootTransform(addr position, addr rotation)
  RagdollTransform(position: fromRaw(position), rotation: fromRaw(rotation))

proc activate*(ragdoll: Ragdoll) =
  ragdoll.requireAlive()
  ragdoll.native.activate()

proc isActive*(ragdoll: Ragdoll): bool =
  ragdoll.isAlive and ragdoll.native.isActive()

proc setGroupId*(ragdoll: Ragdoll; groupId: uint32) =
  ragdoll.requireAlive()
  ragdoll.native.setGroupId(groupId)
  ragdoll.config.groupId = groupId

proc setVelocity*(ragdoll: Ragdoll; linear, angular: Vec3) =
  ragdoll.requireAlive()
  linear.requireFinite("ragdoll linear velocity")
  angular.requireFinite("ragdoll angular velocity")
  ragdoll.native.setVelocity(linear.toRaw, angular.toRaw)

proc addLinearVelocity*(ragdoll: Ragdoll; velocity: Vec3) =
  ragdoll.requireAlive()
  velocity.requireFinite("ragdoll linear velocity")
  ragdoll.native.addLinearVelocity(velocity.toRaw)

proc addImpulse*(ragdoll: Ragdoll; impulse: Vec3) =
  ragdoll.requireAlive()
  impulse.requireFinite("ragdoll impulse")
  ragdoll.native.addImpulse(impulse.toRaw)

proc setPartLinearVelocity*(ragdoll: Ragdoll; index: int; velocity: Vec3) =
  ragdoll.requirePart(index)
  velocity.requireFinite("ragdoll part linear velocity")
  ragdoll.owner.physics.bodyInterface().setLinearVelocity(
    raw.bodyID(ragdoll.bodyIds[index]), velocity.toRaw)

proc setPartAngularVelocity*(ragdoll: Ragdoll; index: int; velocity: Vec3) =
  ragdoll.requirePart(index)
  velocity.requireFinite("ragdoll part angular velocity")
  ragdoll.owner.physics.bodyInterface().setAngularVelocity(
    raw.bodyID(ragdoll.bodyIds[index]), velocity.toRaw)

proc addPartImpulse*(ragdoll: Ragdoll; index: int; impulse: Vec3) =
  ragdoll.requirePart(index)
  impulse.requireFinite("ragdoll part impulse")
  ragdoll.owner.physics.bodyInterface().addImpulse(
    raw.bodyID(ragdoll.bodyIds[index]), impulse.toRaw)

proc addPartImpulseAtPosition*(ragdoll: Ragdoll; index: int;
                               impulse, position: Vec3) =
  ragdoll.requirePart(index)
  impulse.requireFinite("ragdoll part impulse")
  position.requireFinite("ragdoll part impulse position")
  ragdoll.owner.physics.bodyInterface().addImpulse(
    raw.bodyID(ragdoll.bodyIds[index]), impulse.toRaw, position.toRaw)

proc addPartAngularImpulse*(ragdoll: Ragdoll; index: int; impulse: Vec3) =
  ragdoll.requirePart(index)
  impulse.requireFinite("ragdoll part angular impulse")
  ragdoll.owner.physics.bodyInterface().addAngularImpulse(
    raw.bodyID(ragdoll.bodyIds[index]), impulse.toRaw)

proc resetWarmStart*(ragdoll: Ragdoll) =
  ragdoll.requireAlive()
  ragdoll.native.resetWarmStart()

proc validateSkeleton(definition: SkeletonDefinition; name: string) =
  if definition.joints.len == 0:
    raise newException(ValueError, name & " skeleton requires at least one joint")
  if uint64(definition.joints.len) > uint64(high(uint32)):
    raise newException(ValueError, name & " skeleton has too many joints")
  for index, joint in definition.joints:
    if joint.name.len == 0:
      raise newException(ValueError, name & " skeleton joint names must not be empty")
    for previous in 0 ..< index:
      if definition.joints[previous].name == joint.name:
        raise newException(ValueError, name & " skeleton joint names must be unique")
    if index == 0:
      if joint.parent != -1:
        raise newException(ValueError, name & " skeleton root must have no parent")
    elif joint.parent < 0 or joint.parent >= index:
      raise newException(
        ValueError, name & " skeleton parents must precede their children")
    joint.neutralTransform.position.requireFinite(
      name & " skeleton neutral position")
    discard joint.neutralTransform.rotation.normalized

proc copySkeleton(definition: SkeletonDefinition): SkeletonDefinition =
  result.joints = newSeq[SkeletonJoint](definition.joints.len)
  for index, joint in definition.joints:
    result.joints[index] = joint

proc nativeSkeleton(definition: SkeletonDefinition;
                    names: var seq[cstring]; parents: var seq[int32];
                    positions: var seq[raw.Vec3]; rotations: var seq[raw.Quat]) =
  names = newSeq[cstring](definition.joints.len)
  parents = newSeq[int32](definition.joints.len)
  positions = newSeq[raw.Vec3](definition.joints.len)
  rotations = newSeq[raw.Quat](definition.joints.len)
  for index, joint in definition.joints:
    names[index] = cstring(joint.name)
    parents[index] = int32(joint.parent)
    positions[index] = joint.neutralTransform.position.toRaw
    rotations[index] = joint.neutralTransform.rotation.normalized.toRaw

proc copyAnimationTracks(
    tracks: openArray[SkeletalAnimationTrack]): seq[SkeletalAnimationTrack] =
  result = newSeq[SkeletalAnimationTrack](tracks.len)
  for trackIndex, track in tracks:
    result[trackIndex].jointName = track.jointName
    result[trackIndex].keyframes =
      newSeq[SkeletalAnimationKeyframe](track.keyframes.len)
    for keyIndex, keyframe in track.keyframes:
      result[trackIndex].keyframes[keyIndex] = keyframe

proc newSkeletalAnimation*(skeleton: SkeletonDefinition;
                           tracks: openArray[SkeletalAnimationTrack];
                           looping = true): SkeletalAnimation =
  ## Creates an owned Jolt animation. Untracked joints retain their neutral
  ## local transforms when sampled.
  skeleton.validateSkeleton("animation")
  if tracks.len == 0:
    raise newException(ValueError, "animation requires at least one track")
  if uint64(tracks.len) > uint64(high(uint32)):
    raise newException(ValueError, "animation has too many tracks")

  var keyframeTotal = 0'u64
  for trackIndex, track in tracks:
    if track.jointName.len == 0:
      raise newException(ValueError, "animation track names must not be empty")
    var found = false
    for joint in skeleton.joints:
      if joint.name == track.jointName:
        found = true
        break
    if not found:
      raise newException(
        ValueError, "animation track joint does not exist in skeleton")
    for previous in 0 ..< trackIndex:
      if tracks[previous].jointName == track.jointName:
        raise newException(
          ValueError, "animation must not contain duplicate joint tracks")
    if track.keyframes.len == 0:
      raise newException(
        ValueError, "animation tracks require at least one keyframe")
    keyframeTotal += uint64(track.keyframes.len)
    if keyframeTotal > uint64(high(uint32)):
      raise newException(ValueError, "animation has too many keyframes")
    var previousTime = -1.0'f32
    for keyframe in track.keyframes:
      if not keyframe.time.isFinite or keyframe.time < 0:
        raise newException(
          ValueError, "animation keyframe times must be finite and non-negative")
      if keyframe.time <= previousTime:
        raise newException(
          ValueError, "animation keyframe times must be strictly increasing")
      previousTime = keyframe.time
      keyframe.transform.position.requireFinite(
        "animation keyframe position")
      discard keyframe.transform.rotation.normalized

  var jointNames: seq[cstring]
  var jointParents: seq[int32]
  var neutralPositions: seq[raw.Vec3]
  var neutralRotations: seq[raw.Quat]
  skeleton.nativeSkeleton(
    jointNames, jointParents, neutralPositions, neutralRotations)
  var trackNames = newSeq[cstring](tracks.len)
  var trackOffsets = newSeq[uint32](tracks.len + 1)
  var times = newSeq[cfloat](int(keyframeTotal))
  var translations = newSeq[raw.Vec3](int(keyframeTotal))
  var rotations = newSeq[raw.Quat](int(keyframeTotal))
  var flatIndex = 0
  for trackIndex, track in tracks:
    trackNames[trackIndex] = cstring(track.jointName)
    trackOffsets[trackIndex] = uint32(flatIndex)
    for keyframe in track.keyframes:
      times[flatIndex] = cfloat(keyframe.time)
      translations[flatIndex] = keyframe.transform.position.toRaw
      rotations[flatIndex] = keyframe.transform.rotation.normalized.toRaw
      inc flatIndex
  trackOffsets[^1] = uint32(flatIndex)

  new(result)
  if not raw.acquireJolt():
    raise newException(
      JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true
  result.native = raw.newSkeletalAnimation(
    addr jointNames[0], addr jointParents[0], addr neutralPositions[0],
    addr neutralRotations[0], uint32(jointNames.len), addr trackNames[0],
    addr trackOffsets[0], uint32(trackNames.len), addr times[0],
    addr translations[0], addr rotations[0], uint32(flatIndex), looping)
  if result.native.isNil:
    raw.releaseJolt()
    result.acquiredJolt = false
    raise newException(JoltError, "Jolt could not create the skeletal animation")
  result.skeleton = skeleton.copySkeleton()
  result.tracks = tracks.copyAnimationTracks()
  result.alive = true

proc isAlive*(animation: SkeletalAnimation): bool =
  not animation.isNil and animation.alive and not animation.native.isNil

proc requireAlive(animation: SkeletalAnimation) =
  if not animation.isAlive:
    raise newException(JoltError, "Jolt skeletal animation is no longer alive")

proc jointCount*(animation: SkeletalAnimation): int =
  animation.requireAlive()
  animation.skeleton.joints.len

proc trackCount*(animation: SkeletalAnimation): int =
  animation.requireAlive()
  int(animation.native.trackCount)

proc keyframeCount*(animation: SkeletalAnimation): int =
  animation.requireAlive()
  int(animation.native.keyframeCount)

proc duration*(animation: SkeletalAnimation): float32 =
  animation.requireAlive()
  float32(animation.native.duration)

proc isLooping*(animation: SkeletalAnimation): bool =
  animation.requireAlive()
  animation.native.isLooping

proc setLooping*(animation: SkeletalAnimation; looping: bool) =
  animation.requireAlive()
  animation.native.setLooping(looping)

proc scaleJoints*(animation: SkeletalAnimation; scale: float32) =
  animation.requireAlive()
  if not scale.isFinite or scale <= 0:
    raise newException(
      ValueError, "animation joint scale must be finite and positive")
  animation.native.scaleJoints(scale)
  for track in animation.tracks.mitems:
    for keyframe in track.keyframes.mitems:
      keyframe.transform.position.x *= scale
      keyframe.transform.position.y *= scale
      keyframe.transform.position.z *= scale

proc sampleAnimation(animation: SkeletalAnimation; time: float32;
                     modelSpace: bool): seq[SkeletonTransform] =
  animation.requireAlive()
  if not time.isFinite or time < 0:
    raise newException(
      ValueError, "animation sample time must be finite and non-negative")
  var positions = newSeq[raw.Vec3](animation.skeleton.joints.len)
  var rotations = newSeq[raw.Quat](animation.skeleton.joints.len)
  animation.native.sample(
    cfloat(time), modelSpace, addr positions[0], addr rotations[0])
  result = newSeq[SkeletonTransform](animation.skeleton.joints.len)
  for index in 0 ..< result.len:
    result[index] = skeletonTransform(
      fromRaw(positions[index]), fromRaw(rotations[index]))

proc sampleLocalPose*(animation: SkeletalAnimation;
                      time: float32): seq[SkeletonTransform] =
  ## Samples parent-relative joint transforms.
  animation.sampleAnimation(time, false)

proc sampleModelPose*(animation: SkeletalAnimation;
                      time: float32): seq[SkeletonTransform] =
  ## Samples skeleton-root-relative joint transforms.
  animation.sampleAnimation(time, true)

proc validateRagdollSkeleton(animation: SkeletalAnimation;
                             ragdoll: Ragdoll) =
  animation.requireAlive()
  ragdoll.requireAlive()
  if animation.skeleton.joints.len != ragdoll.config.parts.len:
    raise newException(
      ValueError, "animation skeleton must match the ragdoll part count")
  for index, joint in animation.skeleton.joints:
    if joint.name != ragdoll.config.parts[index].name or
        joint.parent != ragdoll.config.parts[index].joint.parent:
      raise newException(
        ValueError, "animation skeleton must match ragdoll names and hierarchy")

proc sampleRagdollLocalPose*(animation: SkeletalAnimation; ragdoll: Ragdoll;
                             time: float32): seq[RagdollTransform] =
  animation.validateRagdollSkeleton(ragdoll)
  let sampled = animation.sampleLocalPose(time)
  result = newSeq[RagdollTransform](sampled.len)
  for index, transform in sampled:
    result[index] = RagdollTransform(
      position: transform.position, rotation: transform.rotation)

proc driveMotors*(ragdoll: Ragdoll; animation: SkeletalAnimation;
                  time: float32) =
  ## Samples an animation and drives dynamic ragdoll joint motors.
  ragdoll.driveMotors(animation.sampleRagdollLocalPose(ragdoll, time))

proc driveMotors*(ragdoll: Ragdoll; animation: SkeletalAnimation;
                  previousTime, time, deltaTime: float32) =
  ## Drives position and angular velocity targets from two animation samples.
  let previousPose = animation.sampleRagdollLocalPose(ragdoll, previousTime)
  let pose = animation.sampleRagdollLocalPose(ragdoll, time)
  ragdoll.driveMotors(previousPose, pose, deltaTime)

proc capturePhysicsScene*(world: World): PhysicsScene =
  ## Captures rigid bodies, soft bodies and two-body constraints as reusable
  ## creation settings. Runtime solver caches and helper controllers are not
  ## part of PhysicsScene; use saveState for same-world rollback.
  world.requireOpen()
  new(result)
  if not raw.acquireJolt():
    raise newException(
      JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true
  result.native = raw.capturePhysicsScene(world.physics)
  if result.native.isNil:
    raw.releaseJolt()
    result.acquiredJolt = false
    raise newException(JoltError, "Jolt could not capture the physics scene")
  result.alive = true

proc newPhysicsScene*(): PhysicsScene =
  ## Creates an empty authoring scene that can be exported as a Jolt
  ## ObjectStream after supported ShapeSettings bodies have been added.
  new(result)
  if not raw.acquireJolt():
    raise newException(
      JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true
  result.native = raw.newPhysicsScene()
  if result.native.isNil:
    raw.releaseJolt()
    result.acquiredJolt = false
    raise newException(JoltError, "Jolt could not create a physics scene")
  result.alive = true

proc isAlive*(scene: PhysicsScene): bool =
  not scene.isNil and scene.alive and not scene.native.isNil

proc requireAlive(scene: PhysicsScene) =
  if not scene.isAlive:
    raise newException(JoltError, "Jolt physics scene is no longer alive")

proc validateAuthoredShape(shape: Shape; motionType: MotionType): Shape =
  case shape.kind
  of ShapeKind.Box:
    result = boxShape(shape.halfExtent, shape.convexRadius)
  of ShapeKind.Sphere:
    result = sphereShape(shape.radius)
  of ShapeKind.Capsule:
    result = capsuleShape(shape.halfHeight, shape.radius)
  of ShapeKind.Cylinder:
    result = cylinderShape(shape.halfHeight, shape.radius, shape.convexRadius)
  of ShapeKind.TaperedCapsule:
    result = taperedCapsuleShape(
      shape.halfHeight, shape.topRadius, shape.bottomRadius)
  of ShapeKind.TaperedCylinder:
    result = taperedCylinderShape(
      shape.halfHeight, shape.topRadius, shape.bottomRadius,
      shape.convexRadius)
  of ShapeKind.Triangle:
    if shape.points.len != 3:
      raise newException(ValueError, "triangle shapes require three vertices")
    result = triangleShape(
      shape.points[0], shape.points[1], shape.points[2], shape.convexRadius)
  of ShapeKind.Plane:
    if motionType != MotionType.Static:
      raise newException(ValueError, "plane scene bodies must be static")
    result = planeShape(
      shape.planeNormal, shape.planeConstant, shape.planeHalfExtent)
  of ShapeKind.Empty:
    result = emptyShape(shape.centerOfMass)
  of ShapeKind.ConvexHull:
    if shape.points.len > int(high(cint)):
      raise newException(ValueError, "convex hull point count must fit in int32")
    result = convexHullShape(shape.points, shape.convexRadius)
  of ShapeKind.TriangleMesh:
    if motionType != MotionType.Static:
      raise newException(ValueError, "triangle-mesh scene bodies must be static")
    result = triangleMeshShape(shape.vertices, shape.triangleIndices)
  of ShapeKind.HeightField:
    if motionType != MotionType.Static:
      raise newException(ValueError, "height-field scene bodies must be static")
    result = heightFieldShape(
      shape.heightSamples, int(shape.sampleCount),
      shape.heightOffset, shape.heightScale,
      shape.blockSize, shape.bitsPerSample)
  of ShapeKind.StaticCompound:
    var children = newSeq[CompoundChild](shape.children.len)
    for index, child in shape.children:
      children[index] = compoundChild(
        validateAuthoredShape(child.shape, motionType),
        child.position, child.rotation)
    result = staticCompoundShape(children)
  of ShapeKind.MutableCompound:
    var children = newSeq[CompoundChild](shape.children.len)
    for index, child in shape.children:
      children[index] = compoundChild(
        validateAuthoredShape(child.shape, motionType),
        child.position, child.rotation)
    result = mutableCompoundShape(children)
  of ShapeKind.Scaled:
    result = scaledShape(
      validateAuthoredShape(shape.innerShape, motionType), shape.shapeScale)
  of ShapeKind.RotatedTranslated:
    result = rotatedTranslatedShape(
      validateAuthoredShape(shape.innerShape, motionType),
      shape.shapePosition, shape.shapeRotation)
  of ShapeKind.OffsetCenterOfMass:
    result = offsetCenterOfMassShape(
      validateAuthoredShape(shape.innerShape, motionType),
      shape.centerOfMassOffset)
  if shape.material.isSome:
    result = result.withMaterial(shape.material.get)
  elif shape.materials.len > 0:
    result = result.withMaterials(shape.materials, shape.materialIndices)
  elif shape.materialIndices.len > 0:
    raise newException(
      ValueError, "material indices require authored physics materials")

proc authoredBodySettings(shape: Shape; position: Vec3; rotation: Quat;
                          motionType: MotionType;
                          layer: CollisionLayer): raw.BodyCreationSettings =
  case shape.kind
  of ShapeKind.Box, ShapeKind.Sphere, ShapeKind.Capsule,
      ShapeKind.Cylinder, ShapeKind.TaperedCapsule,
      ShapeKind.TaperedCylinder, ShapeKind.Triangle, ShapeKind.Plane,
      ShapeKind.Empty, ShapeKind.ConvexHull:
    var material: ptr raw.PhysicsMaterial
    if shape.material.isSome:
      material = cookMaterial(shape.material.get)
    defer:
      if not material.isNil:
        material.release()
    var nativePoints = newSeq[raw.Vec3](shape.points.len)
    for index, point in shape.points:
      nativePoints[index] = point.toRaw
    var points: ptr raw.Vec3
    if nativePoints.len > 0:
      points = addr nativePoints[0]
    result = raw.physicsScenePrimitiveBodySettings(
      uint8(ord(shape.kind)),
      shape.halfExtent.toRaw,
      shape.halfHeight,
      shape.radius,
      shape.topRadius,
      shape.bottomRadius,
      shape.convexRadius,
      points,
      uint32(nativePoints.len),
      shape.planeNormal.toRaw,
      shape.planeConstant,
      shape.planeHalfExtent,
      shape.centerOfMass.toRaw,
      material,
      position.toRaw,
      rotation.toRaw,
      motionType.toRaw,
      layer)
  of ShapeKind.TriangleMesh:
    let materials = cookMaterials(shape.materials)
    defer: materials.releaseMaterials()
    var vertices = newSeq[raw.Vec3](shape.vertices.len)
    for index, vertex in shape.vertices:
      vertices[index] = vertex.toRaw
    var materialPtr: ptr ptr raw.PhysicsMaterial
    var materialIndexPtr: ptr uint32
    if materials.len > 0:
      materialPtr = unsafeAddr materials[0]
      materialIndexPtr = unsafeAddr shape.materialIndices[0]
    result = raw.physicsSceneMeshBodySettings(
      addr vertices[0], uint32(vertices.len),
      unsafeAddr shape.triangleIndices[0],
      uint32(shape.triangleIndices.len div 3),
      materialPtr, uint32(materials.len), materialIndexPtr,
      position.toRaw, rotation.toRaw, motionType.toRaw, layer)
  of ShapeKind.HeightField:
    let materials = cookMaterials(shape.materials)
    defer: materials.releaseMaterials()
    var materialIndices = newSeq[uint8](shape.materialIndices.len)
    for index, materialIndex in shape.materialIndices:
      materialIndices[index] = uint8(materialIndex)
    var materialPtr: ptr ptr raw.PhysicsMaterial
    var materialIndexPtr: ptr uint8
    if materials.len > 0:
      materialPtr = unsafeAddr materials[0]
      materialIndexPtr = addr materialIndices[0]
    result = raw.physicsSceneHeightFieldBodySettings(
      unsafeAddr shape.heightSamples[0], shape.sampleCount,
      shape.heightOffset.toRaw, shape.heightScale.toRaw,
      shape.blockSize, shape.bitsPerSample,
      materialIndexPtr, materialPtr, uint32(materials.len),
      position.toRaw, rotation.toRaw, motionType.toRaw, layer)
  of ShapeKind.StaticCompound, ShapeKind.MutableCompound:
    let compound = raw.newPhysicsSceneCompoundSettings(
      shape.kind == ShapeKind.MutableCompound)
    if compound.isNil:
      raise newException(JoltError, "Jolt could not allocate compound settings")
    defer: compound.delete()
    for child in shape.children:
      let childSettings = authoredBodySettings(
        child.shape, vec3(0, 0, 0), quatIdentity(),
        MotionType.Static, nonMovingLayer)
      if not compound.addChild(
          childSettings, child.position.toRaw, child.rotation.toRaw):
        raise newException(
          JoltError, "Jolt could not add an authored compound child")
    result = raw.physicsSceneCompoundBodySettings(
      compound, position.toRaw, rotation.toRaw, motionType.toRaw, layer)
  of ShapeKind.Scaled, ShapeKind.RotatedTranslated,
      ShapeKind.OffsetCenterOfMass:
    let childSettings = authoredBodySettings(
      shape.innerShape, vec3(0, 0, 0), quatIdentity(),
      MotionType.Static, nonMovingLayer)
    let decoratorKind = case shape.kind
      of ShapeKind.Scaled: 0'u8
      of ShapeKind.RotatedTranslated: 1'u8
      else: 2'u8
    result = raw.physicsSceneDecoratedBodySettings(
      childSettings, decoratorKind,
      shape.shapeScale.toRaw, shape.shapePosition.toRaw,
      shape.shapeRotation.toRaw, shape.centerOfMassOffset.toRaw,
      position.toRaw, rotation.toRaw, motionType.toRaw, layer)

proc addBody*(scene: PhysicsScene; spec: BodySpec): int =
  ## Adds one authoring-time BodyCreationSettings entry. The returned index is
  ## stable within the PhysicsScene and is not a live World BodyID.
  scene.requireAlive()
  spec.position.requireFinite("position")
  spec.config.validate(spec.motionType)
  let shape = validateAuthoredShape(spec.shape, spec.motionType)
  var settings = authoredBodySettings(
    shape, spec.position, spec.rotation.normalized,
    spec.motionType, spec.layer)
  settings.setSensor(spec.sensor)
  settings.configureBodySettings(spec.config)
  let index = scene.native.addBody(settings)
  if index == high(uint32):
    raise newException(JoltError, "Jolt could not add the authored scene body")
  int(index)

proc addBodies*(scene: PhysicsScene; specs: openArray[BodySpec]): seq[int] =
  ## Adds authoring bodies in order after validating the entire batch.
  scene.requireAlive()
  if specs.len > int(high(uint32)) - int(scene.native.bodyCount):
    raise newException(ValueError, "authored physics scene body batch is too large")
  for spec in specs:
    spec.position.requireFinite("position")
    spec.config.validate(spec.motionType)
    discard spec.rotation.normalized
    discard validateAuthoredShape(spec.shape, spec.motionType)
  result = newSeq[int](specs.len)
  for index, spec in specs:
    result[index] = scene.addBody(spec)

proc newPhysicsScene*(specs: openArray[BodySpec]): PhysicsScene =
  ## Creates an ObjectStream-ready authoring scene from validated body specs.
  result = newPhysicsScene()
  try:
    discard result.addBodies(specs)
  except:
    result.close()
    raise

proc authoredConstraintBodies(scene: PhysicsScene; body1, body2: int):
    tuple[first, second: uint32] =
  scene.requireAlive()
  let count = int(scene.native.bodyCount)
  if body1 < fixedWorldBodyIndex or body1 >= count or
      body2 < fixedWorldBodyIndex or body2 >= count:
    raise newException(
      ValueError, "authored constraint body index is out of bounds")
  if body1 == body2:
    raise newException(
      ValueError, "an authored constraint requires two different bodies")
  result.first = if body1 == fixedWorldBodyIndex:
      high(uint32)
    else:
      uint32(body1)
  result.second = if body2 == fixedWorldBodyIndex:
      high(uint32)
    else:
      uint32(body2)

proc authoredUnitAxis(axis: Vec3; name: string): Vec3 =
  axis.requireFinite(name)
  let lengthSquared = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, name & " must have non-zero length")
  let inverseLength = 1.0'f32 / sqrt(lengthSquared)
  Vec3(
    x: axis.x * inverseLength,
    y: axis.y * inverseLength,
    z: axis.z * inverseLength)

proc finishAuthoredConstraint(scene: PhysicsScene; added: bool): int =
  if not added:
    raise newException(JoltError, "Jolt could not add the authored constraint")
  int(scene.native.constraintCount) - 1

proc addPointConstraint*(scene: PhysicsScene; body1, body2: int;
                         localPoint1, localPoint2: Vec3): int =
  ## Points are relative to each body's center of mass.
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  scene.finishAuthoredConstraint(scene.native.addPointConstraint(
    bodies.first, bodies.second, localPoint1.toRaw, localPoint2.toRaw))

proc addDistanceConstraint*(scene: PhysicsScene; body1, body2: int;
                            localPoint1, localPoint2: Vec3;
                            minDistance, maxDistance: float32): int =
  ## Points are relative to each body's center of mass.
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  if not minDistance.isFinite or not maxDistance.isFinite or
      minDistance < 0 or minDistance > maxDistance:
    raise newException(
      ValueError,
      "constraint distances must be finite, non-negative, and ordered")
  scene.finishAuthoredConstraint(scene.native.addDistanceConstraint(
    bodies.first, bodies.second,
    localPoint1.toRaw, localPoint2.toRaw, minDistance, maxDistance))

proc addFixedConstraint*(scene: PhysicsScene; body1, body2: int): int =
  let bodies = scene.authoredConstraintBodies(body1, body2)
  scene.finishAuthoredConstraint(
    scene.native.addFixedConstraint(bodies.first, bodies.second))

proc addFixedConstraint*(scene: PhysicsScene; body1, body2: int;
                         localPoint1, localPoint2,
                         localAxisX1, localAxisY1,
                         localAxisX2, localAxisY2: Vec3): int =
  ## Creates a fixed constraint from explicit local center-of-mass frames.
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  let axisX1 = authoredUnitAxis(localAxisX1, "localAxisX1")
  let axisY1 = authoredUnitAxis(localAxisY1, "localAxisY1")
  let axisX2 = authoredUnitAxis(localAxisX2, "localAxisX2")
  let axisY2 = authoredUnitAxis(localAxisY2, "localAxisY2")
  let dot1 = axisX1.x * axisY1.x +
    axisX1.y * axisY1.y + axisX1.z * axisY1.z
  let dot2 = axisX2.x * axisY2.x +
    axisX2.y * axisY2.y + axisX2.z * axisY2.z
  if abs(dot1) > 1.0e-4'f32 or abs(dot2) > 1.0e-4'f32:
    raise newException(ValueError, "fixed constraint frame axes must be perpendicular")
  scene.finishAuthoredConstraint(scene.native.addFixedConstraint(
    bodies.first, bodies.second, localPoint1.toRaw, localPoint2.toRaw,
    axisX1.toRaw, axisY1.toRaw, axisX2.toRaw, axisY2.toRaw))

proc addHingeConstraint*(scene: PhysicsScene; body1, body2: int;
                         localPoint1, localPoint2, localAxis1,
                         localAxis2: Vec3;
                         minAngle = -PI.float32;
                         maxAngle = PI.float32): int =
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  let axis1 = authoredUnitAxis(localAxis1, "localAxis1")
  let axis2 = authoredUnitAxis(localAxis2, "localAxis2")
  if not minAngle.isFinite or not maxAngle.isFinite or
      minAngle < -PI.float32 or minAngle > 0 or
      maxAngle < 0 or maxAngle > PI.float32:
    raise newException(
      ValueError,
      "hinge limits must be finite with -PI <= minimum <= 0 <= maximum <= PI")
  scene.finishAuthoredConstraint(scene.native.addHingeConstraint(
    bodies.first, bodies.second,
    localPoint1.toRaw, localPoint2.toRaw,
    axis1.toRaw, axis2.toRaw, minAngle, maxAngle))

proc addSliderConstraint*(scene: PhysicsScene; body1, body2: int;
                          localPoint1, localPoint2, localAxis: Vec3;
                          minPosition, maxPosition: float32): int =
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  let axis = authoredUnitAxis(localAxis, "localAxis")
  if not minPosition.isFinite or not maxPosition.isFinite or
      minPosition > 0 or maxPosition < 0:
    raise newException(
      ValueError,
      "slider limits must be finite with minimum <= 0 <= maximum")
  scene.finishAuthoredConstraint(scene.native.addSliderConstraint(
    bodies.first, bodies.second,
    localPoint1.toRaw, localPoint2.toRaw, axis.toRaw,
    minPosition, maxPosition))

proc addConeConstraint*(scene: PhysicsScene; body1, body2: int;
                        localPoint1, localPoint2,
                        localTwistAxis1, localTwistAxis2: Vec3;
                        halfConeAngle: float32): int =
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  let axis1 = authoredUnitAxis(localTwistAxis1, "localTwistAxis1")
  let axis2 = authoredUnitAxis(localTwistAxis2, "localTwistAxis2")
  if not halfConeAngle.isFinite or
      halfConeAngle < 0 or halfConeAngle > PI.float32:
    raise newException(ValueError, "halfConeAngle must be finite and in [0, PI]")
  scene.finishAuthoredConstraint(scene.native.addConeConstraint(
    bodies.first, bodies.second,
    localPoint1.toRaw, localPoint2.toRaw,
    axis1.toRaw, axis2.toRaw, halfConeAngle))

proc addSwingTwistConstraint*(
    scene: PhysicsScene; body1, body2: int;
    localPoint1, localPoint2, localTwistAxis, localPlaneAxis: Vec3;
    normalHalfConeAngle, planeHalfConeAngle,
    twistMinAngle, twistMaxAngle: float32): int =
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  let twistAxis = authoredUnitAxis(localTwistAxis, "localTwistAxis")
  let planeAxis = authoredUnitAxis(localPlaneAxis, "localPlaneAxis")
  let dot = twistAxis.x * planeAxis.x +
    twistAxis.y * planeAxis.y + twistAxis.z * planeAxis.z
  if abs(dot) > 1.0e-4'f32:
    raise newException(ValueError, "swing-twist axes must be perpendicular")
  for angle in [normalHalfConeAngle, planeHalfConeAngle]:
    if not angle.isFinite or angle < 0 or angle > PI.float32:
      raise newException(
        ValueError, "swing half-cone angles must be finite and in [0, PI]")
  if not twistMinAngle.isFinite or not twistMaxAngle.isFinite or
      twistMinAngle < -PI.float32 or twistMaxAngle > PI.float32 or
      twistMinAngle > twistMaxAngle:
    raise newException(
      ValueError, "twist limits must be finite, ordered, and within [-PI, PI]")
  scene.finishAuthoredConstraint(scene.native.addSwingTwistConstraint(
    bodies.first, bodies.second,
    localPoint1.toRaw, localPoint2.toRaw,
    twistAxis.toRaw, planeAxis.toRaw,
    normalHalfConeAngle, planeHalfConeAngle,
    twistMinAngle, twistMaxAngle))

proc addSixDOFConstraint*(scene: PhysicsScene; body1, body2: int;
                          localPoint1, localPoint2: Vec3;
                          config = defaultSixDOFConfig()): int =
  let bodies = scene.authoredConstraintBodies(body1, body2)
  localPoint1.requireFinite("localPoint1")
  localPoint2.requireFinite("localPoint2")
  let axisX = authoredUnitAxis(config.axisX, "SixDOF axisX")
  let axisY = authoredUnitAxis(config.axisY, "SixDOF axisY")
  let dot = axisX.x * axisY.x + axisX.y * axisY.y + axisX.z * axisY.z
  if abs(dot) > 1.0e-4'f32:
    raise newException(ValueError, "SixDOF axes must be perpendicular")
  config.validateLimits()
  var limitMin, limitMax: array[6, cfloat]
  for axis in SixDOFAxis:
    let limit = config.limits[axis]
    case limit.mode
    of SixDOFAxisMode.AxisFree:
      limitMin[ord(axis)] = -maxFiniteFloat32
      limitMax[ord(axis)] = maxFiniteFloat32
    of SixDOFAxisMode.AxisFixed:
      limitMin[ord(axis)] = maxFiniteFloat32
      limitMax[ord(axis)] = -maxFiniteFloat32
    of SixDOFAxisMode.AxisLimited:
      limitMin[ord(axis)] = limit.minimum
      limitMax[ord(axis)] = limit.maximum
  scene.finishAuthoredConstraint(scene.native.addSixDOFConstraint(
    bodies.first, bodies.second,
    localPoint1.toRaw, localPoint2.toRaw,
    axisX.toRaw, axisY.toRaw,
    uint8(ord(config.swingType)),
    addr limitMin[0], addr limitMax[0]))

proc addGearConstraint*(scene: PhysicsScene; body1, body2: int;
                        localAxis1, localAxis2: Vec3;
                        ratio: float32): int =
  ## Axes are relative to each body's center of mass. ObjectStream does not
  ## preserve the optional companion hinge references used for drift repair.
  let bodies = scene.authoredConstraintBodies(body1, body2)
  if bodies.first == high(uint32) or bodies.second == high(uint32):
    raise newException(ValueError, "authored gear bodies must both be dynamic")
  let axis1 = authoredUnitAxis(localAxis1, "localAxis1")
  let axis2 = authoredUnitAxis(localAxis2, "localAxis2")
  if not ratio.isFinite or ratio <= 0:
    raise newException(ValueError, "gear ratio must be finite and positive")
  scene.finishAuthoredConstraint(scene.native.addGearConstraint(
    bodies.first, bodies.second, axis1.toRaw, axis2.toRaw, ratio))

proc addPulleyConstraint*(scene: PhysicsScene; body1, body2: int;
                          bodyPoint1, fixedPoint1,
                          bodyPoint2, fixedPoint2: Vec3;
                          ratio = 1.0'f32;
                          minLength = 0.0'f32;
                          maxLength = -1.0'f32): int =
  ## Attachment and fixed points are world-space authoring coordinates.
  let bodies = scene.authoredConstraintBodies(body1, body2)
  bodyPoint1.requireFinite("bodyPoint1")
  fixedPoint1.requireFinite("fixedPoint1")
  bodyPoint2.requireFinite("bodyPoint2")
  fixedPoint2.requireFinite("fixedPoint2")
  if not ratio.isFinite or ratio <= 0:
    raise newException(ValueError, "pulley ratio must be finite and positive")
  if not minLength.isFinite or minLength < 0 or
      not maxLength.isFinite or
      (maxLength != -1.0'f32 and maxLength < minLength):
    raise newException(
      ValueError,
      "pulley lengths must use 0 <= minimum <= maximum, or maximum = -1")
  scene.finishAuthoredConstraint(scene.native.addPulleyConstraint(
    bodies.first, bodies.second,
    bodyPoint1.toRaw, fixedPoint1.toRaw,
    bodyPoint2.toRaw, fixedPoint2.toRaw,
    ratio, minLength, maxLength))

proc addRackAndPinionConstraint*(scene: PhysicsScene;
                                 pinionBody, rackBody: int;
                                 localHingeAxis,
                                 localSliderAxis: Vec3;
                                 ratio: float32): int =
  ## Axes are relative to their bodies' centers of mass. ObjectStream does not
  ## preserve the optional companion hinge/slider references for drift repair.
  let bodies = scene.authoredConstraintBodies(pinionBody, rackBody)
  if bodies.first == high(uint32) or bodies.second == high(uint32):
    raise newException(
      ValueError, "authored rack-and-pinion bodies must both be dynamic")
  let hingeAxis = authoredUnitAxis(localHingeAxis, "localHingeAxis")
  let sliderAxis = authoredUnitAxis(localSliderAxis, "localSliderAxis")
  if not ratio.isFinite or ratio <= 0:
    raise newException(
      ValueError, "rack-and-pinion ratio must be finite and positive")
  scene.finishAuthoredConstraint(scene.native.addRackAndPinionConstraint(
    bodies.first, bodies.second,
    hingeAxis.toRaw, sliderAxis.toRaw, ratio))

proc addPathConstraint*(scene: PhysicsScene; pathBody, movingBody: int;
                        points: openArray[PathPoint];
                        pathPosition = vec3(0, 0, 0);
                        pathRotation = quatIdentity();
                        pathFraction = 0.0'f32;
                        looping = false;
                        rotationConstraint =
                          PathRotationConstraintType.PathRotationFree;
                        maxFrictionForce = 0.0'f32): int =
  ## Adds a serializable Hermite path relative to pathBody's transform.
  let bodies = scene.authoredConstraintBodies(pathBody, movingBody)
  if points.len < (if looping: 3 else: 2):
    raise newException(
      ValueError,
      if looping:
        "a looping Hermite path requires at least three points"
      else:
        "a Hermite path requires at least two points")
  if uint64(points.len) > uint64(high(uint32)):
    raise newException(ValueError, "path point count must fit in uint32")
  pathPosition.requireFinite("pathPosition")
  let normalizedRotation = pathRotation.normalized
  let maximumFraction = if looping:
      float32(points.len)
    else:
      float32(points.len - 1)
  if not pathFraction.isFinite or pathFraction < 0 or
      pathFraction > maximumFraction:
    raise newException(ValueError, "path fraction is outside the path")
  if not maxFrictionForce.isFinite or maxFrictionForce < 0:
    raise newException(
      ValueError, "path friction force must be finite and non-negative")

  var positions = newSeq[raw.Vec3](points.len)
  var tangents = newSeq[raw.Vec3](points.len)
  var normals = newSeq[raw.Vec3](points.len)
  for index, point in points:
    point.position.requireFinite("path point position")
    let tangent = authoredUnitAxis(point.tangent, "path point tangent")
    let normal = authoredUnitAxis(point.normal, "path point normal")
    let dot = tangent.x * normal.x +
      tangent.y * normal.y + tangent.z * normal.z
    if abs(dot) > 1.0e-4'f32:
      raise newException(ValueError, "path point axes must be perpendicular")
    positions[index] = point.position.toRaw
    tangents[index] = point.tangent.toRaw
    normals[index] = normal.toRaw
  if looping:
    let first = points[0].position
    let last = points[^1].position
    let dx = last.x - first.x
    let dy = last.y - first.y
    let dz = last.z - first.z
    if dx * dx + dy * dy + dz * dz <= 1.0e-12'f32:
      raise newException(
        ValueError, "a looping path must not repeat its first point at the end")

  scene.finishAuthoredConstraint(scene.native.addPathConstraint(
    bodies.first, bodies.second,
    addr positions[0], addr tangents[0], addr normals[0],
    uint32(points.len), looping, pathPosition.toRaw,
    normalizedRotation.toRaw, pathFraction, maxFrictionForce,
    uint8(ord(rotationConstraint))))

proc requireConstraint(scene: PhysicsScene; constraintIndex: int) =
  scene.requireAlive()
  if constraintIndex < 0 or constraintIndex >= int(scene.native.constraintCount):
    raise newException(
      IndexDefect, "physics scene constraint index is out of bounds")

proc validateAuthoredFriction(value: float32; name: string) =
  if not value.isFinite or value < 0:
    raise newException(
      ValueError, name & " must be finite and non-negative")

proc configureConstraint*(scene: PhysicsScene; constraintIndex: int;
                          config: AuthoredConstraintConfig) =
  ## Applies common creation settings before serialization or instantiation.
  scene.requireConstraint(constraintIndex)
  if config.velocityStepsOverride >= 256 or
      config.positionStepsOverride >= 256:
    raise newException(
      ValueError, "constraint solver step overrides must be below 256")
  if not config.drawSize.isFinite or config.drawSize <= 0:
    raise newException(
      ValueError, "constraint draw size must be finite and positive")
  if not scene.native.configureConstraint(
      uint32(constraintIndex), config.enabled, config.priority,
      config.velocityStepsOverride, config.positionStepsOverride,
      config.drawSize, config.userData):
    raise newException(
      JoltError,
      "Jolt could not configure the authored constraint settings")

proc configureAuthoredHingeTuning*(scene: PhysicsScene;
                                   constraintIndex: int;
                                   maximumFrictionTorque: float32;
                                   limitSpring: SpringSettings) =
  ## Sets serialized hinge friction and limit-spring creation settings.
  scene.requireConstraint(constraintIndex)
  maximumFrictionTorque.validateAuthoredFriction("hinge friction torque")
  limitSpring.validate()
  if not scene.native.configureHingeTuning(
      uint32(constraintIndex), maximumFrictionTorque,
      uint8(ord(limitSpring.mode)), limitSpring.value,
      limitSpring.damping):
    raise newException(
      ValueError, "constraint index does not refer to unique hinge settings")

proc configureAuthoredDistanceSpring*(scene: PhysicsScene;
                                      constraintIndex: int;
                                      settings: SpringSettings) =
  ## Sets serialized soft-limit spring settings for a distance constraint.
  scene.requireConstraint(constraintIndex)
  settings.validate()
  if not scene.native.configureDistanceSpring(
      uint32(constraintIndex), uint8(ord(settings.mode)),
      settings.value, settings.damping):
    raise newException(
      ValueError,
      "constraint index does not refer to unique distance settings")

proc configureAuthoredSliderTuning*(scene: PhysicsScene;
                                    constraintIndex: int;
                                    maximumFrictionForce: float32;
                                    limitSpring: SpringSettings) =
  ## Sets serialized slider friction and limit-spring creation settings.
  scene.requireConstraint(constraintIndex)
  maximumFrictionForce.validateAuthoredFriction("slider friction force")
  limitSpring.validate()
  if not scene.native.configureSliderTuning(
      uint32(constraintIndex), maximumFrictionForce,
      uint8(ord(limitSpring.mode)), limitSpring.value,
      limitSpring.damping):
    raise newException(
      ValueError, "constraint index does not refer to unique slider settings")

proc setAuthoredSwingTwistFriction*(scene: PhysicsScene;
                                    constraintIndex: int;
                                    maximumFrictionTorque: float32) =
  ## Sets serialized swing-twist friction creation settings.
  scene.requireConstraint(constraintIndex)
  maximumFrictionTorque.validateAuthoredFriction(
    "swing-twist friction torque")
  if not scene.native.setSwingTwistFriction(
      uint32(constraintIndex), maximumFrictionTorque):
    raise newException(
      ValueError,
      "constraint index does not refer to unique swing-twist settings")

proc setAuthoredSixDOFFriction*(scene: PhysicsScene; constraintIndex: int;
                                axis: SixDOFAxis;
                                maximumFriction: float32) =
  ## Sets serialized SixDOF friction for one translation or rotation axis.
  scene.requireConstraint(constraintIndex)
  maximumFriction.validateAuthoredFriction("SixDOF friction")
  if not scene.native.setSixDOFFriction(
      uint32(constraintIndex), uint8(ord(axis)), maximumFriction):
    raise newException(
      ValueError, "constraint index does not refer to unique SixDOF settings")

proc setAuthoredSixDOFTranslationSpring*(
    scene: PhysicsScene; constraintIndex: int; axis: SixDOFAxis;
    settings: SpringSettings) =
  ## Sets a serialized SixDOF soft-limit spring on a translation axis.
  scene.requireConstraint(constraintIndex)
  if axis > SixDOFAxis.TranslationZ:
    raise newException(
      ValueError, "SixDOF limit springs only support translation axes")
  settings.validate()
  if not scene.native.setSixDOFTranslationSpring(
      uint32(constraintIndex), uint8(ord(axis)), uint8(ord(settings.mode)),
      settings.value, settings.damping):
    raise newException(
      ValueError, "constraint index does not refer to unique SixDOF settings")

proc configureAuthoredMotorSettings(
    scene: PhysicsScene; constraintIndex: int; motorKind, axis: uint8;
    settings: MotorSettings; family: string) =
  scene.requireConstraint(constraintIndex)
  settings.validate()
  if not scene.native.configureMotor(
      uint32(constraintIndex), motorKind, axis,
      uint8(ord(settings.spring.mode)), settings.spring.value,
      settings.spring.damping, settings.minForce, settings.maxForce,
      settings.minTorque, settings.maxTorque):
    raise newException(
      ValueError,
      "constraint index does not refer to unique " & family & " settings")

proc configureAuthoredHingeMotor*(scene: PhysicsScene; constraintIndex: int;
                                  settings: MotorSettings) =
  ## Sets serialized hinge motor tuning; state and targets are runtime state.
  scene.configureAuthoredMotorSettings(
    constraintIndex, 0, 0, settings, "hinge")

proc configureAuthoredSliderMotor*(scene: PhysicsScene; constraintIndex: int;
                                   settings: MotorSettings) =
  ## Sets serialized slider motor tuning; state and targets are runtime state.
  scene.configureAuthoredMotorSettings(
    constraintIndex, 1, 0, settings, "slider")

proc configureAuthoredSwingMotor*(scene: PhysicsScene; constraintIndex: int;
                                  settings: MotorSettings) =
  ## Sets serialized swing motor tuning on a SwingTwist constraint.
  scene.configureAuthoredMotorSettings(
    constraintIndex, 2, 0, settings, "swing-twist")

proc configureAuthoredTwistMotor*(scene: PhysicsScene; constraintIndex: int;
                                  settings: MotorSettings) =
  ## Sets serialized twist motor tuning on a SwingTwist constraint.
  scene.configureAuthoredMotorSettings(
    constraintIndex, 3, 0, settings, "swing-twist")

proc configureAuthoredSixDOFMotor*(scene: PhysicsScene; constraintIndex: int;
                                   axis: SixDOFAxis;
                                   settings: MotorSettings) =
  ## Sets serialized SixDOF motor tuning for one axis.
  scene.configureAuthoredMotorSettings(
    constraintIndex, 4, uint8(ord(axis)), settings, "SixDOF")

proc configureAuthoredPathMotor*(scene: PhysicsScene; constraintIndex: int;
                                 settings: MotorSettings) =
  ## Sets serialized Hermite-path position motor tuning.
  scene.configureAuthoredMotorSettings(
    constraintIndex, 5, 0, settings, "path")

proc rigidBodyCount*(scene: PhysicsScene): int =
  scene.requireAlive()
  int(scene.native.bodyCount)

proc softBodyCount*(scene: PhysicsScene): int =
  scene.requireAlive()
  int(scene.native.softBodyCount)

proc totalBodyCount*(scene: PhysicsScene): int =
  scene.rigidBodyCount + scene.softBodyCount

proc constraintCount*(scene: PhysicsScene): int =
  scene.requireAlive()
  int(scene.native.constraintCount)

proc fixInvalidScales*(scene: PhysicsScene): bool =
  scene.requireAlive()
  scene.native.fixInvalidScales()

proc writeUint32LE(data: var seq[byte]; offset: int; value: uint32) =
  for shift in 0 ..< 4:
    data[offset + shift] = byte(value shr (shift * 8))

proc writeUint64LE(data: var seq[byte]; offset: int; value: uint64) =
  for shift in 0 ..< 8:
    data[offset + shift] = byte(value shr (shift * 8))

proc readUint32LE(data: openArray[byte]; offset: int): uint32 =
  for shift in 0 ..< 4:
    result = result or (uint32(data[offset + shift]) shl (shift * 8))

proc readUint64LE(data: openArray[byte]; offset: int): uint64 =
  for shift in 0 ..< 8:
    result = result or (uint64(data[offset + shift]) shl (shift * 8))

proc dataChecksum(data: openArray[byte]; offset, size: int): uint64 =
  result = fnvOffsetBasis64
  for index in offset ..< offset + size:
    result = (result xor uint64(data[index])) * fnvPrime64

proc serialize*(animation: SkeletalAnimation): seq[byte] =
  ## Saves Jolt's native binary animation state in a checked envelope. The
  ## payload requires a compatible Jolt build and the original skeleton.
  animation.requireAlive()
  if not animation.native.serialize():
    raise newException(JoltError, "Jolt could not serialize the animation")
  let size = animation.native.serializedSize
  if size == 0 or uint64(size) >
      uint64(maxAnimationSerializedBytes - skeletalAnimationHeaderSize):
    raise newException(JoltError, "serialized animation has an invalid size")
  result = newSeq[byte](skeletalAnimationHeaderSize + int(size))
  for index, value in skeletalAnimationMagic:
    result[index] = value
  result.writeUint32LE(8, skeletalAnimationFormatVersion)
  result.writeUint64LE(12, uint64(size))
  animation.native.copySerializedData(
    addr result[skeletalAnimationHeaderSize])
  result.writeUint64LE(
    20, result.dataChecksum(skeletalAnimationHeaderSize, int(size)))

proc restoreSkeletalAnimation*(skeleton: SkeletonDefinition;
                               data: openArray[byte]): SkeletalAnimation =
  ## Restores Jolt's native binary animation and binds it to a checked skeleton.
  skeleton.validateSkeleton("animation restore")
  if data.len == 0:
    raise newException(ValueError, "serialized animation must not be empty")
  if data.len < skeletalAnimationHeaderSize:
    raise newException(ValueError, "serialized animation header is truncated")
  if data.len > maxAnimationSerializedBytes:
    raise newException(ValueError, "serialized animation is too large")
  for index, value in skeletalAnimationMagic:
    if data[index] != value:
      raise newException(ValueError, "serialized animation has an invalid header")
  if data.readUint32LE(8) != skeletalAnimationFormatVersion:
    raise newException(
      ValueError, "serialized animation uses an unsupported format version")
  let payloadSize = data.readUint64LE(12)
  if payloadSize == 0 or payloadSize >
      uint64(maxAnimationSerializedBytes - skeletalAnimationHeaderSize) or
      payloadSize != uint64(data.len - skeletalAnimationHeaderSize):
    raise newException(ValueError, "serialized animation has an invalid size")
  if data.readUint64LE(20) !=
      data.dataChecksum(skeletalAnimationHeaderSize, int(payloadSize)):
    raise newException(ValueError, "serialized animation checksum failed")

  var jointNames: seq[cstring]
  var jointParents: seq[int32]
  var neutralPositions: seq[raw.Vec3]
  var neutralRotations: seq[raw.Quat]
  skeleton.nativeSkeleton(
    jointNames, jointParents, neutralPositions, neutralRotations)
  new(result)
  if not raw.acquireJolt():
    raise newException(
      JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true
  result.native = raw.restoreSkeletalAnimation(
    addr jointNames[0], addr jointParents[0], addr neutralPositions[0],
    addr neutralRotations[0], uint32(jointNames.len),
    cast[ptr uint8](unsafeAddr data[skeletalAnimationHeaderSize]),
    csize_t(payloadSize))
  if result.native.isNil:
    raw.releaseJolt()
    result.acquiredJolt = false
    raise newException(
      ValueError, "serialized animation is invalid or mismatched with the skeleton")
  result.skeleton = skeleton.copySkeleton()
  result.alive = true

proc serialize*(scene: PhysicsScene): seq[byte] =
  ## Includes shapes, materials and collision-group filters. A small checked
  ## envelope rejects truncated or accidentally corrupted data before Jolt
  ## reads it. The payload still requires a matching Jolt version and build.
  scene.requireAlive()
  if not scene.native.serialize():
    raise newException(JoltError, "Jolt could not serialize the physics scene")
  let size = scene.native.serializedSize
  if size == 0 or uint64(size) >
      uint64(maxSceneSerializedBytes - physicsSceneHeaderSize):
    raise newException(JoltError, "serialized physics scene has an invalid size")
  result = newSeq[byte](physicsSceneHeaderSize + int(size))
  for index, value in physicsSceneMagic:
    result[index] = value
  result.writeUint32LE(8, physicsSceneFormatVersion)
  result.writeUint64LE(12, uint64(size))
  scene.native.copySerializedData(addr result[physicsSceneHeaderSize])
  result.writeUint64LE(
    20, result.dataChecksum(physicsSceneHeaderSize, int(size)))

proc objectStreamSerializable*(scene: PhysicsScene): bool =
  ## Reports whether the scene retains authored ShapeSettings for every rigid
  ## body and can therefore be written as a Jolt ObjectStream.
  scene.requireAlive()
  scene.native.objectStreamSerializable()

proc serializeObjectStream*(scene: PhysicsScene;
                            format = PhysicsSceneStreamText): seq[byte] =
  ## Serializes the RTTI-described Jolt ObjectStream representation. Text is
  ## inspectable and Binary is smaller; both are auto-detected when restored.
  ## Unlike serialize(), this is intended for authored/exchange data rather
  ## than compact same-build state transfer.
  scene.requireAlive()
  if not scene.native.objectStreamSerializable():
    raise newException(
      JoltError,
      "physics scene does not retain the ShapeSettings required by ObjectStream")
  if not scene.native.serializeObjectStream(
      format == PhysicsSceneStreamBinary):
    raise newException(
      JoltError,
      "Jolt ObjectStream support is unavailable or serialization failed")
  let size = scene.native.serializedSize
  if size == 0 or uint64(size) > uint64(maxSceneSerializedBytes):
    raise newException(
      JoltError, "serialized physics scene ObjectStream has an invalid size")
  result = newSeq[byte](int(size))
  scene.native.copySerializedData(addr result[0])

proc serializeObjectStreamText*(scene: PhysicsScene): string =
  let data = scene.serializeObjectStream(PhysicsSceneStreamText)
  result = newString(data.len)
  if data.len > 0:
    copyMem(addr result[0], unsafeAddr data[0], data.len)

proc restorePhysicsScenePayload(data: pointer; size: int): PhysicsScene =
  new(result)
  if not raw.acquireJolt():
    raise newException(
      JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true
  result.native = raw.restorePhysicsScene(
    cast[ptr uint8](data), csize_t(size))
  if result.native.isNil:
    raw.releaseJolt()
    result.acquiredJolt = false
    raise newException(ValueError, "serialized physics scene is invalid")
  result.alive = true

proc restorePhysicsSceneObjectStreamPayload(
    data: pointer; size: int): PhysicsScene =
  new(result)
  if not raw.acquireJolt():
    raise newException(
      JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true
  result.native = raw.restorePhysicsSceneObjectStream(
    cast[ptr uint8](data), csize_t(size))
  if result.native.isNil:
    raw.releaseJolt()
    result.acquiredJolt = false
    raise newException(
      ValueError,
      "physics scene ObjectStream is invalid or unsupported by this Jolt build")
  result.alive = true

proc restorePhysicsScene*(data: openArray[byte]): PhysicsScene =
  if data.len == 0:
    raise newException(ValueError, "serialized physics scene must not be empty")
  if data.len < physicsSceneHeaderSize:
    raise newException(ValueError, "serialized physics scene header is truncated")
  if data.len > maxSceneSerializedBytes:
    raise newException(ValueError, "serialized physics scene is too large")
  for index, value in physicsSceneMagic:
    if data[index] != value:
      raise newException(ValueError, "serialized physics scene has an invalid header")
  if data.readUint32LE(8) != physicsSceneFormatVersion:
    raise newException(
      ValueError, "serialized physics scene uses an unsupported format version")
  let payloadSize = data.readUint64LE(12)
  if payloadSize == 0 or payloadSize >
      uint64(maxSceneSerializedBytes - physicsSceneHeaderSize) or
      payloadSize != uint64(data.len - physicsSceneHeaderSize):
    raise newException(ValueError, "serialized physics scene has an invalid size")
  if data.readUint64LE(20) !=
      data.dataChecksum(physicsSceneHeaderSize, int(payloadSize)):
    raise newException(ValueError, "serialized physics scene checksum failed")
  restorePhysicsScenePayload(
    unsafeAddr data[physicsSceneHeaderSize], int(payloadSize))

proc restorePhysicsSceneObjectStream*(data: openArray[byte]): PhysicsScene =
  if data.len == 0:
    raise newException(
      ValueError, "physics scene ObjectStream must not be empty")
  if data.len > maxSceneSerializedBytes:
    raise newException(ValueError, "physics scene ObjectStream is too large")
  restorePhysicsSceneObjectStreamPayload(unsafeAddr data[0], data.len)

proc restorePhysicsSceneObjectStream*(data: string): PhysicsScene =
  if data.len == 0:
    raise newException(
      ValueError, "physics scene ObjectStream must not be empty")
  if data.len > maxSceneSerializedBytes:
    raise newException(ValueError, "physics scene ObjectStream is too large")
  restorePhysicsSceneObjectStreamPayload(unsafeAddr data[0], data.len)

proc instantiate*(scene: PhysicsScene; world: World): PhysicsSceneInstance =
  ## Instantiates the scene as one owned group in an existing world. Closing
  ## the instance removes only the bodies and constraints created by it.
  scene.requireAlive()
  world.requireOpen()
  new(result)
  result.owner = world
  result.native = scene.native.instantiate(world.physics, world.layerCount)
  if result.native.isNil:
    raise newException(
      JoltError, "Jolt could not instantiate the physics scene in this world")
  let bodyCount = int(result.native.bodyCount)
  let constraintCount = int(result.native.constraintCount)
  result.bodyIds = newSeq[uint32](bodyCount)
  result.constraints = newSeq[ptr raw.Constraint](constraintCount)
  for index in 0 ..< bodyCount:
    result.bodyIds[index] = result.native.bodyId(uint32(index))
    world.bodyIds.add(result.bodyIds[index])
  for index in 0 ..< constraintCount:
    result.constraints[index] = result.native.constraint(uint32(index))
    world.constraints.add(result.constraints[index])
  world.sceneInstances.add(result.native)
  result.alive = true

proc isAlive*(instance: PhysicsSceneInstance): bool =
  not instance.isNil and instance.alive and not instance.native.isNil and
    not instance.owner.isNil and instance.owner.isOpen

proc requireAlive(instance: PhysicsSceneInstance) =
  if not instance.isAlive:
    raise newException(JoltError, "Jolt physics scene instance is no longer alive")

proc requireBody(instance: PhysicsSceneInstance; index: int) =
  instance.requireAlive()
  if index < 0 or index >= instance.bodyIds.len:
    raise newException(IndexDefect, "physics scene body index is out of bounds")

proc requireConstraint(instance: PhysicsSceneInstance; index: int) =
  instance.requireAlive()
  if index < 0 or index >= instance.constraints.len:
    raise newException(
      IndexDefect, "physics scene constraint index is out of bounds")

proc wakeInstanceBodies(instance: PhysicsSceneInstance) =
  let bodyInterface = instance.owner.physics.bodyInterface()
  for index, id in instance.bodyIds:
    if instance.native.motionType(uint32(index)) !=
        uint8(ord(raw.EMotionType.Static)):
      bodyInterface.activate(raw.bodyID(id))

proc bodyCount*(instance: PhysicsSceneInstance): int =
  instance.requireAlive()
  instance.bodyIds.len

proc constraintCount*(instance: PhysicsSceneInstance): int =
  instance.requireAlive()
  instance.constraints.len

proc constraintKind*(instance: PhysicsSceneInstance;
                     index: int): ConstraintKind =
  instance.requireConstraint(index)
  case instance.constraints[index].subType
  of 0: ConstraintKind.Fixed
  of 1: ConstraintKind.Point
  of 2: ConstraintKind.Hinge
  of 3: ConstraintKind.Slider
  of 4: ConstraintKind.Distance
  of 5: ConstraintKind.Cone
  of 6: ConstraintKind.SwingTwist
  of 7: ConstraintKind.SixDOF
  of 8: ConstraintKind.Path
  of 10: ConstraintKind.RackAndPinion
  of 11: ConstraintKind.Gear
  of 12: ConstraintKind.Pulley
  else:
    raise newException(JoltError, "Jolt scene constraint kind is unsupported")

proc constraintBodyIds*(instance: PhysicsSceneInstance; index: int):
    tuple[body1, body2: Option[BodyId]] =
  ## A fixed-world endpoint is returned as `none(BodyId)`.
  instance.requireConstraint(index)
  var body1, body2: uint32
  var body1IsFixed, body2IsFixed: bool
  if not instance.constraints[index].twoBodyIDs(
      addr body1, addr body1IsFixed, addr body2, addr body2IsFixed):
    raise newException(JoltError, "Jolt constraint is not a two-body constraint")
  result.body1 = if body1IsFixed: none(BodyId) else: some(BodyId(body1))
  result.body2 = if body2IsFixed: none(BodyId) else: some(BodyId(body2))

proc constraintSolverImpulse*(instance: PhysicsSceneInstance;
                              index: int): ConstraintSolverImpulse =
  ## Returns the latest native solver impulse without escaping scene ownership.
  instance.requireConstraint(index)
  var position, rotation, motorTranslation, motorRotation: raw.Vec3
  var limit: cfloat
  if not instance.constraints[index].solverImpulse(
      addr position, addr rotation, addr limit,
      addr motorTranslation, addr motorRotation):
    raise newException(JoltError, "Jolt could not inspect constraint impulses")
  ConstraintSolverImpulse(
    position: position.fromRaw,
    rotation: rotation.fromRaw,
    limit: limit,
    motorTranslation: motorTranslation.fromRaw,
    motorRotation: motorRotation.fromRaw)

proc constraintFriction*(instance: PhysicsSceneInstance;
                         index: int): float32 =
  ## Returns scalar friction for hinge, slider, swing-twist, or path.
  instance.requireConstraint(index)
  let kind = instance.constraintKind(index)
  if kind notin {ConstraintKind.Hinge, ConstraintKind.Slider,
      ConstraintKind.SwingTwist, ConstraintKind.Path}:
    raise newException(
      ValueError,
      "constraint does not expose scalar friction; SixDOF uses per-axis friction"
    )
  var value: cfloat
  if not instance.constraints[index].friction(0, addr value):
    raise newException(JoltError, "Jolt could not inspect constraint friction")
  float32(value)

proc readSixDOFAxisLimit(native: ptr raw.Constraint;
                         axis: SixDOFAxis): SixDOFAxisLimit =
  var minimum, maximum: cfloat
  var mode: uint8
  native.sixDOFAxisLimit(
    uint8(ord(axis)), addr minimum, addr maximum, addr mode)
  case mode
  of 0: freeAxis()
  of 1: fixedAxis()
  of 2: limitedAxis(minimum, maximum)
  else: raise newException(JoltError, "Jolt returned an unknown SixDOF axis mode")

proc sixDOFSwingType*(instance: PhysicsSceneInstance;
                      index: int): SixDOFSwingType =
  ## Returns the authored cone or pyramid swing-limit representation.
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  let value = instance.constraints[index].sixDOFSwingType
  if value > uint8(ord(high(SixDOFSwingType))):
    raise newException(JoltError, "Jolt returned an invalid SixDOF swing type")
  SixDOFSwingType(value)

proc axisLimit*(instance: PhysicsSceneInstance; index: int;
                axis: SixDOFAxis): SixDOFAxisLimit =
  ## Returns one runtime axis limit without escaping scene ownership.
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  readSixDOFAxisLimit(instance.constraints[index], axis)

proc setAxisLimit*(instance: PhysicsSceneInstance; index: int;
                   axis: SixDOFAxis; limit: SixDOFAxisLimit) =
  ## Changes one runtime axis limit on an instantiated authored scene.
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  limit.validate(axis, instance.sixDOFSwingType(index))
  instance.constraints[index].setSixDOFAxisLimit(
    uint8(ord(axis)), uint8(ord(limit.mode)), limit.minimum, limit.maximum)
  instance.wakeInstanceBodies()

proc sixDOFConstraintFriction*(instance: PhysicsSceneInstance; index: int;
                               axis: SixDOFAxis): float32 =
  ## Returns SixDOF friction force or torque for one axis.
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  var value: cfloat
  if not instance.constraints[index].friction(uint8(ord(axis)), addr value):
    raise newException(JoltError, "Jolt could not inspect SixDOF friction")
  float32(value)

proc setAxisFriction*(instance: PhysicsSceneInstance; index: int;
                      axis: SixDOFAxis; maximum: float32) =
  ## Changes SixDOF friction on an instantiated authored scene.
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  if not maximum.isFinite or maximum < 0:
    raise newException(ValueError, "axis friction must be finite and non-negative")
  instance.constraints[index].setSixDOFFriction(uint8(ord(axis)), maximum)
  instance.wakeInstanceBodies()

proc readConstraintLimitSpring(instance: PhysicsSceneInstance; index: int;
                               axis: uint8): SpringSettings =
  var mode: uint8
  var value, damping: cfloat
  if not instance.constraints[index].limitSpring(
      axis, addr mode, addr value, addr damping):
    raise newException(JoltError, "Jolt could not inspect constraint limit spring")
  if mode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid spring mode")
  SpringSettings(
    mode: SpringMode(mode), value: float32(value), damping: float32(damping))

proc constraintLimitSpring*(instance: PhysicsSceneInstance;
                            index: int): SpringSettings =
  ## Returns distance, hinge, or slider limit-spring settings.
  instance.requireConstraint(index)
  if instance.constraintKind(index) notin
      {ConstraintKind.Distance, ConstraintKind.Hinge, ConstraintKind.Slider}:
    raise newException(
      ValueError, "constraint does not expose a scalar limit spring")
  instance.readConstraintLimitSpring(index, 0)

proc sixDOFConstraintLimitSpring*(instance: PhysicsSceneInstance;
                                  index: int;
                                  axis: SixDOFAxis): SpringSettings =
  ## Returns SixDOF soft-limit settings for one translation axis.
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  if axis > SixDOFAxis.TranslationZ:
    raise newException(
      ValueError, "SixDOF limit springs only support translation axes")
  instance.readConstraintLimitSpring(index, uint8(ord(axis)))

proc setAxisLimitSpring*(instance: PhysicsSceneInstance; index: int;
                         axis: SixDOFAxis; settings: SpringSettings) =
  ## Changes one SixDOF translation limit spring on an instantiated scene.
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  if axis > SixDOFAxis.TranslationZ:
    raise newException(
      ValueError, "SixDOF limit springs only support translation axes")
  settings.validate()
  instance.constraints[index].setSixDOFLimitSpring(
    uint8(ord(axis)), uint8(ord(settings.mode)),
    settings.value, settings.damping)
  instance.wakeInstanceBodies()

proc distanceLimits*(instance: PhysicsSceneInstance; index: int): tuple[
    minimum, maximum: float32] =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.Distance:
    raise newException(ValueError, "constraint is not a distance constraint")
  instance.constraints[index].distanceLimits(
    addr result.minimum, addr result.maximum)

proc setDistanceLimits*(instance: PhysicsSceneInstance; index: int;
                        minimum, maximum: float32) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.Distance:
    raise newException(ValueError, "constraint is not a distance constraint")
  if not minimum.isFinite or not maximum.isFinite or
      minimum < 0 or minimum > maximum:
    raise newException(
      ValueError, "distance limits must be finite, non-negative, and ordered")
  instance.constraints[index].setDistanceLimits(minimum, maximum)
  instance.wakeInstanceBodies()

proc setDistanceLimitSpring*(instance: PhysicsSceneInstance; index: int;
                             settings: SpringSettings) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.Distance:
    raise newException(ValueError, "constraint is not a distance constraint")
  settings.validate()
  instance.constraints[index].setDistanceLimitSpring(
    uint8(ord(settings.mode)), settings.value, settings.damping)
  instance.wakeInstanceBodies()

proc readConstraintMotorSettings(instance: PhysicsSceneInstance; index: int;
                                 motorIndex: uint8): MotorSettings =
  var springMode: uint8
  var springValue, springDamping, minForce, maxForce,
    minTorque, maxTorque: cfloat
  if not instance.constraints[index].motorSettings(
      motorIndex, addr springMode, addr springValue, addr springDamping,
      addr minForce, addr maxForce, addr minTorque, addr maxTorque):
    raise newException(JoltError, "Jolt could not inspect constraint motor settings")
  if springMode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid motor spring mode")
  MotorSettings(
    spring: SpringSettings(
      mode: SpringMode(springMode), value: float32(springValue),
      damping: float32(springDamping)),
    minForce: float32(minForce), maxForce: float32(maxForce),
    minTorque: float32(minTorque), maxTorque: float32(maxTorque))

proc motorSettings*(instance: PhysicsSceneInstance;
                    index: int): MotorSettings =
  ## Returns motor tuning for a hinge, slider, or path constraint.
  instance.requireConstraint(index)
  if instance.constraintKind(index) notin
      {ConstraintKind.Hinge, ConstraintKind.Slider, ConstraintKind.Path}:
    raise newException(
      ValueError, "constraint does not expose one scalar motor")
  instance.readConstraintMotorSettings(index, 0)

proc swingMotorSettings*(instance: PhysicsSceneInstance;
                         index: int): MotorSettings =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "constraint is not a SwingTwist constraint")
  instance.readConstraintMotorSettings(index, 0)

proc twistMotorSettings*(instance: PhysicsSceneInstance;
                         index: int): MotorSettings =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "constraint is not a SwingTwist constraint")
  instance.readConstraintMotorSettings(index, 1)

proc sixDOFAxisMotorSettings*(instance: PhysicsSceneInstance; index: int;
                              axis: SixDOFAxis): MotorSettings =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  instance.readConstraintMotorSettings(index, uint8(ord(axis)))

proc setMotor*(instance: PhysicsSceneInstance; index: int; state: MotorState;
               targetVelocity, targetPosition: float32) =
  ## Sets runtime targets and state for an instantiated hinge or slider.
  instance.requireConstraint(index)
  if not targetVelocity.isFinite or not targetPosition.isFinite:
    raise newException(ValueError, "motor targets must be finite")
  case instance.constraintKind(index)
  of ConstraintKind.Hinge:
    instance.constraints[index].setHingeMotorTarget(
      targetVelocity, targetPosition)
    instance.constraints[index].setHingeMotorState(uint8(ord(state)))
  of ConstraintKind.Slider:
    instance.constraints[index].setSliderMotorTarget(
      targetVelocity, targetPosition)
    instance.constraints[index].setSliderMotorState(uint8(ord(state)))
  else:
    raise newException(ValueError, "constraint is not a hinge or slider")
  instance.wakeInstanceBodies()

proc motor*(instance: PhysicsSceneInstance; index: int): tuple[
    state: MotorState, targetVelocity, targetPosition: float32] =
  instance.requireConstraint(index)
  var state: uint8
  var velocity, position: cfloat
  case instance.constraintKind(index)
  of ConstraintKind.Hinge:
    instance.constraints[index].hingeMotor(
      addr state, addr velocity, addr position)
  of ConstraintKind.Slider:
    instance.constraints[index].sliderMotor(
      addr state, addr velocity, addr position)
  else:
    raise newException(ValueError, "constraint is not a hinge or slider")
  (decodeMotorState(state, "constraint motor"),
   float32(velocity), float32(position))

proc setPathMotor*(instance: PhysicsSceneInstance; index: int;
                   state: MotorState; targetVelocity,
                   targetFraction: float32) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.Path:
    raise newException(ValueError, "constraint is not a path constraint")
  if not targetVelocity.isFinite or not targetFraction.isFinite or
      targetFraction < 0 or
      targetFraction > instance.constraints[index].pathMaxFraction:
    raise newException(ValueError, "path motor targets are outside the path")
  instance.constraints[index].setPathMotorTargets(
    targetVelocity, targetFraction)
  instance.constraints[index].setPathMotorState(uint8(ord(state)))
  instance.wakeInstanceBodies()

proc pathMotor*(instance: PhysicsSceneInstance; index: int): tuple[
    state: MotorState, targetVelocity, targetFraction: float32] =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.Path:
    raise newException(ValueError, "constraint is not a path constraint")
  var state: uint8
  var velocity, fraction: cfloat
  instance.constraints[index].pathMotor(
    addr state, addr velocity, addr fraction)
  (decodeMotorState(state, "path motor"),
   float32(velocity), float32(fraction))

proc setSwingMotorState*(instance: PhysicsSceneInstance; index: int;
                         state: MotorState) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "constraint is not a SwingTwist constraint")
  instance.constraints[index].setSwingTwistMotorState(true, uint8(ord(state)))
  instance.wakeInstanceBodies()

proc setTwistMotorState*(instance: PhysicsSceneInstance; index: int;
                         state: MotorState) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "constraint is not a SwingTwist constraint")
  instance.constraints[index].setSwingTwistMotorState(false, uint8(ord(state)))
  instance.wakeInstanceBodies()

proc setSwingTwistMotorTargets*(instance: PhysicsSceneInstance; index: int;
                                angularVelocity: Vec3;
                                orientation: Quat) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "constraint is not a SwingTwist constraint")
  angularVelocity.requireFinite("swing-twist target angular velocity")
  instance.constraints[index].setSwingTwistMotorTargets(
    angularVelocity.toRaw, orientation.normalized.toRaw)
  instance.wakeInstanceBodies()

proc swingTwistMotor*(instance: PhysicsSceneInstance; index: int): tuple[
    swingState, twistState: MotorState; targetAngularVelocity: Vec3;
    targetOrientation: Quat] =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SwingTwist:
    raise newException(ValueError, "constraint is not a SwingTwist constraint")
  var swingState, twistState: uint8
  var angularVelocity: raw.Vec3
  var orientation: raw.Quat
  if not instance.constraints[index].swingTwistMotorState(
      addr swingState, addr twistState, addr angularVelocity,
      addr orientation):
    raise newException(JoltError, "Jolt could not inspect SwingTwist motor state")
  (decodeMotorState(swingState, "swing motor"),
   decodeMotorState(twistState, "twist motor"),
   angularVelocity.fromRaw, orientation.fromRaw)

proc setAxisMotorState*(instance: PhysicsSceneInstance; index: int;
                        axis: SixDOFAxis; state: MotorState) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  instance.constraints[index].setSixDOFMotorState(
    uint8(ord(axis)), uint8(ord(state)))
  instance.wakeInstanceBodies()

proc setSixDOFMotorTargets*(instance: PhysicsSceneInstance; index: int;
                            velocity, angularVelocity, position: Vec3;
                            orientation: Quat) =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  velocity.requireFinite("SixDOF target velocity")
  angularVelocity.requireFinite("SixDOF target angular velocity")
  position.requireFinite("SixDOF target position")
  instance.constraints[index].setSixDOFMotorTargets(
    velocity.toRaw, angularVelocity.toRaw, position.toRaw,
    orientation.normalized.toRaw)
  instance.wakeInstanceBodies()

proc sixDOFMotor*(instance: PhysicsSceneInstance; index: int;
                  axis: SixDOFAxis): tuple[
    state: MotorState; targetVelocity, targetAngularVelocity,
    targetPosition: Vec3; targetOrientation: Quat] =
  instance.requireConstraint(index)
  if instance.constraintKind(index) != ConstraintKind.SixDOF:
    raise newException(ValueError, "constraint is not a SixDOF constraint")
  var state: uint8
  var velocity, angularVelocity, position: raw.Vec3
  var orientation: raw.Quat
  if not instance.constraints[index].sixDOFMotorState(
      uint8(ord(axis)), addr state, addr velocity, addr angularVelocity,
      addr position, addr orientation):
    raise newException(JoltError, "Jolt could not inspect SixDOF motor state")
  (decodeMotorState(state, "SixDOF motor"), velocity.fromRaw,
   angularVelocity.fromRaw, position.fromRaw, orientation.fromRaw)

proc constraintEnabled*(instance: PhysicsSceneInstance; index: int): bool =
  instance.requireConstraint(index)
  instance.constraints[index].enabled

proc setConstraintEnabled*(instance: PhysicsSceneInstance; index: int;
                           enabled: bool) =
  instance.requireConstraint(index)
  instance.constraints[index].setEnabled(enabled)
  instance.wakeInstanceBodies()

proc constraintPriority*(instance: PhysicsSceneInstance;
                         index: int): uint32 =
  instance.requireConstraint(index)
  instance.constraints[index].priority

proc setConstraintPriority*(instance: PhysicsSceneInstance; index: int;
                            priority: uint32) =
  instance.requireConstraint(index)
  instance.constraints[index].setPriority(priority)

proc constraintSolverStepOverrides*(instance: PhysicsSceneInstance;
                                    index: int):
    tuple[velocity, position: uint32] =
  instance.requireConstraint(index)
  result.velocity = instance.constraints[index].velocityStepsOverride
  result.position = instance.constraints[index].positionStepsOverride

proc setConstraintSolverStepOverrides*(instance: PhysicsSceneInstance;
                                       index: int;
                                       velocity, position: uint32) =
  instance.requireConstraint(index)
  if velocity >= 256 or position >= 256:
    raise newException(
      ValueError, "constraint solver step overrides must be below 256")
  instance.constraints[index].setVelocityStepsOverride(velocity)
  instance.constraints[index].setPositionStepsOverride(position)
  instance.wakeInstanceBodies()

proc constraintUserData*(instance: PhysicsSceneInstance;
                         index: int): uint64 =
  instance.requireConstraint(index)
  instance.constraints[index].userData

proc setConstraintUserData*(instance: PhysicsSceneInstance; index: int;
                            value: uint64) =
  instance.requireConstraint(index)
  instance.constraints[index].setUserData(value)

proc resetConstraintWarmStart*(instance: PhysicsSceneInstance; index: int) =
  instance.requireConstraint(index)
  instance.constraints[index].resetWarmStart()

proc bodyId*(instance: PhysicsSceneInstance; index: int): BodyId =
  instance.requireBody(index)
  BodyId(instance.bodyIds[index])

proc isSoftBody*(instance: PhysicsSceneInstance; index: int): bool =
  instance.requireBody(index)
  instance.native.isSoftBody(uint32(index))

proc bodyMotionType*(instance: PhysicsSceneInstance; index: int): MotionType =
  instance.requireBody(index)
  case instance.native.motionType(uint32(index))
  of uint8(ord(raw.EMotionType.Static)): MotionType.Static
  of uint8(ord(raw.EMotionType.Kinematic)): MotionType.Kinematic
  of uint8(ord(raw.EMotionType.Dynamic)): MotionType.Dynamic
  else:
    raise newException(JoltError, "Jolt scene body has an invalid motion type")

proc bodyPosition*(instance: PhysicsSceneInstance; index: int): Vec3 =
  instance.requireBody(index)
  fromRaw(instance.owner.physics.bodyInterface().position(
    raw.bodyID(instance.bodyIds[index])))

proc bodyRotation*(instance: PhysicsSceneInstance; index: int): Quat =
  instance.requireBody(index)
  fromRaw(instance.owner.physics.bodyInterface().rotation(
    raw.bodyID(instance.bodyIds[index])))

proc bodyLinearVelocity*(instance: PhysicsSceneInstance; index: int): Vec3 =
  instance.requireBody(index)
  fromRaw(instance.owner.physics.bodyInterface().linearVelocity(
    raw.bodyID(instance.bodyIds[index])))

proc bodyAngularVelocity*(instance: PhysicsSceneInstance; index: int): Vec3 =
  instance.requireBody(index)
  fromRaw(instance.owner.physics.bodyInterface().angularVelocity(
    raw.bodyID(instance.bodyIds[index])))

proc bodyCollisionLayer*(instance: PhysicsSceneInstance;
                         index: int): CollisionLayer =
  instance.requireBody(index)
  instance.owner.physics.bodyInterface().objectLayer(
    raw.bodyID(instance.bodyIds[index]))

proc setBodyTransform*(instance: PhysicsSceneInstance; index: int;
                       position: Vec3; rotation = quatIdentity();
                       activate = true) =
  instance.requireBody(index)
  position.requireFinite("physics scene body position")
  instance.owner.physics.bodyInterface().setPositionAndRotation(
    raw.bodyID(instance.bodyIds[index]), position.toRaw,
    rotation.normalized.toRaw,
    if activate: raw.EActivation.Activate else: raw.EActivation.DontActivate)

proc setBodyLinearVelocity*(instance: PhysicsSceneInstance; index: int;
                            velocity: Vec3) =
  instance.requireBody(index)
  velocity.requireFinite("physics scene body linear velocity")
  if instance.bodyMotionType(index) == MotionType.Static:
    raise newException(ValueError, "static scene bodies do not have velocity")
  instance.owner.physics.bodyInterface().setLinearVelocity(
    raw.bodyID(instance.bodyIds[index]), velocity.toRaw)

proc setBodyAngularVelocity*(instance: PhysicsSceneInstance; index: int;
                             velocity: Vec3) =
  instance.requireBody(index)
  velocity.requireFinite("physics scene body angular velocity")
  if instance.bodyMotionType(index) == MotionType.Static:
    raise newException(ValueError, "static scene bodies do not have velocity")
  instance.owner.physics.bodyInterface().setAngularVelocity(
    raw.bodyID(instance.bodyIds[index]), velocity.toRaw)

proc newSkeletonMapper*(source, target: SkeletonDefinition): SkeletonMapper =
  ## Creates a name-based mapper from a low-detail source skeleton to a
  ## higher-detail target skeleton. Every source joint must exist in target.
  source.validateSkeleton("source")
  target.validateSkeleton("target")
  for sourceJoint in source.joints:
    var found = false
    for targetJoint in target.joints:
      if sourceJoint.name == targetJoint.name:
        found = true
        break
    if not found:
      raise newException(
        ValueError, "every source skeleton joint must exist in target")

  var sourceNames, targetNames: seq[cstring]
  var sourceParents, targetParents: seq[int32]
  var sourcePositions, targetPositions: seq[raw.Vec3]
  var sourceRotations, targetRotations: seq[raw.Quat]
  source.nativeSkeleton(
    sourceNames, sourceParents, sourcePositions, sourceRotations)
  target.nativeSkeleton(
    targetNames, targetParents, targetPositions, targetRotations)

  new(result)
  if not raw.acquireJolt():
    raise newException(
      JoltError, "linked Jolt library is not ABI-compatible with its headers")
  result.acquiredJolt = true
  result.native = raw.newSkeletonMapper(
    addr sourceNames[0], addr sourceParents[0], addr sourcePositions[0],
    addr sourceRotations[0], uint32(sourceNames.len),
    addr targetNames[0], addr targetParents[0], addr targetPositions[0],
    addr targetRotations[0], uint32(targetNames.len))
  if result.native.isNil:
    raw.releaseJolt()
    result.acquiredJolt = false
    raise newException(JoltError, "Jolt could not create the skeleton mapper")
  result.source = source.copySkeleton()
  result.target = target.copySkeleton()
  result.alive = true

proc isAlive*(mapper: SkeletonMapper): bool =
  not mapper.isNil and mapper.alive and not mapper.native.isNil

proc requireAlive(mapper: SkeletonMapper) =
  if not mapper.isAlive:
    raise newException(JoltError, "Jolt skeleton mapper is no longer alive")

proc sourceJointCount*(mapper: SkeletonMapper): int =
  mapper.requireAlive()
  mapper.source.joints.len

proc targetJointCount*(mapper: SkeletonMapper): int =
  mapper.requireAlive()
  mapper.target.joints.len

proc mappingCount*(mapper: SkeletonMapper): int =
  mapper.requireAlive()
  int(mapper.native.mappingCount)

proc chainCount*(mapper: SkeletonMapper): int =
  mapper.requireAlive()
  int(mapper.native.chainCount)

proc unmappedJointCount*(mapper: SkeletonMapper): int =
  mapper.requireAlive()
  int(mapper.native.unmappedCount)

proc mappedJoint*(mapper: SkeletonMapper; sourceJoint: int): Option[int] =
  mapper.requireAlive()
  if sourceJoint < 0 or sourceJoint >= mapper.source.joints.len:
    raise newException(IndexDefect, "source skeleton joint is out of bounds")
  let index = mapper.native.mappedJoint(int32(sourceJoint))
  if index < 0: none(int) else: some(int(index))

proc isTranslationLocked*(mapper: SkeletonMapper; targetJoint: int): bool =
  mapper.requireAlive()
  if targetJoint < 0 or targetJoint >= mapper.target.joints.len:
    raise newException(IndexDefect, "target skeleton joint is out of bounds")
  mapper.native.isTranslationLocked(int32(targetJoint))

proc targetNeutralPose(mapper: SkeletonMapper;
                       positions: var seq[raw.Vec3];
                       rotations: var seq[raw.Quat]) =
  positions = newSeq[raw.Vec3](mapper.target.joints.len)
  rotations = newSeq[raw.Quat](mapper.target.joints.len)
  for index, joint in mapper.target.joints:
    positions[index] = joint.neutralTransform.position.toRaw
    rotations[index] = joint.neutralTransform.rotation.normalized.toRaw

proc lockTranslations*(mapper: SkeletonMapper; locked: openArray[bool]) =
  mapper.requireAlive()
  if locked.len != mapper.target.joints.len:
    raise newException(
      ValueError, "translation lock flags must match target joint count")
  var flags = newSeq[bool](locked.len)
  for index, value in locked:
    flags[index] = value
  var positions: seq[raw.Vec3]
  var rotations: seq[raw.Quat]
  mapper.targetNeutralPose(positions, rotations)
  mapper.native.lockTranslations(
    addr flags[0], addr positions[0], addr rotations[0])

proc lockAllTranslations*(mapper: SkeletonMapper) =
  mapper.requireAlive()
  var positions: seq[raw.Vec3]
  var rotations: seq[raw.Quat]
  mapper.targetNeutralPose(positions, rotations)
  mapper.native.lockAllTranslations(addr positions[0], addr rotations[0])

proc validateSkeletonPose(transforms: openArray[SkeletonTransform];
                          expected: int; name: string) =
  if transforms.len != expected:
    raise newException(ValueError, name & " pose has the wrong joint count")
  for transform in transforms:
    transform.position.requireFinite(name & " pose position")
    discard transform.rotation.normalized

proc nativeSkeletonPose(transforms: openArray[SkeletonTransform];
                        positions: var seq[raw.Vec3];
                        rotations: var seq[raw.Quat]) =
  positions = newSeq[raw.Vec3](transforms.len)
  rotations = newSeq[raw.Quat](transforms.len)
  for index, transform in transforms:
    positions[index] = transform.position.toRaw
    rotations[index] = transform.rotation.normalized.toRaw

proc mappedPose*(mapper: SkeletonMapper;
                 sourceModelPose, targetLocalPose:
                   openArray[SkeletonTransform]): seq[SkeletonTransform] =
  mapper.requireAlive()
  sourceModelPose.validateSkeletonPose(
    mapper.source.joints.len, "source model")
  targetLocalPose.validateSkeletonPose(
    mapper.target.joints.len, "target local")
  var sourcePositions, targetPositions: seq[raw.Vec3]
  var sourceRotations, targetRotations: seq[raw.Quat]
  sourceModelPose.nativeSkeletonPose(sourcePositions, sourceRotations)
  targetLocalPose.nativeSkeletonPose(targetPositions, targetRotations)
  var outputPositions = newSeq[raw.Vec3](mapper.target.joints.len)
  var outputRotations = newSeq[raw.Quat](mapper.target.joints.len)
  mapper.native.mapPose(
    addr sourcePositions[0], addr sourceRotations[0],
    addr targetPositions[0], addr targetRotations[0],
    addr outputPositions[0], addr outputRotations[0])
  result = newSeq[SkeletonTransform](mapper.target.joints.len)
  for index in 0 ..< result.len:
    result[index] = SkeletonTransform(
      position: fromRaw(outputPositions[index]),
      rotation: fromRaw(outputRotations[index]))

proc reverseMappedPose*(mapper: SkeletonMapper;
                        targetModelPose:
                          openArray[SkeletonTransform]): seq[SkeletonTransform] =
  mapper.requireAlive()
  targetModelPose.validateSkeletonPose(
    mapper.target.joints.len, "target model")
  var targetPositions: seq[raw.Vec3]
  var targetRotations: seq[raw.Quat]
  targetModelPose.nativeSkeletonPose(targetPositions, targetRotations)
  var outputPositions = newSeq[raw.Vec3](mapper.source.joints.len)
  var outputRotations = newSeq[raw.Quat](mapper.source.joints.len)
  mapper.native.reverseMapPose(
    addr targetPositions[0], addr targetRotations[0],
    addr outputPositions[0], addr outputRotations[0])
  result = newSeq[SkeletonTransform](mapper.source.joints.len)
  for index in 0 ..< result.len:
    result[index] = SkeletonTransform(
      position: fromRaw(outputPositions[index]),
      rotation: fromRaw(outputRotations[index]))

proc copySoftBodyMesh(mesh: SoftBodyMesh): SoftBodyMesh =
  result.vertices = newSeq[SoftBodyVertex](mesh.vertices.len)
  for index, vertex in mesh.vertices:
    result.vertices[index] = vertex
  result.vertexAttributes =
    newSeq[SoftBodyVertexAttributes](mesh.vertexAttributes.len)
  for index, attributes in mesh.vertexAttributes:
    result.vertexAttributes[index] = attributes
  result.faces = newSeq[SoftBodyFace](mesh.faces.len)
  for index, face in mesh.faces:
    result.faces[index] = face
  result.materials = newSeq[PhysicsMaterial](mesh.materials.len)
  for index, material in mesh.materials:
    result.materials[index] = material
  result.edgeConstraints =
    newSeq[SoftBodyEdgeConstraint](mesh.edgeConstraints.len)
  for index, constraint in mesh.edgeConstraints:
    result.edgeConstraints[index] = constraint
  result.dihedralBendConstraints =
    newSeq[SoftBodyDihedralBendConstraint](mesh.dihedralBendConstraints.len)
  for index, constraint in mesh.dihedralBendConstraints:
    result.dihedralBendConstraints[index] = constraint
  result.longRangeConstraints =
    newSeq[SoftBodyLongRangeConstraint](mesh.longRangeConstraints.len)
  for index, constraint in mesh.longRangeConstraints:
    result.longRangeConstraints[index] = constraint
  result.volumeConstraints =
    newSeq[SoftBodyVolumeConstraint](mesh.volumeConstraints.len)
  for index, constraint in mesh.volumeConstraints:
    result.volumeConstraints[index] = constraint
  result.rods = newSeq[SoftBodyRodConstraint](mesh.rods.len)
  for index, rod in mesh.rods:
    result.rods[index] = rod
  result.rodBendTwistConstraints =
    newSeq[SoftBodyRodBendTwistConstraint](mesh.rodBendTwistConstraints.len)
  for index, constraint in mesh.rodBendTwistConstraints:
    result.rodBendTwistConstraints[index] = constraint
  result.skinBindPose = newSeq[SoftBodyJointTransform](mesh.skinBindPose.len)
  for index, joint in mesh.skinBindPose:
    result.skinBindPose[index] = joint
  result.skinConstraints = newSeq[SoftBodySkinConstraint](mesh.skinConstraints.len)
  for index, constraint in mesh.skinConstraints:
    result.skinConstraints[index] = constraint
    result.skinConstraints[index].weights =
      newSeq[SoftBodySkinWeight](constraint.weights.len)
    for weightIndex, weight in constraint.weights:
      result.skinConstraints[index].weights[weightIndex] = weight

proc addSoftBody*(world: World; mesh: SoftBodyMesh; position: Vec3;
                  rotation = quatIdentity(); layer = movingLayer;
                  config = defaultSoftBodyConfig()): SoftBody =
  world.requireLayer(layer)
  mesh.validate()
  config.validate()
  if mesh.materials.len > 0 and config.material.isSome:
    raise newException(
      ValueError, "soft body mesh materials conflict with config material")
  position.requireFinite("soft body position")
  let nativeRotation = rotation.normalized

  var nativeConfig = config
  var hasFixedVertex = false
  var hasDynamicVertex = false
  for vertex in mesh.vertices:
    if vertex.inverseMass == 0:
      hasFixedVertex = true
      nativeConfig.updatePosition = false
    else:
      hasDynamicVertex = true
  if nativeConfig.lraType != SoftBodyLRAType.NoLRA and
      (not hasFixedVertex or not hasDynamicVertex):
    raise newException(
      ValueError, "soft body LRA requires fixed and dynamic vertices")
  for attributes in mesh.vertexAttributes:
    if attributes.lraType != SoftBodyLRAType.NoLRA and
        (not hasFixedVertex or not hasDynamicVertex):
      raise newException(
        ValueError, "soft body vertex LRA requires fixed and dynamic vertices")

  var positions = newSeq[raw.Vec3](mesh.vertices.len)
  var velocities = newSeq[raw.Vec3](mesh.vertices.len)
  var inverseMasses = newSeq[cfloat](mesh.vertices.len)
  for index, vertex in mesh.vertices:
    positions[index] = vertex.position.toRaw
    velocities[index] = vertex.velocity.toRaw
    inverseMasses[index] = vertex.inverseMass
  var attributeEdgeCompliances =
    newSeq[cfloat](mesh.vertexAttributes.len)
  var attributeShearCompliances =
    newSeq[cfloat](mesh.vertexAttributes.len)
  var attributeBendCompliances =
    newSeq[cfloat](mesh.vertexAttributes.len)
  var attributeLRATypes = newSeq[uint8](mesh.vertexAttributes.len)
  var attributeLRAMultipliers =
    newSeq[cfloat](mesh.vertexAttributes.len)
  for index, attributes in mesh.vertexAttributes:
    attributeEdgeCompliances[index] = attributes.edgeCompliance
    attributeShearCompliances[index] = attributes.shearCompliance
    attributeBendCompliances[index] = attributes.bendCompliance
    attributeLRATypes[index] = uint8(ord(attributes.lraType))
    attributeLRAMultipliers[index] = attributes.lraMaxDistanceMultiplier
  var faceVertices = newSeq[uint32](mesh.faces.len * 3)
  var faceMaterialIndices = newSeq[uint32](mesh.faces.len)
  for index, face in mesh.faces:
    for corner in 0 ..< 3:
      faceVertices[index * 3 + corner] = face.vertices[corner]
    faceMaterialIndices[index] = face.materialIndex
  var edgeVertices = newSeq[uint32](mesh.edgeConstraints.len * 2)
  var edgeCompliances = newSeq[cfloat](mesh.edgeConstraints.len)
  for index, constraint in mesh.edgeConstraints:
    edgeVertices[index * 2] = constraint.vertices[0]
    edgeVertices[index * 2 + 1] = constraint.vertices[1]
    edgeCompliances[index] = constraint.compliance
  var dihedralVertices =
    newSeq[uint32](mesh.dihedralBendConstraints.len * 4)
  var dihedralCompliances =
    newSeq[cfloat](mesh.dihedralBendConstraints.len)
  for index, constraint in mesh.dihedralBendConstraints:
    for corner in 0 ..< 4:
      dihedralVertices[index * 4 + corner] = constraint.vertices[corner]
    dihedralCompliances[index] = constraint.compliance
  var lraVertices = newSeq[uint32](mesh.longRangeConstraints.len * 2)
  var lraMaxDistances = newSeq[cfloat](mesh.longRangeConstraints.len)
  for index, constraint in mesh.longRangeConstraints:
    lraVertices[index * 2] = constraint.vertices[0]
    lraVertices[index * 2 + 1] = constraint.vertices[1]
    lraMaxDistances[index] = constraint.maxDistance
  var volumeVertices = newSeq[uint32](mesh.volumeConstraints.len * 4)
  var volumeCompliances = newSeq[cfloat](mesh.volumeConstraints.len)
  for index, constraint in mesh.volumeConstraints:
    for corner in 0 ..< 4:
      volumeVertices[index * 4 + corner] = constraint.vertices[corner]
    volumeCompliances[index] = constraint.compliance
  var rodVertices = newSeq[uint32](mesh.rods.len * 2)
  var rodCompliances = newSeq[cfloat](mesh.rods.len)
  for index, rod in mesh.rods:
    rodVertices[index * 2] = rod.vertices[0]
    rodVertices[index * 2 + 1] = rod.vertices[1]
    rodCompliances[index] = rod.compliance
  var rodPairs = newSeq[uint32](mesh.rodBendTwistConstraints.len * 2)
  var rodPairCompliances = newSeq[cfloat](mesh.rodBendTwistConstraints.len)
  var rodRemap = newSeq[uint32](mesh.rods.len)
  var rodPairRemap = newSeq[uint32](mesh.rodBendTwistConstraints.len)
  for index, constraint in mesh.rodBendTwistConstraints:
    rodPairs[index * 2] = constraint.rods[0]
    rodPairs[index * 2 + 1] = constraint.rods[1]
    rodPairCompliances[index] = constraint.compliance
  var skinBindPositions = newSeq[raw.Vec3](mesh.skinBindPose.len)
  var skinBindRotations = newSeq[raw.Quat](mesh.skinBindPose.len)
  for index, joint in mesh.skinBindPose:
    skinBindPositions[index] = joint.position.toRaw
    skinBindRotations[index] = joint.rotation.normalized.toRaw
  var skinVertices = newSeq[uint32](mesh.skinConstraints.len)
  var skinJointIndices = newSeq[uint32](mesh.skinConstraints.len * 4)
  var skinWeights = newSeq[cfloat](mesh.skinConstraints.len * 4)
  var skinMaxDistances = newSeq[cfloat](mesh.skinConstraints.len)
  var skinBackStopDistances = newSeq[cfloat](mesh.skinConstraints.len)
  var skinBackStopRadii = newSeq[cfloat](mesh.skinConstraints.len)
  for index, constraint in mesh.skinConstraints:
    skinVertices[index] = constraint.vertex
    skinMaxDistances[index] = constraint.maxDistance
    skinBackStopDistances[index] = constraint.backStopDistance
    skinBackStopRadii[index] = constraint.backStopRadius
    for weightIndex, weight in constraint.weights:
      skinJointIndices[index * 4 + weightIndex] = weight.joint
      skinWeights[index * 4 + weightIndex] = weight.weight

  var faceVerticesPtr: ptr uint32
  var attributeEdgeCompliancesPtr: ptr cfloat
  var attributeShearCompliancesPtr: ptr cfloat
  var attributeBendCompliancesPtr: ptr cfloat
  var attributeLRATypesPtr: ptr uint8
  var attributeLRAMultipliersPtr: ptr cfloat
  var faceMaterialIndicesPtr: ptr uint32
  var edgeVerticesPtr: ptr uint32
  var edgeCompliancesPtr: ptr cfloat
  var dihedralVerticesPtr: ptr uint32
  var dihedralCompliancesPtr: ptr cfloat
  var lraVerticesPtr: ptr uint32
  var lraMaxDistancesPtr: ptr cfloat
  var volumeVerticesPtr: ptr uint32
  var volumeCompliancesPtr: ptr cfloat
  var rodVerticesPtr: ptr uint32
  var rodCompliancesPtr: ptr cfloat
  var rodPairsPtr: ptr uint32
  var rodPairCompliancesPtr: ptr cfloat
  var rodRemapPtr: ptr uint32
  var rodPairRemapPtr: ptr uint32
  var skinBindPositionsPtr: ptr raw.Vec3
  var skinBindRotationsPtr: ptr raw.Quat
  var skinVerticesPtr: ptr uint32
  var skinJointIndicesPtr: ptr uint32
  var skinWeightsPtr: ptr cfloat
  var skinMaxDistancesPtr: ptr cfloat
  var skinBackStopDistancesPtr: ptr cfloat
  var skinBackStopRadiiPtr: ptr cfloat
  if attributeEdgeCompliances.len > 0:
    attributeEdgeCompliancesPtr = addr attributeEdgeCompliances[0]
    attributeShearCompliancesPtr = addr attributeShearCompliances[0]
    attributeBendCompliancesPtr = addr attributeBendCompliances[0]
    attributeLRATypesPtr = addr attributeLRATypes[0]
    attributeLRAMultipliersPtr = addr attributeLRAMultipliers[0]
  if faceVertices.len > 0:
    faceVerticesPtr = addr faceVertices[0]
    faceMaterialIndicesPtr = addr faceMaterialIndices[0]
  if edgeVertices.len > 0:
    edgeVerticesPtr = addr edgeVertices[0]
    edgeCompliancesPtr = addr edgeCompliances[0]
  if dihedralVertices.len > 0:
    dihedralVerticesPtr = addr dihedralVertices[0]
    dihedralCompliancesPtr = addr dihedralCompliances[0]
  if lraVertices.len > 0:
    lraVerticesPtr = addr lraVertices[0]
    lraMaxDistancesPtr = addr lraMaxDistances[0]
  if volumeVertices.len > 0: volumeVerticesPtr = addr volumeVertices[0]
  if volumeCompliances.len > 0: volumeCompliancesPtr = addr volumeCompliances[0]
  if rodVertices.len > 0: rodVerticesPtr = addr rodVertices[0]
  if rodCompliances.len > 0: rodCompliancesPtr = addr rodCompliances[0]
  if rodRemap.len > 0: rodRemapPtr = addr rodRemap[0]
  if rodPairs.len > 0: rodPairsPtr = addr rodPairs[0]
  if rodPairCompliances.len > 0:
    rodPairCompliancesPtr = addr rodPairCompliances[0]
  if rodPairRemap.len > 0: rodPairRemapPtr = addr rodPairRemap[0]
  if skinBindPositions.len > 0:
    skinBindPositionsPtr = addr skinBindPositions[0]
    skinBindRotationsPtr = addr skinBindRotations[0]
  if skinVertices.len > 0:
    skinVerticesPtr = addr skinVertices[0]
    skinJointIndicesPtr = addr skinJointIndices[0]
    skinWeightsPtr = addr skinWeights[0]
    skinMaxDistancesPtr = addr skinMaxDistances[0]
    skinBackStopDistancesPtr = addr skinBackStopDistances[0]
    skinBackStopRadiiPtr = addr skinBackStopRadii[0]

  var materialDescriptions = mesh.materials
  if materialDescriptions.len == 0 and nativeConfig.material.isSome:
    materialDescriptions = @[nativeConfig.material.get]
  let nativeMaterials = cookMaterials(materialDescriptions)
  defer: nativeMaterials.releaseMaterials()
  var nativeMaterialsPtr: ptr ptr raw.PhysicsMaterial
  if nativeMaterials.len > 0:
    nativeMaterialsPtr = unsafeAddr nativeMaterials[0]
  let id = world.physics.createSoftBody(
    addr positions[0], addr velocities[0], addr inverseMasses[0],
    uint32(mesh.vertices.len),
    attributeEdgeCompliancesPtr, attributeShearCompliancesPtr,
    attributeBendCompliancesPtr, attributeLRATypesPtr,
    attributeLRAMultipliersPtr, uint32(mesh.vertexAttributes.len),
    faceVerticesPtr, faceMaterialIndicesPtr,
    uint32(mesh.faces.len),
    edgeVerticesPtr, edgeCompliancesPtr, uint32(mesh.edgeConstraints.len),
    dihedralVerticesPtr, dihedralCompliancesPtr,
    uint32(mesh.dihedralBendConstraints.len),
    lraVerticesPtr, lraMaxDistancesPtr,
    uint32(mesh.longRangeConstraints.len),
    volumeVerticesPtr, volumeCompliancesPtr,
    uint32(mesh.volumeConstraints.len),
    rodVerticesPtr, rodCompliancesPtr, uint32(mesh.rods.len),
    rodPairsPtr, rodPairCompliancesPtr,
    uint32(mesh.rodBendTwistConstraints.len),
    rodRemapPtr, rodPairRemapPtr,
    skinBindPositionsPtr, skinBindRotationsPtr,
    uint32(mesh.skinBindPose.len),
    skinVerticesPtr, skinJointIndicesPtr, skinWeightsPtr,
    skinMaxDistancesPtr, skinBackStopDistancesPtr, skinBackStopRadiiPtr,
    uint32(mesh.skinConstraints.len),
    position.toRaw, nativeRotation.toRaw, layer, nativeConfig.userData,
    uint8(ord(nativeConfig.bendType)),
    uint8(ord(nativeConfig.lraType)),
    nativeConfig.lraMaxDistanceMultiplier,
    nativeConfig.edgeCompliance,
    nativeConfig.shearCompliance,
    nativeConfig.bendCompliance,
    nativeConfig.angleTolerance,
    nativeConfig.numIterations,
    nativeConfig.linearDamping,
    nativeConfig.maxLinearVelocity,
    nativeConfig.restitution,
    nativeConfig.friction,
    nativeConfig.pressure,
    nativeConfig.gravityFactor,
    nativeConfig.vertexRadius,
    nativeConfig.updatePosition,
    nativeConfig.makeRotationIdentity,
    nativeConfig.allowSleeping,
    nativeConfig.facesDoubleSided,
    nativeConfig.enableSkinConstraints,
    nativeConfig.skinnedMaxDistanceMultiplier,
    world.allocator,
    nativeMaterialsPtr,
    uint32(nativeMaterials.len))
  if id.isInvalid:
    raise newException(JoltError, "Jolt could not allocate another soft body")

  new(result)
  result.owner = world
  result.rawId = id.value
  result.meshDesc = mesh.copySoftBodyMesh()
  result.rodRemap = rodRemap
  result.rodPairRemap = rodPairRemap
  result.config = nativeConfig
  result.layer = layer
  result.alive = true
  world.bodyIds.add(result.rawId)

proc id*(body: SoftBody): BodyId =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  BodyId(body.rawId)

proc vertexCount*(body: SoftBody): int =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  int(body.owner.physics.softBodyVertexCount(raw.bodyID(body.rawId)))

proc faceCount*(body: SoftBody): int =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.meshDesc.faces.len

proc face*(body: SoftBody; index: int): SoftBodyFace =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if index < 0 or index >= body.meshDesc.faces.len:
    raise newException(IndexDefect, "soft body face index is out of bounds")
  body.meshDesc.faces[index]

proc volumeConstraintCount*(body: SoftBody): int =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.meshDesc.volumeConstraints.len

proc volumeConstraint*(body: SoftBody;
                       index: int): SoftBodyVolumeConstraint =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if index < 0 or index >= body.meshDesc.volumeConstraints.len:
    raise newException(IndexDefect, "soft body volume is out of bounds")
  body.meshDesc.volumeConstraints[index]

proc rodCount*(body: SoftBody): int =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.meshDesc.rods.len

proc rod*(body: SoftBody; index: int): SoftBodyRodConstraint =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if index < 0 or index >= body.meshDesc.rods.len:
    raise newException(IndexDefect, "soft body rod is out of bounds")
  body.meshDesc.rods[index]

proc rodBendTwistConstraintCount*(body: SoftBody): int =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.meshDesc.rodBendTwistConstraints.len

proc rodBendTwistConstraint*(body: SoftBody;
    index: int): SoftBodyRodBendTwistConstraint =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if index < 0 or index >= body.meshDesc.rodBendTwistConstraints.len:
    raise newException(IndexDefect, "rod bend-twist is out of bounds")
  body.meshDesc.rodBendTwistConstraints[index]

proc skinJointCount*(body: SoftBody): int =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.meshDesc.skinBindPose.len

proc skinBindJoint*(body: SoftBody; index: int): SoftBodyJointTransform =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if index < 0 or index >= body.meshDesc.skinBindPose.len:
    raise newException(IndexDefect, "soft body skin joint is out of bounds")
  body.meshDesc.skinBindPose[index]

proc skinConstraintCount*(body: SoftBody): int =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.meshDesc.skinConstraints.len

proc skinConstraint*(body: SoftBody; index: int): SoftBodySkinConstraint =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if index < 0 or index >= body.meshDesc.skinConstraints.len:
    raise newException(IndexDefect, "soft body skin constraint is out of bounds")
  result = body.meshDesc.skinConstraints[index]
  result.weights = newSeq[SoftBodySkinWeight](result.weights.len)
  for weightIndex, weight in body.meshDesc.skinConstraints[index].weights:
    result.weights[weightIndex] = weight

proc mesh*(body: SoftBody): SoftBodyMesh =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.meshDesc.copySoftBodyMesh()

proc configuration*(body: SoftBody): SoftBodyConfig =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.config

proc vertexState*(body: SoftBody; vertex: int): SoftBodyVertexState =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if vertex < 0 or vertex >= body.meshDesc.vertices.len:
    raise newException(IndexDefect, "soft body vertex index is out of bounds")
  var position, velocity: raw.Vec3
  var inverseMass: cfloat
  if not body.owner.physics.softBodyVertexState(
      raw.bodyID(body.rawId), uint32(vertex), addr position, addr velocity,
      addr inverseMass):
    raise newException(JoltError, "Jolt could not read the soft body vertex")
  SoftBodyVertexState(
    position: fromRaw(position),
    velocity: fromRaw(velocity),
    inverseMass: inverseMass)

proc vertices*(body: SoftBody): seq[SoftBodyVertexState] =
  let count = body.vertexCount
  result = newSeq[SoftBodyVertexState](count)
  for index in 0 ..< count:
    result[index] = body.vertexState(index)

proc runtimeState*(body: SoftBody): SoftBodyRuntimeState =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not body.owner.physics.softBodyRuntimeSettings(
      raw.bodyID(body.rawId), addr result.numIterations,
      addr result.pressure, addr result.vertexRadius, addr result.volume,
      addr result.updatePosition, addr result.facesDoubleSided,
      addr result.skinConstraintsEnabled,
      addr result.skinnedMaxDistanceMultiplier):
    raise newException(JoltError, "Jolt could not read soft body settings")

proc constraintCounts*(body: SoftBody): SoftBodyConstraintCounts =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  var edges, dihedralBends, volumes, longRangeAttachments: uint32
  var rods, rodBendTwists, skinned: uint32
  if not body.owner.physics.softBodyConstraintCounts(
      raw.bodyID(body.rawId), addr edges, addr dihedralBends, addr volumes,
      addr longRangeAttachments, addr rods, addr rodBendTwists, addr skinned):
    raise newException(JoltError, "Jolt could not read soft body constraints")
  SoftBodyConstraintCounts(
    edges: int(edges),
    dihedralBends: int(dihedralBends),
    volumes: int(volumes),
    longRangeAttachments: int(longRangeAttachments),
    rods: int(rods),
    rodBendTwists: int(rodBendTwists),
    skinned: int(skinned))

proc rodOptimizationRemap*(body: SoftBody): SoftBodyRodOptimizationRemap =
  ## Returns authored-to-native index maps produced by Jolt's constraint
  ## optimizer. The returned sequences are independent copies.
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  result.stretchShear = newSeq[uint32](body.rodRemap.len)
  for index, nativeIndex in body.rodRemap:
    result.stretchShear[index] = nativeIndex
  result.bendTwist = newSeq[uint32](body.rodPairRemap.len)
  for index, nativeIndex in body.rodPairRemap:
    result.bendTwist[index] = nativeIndex

proc rodNativeIndex*(body: SoftBody; rod: int): int =
  ## Maps an authored stretch/shear rod index to Jolt's optimized order.
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if rod < 0 or rod >= body.rodRemap.len:
    raise newException(IndexDefect, "soft body rod index is out of bounds")
  int(body.rodRemap[rod])

proc rodBendTwistNativeIndex*(body: SoftBody; constraint: int): int =
  ## Maps an authored bend/twist constraint index to Jolt's optimized order.
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if constraint < 0 or constraint >= body.rodPairRemap.len:
    raise newException(
      IndexDefect, "soft body rod bend/twist index is out of bounds")
  int(body.rodPairRemap[constraint])

proc rodState*(body: SoftBody; rod: int): SoftBodyRodState =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if rod < 0 or rod >= body.meshDesc.rods.len:
    raise newException(IndexDefect, "soft body rod index is out of bounds")
  var rotation: raw.Quat
  var angularVelocity: raw.Vec3
  if not body.owner.physics.softBodyRodState(
      raw.bodyID(body.rawId), body.rodRemap[rod], addr rotation,
      addr angularVelocity):
    raise newException(JoltError, "Jolt could not read soft body rod state")
  SoftBodyRodState(
    rotation: fromRaw(rotation), angularVelocity: fromRaw(angularVelocity))

proc localBounds*(body: SoftBody): SoftBodyBounds =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  var minimum, maximum: raw.Vec3
  if not body.owner.physics.softBodyLocalBounds(
      raw.bodyID(body.rawId), addr minimum, addr maximum):
    raise newException(JoltError, "Jolt could not read soft body bounds")
  SoftBodyBounds(minimum: fromRaw(minimum), maximum: fromRaw(maximum))

proc customUpdate*(body: SoftBody; deltaTime: float32) =
  ## Advances this soft body immediately on the calling thread. Call only
  ## between `World.step` operations. The body is temporarily removed from the
  ## broad phase, as required by Jolt, and reinserted with the same BodyId.
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(
      ValueError, "soft body custom update delta time must be positive")
  if not body.owner.physics.customUpdateSoftBody(
      raw.bodyID(body.rawId), deltaTime):
    raise newException(JoltError, "Jolt could not custom-update the soft body")

proc settle*(body: SoftBody; steps: SomeInteger;
             deltaTime = 1.0'f32 / 60.0'f32) =
  ## Performs a fixed number of immediate single-threaded SoftBody updates.
  if steps < 0 or uint64(steps) > uint64(high(int)):
    raise newException(ValueError, "soft body settle step count is invalid")
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(
      ValueError, "soft body settle delta time must be positive")
  for _ in 0 ..< int(steps):
    body.customUpdate(deltaTime)

proc applyRuntimeSettings(body: SoftBody) =
  if not body.owner.physics.setSoftBodyRuntimeSettings(
      raw.bodyID(body.rawId), body.config.numIterations,
      body.config.pressure, body.config.vertexRadius,
      body.config.updatePosition, body.config.facesDoubleSided,
      body.config.enableSkinConstraints,
      body.config.skinnedMaxDistanceMultiplier):
    raise newException(JoltError, "Jolt could not update soft body settings")

proc setNumIterations*(body: SoftBody; iterations: uint32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if iterations == 0:
    raise newException(ValueError, "soft body solver iterations must be positive")
  body.config.numIterations = iterations
  body.applyRuntimeSettings()

proc setPressure*(body: SoftBody; pressure: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not pressure.isFinite or pressure < 0:
    raise newException(ValueError, "soft body pressure must be non-negative")
  body.config.pressure = pressure
  body.applyRuntimeSettings()

proc setVertexRadius*(body: SoftBody; radius: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not radius.isFinite or radius < 0:
    raise newException(ValueError, "soft body vertex radius must be non-negative")
  body.config.vertexRadius = radius
  body.applyRuntimeSettings()

proc setUpdatePosition*(body: SoftBody; enabled: bool) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if enabled:
    for vertex in body.meshDesc.vertices:
      if vertex.inverseMass == 0:
        raise newException(
          ValueError, "a soft body with fixed vertices cannot update its position")
  body.config.updatePosition = enabled
  body.applyRuntimeSettings()

proc setFacesDoubleSided*(body: SoftBody; enabled: bool) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.config.facesDoubleSided = enabled
  body.applyRuntimeSettings()

proc setSkinConstraintsEnabled*(body: SoftBody; enabled: bool) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if body.meshDesc.skinConstraints.len == 0:
    raise newException(ValueError, "soft body has no skin constraints")
  body.config.enableSkinConstraints = enabled
  body.applyRuntimeSettings()

proc setSkinnedMaxDistanceMultiplier*(body: SoftBody; multiplier: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if body.meshDesc.skinConstraints.len == 0:
    raise newException(ValueError, "soft body has no skin constraints")
  if not multiplier.isFinite or multiplier < 0:
    raise newException(
      ValueError, "soft body skin distance multiplier must be non-negative")
  body.config.skinnedMaxDistanceMultiplier = multiplier
  body.applyRuntimeSettings()

proc skinVertices*(body: SoftBody;
                   jointTransforms: openArray[SoftBodyJointTransform];
                   hardSkinAll = false) =
  ## Updates the animated joint pose. Joint transforms are relative to the
  ## soft body's current center-of-mass transform.
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if body.meshDesc.skinConstraints.len == 0:
    raise newException(ValueError, "soft body has no skin constraints")
  if jointTransforms.len != body.meshDesc.skinBindPose.len:
    raise newException(
      ValueError, "soft body skin pose has the wrong number of joints")
  var positions = newSeq[raw.Vec3](jointTransforms.len)
  var rotations = newSeq[raw.Quat](jointTransforms.len)
  for index, joint in jointTransforms:
    if not joint.position.isFinite:
      raise newException(ValueError, "soft body joint position must be finite")
    positions[index] = joint.position.toRaw
    rotations[index] = joint.rotation.normalized.toRaw
  if not body.owner.physics.skinSoftBodyVertices(
      raw.bodyID(body.rawId), addr positions[0], addr rotations[0],
      uint32(jointTransforms.len), hardSkinAll, body.owner.allocator):
    raise newException(JoltError, "Jolt could not update soft body skinning")

proc setVertexVelocity*(body: SoftBody; vertex: int; velocity: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if vertex < 0 or vertex >= body.meshDesc.vertices.len:
    raise newException(IndexDefect, "soft body vertex index is out of bounds")
  velocity.requireFinite("soft body vertex velocity")
  if not body.owner.physics.setSoftBodyVertexVelocity(
      raw.bodyID(body.rawId), uint32(vertex), velocity.toRaw):
    raise newException(JoltError, "Jolt could not update soft body velocity")
  body.meshDesc.vertices[vertex].velocity = velocity

proc setVertexInverseMass*(body: SoftBody; vertex: int;
                           inverseMass: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if vertex < 0 or vertex >= body.meshDesc.vertices.len:
    raise newException(IndexDefect, "soft body vertex index is out of bounds")
  if not inverseMass.isFinite or inverseMass < 0:
    raise newException(
      ValueError, "soft body vertex inverse mass must be non-negative")
  if inverseMass == 0 and body.config.updatePosition:
    body.setUpdatePosition(false)
  if not body.owner.physics.setSoftBodyVertexInverseMass(
      raw.bodyID(body.rawId), uint32(vertex), inverseMass):
    raise newException(JoltError, "Jolt could not update soft body inverse mass")
  body.meshDesc.vertices[vertex].inverseMass = inverseMass

proc position*(body: SoftBody): Vec3 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  fromRaw(body.owner.physics.bodyInterface().position(raw.bodyID(body.rawId)))

proc setTransform*(body: SoftBody; position: Vec3;
                   rotation = quatIdentity(); activate = true) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  position.requireFinite("soft body position")
  body.owner.physics.bodyInterface().setPositionAndRotation(
    raw.bodyID(body.rawId), position.toRaw, rotation.normalized.toRaw,
    if activate: raw.EActivation.Activate else: raw.EActivation.DontActivate)

proc activate*(body: SoftBody) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().activate(raw.bodyID(body.rawId))

proc deactivate*(body: SoftBody) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().deactivate(raw.bodyID(body.rawId))

proc isActive*(body: SoftBody): bool =
  body.isAlive and
    body.owner.physics.bodyInterface().isActive(raw.bodyID(body.rawId))

proc collisionLayer*(body: SoftBody): CollisionLayer =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().objectLayer(raw.bodyID(body.rawId))

proc setCollisionLayer*(body: SoftBody; layer: CollisionLayer) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.requireLayer(layer)
  body.owner.physics.bodyInterface().setObjectLayer(raw.bodyID(body.rawId), layer)
  body.layer = layer

proc setCollisionGroup*(body: SoftBody; group: BodyCollisionGroup) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if group.filter.isNil:
    raise newException(ValueError, "collision group requires a filter")
  group.filter.validateSubgroup(group.subgroupId)
  body.owner.physics.setCollisionGroup(
    raw.bodyID(body.rawId), group.filter.native, group.groupId,
    group.subgroupId)
  body.group = some(group)

proc clearCollisionGroup*(body: SoftBody) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.clearCollisionGroup(raw.bodyID(body.rawId))
  body.group = none(BodyCollisionGroup)

proc collisionGroup*(body: SoftBody): Option[BodyCollisionGroup] =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  var groupId, subgroupId: uint32
  if not body.owner.physics.collisionGroup(
      raw.bodyID(body.rawId), addr groupId, addr subgroupId):
    return none(BodyCollisionGroup)
  if body.group.isNone:
    raise newException(
      JoltError, "native collision group has no matching Nim filter handle")
  let stored = body.group.get
  some(BodyCollisionGroup(
    filter: stored.filter, groupId: groupId, subgroupId: subgroupId))

proc userData*(body: SoftBody): uint64 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().userData(raw.bodyID(body.rawId))

proc setUserData*(body: SoftBody; value: uint64) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().setUserData(raw.bodyID(body.rawId), value)
  body.config.userData = value

proc allowsSleeping*(body: SoftBody): bool =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  var allowSleeping, collideKinematicVsNonDynamic: bool
  var applyGyroscopicForce, enhancedInternalEdgeRemoval: bool
  if not body.owner.physics.bodyCreationFlags(
      raw.bodyID(body.rawId), addr allowSleeping,
      addr collideKinematicVsNonDynamic, addr applyGyroscopicForce,
      addr enhancedInternalEdgeRemoval):
    raise newException(JoltError, "Jolt could not read soft body sleep setting")
  allowSleeping

proc setAllowSleeping*(body: SoftBody; enabled: bool) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not body.owner.physics.setBodyCreationFlag(
      raw.bodyID(body.rawId), 0, enabled):
    raise newException(JoltError, "Jolt could not update soft body sleep setting")
  body.config.allowSleeping = enabled

proc friction*(body: SoftBody): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().friction(raw.bodyID(body.rawId))

proc setFriction*(body: SoftBody; friction: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not friction.isFinite or friction < 0:
    raise newException(ValueError, "soft body friction must be non-negative")
  body.owner.physics.bodyInterface().setFriction(raw.bodyID(body.rawId), friction)
  body.config.friction = friction

proc restitution*(body: SoftBody): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().restitution(raw.bodyID(body.rawId))

proc setRestitution*(body: SoftBody; restitution: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not restitution.isFinite or restitution < 0:
    raise newException(ValueError, "soft body restitution must be non-negative")
  body.owner.physics.bodyInterface().setRestitution(
    raw.bodyID(body.rawId), restitution)
  body.config.restitution = restitution

proc gravityFactor*(body: SoftBody): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().gravityFactor(raw.bodyID(body.rawId))

proc setGravityFactor*(body: SoftBody; factor: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not factor.isFinite:
    raise newException(ValueError, "soft body gravity factor must be finite")
  body.owner.physics.bodyInterface().setGravityFactor(raw.bodyID(body.rawId), factor)
  body.config.gravityFactor = factor

proc maxLinearVelocity*(body: SoftBody): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  body.owner.physics.bodyInterface().maxLinearVelocity(raw.bodyID(body.rawId))

proc setMaxLinearVelocity*(body: SoftBody; velocity: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not velocity.isFinite or velocity <= 0:
    raise newException(
      ValueError, "soft body maximum velocity must be positive")
  body.owner.physics.bodyInterface().setMaxLinearVelocity(
    raw.bodyID(body.rawId), velocity)
  body.config.maxLinearVelocity = velocity

proc linearDamping*(body: SoftBody): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  var linear, angular: cfloat
  if not body.owner.physics.damping(
      raw.bodyID(body.rawId), addr linear, addr angular):
    raise newException(JoltError, "Jolt could not read soft body damping")
  linear

proc setLinearDamping*(body: SoftBody; damping: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  if not damping.isFinite or damping < 0:
    raise newException(ValueError, "soft body damping must be non-negative")
  var currentLinear, angular: cfloat
  if not body.owner.physics.damping(
      raw.bodyID(body.rawId), addr currentLinear, addr angular) or
      not body.owner.physics.setDamping(
        raw.bodyID(body.rawId), damping, angular):
    raise newException(JoltError, "Jolt could not update soft body damping")
  body.config.linearDamping = damping

proc validate(config: CharacterConfig) =
  if not config.maxSlopeAngle.isFinite or config.maxSlopeAngle < 0 or
      config.maxSlopeAngle >= PI.float32 * 0.5'f32:
    raise newException(
      ValueError,
      "character maxSlopeAngle must be finite and in [0, PI / 2)"
    )
  if not config.mass.isFinite or config.mass <= 0:
    raise newException(ValueError, "character mass must be finite and positive")
  if not config.maxStrength.isFinite or config.maxStrength < 0:
    raise newException(
      ValueError,
      "character maxStrength must be finite and non-negative"
    )
  if not config.padding.isFinite or config.padding <= 0:
    raise newException(ValueError, "character padding must be finite and positive")
  if not config.predictiveContactDistance.isFinite or
      config.predictiveContactDistance < 0:
    raise newException(
      ValueError,
      "character predictiveContactDistance must be finite and non-negative"
    )
  if config.maxNumHits == 0:
    raise newException(ValueError, "character maxNumHits must be positive")
  if not config.hitReductionCosMaxAngle.isFinite or
      config.hitReductionCosMaxAngle < -1 or
      config.hitReductionCosMaxAngle > 1:
    raise newException(
      ValueError, "character hitReductionCosMaxAngle must be in [-1, 1]")
  if not config.penetrationRecoverySpeed.isFinite or
      config.penetrationRecoverySpeed < 0 or
      config.penetrationRecoverySpeed > 1:
    raise newException(
      ValueError, "character penetrationRecoverySpeed must be in [0, 1]")
  if not config.stepUp.isFinite or config.stepUp < 0 or
      not config.stepDown.isFinite or config.stepDown < 0:
    raise newException(
      ValueError,
      "character step distances must be finite and non-negative"
    )
  if config.maxCollisionIterations == 0 or
      config.maxConstraintIterations == 0:
    raise newException(
      ValueError, "character collision and constraint iterations must be positive")
  if not config.minTimeRemaining.isFinite or config.minTimeRemaining < 0:
    raise newException(
      ValueError, "character minTimeRemaining must be finite and non-negative")
  if not config.collisionTolerance.isFinite or config.collisionTolerance < 0:
    raise newException(
      ValueError, "character collisionTolerance must be finite and non-negative")
  if config.maxQueuedContactEvents == 0:
    raise newException(
      ValueError, "character maxQueuedContactEvents must be positive")

proc newCharacter*(world: World; shape: Shape; position: Vec3;
                   config = defaultCharacterConfig();
                   rotation = quatIdentity();
                   layer = movingLayer): Character =
  world.requireOpen()
  world.requireLayer(layer)
  if config.innerBodyShape.isSome:
    world.requireLayer(config.innerBodyLayer)
  position.requireFinite("character position")
  config.validate()
  if shape.kind != ShapeKind.Capsule:
    raise newException(
      ValueError,
      "CharacterVirtual currently requires a capsule shape"
    )
  discard capsuleShape(shape.halfHeight, shape.radius)
  if config.padding >= shape.radius:
    raise newException(
      ValueError,
      "character padding must be smaller than the capsule radius"
    )

  let nativeShape = raw.newCapsuleShape(shape.halfHeight, shape.radius, nil)
  var cookedInnerBody: CookedShape
  if config.innerBodyShape.isSome:
    cookedInnerBody = cookShape(config.innerBodyShape.get, MotionType.Kinematic)
  defer: cookedInnerBody.release()
  let native = raw.newCharacter(
    world.physics,
    nativeShape,
    position.toRaw,
    rotation.normalized.toRaw,
    layer,
    shape.halfHeight + shape.radius,
    shape.radius,
    config.maxSlopeAngle,
    config.mass,
    config.maxStrength,
    config.padding,
    config.predictiveContactDistance,
    config.maxNumHits,
    config.hitReductionCosMaxAngle,
    config.penetrationRecoverySpeed,
    config.enhancedInternalEdgeRemoval,
    uint8(ord(config.backFaceMode)),
    config.maxCollisionIterations,
    config.maxConstraintIterations,
    config.minTimeRemaining,
    config.collisionTolerance,
    config.userData,
    cookedInnerBody.native,
    config.innerBodyLayer,
    config.maxQueuedContactEvents,
    config.canPushCharacter,
    config.canReceiveImpulses,
    config.preventSliding,
    world.characterBroadPhase
  )
  if native.isNil:
    raise newException(JoltError, "Jolt could not create the character")

  new(result)
  result.owner = world
  result.native = native
  result.shapeDesc = shape
  result.config = config
  result.layer = layer
  let innerBodyId = native.innerBodyID()
  if not raw.bodyID(innerBodyId).isInvalid:
    result.innerBodyIdValue = some(innerBodyId)
  result.alive = true
  world.characters.add(native)
  if result.innerBodyIdValue.isSome:
    world.characterBodyIds.add(result.innerBodyIdValue.get)

proc shape*(character: Character): Shape =
  if character.isNil:
    raise newException(JoltError, "Jolt character handle is nil")
  character.shapeDesc

proc collisionLayer*(character: Character): CollisionLayer =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.layer

proc configuration*(character: Character): CharacterConfig =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.config

proc characterId*(character: Character): uint32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.characterID()

proc innerBodyId*(character: Character): Option[BodyId] =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if character.innerBodyIdValue.isSome:
    some(BodyId(character.innerBodyIdValue.get))
  else:
    none(BodyId)

proc contacts*(character: Character): seq[CharacterContactInfo] =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  let count = character.native.contactCount()
  result = newSeq[CharacterContactInfo](int(count))
  for index in 0'u32 ..< count:
    var bodyId, otherCharacterId, subShapeId: uint32
    var position, linearVelocity, contactNormal, surfaceNormal: raw.Vec3
    var distance, fraction: cfloat
    var motionType: uint8
    var isSensor, hadCollision, wasDiscarded, canPushCharacter,
      isBackFacing: bool
    var userData: uint64
    character.native.contact(
      index,
      addr bodyId,
      addr otherCharacterId,
      addr subShapeId,
      addr position,
      addr linearVelocity,
      addr contactNormal,
      addr surfaceNormal,
      addr distance,
      addr fraction,
      addr motionType,
      addr isSensor,
      addr userData,
      addr hadCollision,
      addr wasDiscarded,
      addr canPushCharacter,
      addr isBackFacing)
    if not raw.bodyID(bodyId).isInvalid:
      result[int(index)].bodyId = some(BodyId(bodyId))
    if otherCharacterId != high(uint32):
      result[int(index)].characterId = some(otherCharacterId)
    result[int(index)].subShapeId = subShapeId
    result[int(index)].position = fromRaw(position)
    result[int(index)].linearVelocity = fromRaw(linearVelocity)
    result[int(index)].contactNormal = fromRaw(contactNormal)
    result[int(index)].surfaceNormal = fromRaw(surfaceNormal)
    result[int(index)].distance = distance
    result[int(index)].fraction = fraction
    if motionType > uint8(ord(high(MotionType))):
      raise newException(JoltError, "Jolt returned an unknown contact motion type")
    result[int(index)].motionType = MotionType(motionType)
    result[int(index)].isSensor = isSensor
    result[int(index)].userData = userData
    result[int(index)].hadCollision = hadCollision
    result[int(index)].wasDiscarded = wasDiscarded
    result[int(index)].canPushCharacter = canPushCharacter
    result[int(index)].isBackFacing = isBackFacing

proc pendingContactEventCount*(character: Character): uint32 =
  ## Returns events waiting in this character's bounded native queue.
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.pendingContactEventCount()

proc droppedContactEventCount*(character: Character; reset = false): uint64 =
  ## Returns how many oldest events were discarded when the queue was full.
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.droppedContactEventCount(reset)

proc drainContactEvents*(character: Character;
                         limit = high(int)): seq[CharacterContactEvent] =
  ## Moves contact lifecycle and solver events from the native queue to Nim.
  ## No Nim callback executes from inside Jolt.
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if limit < 0:
    raise newException(ValueError, "contact event limit must be non-negative")
  let requested = min(limit, int(character.native.pendingContactEventCount()))
  result = newSeqOfCap[CharacterContactEvent](requested)
  for _ in 0 ..< requested:
    var rawKind: uint8
    var bodyId, otherCharacterId, subShapeId: uint32
    var position, normal, contactVelocity, characterVelocity,
      resultingVelocity: raw.Vec3
    var userData: uint64
    var isSensor, canPushCharacter, canReceiveImpulses: bool
    if not character.native.popContactEvent(
        addr rawKind, addr bodyId, addr otherCharacterId, addr subShapeId,
        addr position, addr normal, addr contactVelocity,
        addr characterVelocity, addr resultingVelocity, addr userData,
        addr isSensor, addr canPushCharacter, addr canReceiveImpulses):
      break
    if rawKind > uint8(ord(high(CharacterContactEventKind))):
      raise newException(JoltError, "Jolt returned an unknown character event kind")
    var event = CharacterContactEvent(
      kind: CharacterContactEventKind(rawKind),
      subShapeId: subShapeId,
      position: fromRaw(position),
      normal: fromRaw(normal),
      contactVelocity: fromRaw(contactVelocity),
      characterVelocity: fromRaw(characterVelocity),
      resultingVelocity: fromRaw(resultingVelocity),
      userData: userData,
      isSensor: isSensor,
      canPushCharacter: canPushCharacter,
      canReceiveImpulses: canReceiveImpulses
    )
    if not raw.bodyID(bodyId).isInvalid:
      event.bodyId = some(BodyId(bodyId))
    if otherCharacterId != high(uint32):
      event.characterId = some(otherCharacterId)
    result.add(event)

proc setContactResponse*(character: Character; canPushCharacter,
                         canReceiveImpulses, preventSliding: bool) =
  ## Changes deterministic listener behavior without invoking Nim callbacks.
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.setContactResponse(
    character.config.maxQueuedContactEvents,
    canPushCharacter, canReceiveImpulses, preventSliding)
  character.config.canPushCharacter = canPushCharacter
  character.config.canReceiveImpulses = canReceiveImpulses
  character.config.preventSliding = preventSliding

proc setMaxQueuedContactEvents*(character: Character; capacity: uint32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if capacity == 0:
    raise newException(ValueError, "character event capacity must be positive")
  character.native.setContactResponse(
    capacity, character.config.canPushCharacter,
    character.config.canReceiveImpulses, character.config.preventSliding)
  character.config.maxQueuedContactEvents = capacity

proc hasCollidedWith*(character: Character; bodyId: BodyId): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  let value = uint32(bodyId)
  if value notin character.owner.bodyIds and
      value notin character.owner.characterBodyIds and
      value notin character.owner.rigidCharacterBodyIds and
      value notin character.owner.ragdollBodyIds:
    raise newException(ValueError, "body ID does not belong to this world")
  character.native.hasCollidedWithBody(value)

proc hasCollidedWith*(character: Character; body: Body): bool =
  if body.isNil or not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if body.owner != character.owner:
    raise newException(ValueError, "character and body belong to different worlds")
  character.native.hasCollidedWithBody(body.rawId)

proc hasCollidedWith*(character, other: Character): bool =
  if not character.isAlive or not other.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if character.owner != other.owner:
    raise newException(ValueError, "characters belong to different worlds")
  character.native.hasCollidedWithCharacter(other.native)

proc position*(character: Character): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  fromRaw(character.native.position())

proc setPosition*(character: Character; position: Vec3;
                  refreshContacts = true) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  position.requireFinite("character position")
  character.native.setPosition(position.toRaw)
  if refreshContacts:
    character.native.refreshContacts(character.owner.allocator)

proc rotation*(character: Character): Quat =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  fromRaw(character.native.rotation())

proc setRotation*(character: Character; rotation: Quat) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.setRotation(rotation.normalized.toRaw)

proc linearVelocity*(character: Character): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  fromRaw(character.native.linearVelocity())

proc setLinearVelocity*(character: Character; velocity: Vec3) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  velocity.requireFinite("character velocity")
  character.native.setLinearVelocity(velocity.toRaw)

proc groundState*(character: Character): CharacterGroundState =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  case character.native.groundState()
  of 0: CharacterGroundState.OnGround
  of 1: CharacterGroundState.OnSteepGround
  of 2: CharacterGroundState.NotSupported
  of 3: CharacterGroundState.InAir
  else: raise newException(JoltError, "Jolt returned an unknown character ground state")

proc isSupported*(character: Character): bool =
  character.isAlive and character.native.isSupported()

proc groundPosition*(character: Character): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  fromRaw(character.native.groundPosition())

proc groundNormal*(character: Character): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  fromRaw(character.native.groundNormal())

proc groundVelocity*(character: Character): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  fromRaw(character.native.groundVelocity())

proc groundBodyId*(character: Character): Option[BodyId] =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  let id = character.native.groundBodyID()
  if raw.bodyID(id).isInvalid:
    none(BodyId)
  else:
    some(BodyId(id))

proc activeContactCount*(character: Character): int =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  int(character.native.activeContactCount())

proc maxNumHits*(character: Character): uint32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.maxNumHits()

proc setMaxNumHits*(character: Character; value: uint32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if value == 0:
    raise newException(ValueError, "character maxNumHits must be positive")
  character.native.setMaxNumHits(value)
  character.config.maxNumHits = value

proc hitReductionCosMaxAngle*(character: Character): float32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.hitReductionCosMaxAngle()

proc setHitReductionCosMaxAngle*(character: Character; value: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if not value.isFinite or value < -1 or value > 1:
    raise newException(
      ValueError, "character hitReductionCosMaxAngle must be in [-1, 1]")
  character.native.setHitReductionCosMaxAngle(value)
  character.config.hitReductionCosMaxAngle = value

proc penetrationRecoverySpeed*(character: Character): float32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.penetrationRecoverySpeed()

proc setPenetrationRecoverySpeed*(character: Character; value: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if not value.isFinite or value < 0 or value > 1:
    raise newException(
      ValueError, "character penetrationRecoverySpeed must be in [0, 1]")
  character.native.setPenetrationRecoverySpeed(value)
  character.config.penetrationRecoverySpeed = value

proc maxHitsExceeded*(character: Character): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.maxHitsExceeded()

proc mass*(character: Character): float32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.mass()

proc setMass*(character: Character; mass: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if not mass.isFinite or mass <= 0:
    raise newException(ValueError, "character mass must be finite and positive")
  character.native.setMass(mass)
  character.config.mass = mass

proc maxStrength*(character: Character): float32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.maxStrength()

proc setMaxStrength*(character: Character; strength: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if not strength.isFinite or strength < 0:
    raise newException(
      ValueError, "character maxStrength must be finite and non-negative")
  character.native.setMaxStrength(strength)
  character.config.maxStrength = strength

proc enhancedInternalEdgeRemoval*(character: Character): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.enhancedInternalEdgeRemoval()

proc setEnhancedInternalEdgeRemoval*(character: Character; enabled: bool) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.setEnhancedInternalEdgeRemoval(enabled)
  character.config.enhancedInternalEdgeRemoval = enabled

proc userData*(character: Character): uint64 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.userData()

proc setUserData*(character: Character; value: uint64) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.setUserData(value)
  character.config.userData = value

proc setShape*(character: Character; shape: Shape;
               maxPenetrationDepth = maxFiniteFloat32): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if shape.kind != ShapeKind.Capsule:
    raise newException(ValueError, "CharacterVirtual requires a capsule shape")
  discard capsuleShape(shape.halfHeight, shape.radius)
  if character.config.padding >= shape.radius:
    raise newException(
      ValueError, "character padding must be smaller than the capsule radius")
  if not maxPenetrationDepth.isFinite or maxPenetrationDepth < 0:
    raise newException(
      ValueError, "maxPenetrationDepth must be finite and non-negative")
  let cooked = cookShape(shape, MotionType.Kinematic)
  defer: cooked.release()
  result = character.native.setShape(
    cooked.native, maxPenetrationDepth, character.owner.allocator)
  if result:
    character.native.setShapeOffset(
      Vec3(y: shape.halfHeight + shape.radius).toRaw)
    character.shapeDesc = shape

proc setInnerBodyShape*(character: Character; shape: Shape) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if character.innerBodyIdValue.isNone:
    raise newException(
      JoltError, "Jolt character does not have an inner body")
  let cooked = cookShape(shape, MotionType.Kinematic)
  defer: cooked.release()
  character.native.setInnerBodyShape(cooked.native)
  character.config.innerBodyShape = some(shape)

proc cancelVelocityTowardsSteepSlopes*(
    character: Character; desiredVelocity: Vec3): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  desiredVelocity.requireFinite("desired character velocity")
  fromRaw(character.native.cancelVelocityTowardsSteepSlopes(
    desiredVelocity.toRaw))

proc canWalkStairs*(character: Character; desiredVelocity: Vec3): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  desiredVelocity.requireFinite("desired character velocity")
  character.native.canWalkStairs(desiredVelocity.toRaw)

proc walkStairs*(character: Character; deltaTime: float32;
                 stepUp, stepForward, stepForwardTest,
                 stepDownExtra: Vec3): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  stepUp.requireFinite("character step up")
  stepForward.requireFinite("character step forward")
  stepForwardTest.requireFinite("character step forward test")
  stepDownExtra.requireFinite("character extra step down")
  character.native.walkStairs(
    deltaTime,
    stepUp.toRaw,
    stepForward.toRaw,
    stepForwardTest.toRaw,
    stepDownExtra.toRaw,
    character.owner.allocator)

proc stickToFloor*(character: Character; stepDown: Vec3): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  stepDown.requireFinite("character step down")
  character.native.stickToFloor(stepDown.toRaw, character.owner.allocator)

proc refreshContacts*(character: Character) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.native.refreshContacts(character.owner.allocator)

proc update*(character: Character; deltaTime: float32; gravity: Vec3) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  gravity.requireFinite("character gravity")
  character.native.update(
    deltaTime,
    gravity.toRaw,
    Vec3(y: character.config.stepUp).toRaw,
    Vec3(y: -character.config.stepDown).toRaw,
    character.owner.allocator
  )

proc update*(character: Character; deltaTime: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  character.update(deltaTime, fromRaw(character.owner.physics.gravity()))

proc move*(character: Character; desiredHorizontalVelocity: Vec3;
           deltaTime: float32; jump = false; jumpSpeed = 6.0'f32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt character is no longer alive")
  desiredHorizontalVelocity.requireFinite("desired horizontal velocity")
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  if not jumpSpeed.isFinite or jumpSpeed < 0:
    raise newException(ValueError, "jumpSpeed must be finite and non-negative")

  character.native.updateGroundVelocity()
  let current = character.linearVelocity()
  let ground = character.groundVelocity()
  let gravity = fromRaw(character.owner.physics.gravity())
  let grounded = character.groundState() == CharacterGroundState.OnGround and
    current.y - ground.y < 0.1'f32

  var velocity: Vec3
  velocity.x = desiredHorizontalVelocity.x
  velocity.y = current.y
  velocity.z = desiredHorizontalVelocity.z
  if grounded:
    velocity.x += ground.x
    velocity.y = ground.y
    velocity.z += ground.z
    if jump:
      velocity.y += jumpSpeed
  velocity.x += gravity.x * deltaTime
  velocity.y += gravity.y * deltaTime
  velocity.z += gravity.z * deltaTime
  character.setLinearVelocity(velocity)
  character.update(deltaTime, gravity)

proc validate(config: RigidCharacterConfig) =
  if not config.maxSlopeAngle.isFinite or config.maxSlopeAngle < 0 or
      config.maxSlopeAngle >= PI.float32 * 0.5'f32:
    raise newException(
      ValueError,
      "rigid character maxSlopeAngle must be finite and in [0, PI / 2)")
  if not config.up.isFinite:
    raise newException(ValueError, "rigid character up vector must be finite")
  let upLengthSquared = config.up.x * config.up.x +
    config.up.y * config.up.y + config.up.z * config.up.z
  if upLengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "rigid character up vector must be non-zero")
  if not config.supportingHeight.isFinite or config.supportingHeight < 0:
    raise newException(
      ValueError, "rigid character supportingHeight must be finite and non-negative")
  if not config.mass.isFinite or config.mass <= 0:
    raise newException(ValueError, "rigid character mass must be finite and positive")
  if not config.friction.isFinite or config.friction < 0:
    raise newException(
      ValueError, "rigid character friction must be finite and non-negative")
  if not config.gravityFactor.isFinite or config.gravityFactor < 0:
    raise newException(
      ValueError, "rigid character gravityFactor must be finite and non-negative")
  if config.allowedDOFs == {}:
    raise newException(
      ValueError, "rigid character requires at least one allowed DOF")
  if not config.maxSeparationDistance.isFinite or
      config.maxSeparationDistance < 0:
    raise newException(
      ValueError,
      "rigid character maxSeparationDistance must be finite and non-negative")

proc newRigidCharacter*(world: World; shape: Shape; position: Vec3;
                        config = defaultRigidCharacterConfig();
                        rotation = quatIdentity();
                        layer = movingLayer;
                        activate = true): RigidCharacter =
  ## Creates Jolt's body-backed Character. Unlike CharacterVirtual, this
  ## character participates in the rigid-body solver and is affected by its
  ## configured gravity factor.
  world.requireOpen()
  world.requireLayer(layer)
  position.requireFinite("rigid character position")
  config.validate()
  let normalizedRotation = rotation.normalized
  let upLength = sqrt(
    config.up.x * config.up.x + config.up.y * config.up.y +
    config.up.z * config.up.z)
  let normalizedUp = Vec3(
    x: config.up.x / upLength,
    y: config.up.y / upLength,
    z: config.up.z / upLength)
  let cooked = cookShape(shape, MotionType.Dynamic)
  defer: cooked.release()
  let native = raw.newRigidCharacter(
    world.physics,
    cooked.native,
    position.toRaw,
    normalizedRotation.toRaw,
    layer,
    normalizedUp.toRaw,
    config.supportingHeight,
    config.maxSlopeAngle,
    config.mass,
    config.friction,
    config.gravityFactor,
    allowedDOFMask(config.allowedDOFs),
    config.enhancedInternalEdgeRemoval,
    config.userData,
    config.maxSeparationDistance,
    activate)
  if native.isNil:
    raise newException(JoltError, "Jolt could not create the rigid character")

  new(result)
  result.owner = world
  result.native = native
  result.shapeDesc = shape
  result.config = config
  result.config.up = normalizedUp
  result.layer = layer
  result.rawId = native.bodyID()
  result.alive = true
  world.rigidCharacters.add(native)
  world.rigidCharacterBodyIds.add(result.rawId)

proc shape*(character: RigidCharacter): Shape =
  if character.isNil:
    raise newException(JoltError, "Jolt rigid character handle is nil")
  character.shapeDesc

proc bodyId*(character: RigidCharacter): BodyId =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  BodyId(character.rawId)

proc collisionLayer*(character: RigidCharacter): CollisionLayer =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.layer

proc setCollisionLayer*(character: RigidCharacter; layer: CollisionLayer) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.owner.requireLayer(layer)
  character.native.setCollisionLayer(layer)
  character.layer = layer

proc configuration*(character: RigidCharacter): RigidCharacterConfig =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.config

proc position*(character: RigidCharacter): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.position())

proc setPosition*(character: RigidCharacter; position: Vec3;
                  activate = true) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  position.requireFinite("rigid character position")
  character.native.setPosition(position.toRaw, activate)

proc rotation*(character: RigidCharacter): Quat =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.rotation())

proc setRotation*(character: RigidCharacter; rotation: Quat;
                  activate = true) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.native.setRotation(rotation.normalized.toRaw, activate)

proc centerOfMassPosition*(character: RigidCharacter): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.centerOfMassPosition())

proc linearVelocity*(character: RigidCharacter): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.linearVelocity())

proc setLinearVelocity*(character: RigidCharacter; velocity: Vec3) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  velocity.requireFinite("rigid character velocity")
  character.native.setLinearVelocity(velocity.toRaw)

proc addLinearVelocity*(character: RigidCharacter; velocity: Vec3) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  velocity.requireFinite("rigid character velocity")
  character.native.addLinearVelocity(velocity.toRaw)

proc addImpulse*(character: RigidCharacter; impulse: Vec3) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  impulse.requireFinite("rigid character impulse")
  character.native.addImpulse(impulse.toRaw)

proc activate*(character: RigidCharacter) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.native.activate()

proc groundState*(character: RigidCharacter): CharacterGroundState =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  case character.native.groundState()
  of 0: CharacterGroundState.OnGround
  of 1: CharacterGroundState.OnSteepGround
  of 2: CharacterGroundState.NotSupported
  of 3: CharacterGroundState.InAir
  else:
    raise newException(
      JoltError, "Jolt returned an unknown rigid character ground state")

proc isSupported*(character: RigidCharacter): bool =
  character.isAlive and character.native.isSupported()

proc groundPosition*(character: RigidCharacter): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.groundPosition())

proc groundNormal*(character: RigidCharacter): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.groundNormal())

proc groundVelocity*(character: RigidCharacter): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.groundVelocity())

proc groundBodyId*(character: RigidCharacter): Option[BodyId] =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  let id = character.native.groundBodyID()
  if raw.bodyID(id).isInvalid: none(BodyId) else: some(BodyId(id))

proc groundSubShapeId*(character: RigidCharacter): Option[uint32] =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  if character.groundBodyId.isSome:
    some(character.native.groundSubShapeID())
  else:
    none(uint32)

proc groundUserData*(character: RigidCharacter): uint64 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.native.groundUserData()

proc refreshGround*(character: RigidCharacter;
                    maxSeparationDistance: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  if not maxSeparationDistance.isFinite or maxSeparationDistance < 0:
    raise newException(
      ValueError, "maxSeparationDistance must be finite and non-negative")
  character.native.refreshGround(maxSeparationDistance)
  character.config.maxSeparationDistance = maxSeparationDistance

proc refreshGround*(character: RigidCharacter) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.native.refreshGround(character.config.maxSeparationDistance)

proc maxSlopeAngle*(character: RigidCharacter): float32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.native.maxSlopeAngle()

proc setMaxSlopeAngle*(character: RigidCharacter; angle: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  if not angle.isFinite or angle < 0 or angle >= PI.float32 * 0.5'f32:
    raise newException(ValueError, "maxSlopeAngle must be in [0, PI / 2)")
  character.native.setMaxSlopeAngle(angle)
  character.config.maxSlopeAngle = angle

proc up*(character: RigidCharacter): Vec3 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  fromRaw(character.native.up())

proc setUp*(character: RigidCharacter; up: Vec3) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  up.requireFinite("rigid character up vector")
  let lengthSquared = up.x * up.x + up.y * up.y + up.z * up.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "rigid character up vector must be non-zero")
  let inverseLength = 1.0'f32 / sqrt(lengthSquared)
  let normalizedUp = Vec3(
    x: up.x * inverseLength,
    y: up.y * inverseLength,
    z: up.z * inverseLength)
  character.native.setUp(normalizedUp.toRaw)
  character.native.setSupportingHeight(character.config.supportingHeight)
  character.config.up = normalizedUp

proc supportingHeight*(character: RigidCharacter): float32 =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  character.native.supportingHeight()

proc setSupportingHeight*(character: RigidCharacter; height: float32) =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  if not height.isFinite or height < 0:
    raise newException(
      ValueError, "supportingHeight must be finite and non-negative")
  character.native.setSupportingHeight(height)
  character.config.supportingHeight = height

proc isSlopeTooSteep*(character: RigidCharacter; normal: Vec3): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  normal.requireFinite("surface normal")
  character.native.isSlopeTooSteep(normal.toRaw)

proc setShape*(character: RigidCharacter; shape: Shape;
               maxPenetrationDepth = maxFiniteFloat32): bool =
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  if not maxPenetrationDepth.isFinite or maxPenetrationDepth < 0:
    raise newException(
      ValueError, "maxPenetrationDepth must be finite and non-negative")
  let cooked = cookShape(shape, MotionType.Dynamic)
  defer: cooked.release()
  result = character.native.setShape(cooked.native, maxPenetrationDepth)
  if result:
    character.shapeDesc = shape

proc move*(character: RigidCharacter; desiredHorizontalVelocity: Vec3;
           jump = false; jumpSpeed = 6.0'f32) =
  ## Sets velocity relative to supporting ground. Gravity is applied by the
  ## body's normal rigid-body integration during World.step.
  if not character.isAlive:
    raise newException(JoltError, "Jolt rigid character is no longer alive")
  desiredHorizontalVelocity.requireFinite("desired horizontal velocity")
  if not jumpSpeed.isFinite or jumpSpeed < 0:
    raise newException(ValueError, "jumpSpeed must be finite and non-negative")
  let up = character.up
  let desiredUp = desiredHorizontalVelocity.x * up.x +
    desiredHorizontalVelocity.y * up.y + desiredHorizontalVelocity.z * up.z
  let horizontal = Vec3(
    x: desiredHorizontalVelocity.x - up.x * desiredUp,
    y: desiredHorizontalVelocity.y - up.y * desiredUp,
    z: desiredHorizontalVelocity.z - up.z * desiredUp)
  let current = character.linearVelocity
  let currentUp = current.x * up.x + current.y * up.y + current.z * up.z
  var velocity = Vec3(
    x: horizontal.x + up.x * currentUp,
    y: horizontal.y + up.y * currentUp,
    z: horizontal.z + up.z * currentUp)
  if character.isSupported:
    let ground = character.groundVelocity
    velocity = Vec3(
      x: horizontal.x + ground.x,
      y: horizontal.y + ground.y,
      z: horizontal.z + ground.z)
    if jump:
      velocity = Vec3(
        x: velocity.x + up.x * jumpSpeed,
        y: velocity.y + up.y * jumpSpeed,
        z: velocity.z + up.z * jumpSpeed)
  character.setLinearVelocity(velocity)

proc validateVehicleTireFrictionCurve(
    curve: openArray[VehicleTireFrictionPoint]; name: string) =
  if curve.len == 0:
    return
  if curve.len < 2 or uint64(curve.len) > uint64(high(uint32)):
    raise newException(
      ValueError, name & " must be empty or contain at least two points")
  var previousSlip = -1.0'f32
  for point in curve:
    if not point.slip.isFinite or point.slip < 0 or
        point.slip <= previousSlip or
        not point.friction.isFinite or point.friction < 0:
      raise newException(
        ValueError,
        name & " slip values must be finite, non-negative, and strictly " &
          "increasing; friction values must be finite and non-negative")
    previousSlip = point.slip

proc validate(config: VehicleConfig; chassis: Shape) =
  if not config.wheelRadius.isFinite or config.wheelRadius <= 0:
    raise newException(ValueError, "vehicle wheelRadius must be finite and positive")
  if not config.wheelWidth.isFinite or config.wheelWidth <= 0:
    raise newException(ValueError, "vehicle wheelWidth must be finite and positive")
  if not config.suspensionMinLength.isFinite or
      not config.suspensionMaxLength.isFinite or
      config.suspensionMinLength < 0 or
      config.suspensionMaxLength < config.suspensionMinLength:
    raise newException(
      ValueError,
      "vehicle suspension lengths must be finite, non-negative, and ordered")
  if not config.suspensionFrequency.isFinite or config.suspensionFrequency <= 0:
    raise newException(
      ValueError, "vehicle suspensionFrequency must be finite and positive")
  if not config.suspensionDamping.isFinite or config.suspensionDamping < 0:
    raise newException(
      ValueError, "vehicle suspensionDamping must be finite and non-negative")
  if not config.maxSteerAngle.isFinite or config.maxSteerAngle < 0 or
      config.maxSteerAngle > PI.float32 * 0.5'f32:
    raise newException(
      ValueError, "vehicle maxSteerAngle must be finite and in [0, PI / 2]")
  if not config.maxPitchRollAngle.isFinite or config.maxPitchRollAngle <= 0 or
      config.maxPitchRollAngle > PI.float32:
    raise newException(
      ValueError, "vehicle maxPitchRollAngle must be finite and in (0, PI]")
  if config.controllerKind == VehicleControllerKind.Motorcycle:
    if config.wheels.len != 2:
      raise newException(
        ValueError, "a motorcycle controller requires exactly two wheels")
    if not config.maxLeanAngle.isFinite or config.maxLeanAngle < 0 or
        config.maxLeanAngle > PI.float32 * 0.5'f32:
      raise newException(
        ValueError, "motorcycle maxLeanAngle must be in [0, PI / 2]")
    if not config.leanSpringConstant.isFinite or
        config.leanSpringConstant < 0 or
        not config.leanSpringDamping.isFinite or
        config.leanSpringDamping < 0 or
        not config.leanSpringIntegrationCoefficient.isFinite or
        config.leanSpringIntegrationCoefficient < 0 or
        not config.leanSpringIntegrationCoefficientDecay.isFinite or
        config.leanSpringIntegrationCoefficientDecay < 0 or
        not config.leanSmoothingFactor.isFinite or
        config.leanSmoothingFactor < 0 or config.leanSmoothingFactor > 1:
      raise newException(
        ValueError, "motorcycle lean controller settings are invalid")
  if not config.engineMaxTorque.isFinite or config.engineMaxTorque <= 0:
    raise newException(
      ValueError, "vehicle engineMaxTorque must be finite and positive")
  if not config.engineMinRPM.isFinite or config.engineMinRPM <= 0 or
      not config.engineMaxRPM.isFinite or
      config.engineMaxRPM <= config.engineMinRPM:
    raise newException(
      ValueError, "vehicle engine RPM range must be finite, positive, and ordered")
  if not config.engineInertia.isFinite or config.engineInertia <= 0:
    raise newException(
      ValueError, "vehicle engineInertia must be finite and positive")
  if not config.engineAngularDamping.isFinite or
      config.engineAngularDamping < 0:
    raise newException(
      ValueError, "vehicle engineAngularDamping must be finite and non-negative")
  if config.engineTorqueCurve.len < 2 or
      uint64(config.engineTorqueCurve.len) > uint64(high(uint32)):
    raise newException(
      ValueError, "vehicle engineTorqueCurve must contain at least two points")
  var previousRPMFraction = -1.0'f32
  for point in config.engineTorqueCurve:
    if not point.rpmFraction.isFinite or point.rpmFraction < 0 or
        point.rpmFraction > 1 or point.rpmFraction <= previousRPMFraction or
        not point.torqueFraction.isFinite or point.torqueFraction < 0:
      raise newException(
        ValueError,
        "vehicle torque-curve RPM fractions must increase in [0, 1] and torque fractions must be non-negative")
    previousRPMFraction = point.rpmFraction
  if config.gearRatios.len == 0 or config.reverseGearRatios.len == 0 or
      uint64(config.gearRatios.len) > uint64(high(int32)) or
      uint64(config.reverseGearRatios.len) > uint64(high(int32)):
    raise newException(
      ValueError, "vehicle forward and reverse gear ratios must not be empty")
  for ratio in config.gearRatios:
    if not ratio.isFinite or ratio <= 0:
      raise newException(
        ValueError, "vehicle forward gear ratios must be finite and positive")
  for ratio in config.reverseGearRatios:
    if not ratio.isFinite or ratio >= 0:
      raise newException(
        ValueError, "vehicle reverse gear ratios must be finite and negative")
  if not config.transmissionSwitchTime.isFinite or
      config.transmissionSwitchTime < 0 or
      not config.clutchReleaseTime.isFinite or config.clutchReleaseTime < 0 or
      not config.transmissionSwitchLatency.isFinite or
      config.transmissionSwitchLatency < 0:
    raise newException(
      ValueError, "vehicle transmission timings must be finite and non-negative")
  if not config.shiftUpRPM.isFinite or not config.shiftDownRPM.isFinite or
      config.shiftDownRPM < config.engineMinRPM or
      config.shiftUpRPM > config.engineMaxRPM or
      config.shiftDownRPM >= config.shiftUpRPM:
    raise newException(
      ValueError, "vehicle shift RPM values must be ordered inside the engine range")
  if not config.clutchStrength.isFinite or config.clutchStrength <= 0:
    raise newException(
      ValueError, "vehicle clutchStrength must be finite and positive")
  if not config.frontTorqueRatio.isFinite or config.frontTorqueRatio < 0 or
      config.frontTorqueRatio > 1:
    raise newException(
      ValueError, "vehicle frontTorqueRatio must be finite and in [0, 1]")
  if not config.differentialRatio.isFinite or config.differentialRatio <= 0:
    raise newException(
      ValueError, "vehicle differentialRatio must be finite and positive")
  if not config.differentialLeftRightSplit.isFinite or
      config.differentialLeftRightSplit < 0 or
      config.differentialLeftRightSplit > 1:
    raise newException(
      ValueError, "vehicle differentialLeftRightSplit must be in [0, 1]")
  if not config.differentialLimitedSlipRatio.isFinite or
      config.differentialLimitedSlipRatio <= 1 or
      not config.centerDifferentialLimitedSlipRatio.isFinite or
      config.centerDifferentialLimitedSlipRatio <= 1:
    raise newException(
      ValueError, "vehicle differential limited-slip ratios must exceed 1")
  if not config.wheelTrack.isFinite or config.wheelTrack < 0:
    raise newException(
      ValueError, "vehicle wheelTrack must be finite and non-negative")
  if not config.frontAxleOffset.isFinite or config.frontAxleOffset < 0 or
      not config.rearAxleOffset.isFinite or config.rearAxleOffset < 0:
    raise newException(
      ValueError, "vehicle axle offsets must be finite and non-negative")
  if not config.suspensionAttachmentHeightRatio.isFinite:
    raise newException(
      ValueError, "vehicle suspensionAttachmentHeightRatio must be finite")
  if not config.rearMaxSteerAngle.isFinite or config.rearMaxSteerAngle < 0 or
      config.rearMaxSteerAngle > PI.float32 * 0.5'f32:
    raise newException(
      ValueError, "vehicle rearMaxSteerAngle must be finite and in [0, PI / 2]")
  if not config.frontBrakeTorque.isFinite or config.frontBrakeTorque < 0 or
      not config.rearBrakeTorque.isFinite or config.rearBrakeTorque < 0 or
      not config.rearHandBrakeTorque.isFinite or config.rearHandBrakeTorque < 0:
    raise newException(
      ValueError, "vehicle brake torques must be finite and non-negative")
  if not config.antiRollBarStiffness.isFinite or
      config.antiRollBarStiffness < 0:
    raise newException(
      ValueError, "vehicle antiRollBarStiffness must be finite and non-negative")
  if not config.wheelInertia.isFinite or config.wheelInertia <= 0:
    raise newException(
      ValueError, "vehicle wheelInertia must be finite and positive")
  if not config.wheelAngularDamping.isFinite or
      config.wheelAngularDamping < 0:
    raise newException(
      ValueError, "vehicle wheelAngularDamping must be finite and non-negative")
  if not config.tireLongitudinalImpulseMultiplier.isFinite or
      config.tireLongitudinalImpulseMultiplier < 0 or
      not config.tireLateralImpulseMultiplier.isFinite or
      config.tireLateralImpulseMultiplier < 0:
    raise newException(
      ValueError, "vehicle tire impulse multipliers must be finite and non-negative")
  if not config.wheelCollisionUp.isUnitVector:
    raise newException(
      ValueError, "vehicle wheelCollisionUp must be a finite unit vector")
  if not config.wheelCollisionMaxSlopeAngle.isFinite or
      config.wheelCollisionMaxSlopeAngle <= 0 or
      config.wheelCollisionMaxSlopeAngle > PI.float32 * 0.5'f32:
    raise newException(
      ValueError,
      "vehicle wheelCollisionMaxSlopeAngle must be finite and in (0, PI / 2]")
  if not config.wheelSphereCastRadius.isFinite or
      config.wheelSphereCastRadius <= 0:
    raise newException(
      ValueError, "vehicle wheelSphereCastRadius must be finite and positive")
  if not config.wheelCylinderConvexRadiusFraction.isFinite or
      config.wheelCylinderConvexRadiusFraction < 0 or
      config.wheelCylinderConvexRadiusFraction > 1:
    raise newException(
      ValueError,
      "vehicle wheelCylinderConvexRadiusFraction must be finite and in [0, 1]")
  if config.wheels.len == 0:
    if config.differentials.len > 0 or config.antiRollBars.len > 0:
      raise newException(
        ValueError,
        "custom differentials and anti-roll bars require custom wheels")
  else:
    if config.wheels.len < 2 or
        uint64(config.wheels.len) > uint64(high(int32)):
      raise newException(
        ValueError, "a custom vehicle must contain 2 to int32.high wheels")
    if config.differentials.len == 0 or
        uint64(config.differentials.len) > uint64(high(uint32)) or
        uint64(config.antiRollBars.len) > uint64(high(uint32)):
      raise newException(
        ValueError, "custom vehicles require at least one valid differential")
    for wheel in config.wheels:
      if not wheel.position.isFinite or
          not wheel.suspensionForcePoint.isFinite or
          not wheel.suspensionDirection.isUnitVector or
          not wheel.steeringAxis.isUnitVector or
          not wheel.wheelUp.isUnitVector or
          not wheel.wheelForward.isUnitVector:
        raise newException(
          ValueError,
          "custom wheel positions must be finite and direction vectors must be unit length")
      if not wheel.suspensionMinLength.isFinite or
          not wheel.suspensionMaxLength.isFinite or
          wheel.suspensionMinLength < 0 or
          wheel.suspensionMaxLength < wheel.suspensionMinLength or
          not wheel.suspensionPreloadLength.isFinite or
          wheel.suspensionPreloadLength < 0:
        raise newException(
          ValueError, "custom wheel suspension lengths must be finite and ordered")
      if not wheel.suspensionFrequency.isFinite or
          wheel.suspensionFrequency <= 0 or
          not wheel.suspensionDamping.isFinite or
          wheel.suspensionDamping < 0:
        raise newException(
          ValueError, "custom wheel suspension spring settings are invalid")
      if not wheel.radius.isFinite or wheel.radius <= 0 or
          not wheel.width.isFinite or wheel.width <= 0 or
          not wheel.inertia.isFinite or wheel.inertia <= 0 or
          not wheel.angularDamping.isFinite or wheel.angularDamping < 0:
        raise newException(
          ValueError, "custom wheel dimensions and inertia must be positive and finite")
      if not wheel.maxSteerAngle.isFinite or wheel.maxSteerAngle < 0 or
          wheel.maxSteerAngle > PI.float32 * 0.5'f32 or
          not wheel.maxBrakeTorque.isFinite or wheel.maxBrakeTorque < 0 or
          not wheel.maxHandBrakeTorque.isFinite or
          wheel.maxHandBrakeTorque < 0:
        raise newException(
          ValueError, "custom wheel steering and brake settings are invalid")
      if not wheel.longitudinalImpulseMultiplier.isFinite or
          wheel.longitudinalImpulseMultiplier < 0 or
          not wheel.lateralImpulseMultiplier.isFinite or
          wheel.lateralImpulseMultiplier < 0:
        raise newException(
          ValueError, "custom wheel tire impulse multipliers must be non-negative")
      wheel.longitudinalFrictionCurve.validateVehicleTireFrictionCurve(
        "custom wheel longitudinalFrictionCurve")
      wheel.lateralFrictionCurve.validateVehicleTireFrictionCurve(
        "custom wheel lateralFrictionCurve")
    var totalEngineTorqueRatio = 0.0'f32
    for differential in config.differentials:
      if differential.leftWheel < -1 or
          differential.leftWheel >= config.wheels.len or
          differential.rightWheel < -1 or
          differential.rightWheel >= config.wheels.len or
          (differential.leftWheel < 0 and differential.rightWheel < 0) or
          differential.leftWheel == differential.rightWheel:
        raise newException(
          ValueError, "custom differential wheel indices are invalid")
      if not differential.differentialRatio.isFinite or
          differential.differentialRatio <= 0 or
          not differential.leftRightSplit.isFinite or
          differential.leftRightSplit < 0 or differential.leftRightSplit > 1 or
          not differential.limitedSlipRatio.isFinite or
          differential.limitedSlipRatio <= 1 or
          not differential.engineTorqueRatio.isFinite or
          differential.engineTorqueRatio < 0:
        raise newException(
          ValueError, "custom differential ratios and split are invalid")
      totalEngineTorqueRatio += differential.engineTorqueRatio
    if abs(totalEngineTorqueRatio - 1.0'f32) > 1.0e-3'f32:
      raise newException(
        ValueError, "custom differential engine torque ratios must sum to 1")
    for antiRollBar in config.antiRollBars:
      if antiRollBar.leftWheel < 0 or
          antiRollBar.leftWheel >= config.wheels.len or
          antiRollBar.rightWheel < 0 or
          antiRollBar.rightWheel >= config.wheels.len or
          antiRollBar.leftWheel == antiRollBar.rightWheel or
          not antiRollBar.stiffness.isFinite or antiRollBar.stiffness < 0:
        raise newException(
          ValueError, "custom anti-roll bar wheel indices or stiffness are invalid")
  if config.wheels.len == 0 and
      chassis.halfExtent.z <= 2.0'f32 * config.wheelRadius:
    raise newException(
      ValueError, "vehicle chassis is too short for the configured wheel radius")

proc copyVehicleConfig(config: VehicleConfig): VehicleConfig =
  result = config
  result.engineTorqueCurve = newSeq[VehicleTorquePoint](
    config.engineTorqueCurve.len)
  for index, point in config.engineTorqueCurve:
    result.engineTorqueCurve[index] = point
  result.gearRatios = newSeq[float32](config.gearRatios.len)
  for index, ratio in config.gearRatios:
    result.gearRatios[index] = ratio
  result.reverseGearRatios = newSeq[float32](config.reverseGearRatios.len)
  for index, ratio in config.reverseGearRatios:
    result.reverseGearRatios[index] = ratio
  result.wheels = newSeq[VehicleWheelConfig](config.wheels.len)
  for index, wheel in config.wheels:
    result.wheels[index] = wheel
    result.wheels[index].longitudinalFrictionCurve =
      newSeq[VehicleTireFrictionPoint](wheel.longitudinalFrictionCurve.len)
    for pointIndex, point in wheel.longitudinalFrictionCurve:
      result.wheels[index].longitudinalFrictionCurve[pointIndex] = point
    result.wheels[index].lateralFrictionCurve =
      newSeq[VehicleTireFrictionPoint](wheel.lateralFrictionCurve.len)
    for pointIndex, point in wheel.lateralFrictionCurve:
      result.wheels[index].lateralFrictionCurve[pointIndex] = point
  result.differentials = newSeq[VehicleDifferentialConfig](
    config.differentials.len)
  for index, differential in config.differentials:
    result.differentials[index] = differential
  result.antiRollBars = newSeq[VehicleAntiRollBarConfig](
    config.antiRollBars.len)
  for index, antiRollBar in config.antiRollBars:
    result.antiRollBars[index] = antiRollBar

proc validate(config: TrackedVehicleConfig; chassis: Shape) =
  if not config.maxPitchRollAngle.isFinite or
      config.maxPitchRollAngle <= 0 or config.maxPitchRollAngle > PI.float32:
    raise newException(
      ValueError, "tracked vehicle maxPitchRollAngle must be in (0, PI]")
  if not config.engineMaxTorque.isFinite or config.engineMaxTorque <= 0 or
      not config.engineMinRPM.isFinite or config.engineMinRPM <= 0 or
      not config.engineMaxRPM.isFinite or
      config.engineMaxRPM <= config.engineMinRPM or
      not config.engineInertia.isFinite or config.engineInertia <= 0 or
      not config.engineAngularDamping.isFinite or
      config.engineAngularDamping < 0:
    raise newException(
      ValueError, "tracked vehicle engine settings are invalid")
  if config.engineTorqueCurve.len < 2 or
      uint64(config.engineTorqueCurve.len) > uint64(high(uint32)):
    raise newException(
      ValueError, "tracked vehicle engineTorqueCurve requires at least two points")
  var previousRPMFraction = -1.0'f32
  for point in config.engineTorqueCurve:
    if not point.rpmFraction.isFinite or point.rpmFraction < 0 or
        point.rpmFraction > 1 or point.rpmFraction <= previousRPMFraction or
        not point.torqueFraction.isFinite or point.torqueFraction < 0:
      raise newException(ValueError, "tracked vehicle torque curve is invalid")
    previousRPMFraction = point.rpmFraction
  if config.gearRatios.len == 0 or config.reverseGearRatios.len == 0 or
      uint64(config.gearRatios.len) > uint64(high(int32)) or
      uint64(config.reverseGearRatios.len) > uint64(high(int32)):
    raise newException(
      ValueError, "tracked vehicle forward and reverse gears must not be empty")
  for ratio in config.gearRatios:
    if not ratio.isFinite or ratio <= 0:
      raise newException(
        ValueError, "tracked vehicle forward gear ratios must be positive")
  for ratio in config.reverseGearRatios:
    if not ratio.isFinite or ratio >= 0:
      raise newException(
        ValueError, "tracked vehicle reverse gear ratios must be negative")
  if not config.transmissionSwitchTime.isFinite or
      config.transmissionSwitchTime < 0 or
      not config.clutchReleaseTime.isFinite or config.clutchReleaseTime < 0 or
      not config.transmissionSwitchLatency.isFinite or
      config.transmissionSwitchLatency < 0 or
      not config.shiftUpRPM.isFinite or not config.shiftDownRPM.isFinite or
      config.shiftDownRPM < config.engineMinRPM or
      config.shiftUpRPM > config.engineMaxRPM or
      config.shiftDownRPM >= config.shiftUpRPM or
      not config.clutchStrength.isFinite or config.clutchStrength <= 0:
    raise newException(
      ValueError, "tracked vehicle transmission settings are invalid")
  if config.wheels.len < 2 or
      uint64(config.wheels.len) > uint64(high(uint32)):
    raise newException(
      ValueError, "tracked vehicle requires at least two wheels")
  for wheel in config.wheels:
    if not wheel.position.isFinite or
        not wheel.suspensionForcePoint.isFinite or
        not wheel.suspensionDirection.isUnitVector or
        not wheel.steeringAxis.isUnitVector or
        not wheel.wheelUp.isUnitVector or
        not wheel.wheelForward.isUnitVector:
      raise newException(
        ValueError, "tracked vehicle wheel frame is invalid")
    if not wheel.suspensionMinLength.isFinite or
        not wheel.suspensionMaxLength.isFinite or
        wheel.suspensionMinLength < 0 or
        wheel.suspensionMaxLength < wheel.suspensionMinLength or
        not wheel.suspensionPreloadLength.isFinite or
        wheel.suspensionPreloadLength < 0 or
        not wheel.suspensionFrequency.isFinite or
        wheel.suspensionFrequency <= 0 or
        not wheel.suspensionDamping.isFinite or wheel.suspensionDamping < 0:
      raise newException(
        ValueError, "tracked vehicle wheel suspension is invalid")
    if not wheel.radius.isFinite or wheel.radius <= 0 or
        not wheel.width.isFinite or wheel.width <= 0 or
        not wheel.longitudinalFriction.isFinite or
        wheel.longitudinalFriction < 0 or
        not wheel.lateralFriction.isFinite or wheel.lateralFriction < 0:
      raise newException(
        ValueError, "tracked vehicle wheel dimensions or friction are invalid")
  var assigned = newSeq[bool](config.wheels.len)
  for side in TrackedVehicleSide:
    let track = config.tracks[side]
    if track.wheelIndices.len == 0 or
        uint64(track.wheelIndices.len) > uint64(high(uint32)):
      raise newException(
        ValueError, "each tracked vehicle track requires wheels")
    var drivenFound = false
    for wheelIndex in track.wheelIndices:
      if wheelIndex < 0 or wheelIndex >= config.wheels.len or
          assigned[wheelIndex]:
        raise newException(
          ValueError, "tracked vehicle wheel indices are invalid or duplicated")
      assigned[wheelIndex] = true
      if wheelIndex == track.drivenWheel:
        drivenFound = true
    if not drivenFound:
      raise newException(
        ValueError, "tracked vehicle drivenWheel must belong to its track")
    if not track.inertia.isFinite or track.inertia <= 0 or
        not track.angularDamping.isFinite or track.angularDamping < 0 or
        not track.maxBrakeTorque.isFinite or track.maxBrakeTorque < 0 or
        not track.differentialRatio.isFinite or track.differentialRatio <= 0:
      raise newException(ValueError, "tracked vehicle track settings are invalid")
  for isAssigned in assigned:
    if not isAssigned:
      raise newException(
        ValueError, "every tracked vehicle wheel must belong to one track")
  if not config.wheelCollisionUp.isUnitVector or
      not config.wheelCollisionMaxSlopeAngle.isFinite or
      config.wheelCollisionMaxSlopeAngle <= 0 or
      config.wheelCollisionMaxSlopeAngle > PI.float32 * 0.5 or
      not config.wheelSphereCastRadius.isFinite or
      config.wheelSphereCastRadius <= 0 or
      not config.wheelCylinderConvexRadiusFraction.isFinite or
      config.wheelCylinderConvexRadiusFraction < 0 or
      config.wheelCylinderConvexRadiusFraction > 1:
    raise newException(
      ValueError, "tracked vehicle wheel collision settings are invalid")
  if chassis.halfExtent.x <= 0 or chassis.halfExtent.z <= 0:
    raise newException(ValueError, "tracked vehicle chassis is invalid")

proc copyTrackedVehicleConfig(
    config: TrackedVehicleConfig): TrackedVehicleConfig =
  result = config
  result.engineTorqueCurve = newSeq[VehicleTorquePoint](
    config.engineTorqueCurve.len)
  for index, point in config.engineTorqueCurve:
    result.engineTorqueCurve[index] = point
  result.gearRatios = newSeq[float32](config.gearRatios.len)
  for index, ratio in config.gearRatios:
    result.gearRatios[index] = ratio
  result.reverseGearRatios = newSeq[float32](config.reverseGearRatios.len)
  for index, ratio in config.reverseGearRatios:
    result.reverseGearRatios[index] = ratio
  result.wheels = newSeq[TrackedVehicleWheelConfig](config.wheels.len)
  for index, wheel in config.wheels:
    result.wheels[index] = wheel
  for side in TrackedVehicleSide:
    result.tracks[side].wheelIndices = newSeq[int](
      config.tracks[side].wheelIndices.len)
    for index, wheelIndex in config.tracks[side].wheelIndices:
      result.tracks[side].wheelIndices[index] = wheelIndex

proc newVehicle*(chassis: Body;
                 config = defaultVehicleConfig();
                 wheelCollisionLayer = movingLayer): Vehicle =
  if not chassis.isAlive:
    raise newException(JoltError, "vehicle chassis must be alive")
  if chassis.motion != MotionType.Dynamic:
    raise newException(ValueError, "vehicle chassis must be dynamic")
  let chassisBox =
    if chassis.shapeDesc.kind == ShapeKind.Box:
      chassis.shapeDesc
    elif chassis.shapeDesc.kind == ShapeKind.OffsetCenterOfMass and
        chassis.shapeDesc.innerShapes.len == 1 and
        chassis.shapeDesc.innerShapes[0].kind == ShapeKind.Box:
      chassis.shapeDesc.innerShapes[0]
    else:
      raise newException(
        ValueError,
        "vehicle chassis must use a box or center-of-mass-offset box shape")
  config.validate(chassisBox)
  chassis.owner.requireLayer(wheelCollisionLayer)

  let extent = chassisBox.halfExtent
  var torqueRPMFractions = newSeq[cfloat](config.engineTorqueCurve.len)
  var torqueFractions = newSeq[cfloat](config.engineTorqueCurve.len)
  for index, point in config.engineTorqueCurve:
    torqueRPMFractions[index] = point.rpmFraction
    torqueFractions[index] = point.torqueFraction
  var gearRatios = newSeq[cfloat](config.gearRatios.len)
  var reverseGearRatios = newSeq[cfloat](config.reverseGearRatios.len)
  for index, ratio in config.gearRatios:
    gearRatios[index] = ratio
  for index, ratio in config.reverseGearRatios:
    reverseGearRatios[index] = ratio
  var longitudinalFrictionSlips: seq[cfloat]
  var longitudinalFrictionValues: seq[cfloat]
  var lateralFrictionSlips: seq[cfloat]
  var lateralFrictionValues: seq[cfloat]
  var longitudinalFrictionOffsets = newSeq[int](config.wheels.len)
  var lateralFrictionOffsets = newSeq[int](config.wheels.len)
  for index, wheel in config.wheels:
    longitudinalFrictionOffsets[index] = longitudinalFrictionSlips.len
    for point in wheel.longitudinalFrictionCurve:
      longitudinalFrictionSlips.add(point.slip)
      longitudinalFrictionValues.add(point.friction)
    lateralFrictionOffsets[index] = lateralFrictionSlips.len
    for point in wheel.lateralFrictionCurve:
      lateralFrictionSlips.add(point.slip)
      lateralFrictionValues.add(point.friction)
  var nativeWheels = newSeq[raw.VehicleWheelConfigData](config.wheels.len)
  for index, wheel in config.wheels:
    let longitudinalOffset = longitudinalFrictionOffsets[index]
    let lateralOffset = lateralFrictionOffsets[index]
    nativeWheels[index] = raw.vehicleWheelConfigData(
      wheel.position.toRaw,
      wheel.suspensionForcePoint.toRaw,
      wheel.suspensionDirection.toRaw,
      wheel.steeringAxis.toRaw,
      wheel.wheelUp.toRaw,
      wheel.wheelForward.toRaw,
      wheel.suspensionMinLength,
      wheel.suspensionMaxLength,
      wheel.suspensionPreloadLength,
      wheel.suspensionFrequency,
      wheel.suspensionDamping,
      wheel.radius,
      wheel.width,
      wheel.enableSuspensionForcePoint,
      wheel.inertia,
      wheel.angularDamping,
      wheel.maxSteerAngle,
      wheel.maxBrakeTorque,
      wheel.maxHandBrakeTorque,
      wheel.longitudinalImpulseMultiplier,
      wheel.lateralImpulseMultiplier,
      if wheel.longitudinalFrictionCurve.len > 0:
        addr longitudinalFrictionSlips[longitudinalOffset]
      else:
        nil,
      if wheel.longitudinalFrictionCurve.len > 0:
        addr longitudinalFrictionValues[longitudinalOffset]
      else:
        nil,
      uint32(wheel.longitudinalFrictionCurve.len),
      if wheel.lateralFrictionCurve.len > 0:
        addr lateralFrictionSlips[lateralOffset]
      else:
        nil,
      if wheel.lateralFrictionCurve.len > 0:
        addr lateralFrictionValues[lateralOffset]
      else:
        nil,
      uint32(wheel.lateralFrictionCurve.len))
  var nativeDifferentials = newSeq[raw.VehicleDifferentialConfigData](
    config.differentials.len)
  for index, differential in config.differentials:
    nativeDifferentials[index] = raw.vehicleDifferentialConfigData(
      int32(differential.leftWheel),
      int32(differential.rightWheel),
      differential.differentialRatio,
      differential.leftRightSplit,
      differential.limitedSlipRatio,
      differential.engineTorqueRatio)
  var nativeAntiRollBars = newSeq[raw.VehicleAntiRollBarConfigData](
    config.antiRollBars.len)
  for index, antiRollBar in config.antiRollBars:
    nativeAntiRollBars[index] = raw.vehicleAntiRollBarConfigData(
      int32(antiRollBar.leftWheel),
      int32(antiRollBar.rightWheel),
      antiRollBar.stiffness)
  let native = raw.newVehicle(
    chassis.owner.physics,
    raw.bodyID(chassis.rawId),
    extent.x,
    extent.y,
    extent.z,
    config.wheelRadius,
    config.wheelWidth,
    config.suspensionMinLength,
    config.suspensionMaxLength,
    config.suspensionFrequency,
    config.suspensionDamping,
    config.maxSteerAngle,
    config.maxPitchRollAngle,
    config.engineMaxTorque,
    config.engineMinRPM,
    config.engineMaxRPM,
    config.engineInertia,
    config.engineAngularDamping,
    addr torqueRPMFractions[0],
    addr torqueFractions[0],
    uint32(torqueRPMFractions.len),
    uint8(config.transmissionMode),
    addr gearRatios[0],
    uint32(gearRatios.len),
    addr reverseGearRatios[0],
    uint32(reverseGearRatios.len),
    config.transmissionSwitchTime,
    config.clutchReleaseTime,
    config.transmissionSwitchLatency,
    config.shiftUpRPM,
    config.shiftDownRPM,
    config.clutchStrength,
    config.fourWheelDrive,
    config.frontWheelDrive,
    config.frontTorqueRatio,
    config.differentialRatio,
    config.differentialLeftRightSplit,
    config.differentialLimitedSlipRatio,
    config.centerDifferentialLimitedSlipRatio,
    config.wheelTrack,
    config.frontAxleOffset,
    config.rearAxleOffset,
    config.suspensionAttachmentHeightRatio,
    config.rearMaxSteerAngle,
    config.frontBrakeTorque,
    config.rearBrakeTorque,
    config.rearHandBrakeTorque,
    config.antiRollBarStiffness,
    config.wheelInertia,
    config.wheelAngularDamping,
    config.tireLongitudinalImpulseMultiplier,
    config.tireLateralImpulseMultiplier,
    if nativeWheels.len > 0: addr nativeWheels[0] else: nil,
    uint32(nativeWheels.len),
    if nativeDifferentials.len > 0: addr nativeDifferentials[0] else: nil,
    uint32(nativeDifferentials.len),
    if nativeAntiRollBars.len > 0: addr nativeAntiRollBars[0] else: nil,
    uint32(nativeAntiRollBars.len),
    uint8(config.wheelCollisionMode),
    config.wheelCollisionUp.toRaw,
    config.wheelCollisionMaxSlopeAngle,
    config.wheelSphereCastRadius,
    config.wheelCylinderConvexRadiusFraction,
    wheelCollisionLayer,
    uint8(ord(config.controllerKind)),
    config.maxLeanAngle,
    config.leanSpringConstant,
    config.leanSpringDamping,
    config.leanSpringIntegrationCoefficient,
    config.leanSpringIntegrationCoefficientDecay,
    config.leanSmoothingFactor,
    config.enableLeanController,
    config.enableLeanSteeringLimit)
  if native.isNil:
    raise newException(JoltError, "Jolt could not create the vehicle")

  new(result)
  result.owner = chassis.owner
  result.native = native
  result.chassisBody = chassis
  result.config = config.copyVehicleConfig()
  result.wheelLayer = wheelCollisionLayer
  result.alive = true
  inc chassis.constraintCount
  chassis.owner.vehicles.add(native)

proc newMotorcycle*(chassis: Body;
                    config = defaultMotorcycleConfig();
                    wheelCollisionLayer = movingLayer): Vehicle =
  ## Creates Jolt's two-wheel MotorcycleController while retaining the common
  ## Vehicle powertrain, wheel telemetry, rollback and ownership API.
  if config.controllerKind != VehicleControllerKind.Motorcycle:
    raise newException(
      ValueError, "newMotorcycle requires a motorcycle controller config")
  chassis.newVehicle(config, wheelCollisionLayer)

proc chassis*(vehicle: Vehicle): Body =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  vehicle.chassisBody

proc configuration*(vehicle: Vehicle): VehicleConfig =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  vehicle.config.copyVehicleConfig()

proc wheelCollisionLayer*(vehicle: Vehicle): CollisionLayer =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  vehicle.wheelLayer

proc isMotorcycle*(vehicle: Vehicle): bool =
  vehicle.isAlive and
    vehicle.config.controllerKind == VehicleControllerKind.Motorcycle

proc motorcycleControllerState*(
    vehicle: Vehicle): MotorcycleControllerState =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  if vehicle.config.controllerKind != VehicleControllerKind.Motorcycle:
    raise newException(ValueError, "vehicle does not use MotorcycleController")
  vehicle.native.motorcycleControllerState(
    addr result.wheelBase,
    addr result.leanControllerEnabled,
    addr result.leanSteeringLimitEnabled,
    addr result.leanSpringConstant,
    addr result.leanSpringDamping,
    addr result.leanSpringIntegrationCoefficient,
    addr result.leanSpringIntegrationCoefficientDecay,
    addr result.leanSmoothingFactor)

proc configureMotorcycleLean*(vehicle: Vehicle;
    leanControllerEnabled, leanSteeringLimitEnabled: bool;
    springConstant, springDamping, integrationCoefficient,
    integrationCoefficientDecay, smoothingFactor: float32) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  if vehicle.config.controllerKind != VehicleControllerKind.Motorcycle:
    raise newException(ValueError, "vehicle does not use MotorcycleController")
  if not springConstant.isFinite or springConstant < 0 or
      not springDamping.isFinite or springDamping < 0 or
      not integrationCoefficient.isFinite or integrationCoefficient < 0 or
      not integrationCoefficientDecay.isFinite or
      integrationCoefficientDecay < 0 or
      not smoothingFactor.isFinite or smoothingFactor < 0 or
      smoothingFactor > 1:
    raise newException(ValueError, "motorcycle lean settings are invalid")
  vehicle.native.configureMotorcycleController(
    leanControllerEnabled, leanSteeringLimitEnabled,
    springConstant, springDamping, integrationCoefficient,
    integrationCoefficientDecay, smoothingFactor)
  vehicle.config.enableLeanController = leanControllerEnabled
  vehicle.config.enableLeanSteeringLimit = leanSteeringLimitEnabled
  vehicle.config.leanSpringConstant = springConstant
  vehicle.config.leanSpringDamping = springDamping
  vehicle.config.leanSpringIntegrationCoefficient = integrationCoefficient
  vehicle.config.leanSpringIntegrationCoefficientDecay =
    integrationCoefficientDecay
  vehicle.config.leanSmoothingFactor = smoothingFactor

proc setMotorcycleLeanControllerEnabled*(vehicle: Vehicle; enabled: bool) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  let config = vehicle.config
  vehicle.configureMotorcycleLean(
    enabled, config.enableLeanSteeringLimit,
    config.leanSpringConstant, config.leanSpringDamping,
    config.leanSpringIntegrationCoefficient,
    config.leanSpringIntegrationCoefficientDecay,
    config.leanSmoothingFactor)

proc setMotorcycleLeanSteeringLimitEnabled*(vehicle: Vehicle; enabled: bool) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  let config = vehicle.config
  vehicle.configureMotorcycleLean(
    config.enableLeanController, enabled,
    config.leanSpringConstant, config.leanSpringDamping,
    config.leanSpringIntegrationCoefficient,
    config.leanSpringIntegrationCoefficientDecay,
    config.leanSmoothingFactor)

proc wheelCount*(vehicle: Vehicle): int =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  int(vehicle.native.wheelCount())

proc setInput*(vehicle: Vehicle; forward, steering: float32;
               brake = 0.0'f32; handBrake = 0.0'f32) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  if not forward.isFinite or forward < -1 or forward > 1:
    raise newException(ValueError, "vehicle forward input must be finite and in [-1, 1]")
  if not steering.isFinite or steering < -1 or steering > 1:
    raise newException(ValueError, "vehicle steering input must be finite and in [-1, 1]")
  if not brake.isFinite or brake < 0 or brake > 1 or
      not handBrake.isFinite or handBrake < 0 or handBrake > 1:
    raise newException(
      ValueError, "vehicle brake inputs must be finite and in [0, 1]")
  vehicle.native.setInput(forward, steering, brake, handBrake)

proc powertrainState*(vehicle: Vehicle): VehiclePowertrainState =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  var engineRPM, clutchFriction, transmissionRatio,
    wheelSpeedAtClutch: cfloat
  var currentGear: int32
  var switchingGear: bool
  vehicle.native.powertrainState(
    addr engineRPM,
    addr currentGear,
    addr clutchFriction,
    addr switchingGear,
    addr transmissionRatio,
    addr wheelSpeedAtClutch)
  VehiclePowertrainState(
    engineRPM: engineRPM,
    currentGear: int(currentGear),
    clutchFriction: clutchFriction,
    switchingGear: switchingGear,
    transmissionRatio: transmissionRatio,
    wheelSpeedAtClutch: wheelSpeedAtClutch)

proc setEngineRPM*(vehicle: Vehicle; rpm: float32) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  if not rpm.isFinite or rpm < 0:
    raise newException(
      ValueError, "vehicle engine RPM must be finite and non-negative")
  vehicle.native.setEngineRPM(rpm)

proc setTransmission*(vehicle: Vehicle; gear: int;
                      clutchFriction: float32) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  if vehicle.config.transmissionMode != VehicleTransmissionMode.Manual:
    raise newException(
      ValueError, "manual transmission input requires manual mode")
  if gear < -vehicle.config.reverseGearRatios.len or
      gear > vehicle.config.gearRatios.len:
    raise newException(ValueError, "vehicle gear is outside the configured range")
  if not clutchFriction.isFinite or clutchFriction < 0 or clutchFriction > 1:
    raise newException(
      ValueError, "vehicle clutch friction must be finite and in [0, 1]")
  vehicle.native.setTransmission(int32(gear), clutchFriction)

proc differentialCount*(vehicle: Vehicle): int =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  int(vehicle.native.differentialCount())

proc differentialState*(vehicle: Vehicle;
                        differential: int): VehicleDifferentialState =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  let count = vehicle.differentialCount()
  if differential < 0 or differential >= count:
    raise newException(IndexDefect, "vehicle differential index is out of bounds")
  var leftWheel, rightWheel: int32
  var ratio, split, limitedSlip, engineTorque: cfloat
  vehicle.native.differentialState(
    uint32(differential), addr leftWheel, addr rightWheel,
    addr ratio, addr split, addr limitedSlip, addr engineTorque)
  VehicleDifferentialState(
    leftWheel: int(leftWheel),
    rightWheel: int(rightWheel),
    differentialRatio: ratio,
    leftRightSplit: split,
    limitedSlipRatio: limitedSlip,
    engineTorqueRatio: engineTorque)

proc antiRollBarCount*(vehicle: Vehicle): int =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  int(vehicle.native.antiRollBarCount())

proc antiRollBarState*(vehicle: Vehicle;
                       antiRollBar: int): VehicleAntiRollBarConfig =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  let count = vehicle.antiRollBarCount()
  if antiRollBar < 0 or antiRollBar >= count:
    raise newException(IndexDefect, "vehicle anti-roll bar index is out of bounds")
  var leftWheel, rightWheel: int32
  var stiffness: cfloat
  vehicle.native.antiRollBarState(
    uint32(antiRollBar), addr leftWheel, addr rightWheel, addr stiffness)
  VehicleAntiRollBarConfig(
    leftWheel: int(leftWheel),
    rightWheel: int(rightWheel),
    stiffness: stiffness)

proc wheelState*(vehicle: Vehicle; wheel: int): VehicleWheelState =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt vehicle is no longer alive")
  let count = vehicle.wheelCount()
  if wheel < 0 or wheel >= count:
    raise newException(IndexDefect, "vehicle wheel index is out of bounds")
  let index = uint32(wheel)
  result.position = fromRaw(vehicle.native.wheelPosition(index))
  result.rotation = fromRaw(vehicle.native.wheelRotation(index))
  result.hasContact = vehicle.native.wheelHasContact(index)
  result.suspensionLength = vehicle.native.wheelSuspensionLength(index)
  result.angularVelocity = vehicle.native.wheelAngularVelocity(index)
  result.steerAngle = vehicle.native.wheelSteerAngle(index)
  vehicle.native.wheelDynamics(
    index,
    addr result.longitudinalSlip,
    addr result.lateralSlip,
    addr result.combinedLongitudinalFriction,
    addr result.combinedLateralFriction)
  vehicle.native.wheelConstraintState(
    index,
    addr result.hitHardPoint,
    addr result.suspensionImpulse,
    addr result.longitudinalImpulse,
    addr result.lateralImpulse)
  if result.hasContact:
    let contactId = vehicle.native.wheelContactBodyID(index)
    if not raw.bodyID(contactId).isInvalid:
      result.contactBodyId = some(BodyId(contactId))
    var subShapeId: uint32
    var pointVelocity, longitudinal, lateral: raw.Vec3
    vehicle.native.wheelContactDetails(
      index,
      addr subShapeId,
      addr pointVelocity,
      addr longitudinal,
      addr lateral)
    result.contactSubShapeId = some(subShapeId)
    result.contactPosition = fromRaw(
      vehicle.native.wheelContactPosition(index))
    result.contactNormal = fromRaw(vehicle.native.wheelContactNormal(index))
    result.contactPointVelocity = fromRaw(pointVelocity)
    result.contactLongitudinal = fromRaw(longitudinal)
    result.contactLateral = fromRaw(lateral)

proc material*(state: VehicleWheelState;
               world: World): Option[PhysicsMaterial] =
  if state.contactBodyId.isNone or state.contactSubShapeId.isNone:
    return none(PhysicsMaterial)
  world.requireOpen()
  var name: cstring
  var red, green, blue, alpha: uint8
  if not world.physics.bodyMaterial(
      raw.bodyID(uint32(state.contactBodyId.get)),
      state.contactSubShapeId.get,
      addr name, addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

proc newTrackedVehicle*(
    chassis: Body; config = defaultTrackedVehicleConfig();
    wheelCollisionLayer = movingLayer): TrackedVehicle =
  if not chassis.isAlive:
    raise newException(JoltError, "tracked vehicle chassis must be alive")
  if chassis.motion != MotionType.Dynamic:
    raise newException(ValueError, "tracked vehicle chassis must be dynamic")
  if chassis.shapeDesc.kind != ShapeKind.Box:
    raise newException(
      ValueError, "tracked vehicle chassis must currently use a box shape")
  config.validate(chassis.shapeDesc)
  chassis.owner.requireLayer(wheelCollisionLayer)

  var torqueRPMFractions = newSeq[cfloat](config.engineTorqueCurve.len)
  var torqueFractions = newSeq[cfloat](config.engineTorqueCurve.len)
  for index, point in config.engineTorqueCurve:
    torqueRPMFractions[index] = point.rpmFraction
    torqueFractions[index] = point.torqueFraction
  var gearRatios = newSeq[cfloat](config.gearRatios.len)
  var reverseGearRatios = newSeq[cfloat](config.reverseGearRatios.len)
  for index, ratio in config.gearRatios:
    gearRatios[index] = ratio
  for index, ratio in config.reverseGearRatios:
    reverseGearRatios[index] = ratio
  var nativeWheels = newSeq[raw.TrackedVehicleWheelConfigData](
    config.wheels.len)
  for index, wheel in config.wheels:
    nativeWheels[index] = raw.trackedVehicleWheelConfigData(
      wheel.position.toRaw,
      wheel.suspensionForcePoint.toRaw,
      wheel.suspensionDirection.toRaw,
      wheel.steeringAxis.toRaw,
      wheel.wheelUp.toRaw,
      wheel.wheelForward.toRaw,
      wheel.suspensionMinLength,
      wheel.suspensionMaxLength,
      wheel.suspensionPreloadLength,
      wheel.suspensionFrequency,
      wheel.suspensionDamping,
      wheel.radius,
      wheel.width,
      wheel.enableSuspensionForcePoint,
      wheel.longitudinalFriction,
      wheel.lateralFriction)
  var trackIndices: array[TrackedVehicleSide, seq[uint32]]
  for side in TrackedVehicleSide:
    trackIndices[side] = newSeq[uint32](config.tracks[side].wheelIndices.len)
    for index, wheelIndex in config.tracks[side].wheelIndices:
      trackIndices[side][index] = uint32(wheelIndex)
  let left = config.tracks[TrackedVehicleSide.LeftTrack]
  let right = config.tracks[TrackedVehicleSide.RightTrack]
  let native = raw.newTrackedVehicle(
    chassis.owner.physics,
    raw.bodyID(chassis.rawId),
    config.maxPitchRollAngle,
    config.engineMaxTorque,
    config.engineMinRPM,
    config.engineMaxRPM,
    config.engineInertia,
    config.engineAngularDamping,
    addr torqueRPMFractions[0],
    addr torqueFractions[0],
    uint32(torqueRPMFractions.len),
    uint8(config.transmissionMode),
    addr gearRatios[0],
    uint32(gearRatios.len),
    addr reverseGearRatios[0],
    uint32(reverseGearRatios.len),
    config.transmissionSwitchTime,
    config.clutchReleaseTime,
    config.transmissionSwitchLatency,
    config.shiftUpRPM,
    config.shiftDownRPM,
    config.clutchStrength,
    addr nativeWheels[0],
    uint32(nativeWheels.len),
    addr trackIndices[TrackedVehicleSide.LeftTrack][0],
    uint32(trackIndices[TrackedVehicleSide.LeftTrack].len),
    uint32(left.drivenWheel),
    left.inertia,
    left.angularDamping,
    left.maxBrakeTorque,
    left.differentialRatio,
    addr trackIndices[TrackedVehicleSide.RightTrack][0],
    uint32(trackIndices[TrackedVehicleSide.RightTrack].len),
    uint32(right.drivenWheel),
    right.inertia,
    right.angularDamping,
    right.maxBrakeTorque,
    right.differentialRatio,
    uint8(config.wheelCollisionMode),
    config.wheelCollisionUp.toRaw,
    config.wheelCollisionMaxSlopeAngle,
    config.wheelSphereCastRadius,
    config.wheelCylinderConvexRadiusFraction,
    wheelCollisionLayer)
  if native.isNil:
    raise newException(JoltError, "Jolt could not create the tracked vehicle")

  new(result)
  result.owner = chassis.owner
  result.native = native
  result.chassisBody = chassis
  result.config = config.copyTrackedVehicleConfig()
  result.wheelLayer = wheelCollisionLayer
  result.alive = true
  inc chassis.constraintCount
  chassis.owner.vehicles.add(native)

proc chassis*(vehicle: TrackedVehicle): Body =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  vehicle.chassisBody

proc configuration*(vehicle: TrackedVehicle): TrackedVehicleConfig =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  vehicle.config.copyTrackedVehicleConfig()

proc wheelCollisionLayer*(vehicle: TrackedVehicle): CollisionLayer =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  vehicle.wheelLayer

proc wheelCount*(vehicle: TrackedVehicle): int =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  int(vehicle.native.wheelCount())

proc setInput*(vehicle: TrackedVehicle; forward, leftRatio,
               rightRatio: float32; brake = 0.0'f32) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  if not forward.isFinite or forward < -1 or forward > 1 or
      not leftRatio.isFinite or leftRatio < -1 or leftRatio > 1 or
      abs(leftRatio) < 1.0e-6 or
      not rightRatio.isFinite or rightRatio < -1 or rightRatio > 1 or
      abs(rightRatio) < 1.0e-6 or
      not brake.isFinite or brake < 0 or brake > 1:
    raise newException(
      ValueError,
      "tracked input must be finite; forward/ratios in [-1, 1], ratios " &
        "nonzero, and brake in [0, 1]")
  vehicle.native.setTrackedInput(forward, leftRatio, rightRatio, brake)

proc powertrainState*(
    vehicle: TrackedVehicle): TrackedVehiclePowertrainState =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  var gear: int32
  vehicle.native.trackedPowertrainState(
    addr result.engineRPM,
    addr gear,
    addr result.clutchFriction,
    addr result.switchingGear,
    addr result.transmissionRatio)
  result.currentGear = int(gear)

proc trackState*(vehicle: TrackedVehicle;
                 side: TrackedVehicleSide): TrackedVehicleTrackState =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  var drivenWheel: uint32
  vehicle.native.trackedTrackState(
    uint32(ord(side)),
    addr drivenWheel,
    addr result.inertia,
    addr result.angularDamping,
    addr result.maxBrakeTorque,
    addr result.differentialRatio,
    addr result.angularVelocity)
  result.drivenWheel = int(drivenWheel)
  result.wheelIndices = newSeq[int](vehicle.config.tracks[side].wheelIndices.len)
  for index, wheelIndex in vehicle.config.tracks[side].wheelIndices:
    result.wheelIndices[index] = wheelIndex

proc setEngineRPM*(vehicle: TrackedVehicle; rpm: float32) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  if not rpm.isFinite or rpm < 0:
    raise newException(ValueError, "tracked vehicle engine RPM must be non-negative")
  vehicle.native.setTrackedEngineRPM(rpm)

proc setTransmission*(vehicle: TrackedVehicle; gear: int;
                      clutchFriction: float32) =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  if gear < -vehicle.config.reverseGearRatios.len or
      gear > vehicle.config.gearRatios.len:
    raise newException(ValueError, "tracked vehicle gear is out of range")
  if not clutchFriction.isFinite or clutchFriction < 0 or clutchFriction > 1:
    raise newException(
      ValueError, "tracked vehicle clutch friction must be in [0, 1]")
  vehicle.native.setTrackedTransmission(int32(gear), clutchFriction)

proc wheelState*(vehicle: TrackedVehicle; wheel: int): VehicleWheelState =
  if not vehicle.isAlive:
    raise newException(JoltError, "Jolt tracked vehicle is no longer alive")
  let count = vehicle.wheelCount()
  if wheel < 0 or wheel >= count:
    raise newException(
      IndexDefect, "tracked vehicle wheel index is out of bounds")
  let index = uint32(wheel)
  result.position = fromRaw(vehicle.native.wheelPosition(index))
  result.rotation = fromRaw(vehicle.native.wheelRotation(index))
  result.hasContact = vehicle.native.wheelHasContact(index)
  result.suspensionLength = vehicle.native.wheelSuspensionLength(index)
  result.angularVelocity = vehicle.native.wheelAngularVelocity(index)
  result.steerAngle = vehicle.native.wheelSteerAngle(index)
  vehicle.native.trackedWheelDynamics(
    index,
    addr result.combinedLongitudinalFriction,
    addr result.combinedLateralFriction)
  vehicle.native.wheelConstraintState(
    index,
    addr result.hitHardPoint,
    addr result.suspensionImpulse,
    addr result.longitudinalImpulse,
    addr result.lateralImpulse)
  if result.hasContact:
    let contactId = vehicle.native.wheelContactBodyID(index)
    if not raw.bodyID(contactId).isInvalid:
      result.contactBodyId = some(BodyId(contactId))
    var subShapeId: uint32
    var pointVelocity, longitudinal, lateral: raw.Vec3
    vehicle.native.wheelContactDetails(
      index,
      addr subShapeId,
      addr pointVelocity,
      addr longitudinal,
      addr lateral)
    result.contactSubShapeId = some(subShapeId)
    result.contactPosition = fromRaw(
      vehicle.native.wheelContactPosition(index))
    result.contactNormal = fromRaw(vehicle.native.wheelContactNormal(index))
    result.contactPointVelocity = fromRaw(pointVelocity)
    result.contactLongitudinal = fromRaw(longitudinal)
    result.contactLateral = fromRaw(lateral)

proc constraintWorld(body1, body2: Body): World =
  if not body1.isAlive or not body2.isAlive:
    raise newException(JoltError, "constraint bodies must both be alive")
  if body1 == body2:
    raise newException(ValueError, "a constraint requires two different bodies")
  if body1.owner != body2.owner:
    raise newException(JoltError, "constraint bodies must belong to the same world")
  body1.owner

proc finishConstraint(world: World; body1, body2: Body;
                      kind: ConstraintKind;
                      native: ptr raw.Constraint): Constraint =
  if native.isNil:
    raise newException(JoltError, "Jolt could not create the constraint")
  new(result)
  result.owner = world
  result.native = native
  result.body1 = body1
  result.body2 = body2
  result.constraintKind = kind
  result.alive = true
  inc body1.constraintCount
  inc body2.constraintCount
  world.constraints.add(native)

proc addPointConstraint*(body1, body2: Body; point1, point2: Vec3): Constraint =
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Point,
    world.physics.createPointConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      point1.toRaw,
      point2.toRaw
    )
  )

proc addPointConstraint*(body1, body2: Body; worldPoint: Vec3): Constraint =
  addPointConstraint(body1, body2, worldPoint, worldPoint)

proc addDistanceConstraint*(body1, body2: Body; point1, point2: Vec3;
                            minDistance, maxDistance: float32): Constraint =
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  if not minDistance.isFinite or not maxDistance.isFinite or
      minDistance < 0 or minDistance > maxDistance:
    raise newException(
      ValueError,
      "constraint distances must be finite, non-negative, and ordered"
    )
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Distance,
    world.physics.createDistanceConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      point1.toRaw,
      point2.toRaw,
      minDistance,
      maxDistance
    )
  )

proc addFixedConstraint*(body1, body2: Body): Constraint =
  let world = constraintWorld(body1, body2)
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Fixed,
    world.physics.createFixedConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId)
    )
  )

proc normalizedDirection(value: Vec3; name: string): Vec3 =
  value.requireFinite(name)
  let lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, name & " must have non-zero length")
  let inverseLength = 1.0'f32 / sqrt(lengthSquared)
  Vec3(
    x: value.x * inverseLength,
    y: value.y * inverseLength,
    z: value.z * inverseLength
  )

proc validateHingeLimits(minAngle, maxAngle: float32) =
  if not minAngle.isFinite or not maxAngle.isFinite or
      minAngle < -PI.float32 or minAngle > 0 or
      maxAngle < 0 or maxAngle > PI.float32:
    raise newException(
      ValueError,
      "hinge limits must be finite with -PI <= minimum <= 0 <= maximum <= PI"
    )

proc validateSliderLimits(minPosition, maxPosition: float32) =
  if not minPosition.isFinite or not maxPosition.isFinite or
      minPosition > 0 or maxPosition < 0:
    raise newException(
      ValueError,
      "slider limits must be finite with minimum <= 0 <= maximum"
    )

proc validateHalfConeAngle(angle: float32; name: string) =
  if not angle.isFinite or angle < 0 or angle > PI.float32:
    raise newException(ValueError, name & " must be finite and in [0, PI]")

proc validateTwistLimits(minimum, maximum: float32) =
  if not minimum.isFinite or not maximum.isFinite or
      minimum < -PI.float32 or maximum > PI.float32 or minimum > maximum:
    raise newException(
      ValueError,
      "twist limits must be finite, ordered, and within [-PI, PI]"
    )

proc requirePerpendicular(axisX, axisY: Vec3; name: string) =
  let dot = axisX.x * axisY.x + axisX.y * axisY.y + axisX.z * axisY.z
  if abs(dot) > 1.0e-4'f32:
    raise newException(ValueError, name & " axes must be perpendicular")

proc addFixedConstraint*(body1, body2: Body; point1, point2,
                         axisX1, axisY1, axisX2, axisY2: Vec3): Constraint =
  ## Creates a fixed constraint from explicit world-space attachment frames.
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  let normalizedX1 = axisX1.normalizedDirection("axisX1")
  let normalizedY1 = axisY1.normalizedDirection("axisY1")
  let normalizedX2 = axisX2.normalizedDirection("axisX2")
  let normalizedY2 = axisY2.normalizedDirection("axisY2")
  requirePerpendicular(normalizedX1, normalizedY1, "body1 fixed frame")
  requirePerpendicular(normalizedX2, normalizedY2, "body2 fixed frame")
  finishConstraint(
    world, body1, body2, ConstraintKind.Fixed,
    world.physics.createFixedConstraint(
      raw.bodyID(body1.rawId), raw.bodyID(body2.rawId),
      point1.toRaw, point2.toRaw,
      normalizedX1.toRaw, normalizedY1.toRaw,
      normalizedX2.toRaw, normalizedY2.toRaw))

proc wakeConstraintBodies(constraint: Constraint) =
  if constraint.body1.motion != MotionType.Static:
    constraint.owner.physics.bodyInterface().activate(
      raw.bodyID(constraint.body1.rawId))
  if constraint.body2.motion != MotionType.Static:
    constraint.owner.physics.bodyInterface().activate(
      raw.bodyID(constraint.body2.rawId))

proc addHingeConstraint*(body1, body2: Body; point1, point2, axis: Vec3;
                         minAngle = -PI.float32;
                         maxAngle = PI.float32): Constraint =
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  let unitAxis = axis.normalizedDirection("hinge axis")
  validateHingeLimits(minAngle, maxAngle)
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Hinge,
    world.physics.createHingeConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      point1.toRaw,
      point2.toRaw,
      unitAxis.toRaw,
      minAngle,
      maxAngle
    )
  )

proc addHingeConstraint*(body1, body2: Body; worldPoint, axis: Vec3;
                         minAngle = -PI.float32;
                         maxAngle = PI.float32): Constraint =
  addHingeConstraint(
    body1, body2, worldPoint, worldPoint, axis, minAngle, maxAngle)

proc addSliderConstraint*(body1, body2: Body; point1, point2, axis: Vec3;
                          minPosition, maxPosition: float32): Constraint =
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  let unitAxis = axis.normalizedDirection("slider axis")
  validateSliderLimits(minPosition, maxPosition)
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Slider,
    world.physics.createSliderConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      point1.toRaw,
      point2.toRaw,
      unitAxis.toRaw,
      minPosition,
      maxPosition
    )
  )

proc addSliderConstraint*(body1, body2: Body; worldPoint, axis: Vec3;
                          minPosition, maxPosition: float32): Constraint =
  addSliderConstraint(
    body1, body2, worldPoint, worldPoint, axis, minPosition, maxPosition)

proc addConeConstraint*(body1, body2: Body; point1, point2,
                        twistAxis1, twistAxis2: Vec3;
                        halfConeAngle: float32): Constraint =
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  let axis1 = twistAxis1.normalizedDirection("cone twistAxis1")
  let axis2 = twistAxis2.normalizedDirection("cone twistAxis2")
  validateHalfConeAngle(halfConeAngle, "halfConeAngle")
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Cone,
    world.physics.createConeConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      point1.toRaw,
      point2.toRaw,
      axis1.toRaw,
      axis2.toRaw,
      halfConeAngle))

proc addConeConstraint*(body1, body2: Body; worldPoint, twistAxis: Vec3;
                        halfConeAngle: float32): Constraint =
  addConeConstraint(
    body1, body2, worldPoint, worldPoint,
    twistAxis, twistAxis, halfConeAngle)

proc addSwingTwistConstraint*(body1, body2: Body; point1, point2,
                              twistAxis, planeAxis: Vec3;
                              normalHalfConeAngle,
                              planeHalfConeAngle,
                              twistMinAngle,
                              twistMaxAngle: float32): Constraint =
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  let unitTwist = twistAxis.normalizedDirection("swing-twist twist axis")
  let unitPlane = planeAxis.normalizedDirection("swing-twist plane axis")
  requirePerpendicular(unitTwist, unitPlane, "swing-twist")
  validateHalfConeAngle(normalHalfConeAngle, "normalHalfConeAngle")
  validateHalfConeAngle(planeHalfConeAngle, "planeHalfConeAngle")
  validateTwistLimits(twistMinAngle, twistMaxAngle)
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.SwingTwist,
    world.physics.createSwingTwistConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      point1.toRaw,
      point2.toRaw,
      unitTwist.toRaw,
      unitPlane.toRaw,
      normalHalfConeAngle,
      planeHalfConeAngle,
      twistMinAngle,
      twistMaxAngle))

proc addSwingTwistConstraint*(body1, body2: Body; worldPoint,
                              twistAxis, planeAxis: Vec3;
                              normalHalfConeAngle,
                              planeHalfConeAngle,
                              twistMinAngle,
                              twistMaxAngle: float32): Constraint =
  addSwingTwistConstraint(
    body1, body2, worldPoint, worldPoint, twistAxis, planeAxis,
    normalHalfConeAngle, planeHalfConeAngle, twistMinAngle, twistMaxAngle)

proc addSixDOFConstraint*(body1, body2: Body; point1, point2: Vec3;
                          config = defaultSixDOFConfig()): Constraint =
  let world = constraintWorld(body1, body2)
  point1.requireFinite("point1")
  point2.requireFinite("point2")
  let axisX = config.axisX.normalizedDirection("SixDOF axisX")
  let axisY = config.axisY.normalizedDirection("SixDOF axisY")
  requirePerpendicular(axisX, axisY, "SixDOF")
  config.validateLimits()
  var limitMin, limitMax: array[6, cfloat]
  for axis in SixDOFAxis:
    let limit = config.limits[axis]
    limit.validate(axis)
    case limit.mode
    of SixDOFAxisMode.AxisFree:
      limitMin[ord(axis)] = -maxFiniteFloat32
      limitMax[ord(axis)] = maxFiniteFloat32
    of SixDOFAxisMode.AxisFixed:
      limitMin[ord(axis)] = maxFiniteFloat32
      limitMax[ord(axis)] = -maxFiniteFloat32
    of SixDOFAxisMode.AxisLimited:
      limitMin[ord(axis)] = limit.minimum
      limitMax[ord(axis)] = limit.maximum
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.SixDOF,
    world.physics.createSixDOFConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      point1.toRaw,
      point2.toRaw,
      axisX.toRaw,
      axisY.toRaw,
      uint8(ord(config.swingType)),
      addr limitMin[0],
      addr limitMax[0]))

proc addSixDOFConstraint*(body1, body2: Body; worldPoint: Vec3;
                          config = defaultSixDOFConfig()): Constraint =
  addSixDOFConstraint(body1, body2, worldPoint, worldPoint, config)

proc constraintDependency(value: Constraint; expected: ConstraintKind;
                          world: World; body: Body;
                          name: string): ptr raw.Constraint =
  if value.isNil:
    return nil
  if not value.isAlive:
    raise newException(JoltError, name & " must be alive")
  if value.owner != world:
    raise newException(JoltError, name & " must belong to the same world")
  if value.constraintKind != expected:
    raise newException(ValueError, name & " has the wrong constraint kind")
  if value.body1 != body and value.body2 != body:
    raise newException(ValueError, name & " must constrain its coupled body")
  value.native

proc addGearConstraint*(body1, body2: Body; axis1, axis2: Vec3;
                        ratio: float32; hinge1: Constraint = nil;
                        hinge2: Constraint = nil): Constraint =
  let world = constraintWorld(body1, body2)
  let unitAxis1 = axis1.normalizedDirection("gear axis1")
  let unitAxis2 = axis2.normalizedDirection("gear axis2")
  if not ratio.isFinite or ratio <= 0:
    raise newException(ValueError, "gear ratio must be finite and positive")
  let nativeHinge1 = constraintDependency(
    hinge1, ConstraintKind.Hinge, world, body1, "gear hinge1")
  let nativeHinge2 = constraintDependency(
    hinge2, ConstraintKind.Hinge, world, body2, "gear hinge2")
  result = finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Gear,
    world.physics.createGearConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      unitAxis1.toRaw,
      unitAxis2.toRaw,
      ratio,
      nativeHinge1,
      nativeHinge2))
  if not hinge1.isNil:
    result.dependencies.add hinge1
  if not hinge2.isNil:
    result.dependencies.add hinge2

proc addPulleyConstraint*(body1, body2: Body;
                          bodyPoint1, fixedPoint1,
                          bodyPoint2, fixedPoint2: Vec3;
                          ratio = 1.0'f32;
                          minLength = 0.0'f32;
                          maxLength = -1.0'f32): Constraint =
  let world = constraintWorld(body1, body2)
  bodyPoint1.requireFinite("pulley bodyPoint1")
  fixedPoint1.requireFinite("pulley fixedPoint1")
  bodyPoint2.requireFinite("pulley bodyPoint2")
  fixedPoint2.requireFinite("pulley fixedPoint2")
  if not ratio.isFinite or ratio <= 0:
    raise newException(ValueError, "pulley ratio must be finite and positive")
  if not minLength.isFinite or minLength < 0 or
      not maxLength.isFinite or
      (maxLength != -1.0'f32 and maxLength < minLength):
    raise newException(
      ValueError,
      "pulley lengths must use 0 <= minimum <= maximum, or maximum = -1")
  finishConstraint(
    world,
    body1,
    body2,
    ConstraintKind.Pulley,
    world.physics.createPulleyConstraint(
      raw.bodyID(body1.rawId),
      raw.bodyID(body2.rawId),
      bodyPoint1.toRaw,
      fixedPoint1.toRaw,
      bodyPoint2.toRaw,
      fixedPoint2.toRaw,
      ratio,
      minLength,
      maxLength))

proc addRackAndPinionConstraint*(pinionBody, rackBody: Body;
                                 hingeAxis, sliderAxis: Vec3;
                                 ratio: float32;
                                 hinge: Constraint = nil;
                                 slider: Constraint = nil): Constraint =
  let world = constraintWorld(pinionBody, rackBody)
  let unitHingeAxis = hingeAxis.normalizedDirection("pinion hinge axis")
  let unitSliderAxis = sliderAxis.normalizedDirection("rack slider axis")
  if not ratio.isFinite or ratio <= 0:
    raise newException(
      ValueError, "rack-and-pinion ratio must be finite and positive")
  let nativeHinge = constraintDependency(
    hinge, ConstraintKind.Hinge, world, pinionBody, "pinion hinge")
  let nativeSlider = constraintDependency(
    slider, ConstraintKind.Slider, world, rackBody, "rack slider")
  result = finishConstraint(
    world,
    pinionBody,
    rackBody,
    ConstraintKind.RackAndPinion,
    world.physics.createRackAndPinionConstraint(
      raw.bodyID(pinionBody.rawId),
      raw.bodyID(rackBody.rawId),
      unitHingeAxis.toRaw,
      unitSliderAxis.toRaw,
      ratio,
      nativeHinge,
      nativeSlider))
  if not hinge.isNil:
    result.dependencies.add hinge
  if not slider.isNil:
    result.dependencies.add slider

proc addPathConstraint*(pathBody, movingBody: Body;
                        points: openArray[PathPoint];
                        pathPosition = vec3(0, 0, 0);
                        pathRotation = quatIdentity();
                        pathFraction = 0.0'f32;
                        looping = false;
                        rotationConstraint =
                          PathRotationConstraintType.PathRotationFree;
                        maxFrictionForce = 0.0'f32): Constraint =
  let world = constraintWorld(pathBody, movingBody)
  if points.len < (if looping: 3 else: 2):
    raise newException(
      ValueError,
      if looping:
        "a looping Hermite path requires at least three points"
      else:
        "a Hermite path requires at least two points")
  if uint64(points.len) > uint64(high(uint32)):
    raise newException(ValueError, "path point count must fit in uint32")
  pathPosition.requireFinite("path position")
  let unitPathRotation = pathRotation.normalized
  let maximumFraction = if looping:
      float32(points.len)
    else:
      float32(points.len - 1)
  if not pathFraction.isFinite or pathFraction < 0 or
      pathFraction > maximumFraction:
    raise newException(ValueError, "path fraction is outside the path")
  if not maxFrictionForce.isFinite or maxFrictionForce < 0:
    raise newException(
      ValueError, "path friction force must be finite and non-negative")

  var positions = newSeq[raw.Vec3](points.len)
  var tangents = newSeq[raw.Vec3](points.len)
  var normals = newSeq[raw.Vec3](points.len)
  for index, point in points:
    point.position.requireFinite("path point position")
    let unitTangent = point.tangent.normalizedDirection("path point tangent")
    let unitNormal = point.normal.normalizedDirection("path point normal")
    requirePerpendicular(unitTangent, unitNormal, "path point")
    positions[index] = point.position.toRaw
    tangents[index] = point.tangent.toRaw
    normals[index] = unitNormal.toRaw
  if looping:
    let first = points[0].position
    let last = points[^1].position
    let delta = vec3(last.x - first.x, last.y - first.y, last.z - first.z)
    if delta.x * delta.x + delta.y * delta.y + delta.z * delta.z <= 1.0e-12'f32:
      raise newException(
        ValueError, "a looping path must not repeat its first point at the end")

  result = finishConstraint(
    world,
    pathBody,
    movingBody,
    ConstraintKind.Path,
    world.physics.createPathConstraint(
      raw.bodyID(pathBody.rawId),
      raw.bodyID(movingBody.rawId),
      addr positions[0],
      addr tangents[0],
      addr normals[0],
      uint32(points.len),
      looping,
      pathPosition.toRaw,
      unitPathRotation.toRaw,
      pathFraction,
      maxFrictionForce,
      uint8(ord(rotationConstraint))))
  result.pathMaxFraction = maximumFraction
  result.pathLooping = looping

proc kind*(constraint: Constraint): ConstraintKind =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.constraintKind

proc isEnabled*(constraint: Constraint): bool =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.native.enabled

proc setEnabled*(constraint: Constraint; enabled: bool) =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.native.setEnabled(enabled)
  constraint.wakeConstraintBodies()

proc priority*(constraint: Constraint): uint32 =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.native.priority

proc setPriority*(constraint: Constraint; priority: uint32) =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.native.setPriority(priority)

proc solverStepOverrides*(constraint: Constraint):
    tuple[velocity, position: uint32] =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  result.velocity = constraint.native.velocityStepsOverride
  result.position = constraint.native.positionStepsOverride

proc setSolverStepOverrides*(constraint: Constraint; velocity,
                             position: uint32) =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  if velocity >= 256 or position >= 256:
    raise newException(
      ValueError, "constraint solver step overrides must be below 256")
  constraint.native.setVelocityStepsOverride(velocity)
  constraint.native.setPositionStepsOverride(position)
  constraint.wakeConstraintBodies()

proc userData*(constraint: Constraint): uint64 =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.native.userData

proc setUserData*(constraint: Constraint; value: uint64) =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.native.setUserData(value)

proc resetWarmStart*(constraint: Constraint) =
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  constraint.native.resetWarmStart()

proc solverImpulse*(constraint: Constraint): ConstraintSolverImpulse =
  ## Returns native accumulated solver impulses for break-force diagnostics.
  ## Divide an impulse magnitude by the step duration to approximate force or
  ## torque. Components depend on the constraint kind and its constrained axes.
  if not constraint.isAlive:
    raise newException(JoltError, "Jolt constraint is no longer alive")
  var position, rotation, motorTranslation, motorRotation: raw.Vec3
  var limit: cfloat
  if not constraint.native.solverImpulse(
      addr position, addr rotation, addr limit,
      addr motorTranslation, addr motorRotation):
    raise newException(JoltError, "Jolt could not inspect constraint impulses")
  ConstraintSolverImpulse(
    position: position.fromRaw,
    rotation: rotation.fromRaw,
    limit: limit,
    motorTranslation: motorTranslation.fromRaw,
    motorRotation: motorRotation.fromRaw)

proc currentAngle*(constraint: Constraint): float32 =
  if constraint.kind != ConstraintKind.Hinge:
    raise newException(ValueError, "currentAngle requires a hinge constraint")
  constraint.native.hingeAngle

proc currentPosition*(constraint: Constraint): float32 =
  if constraint.kind != ConstraintKind.Slider:
    raise newException(ValueError, "currentPosition requires a slider constraint")
  constraint.native.sliderPosition

proc totalLambda*(constraint: Constraint): float32 =
  case constraint.kind
  of ConstraintKind.Gear:
    constraint.native.gearTotalLambda
  of ConstraintKind.RackAndPinion:
    constraint.native.rackAndPinionTotalLambda
  else:
    raise newException(
      ValueError, "totalLambda requires a gear or rack-and-pinion constraint")

proc currentLength*(constraint: Constraint): float32 =
  if constraint.kind != ConstraintKind.Pulley:
    raise newException(ValueError, "currentLength requires a pulley constraint")
  constraint.native.pulleyCurrentLength

proc lengthLimits*(constraint: Constraint): tuple[minimum, maximum: float32] =
  if constraint.kind != ConstraintKind.Pulley:
    raise newException(ValueError, "lengthLimits requires a pulley constraint")
  constraint.native.pulleyLengths(addr result.minimum, addr result.maximum)

proc setLengthLimits*(constraint: Constraint; minimum, maximum: float32) =
  if constraint.kind != ConstraintKind.Pulley:
    raise newException(ValueError, "setLengthLimits requires a pulley constraint")
  if not minimum.isFinite or not maximum.isFinite or
      minimum < 0 or minimum > maximum:
    raise newException(
      ValueError, "pulley lengths must be finite, non-negative, and ordered")
  constraint.native.setPulleyLengths(minimum, maximum)
  constraint.wakeConstraintBodies()

proc pathFraction*(constraint: Constraint): float32 =
  if constraint.kind != ConstraintKind.Path:
    raise newException(ValueError, "pathFraction requires a path constraint")
  constraint.native.pathFraction

proc pathMaxFraction*(constraint: Constraint): float32 =
  if constraint.kind != ConstraintKind.Path:
    raise newException(ValueError, "pathMaxFraction requires a path constraint")
  constraint.native.pathMaxFraction

proc setPathFriction*(constraint: Constraint; maximumForce: float32) =
  if constraint.kind != ConstraintKind.Path:
    raise newException(ValueError, "setPathFriction requires a path constraint")
  if not maximumForce.isFinite or maximumForce < 0:
    raise newException(
      ValueError, "path friction force must be finite and non-negative")
  constraint.native.setPathFriction(maximumForce)
  constraint.wakeConstraintBodies()

proc configurePathMotor*(constraint: Constraint; settings: MotorSettings) =
  if constraint.kind != ConstraintKind.Path:
    raise newException(ValueError, "configurePathMotor requires a path constraint")
  settings.validate()
  constraint.native.configurePathMotor(
    uint8(ord(settings.spring.mode)),
    settings.spring.value,
    settings.spring.damping,
    settings.minForce,
    settings.maxForce)

proc setPathMotorTargets*(constraint: Constraint; velocity,
                          fraction: float32) =
  if constraint.kind != ConstraintKind.Path:
    raise newException(ValueError, "setPathMotorTargets requires a path constraint")
  if not velocity.isFinite or not fraction.isFinite:
    raise newException(ValueError, "path motor targets must be finite")
  if not constraint.pathLooping and
      (fraction < 0 or fraction > constraint.pathMaxFraction):
    raise newException(ValueError, "path motor fraction is outside the path")
  constraint.native.setPathMotorTargets(velocity, fraction)
  constraint.wakeConstraintBodies()

proc setPathMotorState*(constraint: Constraint; state: MotorState) =
  if constraint.kind != ConstraintKind.Path:
    raise newException(ValueError, "setPathMotorState requires a path constraint")
  constraint.native.setPathMotorState(uint8(ord(state)))
  constraint.wakeConstraintBodies()

proc pathMotor*(constraint: Constraint): tuple[
    state: MotorState, targetVelocity, targetFraction: float32] =
  if constraint.kind != ConstraintKind.Path:
    raise newException(ValueError, "pathMotor requires a path constraint")
  var state: uint8
  var velocity, fraction: cfloat
  constraint.native.pathMotor(addr state, addr velocity, addr fraction)
  if state > uint8(ord(high(MotorState))):
    raise newException(JoltError, "Jolt returned an unknown path motor state")
  (MotorState(state), float32(velocity), float32(fraction))

proc setPathMotor*(constraint: Constraint; state: MotorState;
                   targetVelocity, targetFraction: float32;
                   settings = defaultMotorSettings()) =
  constraint.configurePathMotor(settings)
  constraint.setPathMotorTargets(targetVelocity, targetFraction)
  constraint.setPathMotorState(state)

proc distanceLimits*(constraint: Constraint): tuple[
    minimum, maximum: float32] =
  if constraint.kind != ConstraintKind.Distance:
    raise newException(ValueError, "distanceLimits requires a distance constraint")
  constraint.native.distanceLimits(addr result.minimum, addr result.maximum)

proc setDistanceLimits*(constraint: Constraint; minimum, maximum: float32) =
  if constraint.kind != ConstraintKind.Distance:
    raise newException(ValueError, "setDistanceLimits requires a distance constraint")
  if not minimum.isFinite or not maximum.isFinite or
      minimum < 0 or minimum > maximum:
    raise newException(
      ValueError, "distance limits must be finite, non-negative, and ordered")
  constraint.native.setDistanceLimits(minimum, maximum)
  constraint.wakeConstraintBodies()

proc setLimits*(constraint: Constraint; minimum, maximum: float32) =
  case constraint.kind
  of ConstraintKind.Hinge:
    validateHingeLimits(minimum, maximum)
    constraint.native.setHingeLimits(minimum, maximum)
  of ConstraintKind.Slider:
    validateSliderLimits(minimum, maximum)
    constraint.native.setSliderLimits(minimum, maximum)
  else:
    raise newException(ValueError, "setLimits requires a hinge or slider constraint")
  constraint.wakeConstraintBodies()

proc setFriction*(constraint: Constraint; maximum: float32) =
  if not maximum.isFinite or maximum < 0:
    raise newException(ValueError, "constraint friction must be finite and non-negative")
  case constraint.kind
  of ConstraintKind.Hinge:
    constraint.native.setHingeFriction(maximum)
  of ConstraintKind.Slider:
    constraint.native.setSliderFriction(maximum)
  of ConstraintKind.SwingTwist:
    constraint.native.setSwingTwistFriction(maximum)
  else:
    raise newException(
      ValueError,
      "setFriction requires a hinge, slider or swing-twist constraint")
  constraint.wakeConstraintBodies()

proc limitSpring*(constraint: Constraint): SpringSettings =
  if constraint.kind notin
      {ConstraintKind.Distance, ConstraintKind.Hinge, ConstraintKind.Slider}:
    raise newException(
      ValueError, "limitSpring requires a distance, hinge or slider")
  var mode: uint8
  var value, damping: cfloat
  if not constraint.native.limitSpring(
      0, addr mode, addr value, addr damping):
    raise newException(JoltError, "Jolt could not inspect the limit spring")
  if mode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid spring mode")
  SpringSettings(
    mode: SpringMode(mode), value: float32(value), damping: float32(damping))

proc setLimitSpring*(constraint: Constraint; settings: SpringSettings) =
  settings.validate()
  case constraint.kind
  of ConstraintKind.Distance:
    constraint.native.setDistanceLimitSpring(
      uint8(ord(settings.mode)), settings.value, settings.damping)
  of ConstraintKind.Hinge:
    constraint.native.setHingeLimitSpring(
      uint8(ord(settings.mode)), settings.value, settings.damping)
  of ConstraintKind.Slider:
    constraint.native.setSliderLimitSpring(
      uint8(ord(settings.mode)), settings.value, settings.damping)
  else:
    raise newException(
      ValueError, "setLimitSpring requires a distance, hinge or slider")
  constraint.wakeConstraintBodies()

proc configureMotor*(constraint: Constraint; settings: MotorSettings) =
  settings.validate()
  case constraint.kind
  of ConstraintKind.Hinge:
    constraint.native.configureHingeMotor(
      uint8(ord(settings.spring.mode)),
      settings.spring.value,
      settings.spring.damping,
      settings.minTorque,
      settings.maxTorque)
  of ConstraintKind.Slider:
    constraint.native.configureSliderMotor(
      uint8(ord(settings.spring.mode)),
      settings.spring.value,
      settings.spring.damping,
      settings.minForce,
      settings.maxForce)
  else:
    raise newException(ValueError, "configureMotor requires a hinge or slider")

proc setMotorTarget*(constraint: Constraint; velocity, position: float32) =
  if not velocity.isFinite or not position.isFinite:
    raise newException(ValueError, "motor targets must be finite")
  case constraint.kind
  of ConstraintKind.Hinge:
    constraint.native.setHingeMotorTarget(velocity, position)
  of ConstraintKind.Slider:
    constraint.native.setSliderMotorTarget(velocity, position)
  else:
    raise newException(ValueError, "setMotorTarget requires a hinge or slider")
  constraint.wakeConstraintBodies()

proc setMotorState*(constraint: Constraint; state: MotorState) =
  case constraint.kind
  of ConstraintKind.Hinge:
    constraint.native.setHingeMotorState(uint8(ord(state)))
  of ConstraintKind.Slider:
    constraint.native.setSliderMotorState(uint8(ord(state)))
  else:
    raise newException(ValueError, "setMotorState requires a hinge or slider")
  constraint.wakeConstraintBodies()

proc motor*(constraint: Constraint): tuple[
    state: MotorState, targetVelocity, targetPosition: float32] =
  var state: uint8
  var velocity, position: cfloat
  case constraint.kind
  of ConstraintKind.Hinge:
    constraint.native.hingeMotor(addr state, addr velocity, addr position)
  of ConstraintKind.Slider:
    constraint.native.sliderMotor(addr state, addr velocity, addr position)
  else:
    raise newException(ValueError, "motor requires a hinge or slider")
  if state > uint8(ord(high(MotorState))):
    raise newException(JoltError, "Jolt returned an unknown motor state")
  (MotorState(state), float32(velocity), float32(position))

proc setMotor*(constraint: Constraint; state: MotorState;
               targetVelocity, targetPosition: float32;
               settings = defaultMotorSettings()) =
  constraint.configureMotor(settings)
  constraint.setMotorTarget(targetVelocity, targetPosition)
  constraint.setMotorState(state)

proc halfConeAngle*(constraint: Constraint): float32 =
  if constraint.kind != ConstraintKind.Cone:
    raise newException(ValueError, "halfConeAngle requires a cone constraint")
  constraint.native.coneHalfAngle

proc setHalfConeAngle*(constraint: Constraint; angle: float32) =
  if constraint.kind != ConstraintKind.Cone:
    raise newException(ValueError, "setHalfConeAngle requires a cone constraint")
  validateHalfConeAngle(angle, "halfConeAngle")
  constraint.native.setConeHalfAngle(angle)
  constraint.wakeConstraintBodies()

proc setSwingTwistLimits*(constraint: Constraint; normalHalfConeAngle,
                          planeHalfConeAngle, twistMinAngle,
                          twistMaxAngle: float32) =
  if constraint.kind != ConstraintKind.SwingTwist:
    raise newException(
      ValueError, "setSwingTwistLimits requires a swing-twist constraint")
  validateHalfConeAngle(normalHalfConeAngle, "normalHalfConeAngle")
  validateHalfConeAngle(planeHalfConeAngle, "planeHalfConeAngle")
  validateTwistLimits(twistMinAngle, twistMaxAngle)
  constraint.native.setSwingTwistLimits(
    normalHalfConeAngle,
    planeHalfConeAngle,
    twistMinAngle,
    twistMaxAngle)
  constraint.wakeConstraintBodies()

proc rotationInConstraintSpace*(constraint: Constraint): Quat =
  if constraint.kind != ConstraintKind.SwingTwist:
    raise newException(
      ValueError, "rotationInConstraintSpace requires a swing-twist constraint")
  fromRaw(constraint.native.swingTwistRotation)

proc configureSwingMotor*(constraint: Constraint; settings: MotorSettings) =
  if constraint.kind != ConstraintKind.SwingTwist:
    raise newException(ValueError, "configureSwingMotor requires swing-twist")
  settings.validate()
  constraint.native.configureSwingTwistMotor(
    true,
    uint8(ord(settings.spring.mode)),
    settings.spring.value,
    settings.spring.damping,
    settings.minTorque,
    settings.maxTorque)

proc configureTwistMotor*(constraint: Constraint; settings: MotorSettings) =
  if constraint.kind != ConstraintKind.SwingTwist:
    raise newException(ValueError, "configureTwistMotor requires swing-twist")
  settings.validate()
  constraint.native.configureSwingTwistMotor(
    false,
    uint8(ord(settings.spring.mode)),
    settings.spring.value,
    settings.spring.damping,
    settings.minTorque,
    settings.maxTorque)

proc setSwingMotorState*(constraint: Constraint; state: MotorState) =
  if constraint.kind != ConstraintKind.SwingTwist:
    raise newException(ValueError, "setSwingMotorState requires swing-twist")
  constraint.native.setSwingTwistMotorState(true, uint8(ord(state)))
  constraint.wakeConstraintBodies()

proc setTwistMotorState*(constraint: Constraint; state: MotorState) =
  if constraint.kind != ConstraintKind.SwingTwist:
    raise newException(ValueError, "setTwistMotorState requires swing-twist")
  constraint.native.setSwingTwistMotorState(false, uint8(ord(state)))
  constraint.wakeConstraintBodies()

proc setSwingTwistMotorTargets*(constraint: Constraint;
                                angularVelocity: Vec3;
                                orientation: Quat) =
  if constraint.kind != ConstraintKind.SwingTwist:
    raise newException(ValueError, "motor targets require swing-twist")
  angularVelocity.requireFinite("swing-twist target angular velocity")
  constraint.native.setSwingTwistMotorTargets(
    angularVelocity.toRaw, orientation.normalized.toRaw)
  constraint.wakeConstraintBodies()

proc axisLimit*(constraint: Constraint;
                axis: SixDOFAxis): SixDOFAxisLimit =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "axisLimit requires a SixDOF constraint")
  readSixDOFAxisLimit(constraint.native, axis)

proc swingType*(constraint: Constraint): SixDOFSwingType =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "swingType requires a SixDOF constraint")
  let value = constraint.native.sixDOFSwingType
  if value > uint8(ord(high(SixDOFSwingType))):
    raise newException(JoltError, "Jolt returned an invalid SixDOF swing type")
  SixDOFSwingType(value)

proc setAxisLimit*(constraint: Constraint; axis: SixDOFAxis;
                   limit: SixDOFAxisLimit) =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "setAxisLimit requires a SixDOF constraint")
  limit.validate(axis, constraint.swingType)
  constraint.native.setSixDOFAxisLimit(
    uint8(ord(axis)),
    uint8(ord(limit.mode)),
    limit.minimum,
    limit.maximum)
  constraint.wakeConstraintBodies()

proc setAxisFriction*(constraint: Constraint; axis: SixDOFAxis;
                      maximum: float32) =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "setAxisFriction requires a SixDOF constraint")
  if not maximum.isFinite or maximum < 0:
    raise newException(ValueError, "axis friction must be finite and non-negative")
  constraint.native.setSixDOFFriction(uint8(ord(axis)), maximum)
  constraint.wakeConstraintBodies()

proc axisLimitSpring*(constraint: Constraint;
                      axis: SixDOFAxis): SpringSettings =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "axisLimitSpring requires a SixDOF constraint")
  if axis > SixDOFAxis.TranslationZ:
    raise newException(
      ValueError, "SixDOF limit springs only support translation axes")
  var mode: uint8
  var value, damping: cfloat
  if not constraint.native.limitSpring(
      uint8(ord(axis)), addr mode, addr value, addr damping):
    raise newException(JoltError, "Jolt could not inspect SixDOF limit spring")
  if mode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid spring mode")
  SpringSettings(
    mode: SpringMode(mode), value: float32(value), damping: float32(damping))

proc setAxisLimitSpring*(constraint: Constraint; axis: SixDOFAxis;
                         settings: SpringSettings) =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(
      ValueError, "setAxisLimitSpring requires a SixDOF constraint")
  if axis > SixDOFAxis.TranslationZ:
    raise newException(
      ValueError, "SixDOF limit springs only support translation axes")
  settings.validate()
  constraint.native.setSixDOFLimitSpring(
    uint8(ord(axis)), uint8(ord(settings.mode)),
    settings.value, settings.damping)
  constraint.wakeConstraintBodies()

proc configureAxisMotor*(constraint: Constraint; axis: SixDOFAxis;
                         settings: MotorSettings) =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "configureAxisMotor requires a SixDOF constraint")
  settings.validate()
  let minimum = if axis <= SixDOFAxis.TranslationZ:
      settings.minForce
    else:
      settings.minTorque
  let maximum = if axis <= SixDOFAxis.TranslationZ:
      settings.maxForce
    else:
      settings.maxTorque
  constraint.native.configureSixDOFMotor(
    uint8(ord(axis)),
    uint8(ord(settings.spring.mode)),
    settings.spring.value,
    settings.spring.damping,
    minimum,
    maximum)

proc sixDOFAxisMotorSettings*(constraint: Constraint;
                              axis: SixDOFAxis): MotorSettings =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(
      ValueError, "sixDOFAxisMotorSettings requires a SixDOF constraint")
  var springMode: uint8
  var springValue, springDamping, minForce, maxForce,
    minTorque, maxTorque: cfloat
  if not constraint.native.motorSettings(
      uint8(ord(axis)), addr springMode, addr springValue,
      addr springDamping, addr minForce, addr maxForce,
      addr minTorque, addr maxTorque):
    raise newException(JoltError, "Jolt could not inspect SixDOF motor settings")
  if springMode > uint8(ord(high(SpringMode))):
    raise newException(JoltError, "Jolt returned an invalid motor spring mode")
  MotorSettings(
    spring: SpringSettings(
      mode: SpringMode(springMode), value: float32(springValue),
      damping: float32(springDamping)),
    minForce: float32(minForce), maxForce: float32(maxForce),
    minTorque: float32(minTorque), maxTorque: float32(maxTorque))

proc setAxisMotorState*(constraint: Constraint; axis: SixDOFAxis;
                        state: MotorState) =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "setAxisMotorState requires a SixDOF constraint")
  constraint.native.setSixDOFMotorState(uint8(ord(axis)), uint8(ord(state)))
  constraint.wakeConstraintBodies()

proc setSixDOFMotorTargets*(constraint: Constraint; velocity,
                            angularVelocity, position: Vec3;
                            orientation: Quat) =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "motor targets require a SixDOF constraint")
  velocity.requireFinite("SixDOF target velocity")
  angularVelocity.requireFinite("SixDOF target angular velocity")
  position.requireFinite("SixDOF target position")
  constraint.native.setSixDOFMotorTargets(
    velocity.toRaw,
    angularVelocity.toRaw,
    position.toRaw,
    orientation.normalized.toRaw)
  constraint.wakeConstraintBodies()

proc sixDOFMotor*(constraint: Constraint; axis: SixDOFAxis): tuple[
    state: MotorState; targetVelocity, targetAngularVelocity,
    targetPosition: Vec3; targetOrientation: Quat] =
  if constraint.kind != ConstraintKind.SixDOF:
    raise newException(ValueError, "sixDOFMotor requires a SixDOF constraint")
  var state: uint8
  var velocity, angularVelocity, position: raw.Vec3
  var orientation: raw.Quat
  if not constraint.native.sixDOFMotorState(
      uint8(ord(axis)), addr state, addr velocity, addr angularVelocity,
      addr position, addr orientation):
    raise newException(JoltError, "Jolt could not inspect SixDOF motor state")
  (decodeMotorState(state, "SixDOF motor"), velocity.fromRaw,
   angularVelocity.fromRaw, position.fromRaw, orientation.fromRaw)

proc id*(body: Body): BodyId =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  BodyId(body.rawId)

proc bodySnapshots*(world: World; bodies: openArray[Body]): seq[BodySnapshot] =
  ## Captures every requested rigid body under one native multi-body read lock.
  ## The returned values are detached and preserve the input order, including
  ## duplicate handles.
  world.requireOpen()
  if uint64(bodies.len) > uint64(high(uint32)):
    raise newException(ValueError, "too many bodies to snapshot")
  if bodies.len == 0:
    return @[]

  var ids = newSeq[uint32](bodies.len)
  for index, body in bodies:
    if body.isNil or not body.isAlive:
      raise newException(JoltError, "cannot snapshot a closed Jolt body")
    if body.owner != world:
      raise newException(ValueError, "all snapshot bodies must belong to the world")
    ids[index] = body.rawId

  var native = newSeq[raw.BodySnapshotData](bodies.len)
  world.physics.readBodySnapshots(
    addr ids[0], uint32(ids.len), addr native[0])
  result = newSeq[BodySnapshot](bodies.len)
  for index, state in native:
    if not state.mSucceeded:
      raise newException(JoltError, "Jolt could not lock a body for snapshot")
    if state.mMotionType > uint8(ord(high(MotionType))):
      raise newException(JoltError, "Jolt returned an unknown body motion type")
    if uint32(state.mObjectLayer) >= world.layerCount:
      raise newException(JoltError, "Jolt returned an unknown body collision layer")

    var motion = none(BodyMotionSnapshot)
    if state.mHasMotionProperties:
      if state.mMotionQuality > uint8(ord(high(MotionQuality))):
        raise newException(JoltError, "Jolt returned an unknown motion quality")
      if (state.mAllowedDOFs and 0xc0'u8) != 0:
        raise newException(JoltError, "Jolt returned unknown allowed body DOFs")
      var mass = none(float32)
      if state.mHasMass:
        mass = some(float32(state.mMass))
      var properties = none(BodyMassProperties)
      if state.mHasMassProperties:
        properties = some(BodyMassProperties(
          mass: float32(state.mMass),
          inertiaDiagonal: state.mInertiaDiagonal.fromRaw,
          inertiaRotation: state.mInertiaRotation.fromRaw))
      motion = some(BodyMotionSnapshot(
        motionQuality: MotionQuality(state.mMotionQuality),
        allowedDOFs: allowedDOFsFromMask(state.mAllowedDOFs),
        linearDamping: float32(state.mLinearDamping),
        angularDamping: float32(state.mAngularDamping),
        maxLinearVelocity: float32(state.mMaxLinearVelocity),
        maxAngularVelocity: float32(state.mMaxAngularVelocity),
        gravityFactor: float32(state.mGravityFactor),
        allowSleeping: state.mAllowSleeping,
        collideKinematicVsNonDynamic: state.mCollideKinematicVsNonDynamic,
        applyGyroscopicForce: state.mApplyGyroscopicForce,
        enhancedInternalEdgeRemoval: state.mEnhancedInternalEdgeRemoval,
        numVelocityStepsOverride: state.mNumVelocityStepsOverride,
        numPositionStepsOverride: state.mNumPositionStepsOverride,
        mass: mass,
        massProperties: properties))

    result[index] = BodySnapshot(
      bodyId: BodyId(ids[index]),
      motionType: MotionType(state.mMotionType),
      collisionLayer: CollisionLayer(state.mObjectLayer),
      position: state.mPosition.fromRaw,
      centerOfMassPosition: state.mCenterOfMassPosition.fromRaw,
      rotation: state.mRotation.fromRaw,
      linearVelocity: state.mLinearVelocity.fromRaw,
      angularVelocity: state.mAngularVelocity.fromRaw,
      active: state.mActive,
      sensor: state.mSensor,
      inBroadPhase: state.mInBroadPhase,
      collisionCacheInvalid: state.mCollisionCacheInvalid,
      useManifoldReduction: state.mUseManifoldReduction,
      friction: float32(state.mFriction),
      restitution: float32(state.mRestitution),
      userData: state.mUserData,
      motion: motion)

proc snapshot*(body: Body): BodySnapshot =
  ## Captures a detached state value under the body's native read lock.
  if body.isNil or not body.isAlive:
    raise newException(JoltError, "cannot snapshot a closed Jolt body")
  body.owner.bodySnapshots([body])[0]

proc hits*(hit: RayHit; body: Body): bool =
  body.isAlive and body.owner.isOpen and hit.bodyId == BodyId(body.rawId)

proc hits*(hit: OverlapHit; body: Body): bool =
  body.isAlive and body.owner.isOpen and hit.bodyId == BodyId(body.rawId)

proc hits*(hit: ShapeCastHit; body: Body): bool =
  body.isAlive and body.owner.isOpen and hit.bodyId == BodyId(body.rawId)

proc hits*(hit: RayHit; body: SoftBody): bool =
  body.isAlive and body.owner.isOpen and hit.bodyId == BodyId(body.rawId)

proc hits*(hit: OverlapHit; body: SoftBody): bool =
  body.isAlive and body.owner.isOpen and hit.bodyId == BodyId(body.rawId)

proc hits*(hit: ShapeCastHit; body: SoftBody): bool =
  body.isAlive and body.owner.isOpen and hit.bodyId == BodyId(body.rawId)

proc materialAt*(body: Body; subShapeId: uint32): Option[PhysicsMaterial] =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  var name: cstring
  var red, green, blue, alpha: uint8
  if not body.owner.physics.bodyMaterial(
      raw.bodyID(body.rawId), subShapeId, addr name,
      addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

proc materialAt*(body: SoftBody;
                 subShapeId: uint32): Option[PhysicsMaterial] =
  if not body.isAlive:
    raise newException(JoltError, "Jolt soft body is no longer alive")
  var name: cstring
  var red, green, blue, alpha: uint8
  if not body.owner.physics.bodyMaterial(
      raw.bodyID(body.rawId), subShapeId, addr name,
      addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

proc material*(hit: RayHit; world: World): Option[PhysicsMaterial] =
  world.requireOpen()
  var name: cstring
  var red, green, blue, alpha: uint8
  if not world.physics.bodyMaterial(
      raw.bodyID(uint32(hit.bodyId)), hit.subShapeId, addr name,
      addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

proc material*(hit: ShapeCastHit; world: World): Option[PhysicsMaterial] =
  world.requireOpen()
  var name: cstring
  var red, green, blue, alpha: uint8
  if not world.physics.bodyMaterial(
      raw.bodyID(uint32(hit.bodyId)), hit.subShapeId, addr name,
      addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

proc material*(hit: OverlapHit; world: World): Option[PhysicsMaterial] =
  world.requireOpen()
  var name: cstring
  var red, green, blue, alpha: uint8
  if not world.physics.bodyMaterial(
      raw.bodyID(uint32(hit.bodyId)), hit.subShapeId, addr name,
      addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

func isConvexQueryShape(shape: Shape): bool =
  case shape.kind
  of ShapeKind.Box, ShapeKind.Sphere, ShapeKind.Capsule,
      ShapeKind.Cylinder, ShapeKind.TaperedCapsule,
      ShapeKind.TaperedCylinder, ShapeKind.ConvexHull:
    true
  of ShapeKind.Scaled, ShapeKind.RotatedTranslated,
      ShapeKind.OffsetCenterOfMass:
    shape.innerShapes.len == 1 and shape.innerShapes[0].isConvexQueryShape
  else:
    false

proc cookConvexQueryShape(shape: Shape): CookedShape =
  if not shape.isConvexQueryShape:
    raise newException(
      ValueError,
      "shape queries require a convex shape or convex decorated shape"
    )
  result = cookShape(shape, MotionType.Dynamic)
  if not result.ownsNativeReference:
    result.native.addRef()
    result.ownsNativeReference = true

proc castShape*(world: World; shape: Shape; origin, direction: Vec3;
                maxDistance: float32; rotation = quatIdentity();
                layer = none(CollisionLayer);
                bodyFilter = QueryBodyFilter();
                subShapeFilter = QuerySubShapeFilter()): Option[ShapeCastHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("shape cast origin")
  direction.requireFinite("shape cast direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y + direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "shape cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let castDelta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  let cookedShape = cookConvexQueryShape(shape)
  defer: cookedShape.release()
  var bodyId: uint32
  var fraction: cfloat
  var subShapeId: uint32
  var contactPoint, normal: raw.Vec3
  if not world.physics.castConvex(
      cookedShape.native,
      origin.toRaw,
      rotation.normalized.toRaw,
      castDelta.toRaw,
      addr bodyId,
      addr fraction,
      addr contactPoint,
      addr normal,
      addr subShapeId,
      world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
      uint32(bodyFilter.len), bodyFilter.includesOnly,
      subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
      uint32(subShapeFilter.len), subShapeFilter.includesOnly):
    return none(ShapeCastHit)
  let nativeFraction = float32(fraction)
  some(ShapeCastHit(
    bodyId: BodyId(bodyId),
    subShapeId: subShapeId,
    fraction: nativeFraction,
    distance: nativeFraction * maxDistance,
    position: Vec3(
      x: origin.x + castDelta.x * nativeFraction,
      y: origin.y + castDelta.y * nativeFraction,
      z: origin.z + castDelta.z * nativeFraction),
    contactPoint: fromRaw(contactPoint),
    normal: fromRaw(normal)))

proc castShapeAll*(world: World; shape: Shape; origin, direction: Vec3;
                   maxDistance: float32; rotation = quatIdentity();
                   maxHits = 256;
                   layer = none(CollisionLayer);
                   bodyFilter = QueryBodyFilter();
                   subShapeFilter = QuerySubShapeFilter()): seq[ShapeCastHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("shape cast origin")
  direction.requireFinite("shape cast direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  if maxHits <= 0 or uint64(maxHits) > uint64(high(uint32)):
    raise newException(ValueError, "maxHits must fit in a positive uint32")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y + direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "shape cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let castDelta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  let cookedShape = cookConvexQueryShape(shape)
  defer: cookedShape.release()
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  var contactPoints = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.castConvexAll(
    cookedShape.native,
    origin.toRaw,
    rotation.normalized.toRaw,
    castDelta.toRaw,
    addr bodyIds[0],
    addr fractions[0],
    addr contactPoints[0],
    addr normals[0],
    addr subShapeIds[0],
    uint32(maxHits),
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly,
    subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
    uint32(subShapeFilter.len), subShapeFilter.includesOnly)
  result = newSeqOfCap[ShapeCastHit](int(count))
  for index in 0 ..< int(count):
    let fraction = float32(fractions[index])
    result.add(ShapeCastHit(
      bodyId: BodyId(bodyIds[index]),
      subShapeId: subShapeIds[index],
      fraction: fraction,
      distance: fraction * maxDistance,
      position: Vec3(
        x: origin.x + castDelta.x * fraction,
        y: origin.y + castDelta.y * fraction,
        z: origin.z + castDelta.z * fraction),
      contactPoint: fromRaw(contactPoints[index]),
      normal: fromRaw(normals[index])))

proc overlapShape*(world: World; shape: Shape; position: Vec3;
                   rotation = quatIdentity(); maxHits = 256;
                   layer = none(CollisionLayer);
                   bodyFilter = QueryBodyFilter();
                   subShapeFilter = QuerySubShapeFilter()): seq[OverlapHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  position.requireFinite("shape overlap position")
  if maxHits <= 0 or uint64(maxHits) > uint64(high(uint32)):
    raise newException(ValueError, "maxHits must fit in a positive uint32")
  let cookedShape = cookConvexQueryShape(shape)
  defer: cookedShape.release()
  var bodyIds = newSeq[uint32](maxHits)
  var depths = newSeq[cfloat](maxHits)
  var points = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.overlapConvex(
    cookedShape.native,
    position.toRaw,
    rotation.normalized.toRaw,
    addr bodyIds[0],
    addr depths[0],
    addr points[0],
    addr normals[0],
    addr subShapeIds[0],
    uint32(maxHits),
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly,
    subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
    uint32(subShapeFilter.len), subShapeFilter.includesOnly)
  result = newSeqOfCap[OverlapHit](int(count))
  for index in 0 ..< int(count):
    result.add(OverlapHit(
      bodyId: BodyId(bodyIds[index]),
      subShapeId: subShapeIds[index],
      penetrationDepth: depths[index],
      contactPoint: fromRaw(points[index]),
      normal: fromRaw(normals[index])))

proc validateQueryCapacity(maxHits: int): uint32 =
  if maxHits <= 0 or uint64(maxHits) > uint64(high(uint32)):
    raise newException(ValueError, "maxHits must fit in a positive uint32")
  uint32(maxHits)

proc collidePoint*(world: World; point: Vec3; maxHits = 256;
                   layer = none(CollisionLayer);
                   bodyFilter = QueryBodyFilter();
                   subShapeFilter = QuerySubShapeFilter()): seq[BodyId] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  point.requireFinite("point query position")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.collidePoint(
    point.toRaw, addr bodyIds[0], capacity, world.queryLayerValue(layer),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len), bodyFilter.includesOnly,
    subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
    uint32(subShapeFilter.len), subShapeFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQueryBox*(world: World; minimum, maximum: Vec3;
                         maxHits = 256;
                         layer = none(CollisionLayer);
                         bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  minimum.requireFinite("broad-phase box minimum")
  maximum.requireFinite("broad-phase box maximum")
  if minimum.x > maximum.x or minimum.y > maximum.y or
      minimum.z > maximum.z:
    raise newException(ValueError, "broad-phase box bounds must be ordered")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollideAABox(
    minimum.toRaw, maximum.toRaw, addr bodyIds[0], capacity,
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQuerySphere*(world: World; center: Vec3; radius: float32;
                            maxHits = 256;
                            layer = none(CollisionLayer);
                            bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  center.requireFinite("broad-phase sphere center")
  if not radius.isFinite or radius <= 0:
    raise newException(
      ValueError, "broad-phase sphere radius must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollideSphere(
    center.toRaw, radius, addr bodyIds[0], capacity,
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQueryPoint*(world: World; point: Vec3; maxHits = 256;
                           layer = none(CollisionLayer);
                           bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  point.requireFinite("broad-phase point")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollidePoint(
    point.toRaw, addr bodyIds[0], capacity, world.queryLayerValue(layer),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQueryOrientedBox*(
    world: World; center, halfExtent: Vec3;
    rotation = quatIdentity(); maxHits = 256;
    layer = none(CollisionLayer);
    bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  center.requireFinite("broad-phase oriented-box center")
  if not halfExtent.isFinite or halfExtent.x <= 0 or
      halfExtent.y <= 0 or halfExtent.z <= 0:
    raise newException(
      ValueError, "broad-phase oriented-box half extents must be positive")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollideOrientedBox(
    center.toRaw, rotation.normalized.toRaw, halfExtent.toRaw,
    addr bodyIds[0], capacity, world.queryLayerValue(layer),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseCastRay*(world: World; origin, direction: Vec3;
                        maxDistance: float32; maxHits = 256;
                        layer = none(CollisionLayer);
                        bodyFilter = QueryBodyFilter()): seq[BroadPhaseCastHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  origin.requireFinite("broad-phase ray origin")
  direction.requireFinite("broad-phase ray direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "broad-phase ray direction must be non-zero")
  let capacity = validateQueryCapacity(maxHits)
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  let count = world.physics.broadPhaseCastRay(
    origin.toRaw, delta.toRaw, addr bodyIds[0], addr fractions[0],
    capacity, world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BroadPhaseCastHit](int(count))
  for index in 0 ..< int(count):
    result.add(BroadPhaseCastHit(
      bodyId: BodyId(bodyIds[index]),
      fraction: fractions[index],
      distance: fractions[index] * maxDistance))

proc broadPhaseCastBox*(world: World; center, halfExtent, direction: Vec3;
                        maxDistance: float32; maxHits = 256;
                        layer = none(CollisionLayer);
                        bodyFilter = QueryBodyFilter()): seq[BroadPhaseCastHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  center.requireFinite("broad-phase box-cast center")
  direction.requireFinite("broad-phase box-cast direction")
  if not halfExtent.isFinite or halfExtent.x <= 0 or
      halfExtent.y <= 0 or halfExtent.z <= 0:
    raise newException(
      ValueError, "broad-phase box-cast half extents must be positive")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(
      ValueError, "broad-phase box-cast direction must be non-zero")
  let capacity = validateQueryCapacity(maxHits)
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  let count = world.physics.broadPhaseCastAABox(
    center.toRaw, halfExtent.toRaw, delta.toRaw,
    addr bodyIds[0], addr fractions[0], capacity,
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BroadPhaseCastHit](int(count))
  for index in 0 ..< int(count):
    result.add(BroadPhaseCastHit(
      bodyId: BodyId(bodyIds[index]),
      fraction: fractions[index],
      distance: fractions[index] * maxDistance))

proc broadPhaseBounds*(world: World): BroadPhaseBounds =
  world.requireOpen()
  var minimum, maximum: raw.Vec3
  world.physics.broadPhaseBounds(addr minimum, addr maximum)
  BroadPhaseBounds(minimum: fromRaw(minimum), maximum: fromRaw(maximum))

proc castRay*(world: World; origin, direction: Vec3;
              maxDistance: float32;
              layer = none(CollisionLayer);
              bodyFilter = QueryBodyFilter();
              subShapeFilter = QuerySubShapeFilter()): Option[RayHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("ray origin")
  direction.requireFinite("ray direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y + direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "ray direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let rayDelta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale
  )
  var bodyId: uint32
  var fraction: cfloat
  var subShapeId: uint32
  if not world.physics.castRay(
      origin.toRaw,
      rayDelta.toRaw,
      addr bodyId,
      addr fraction,
      addr subShapeId,
      world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
      uint32(bodyFilter.len), bodyFilter.includesOnly,
      subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
      uint32(subShapeFilter.len), subShapeFilter.includesOnly
    ):
    return none(RayHit)
  let hitPosition = Vec3(
    x: origin.x + rayDelta.x * fraction,
    y: origin.y + rayDelta.y * fraction,
    z: origin.z + rayDelta.z * fraction
  )
  some(RayHit(
    bodyId: BodyId(bodyId),
    subShapeId: subShapeId,
    fraction: fraction,
    distance: fraction * maxDistance,
    position: hitPosition
  ))

proc castRayAll*(world: World; origin, direction: Vec3;
                 maxDistance: float32; maxHits = 256;
                 layer = none(CollisionLayer);
                 bodyFilter = QueryBodyFilter();
                 subShapeFilter = QuerySubShapeFilter()): seq[RayHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("ray origin")
  direction.requireFinite("ray direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  if maxHits <= 0 or uint64(maxHits) > uint64(high(uint32)):
    raise newException(ValueError, "maxHits must fit in a positive uint32")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y + direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "ray direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let rayDelta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale
  )
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.castRayAll(
    origin.toRaw,
    rayDelta.toRaw,
    addr bodyIds[0],
    addr fractions[0],
    addr subShapeIds[0],
    uint32(maxHits),
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly,
    subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
    uint32(subShapeFilter.len), subShapeFilter.includesOnly
  )
  result = newSeqOfCap[RayHit](int(count))
  for index in 0 ..< int(count):
    let fraction = float32(fractions[index])
    result.add(RayHit(
      bodyId: BodyId(bodyIds[index]),
      subShapeId: subShapeIds[index],
      fraction: fraction,
      distance: fraction * maxDistance,
      position: Vec3(
        x: origin.x + rayDelta.x * fraction,
        y: origin.y + rayDelta.y * fraction,
        z: origin.z + rayDelta.z * fraction
      )
    ))

proc castSphere*(world: World; radius: float32; origin, direction: Vec3;
                 maxDistance: float32;
                 layer = none(CollisionLayer);
                 bodyFilter = QueryBodyFilter();
                 subShapeFilter = QuerySubShapeFilter()): Option[ShapeCastHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("sphere cast origin")
  direction.requireFinite("sphere cast direction")
  if not radius.isFinite or radius <= 0:
    raise newException(ValueError, "sphere cast radius must be finite and positive")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y + direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "sphere cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let castDelta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyId: uint32
  var fraction: cfloat
  var subShapeId: uint32
  var contactPoint, normal: raw.Vec3
  if not world.physics.castSphere(
      radius,
      origin.toRaw,
      castDelta.toRaw,
      addr bodyId,
      addr fraction,
      addr contactPoint,
      addr normal,
      addr subShapeId,
      world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
      uint32(bodyFilter.len), bodyFilter.includesOnly,
      subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
      uint32(subShapeFilter.len), subShapeFilter.includesOnly):
    return none(ShapeCastHit)
  let nativeFraction = float32(fraction)
  some(ShapeCastHit(
    bodyId: BodyId(bodyId),
    subShapeId: subShapeId,
    fraction: nativeFraction,
    distance: nativeFraction * maxDistance,
    position: Vec3(
      x: origin.x + castDelta.x * nativeFraction,
      y: origin.y + castDelta.y * nativeFraction,
      z: origin.z + castDelta.z * nativeFraction),
    contactPoint: fromRaw(contactPoint),
    normal: fromRaw(normal)))

proc castSphereAll*(world: World; radius: float32; origin, direction: Vec3;
                    maxDistance: float32;
                    maxHits = 256;
                    layer = none(CollisionLayer);
                    bodyFilter = QueryBodyFilter();
                    subShapeFilter = QuerySubShapeFilter()): seq[ShapeCastHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("sphere cast origin")
  direction.requireFinite("sphere cast direction")
  if not radius.isFinite or radius <= 0:
    raise newException(ValueError, "sphere cast radius must be finite and positive")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  if maxHits <= 0 or uint64(maxHits) > uint64(high(uint32)):
    raise newException(ValueError, "maxHits must fit in a positive uint32")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y + direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "sphere cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let castDelta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  var contactPoints = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.castSphereAll(
    radius,
    origin.toRaw,
    castDelta.toRaw,
    addr bodyIds[0],
    addr fractions[0],
    addr contactPoints[0],
    addr normals[0],
    addr subShapeIds[0],
    uint32(maxHits),
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly,
    subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
    uint32(subShapeFilter.len), subShapeFilter.includesOnly)
  result = newSeqOfCap[ShapeCastHit](int(count))
  for index in 0 ..< int(count):
    let fraction = float32(fractions[index])
    result.add(ShapeCastHit(
      bodyId: BodyId(bodyIds[index]),
      subShapeId: subShapeIds[index],
      fraction: fraction,
      distance: fraction * maxDistance,
      position: Vec3(
        x: origin.x + castDelta.x * fraction,
        y: origin.y + castDelta.y * fraction,
        z: origin.z + castDelta.z * fraction),
      contactPoint: fromRaw(contactPoints[index]),
      normal: fromRaw(normals[index])))

proc overlapSphere*(world: World; center: Vec3; radius: float32;
                    maxHits = 256;
                    layer = none(CollisionLayer);
                    bodyFilter = QueryBodyFilter();
                    subShapeFilter = QuerySubShapeFilter()): seq[OverlapHit] =
  world.requireOpen()
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  center.requireFinite("sphere center")
  if not radius.isFinite or radius <= 0:
    raise newException(ValueError, "sphere radius must be finite and positive")
  if maxHits <= 0 or uint64(maxHits) > uint64(high(uint32)):
    raise newException(ValueError, "maxHits must fit in a positive uint32")
  var bodyIds = newSeq[uint32](maxHits)
  var depths = newSeq[cfloat](maxHits)
  var points = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.overlapSphere(
    center.toRaw,
    radius,
    addr bodyIds[0],
    addr depths[0],
    addr points[0],
    addr normals[0],
    addr subShapeIds[0],
    uint32(maxHits),
    world.queryLayerValue(layer), bodyFilter.nativeBodyIds,
    uint32(bodyFilter.len), bodyFilter.includesOnly,
    subShapeFilter.nativeSubShapeBodyIds, subShapeFilter.nativeSubShapeIds,
    uint32(subShapeFilter.len), subShapeFilter.includesOnly
  )
  result = newSeqOfCap[OverlapHit](int(count))
  for index in 0 ..< int(count):
    result.add(OverlapHit(
      bodyId: BodyId(bodyIds[index]),
      subShapeId: subShapeIds[index],
      penetrationDepth: depths[index],
      contactPoint: fromRaw(points[index]),
      normal: fromRaw(normals[index])
    ))

proc castRay*(world: World; origin, direction: Vec3; maxDistance: float32;
              layers: QueryLayerSet;
              bodyFilter = QueryBodyFilter();
              subShapeFilter = QuerySubShapeFilter()): Option[RayHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("ray origin")
  direction.requireFinite("ray direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "ray direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyId, subShapeId: uint32
  var fraction: cfloat
  if not world.physics.castRay(
      origin.toRaw, delta.toRaw, addr bodyId, addr fraction,
      addr subShapeId, unsafeAddr layers.layers[0], uint32(layers.len),
      bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
      bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
      subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
      subShapeFilter.includesOnly):
    return none(RayHit)
  some(RayHit(
    bodyId: BodyId(bodyId), subShapeId: subShapeId,
    fraction: fraction, distance: fraction * maxDistance,
    position: Vec3(
      x: origin.x + delta.x * fraction,
      y: origin.y + delta.y * fraction,
      z: origin.z + delta.z * fraction)))

proc castRayAll*(world: World; origin, direction: Vec3; maxDistance: float32;
                 layers: QueryLayerSet; maxHits = 256;
                 bodyFilter = QueryBodyFilter();
                 subShapeFilter = QuerySubShapeFilter()): seq[RayHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("ray origin")
  direction.requireFinite("ray direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "ray direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.castRayAll(
    origin.toRaw, delta.toRaw, addr bodyIds[0], addr fractions[0],
    addr subShapeIds[0], capacity, unsafeAddr layers.layers[0],
    uint32(layers.len), bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
    subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
    subShapeFilter.includesOnly)
  result = newSeqOfCap[RayHit](int(count))
  for index in 0 ..< int(count):
    let fraction = float32(fractions[index])
    result.add(RayHit(
      bodyId: BodyId(bodyIds[index]), subShapeId: subShapeIds[index],
      fraction: fraction, distance: fraction * maxDistance,
      position: Vec3(
        x: origin.x + delta.x * fraction,
        y: origin.y + delta.y * fraction,
        z: origin.z + delta.z * fraction)))

proc castSphere*(world: World; radius: float32; origin, direction: Vec3;
                 maxDistance: float32;
                 layers: QueryLayerSet;
                 bodyFilter = QueryBodyFilter();
                 subShapeFilter = QuerySubShapeFilter()): Option[ShapeCastHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("sphere cast origin")
  direction.requireFinite("sphere cast direction")
  if not radius.isFinite or radius <= 0:
    raise newException(ValueError, "sphere cast radius must be finite and positive")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "sphere cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyId, subShapeId: uint32
  var fraction: cfloat
  var contactPoint, normal: raw.Vec3
  if not world.physics.castSphere(
      radius, origin.toRaw, delta.toRaw, addr bodyId, addr fraction,
      addr contactPoint, addr normal, addr subShapeId,
      unsafeAddr layers.layers[0], uint32(layers.len),
      bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
      bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
      subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
      subShapeFilter.includesOnly):
    return none(ShapeCastHit)
  let nativeFraction = float32(fraction)
  some(ShapeCastHit(
    bodyId: BodyId(bodyId), subShapeId: subShapeId,
    fraction: nativeFraction, distance: nativeFraction * maxDistance,
    position: Vec3(
      x: origin.x + delta.x * nativeFraction,
      y: origin.y + delta.y * nativeFraction,
      z: origin.z + delta.z * nativeFraction),
    contactPoint: fromRaw(contactPoint), normal: fromRaw(normal)))

proc castSphereAll*(world: World; radius: float32; origin, direction: Vec3;
                    maxDistance: float32; layers: QueryLayerSet;
                    maxHits = 256;
                    bodyFilter = QueryBodyFilter();
                    subShapeFilter = QuerySubShapeFilter()): seq[ShapeCastHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("sphere cast origin")
  direction.requireFinite("sphere cast direction")
  if not radius.isFinite or radius <= 0:
    raise newException(ValueError, "sphere cast radius must be finite and positive")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "sphere cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  var contactPoints = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.castSphereAll(
    radius, origin.toRaw, delta.toRaw, addr bodyIds[0], addr fractions[0],
    addr contactPoints[0], addr normals[0], addr subShapeIds[0], capacity,
    unsafeAddr layers.layers[0], uint32(layers.len),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
    subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
    subShapeFilter.includesOnly)
  result = newSeqOfCap[ShapeCastHit](int(count))
  for index in 0 ..< int(count):
    let fraction = float32(fractions[index])
    result.add(ShapeCastHit(
      bodyId: BodyId(bodyIds[index]), subShapeId: subShapeIds[index],
      fraction: fraction, distance: fraction * maxDistance,
      position: Vec3(
        x: origin.x + delta.x * fraction,
        y: origin.y + delta.y * fraction,
        z: origin.z + delta.z * fraction),
      contactPoint: fromRaw(contactPoints[index]),
      normal: fromRaw(normals[index])))

proc overlapSphere*(world: World; center: Vec3; radius: float32;
                    layers: QueryLayerSet; maxHits = 256;
                    bodyFilter = QueryBodyFilter();
                    subShapeFilter = QuerySubShapeFilter()): seq[OverlapHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  center.requireFinite("sphere center")
  if not radius.isFinite or radius <= 0:
    raise newException(ValueError, "sphere radius must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  var depths = newSeq[cfloat](maxHits)
  var points = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.overlapSphere(
    center.toRaw, radius, addr bodyIds[0], addr depths[0], addr points[0],
    addr normals[0], addr subShapeIds[0], capacity,
    unsafeAddr layers.layers[0], uint32(layers.len),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
    subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
    subShapeFilter.includesOnly)
  result = newSeqOfCap[OverlapHit](int(count))
  for index in 0 ..< int(count):
    result.add(OverlapHit(
      bodyId: BodyId(bodyIds[index]), subShapeId: subShapeIds[index],
      penetrationDepth: depths[index], contactPoint: fromRaw(points[index]),
      normal: fromRaw(normals[index])))

proc castShape*(world: World; shape: Shape; origin, direction: Vec3;
                maxDistance: float32; layers: QueryLayerSet;
                rotation = quatIdentity();
                bodyFilter = QueryBodyFilter();
                subShapeFilter = QuerySubShapeFilter()): Option[ShapeCastHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("shape cast origin")
  direction.requireFinite("shape cast direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "shape cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  let cookedShape = cookConvexQueryShape(shape)
  defer: cookedShape.release()
  var bodyId, subShapeId: uint32
  var fraction: cfloat
  var contactPoint, normal: raw.Vec3
  if not world.physics.castConvex(
      cookedShape.native, origin.toRaw, rotation.normalized.toRaw,
      delta.toRaw, addr bodyId, addr fraction, addr contactPoint,
      addr normal, addr subShapeId, unsafeAddr layers.layers[0],
      uint32(layers.len), bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
      bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
      subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
      subShapeFilter.includesOnly):
    return none(ShapeCastHit)
  let nativeFraction = float32(fraction)
  some(ShapeCastHit(
    bodyId: BodyId(bodyId), subShapeId: subShapeId,
    fraction: nativeFraction, distance: nativeFraction * maxDistance,
    position: Vec3(
      x: origin.x + delta.x * nativeFraction,
      y: origin.y + delta.y * nativeFraction,
      z: origin.z + delta.z * nativeFraction),
    contactPoint: fromRaw(contactPoint), normal: fromRaw(normal)))

proc castShapeAll*(world: World; shape: Shape; origin, direction: Vec3;
                   maxDistance: float32; layers: QueryLayerSet;
                   rotation = quatIdentity(); maxHits = 256;
                   bodyFilter = QueryBodyFilter();
                   subShapeFilter = QuerySubShapeFilter()): seq[ShapeCastHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  origin.requireFinite("shape cast origin")
  direction.requireFinite("shape cast direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "shape cast direction must have non-zero length")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  let cookedShape = cookConvexQueryShape(shape)
  defer: cookedShape.release()
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  var contactPoints = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.castConvexAll(
    cookedShape.native, origin.toRaw, rotation.normalized.toRaw, delta.toRaw,
    addr bodyIds[0], addr fractions[0], addr contactPoints[0],
    addr normals[0], addr subShapeIds[0], capacity,
    unsafeAddr layers.layers[0], uint32(layers.len),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
    subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
    subShapeFilter.includesOnly)
  result = newSeqOfCap[ShapeCastHit](int(count))
  for index in 0 ..< int(count):
    let fraction = float32(fractions[index])
    result.add(ShapeCastHit(
      bodyId: BodyId(bodyIds[index]), subShapeId: subShapeIds[index],
      fraction: fraction, distance: fraction * maxDistance,
      position: Vec3(
        x: origin.x + delta.x * fraction,
        y: origin.y + delta.y * fraction,
        z: origin.z + delta.z * fraction),
      contactPoint: fromRaw(contactPoints[index]),
      normal: fromRaw(normals[index])))

proc overlapShape*(world: World; shape: Shape; position: Vec3;
                   layers: QueryLayerSet; rotation = quatIdentity();
                   maxHits = 256;
                   bodyFilter = QueryBodyFilter();
                   subShapeFilter = QuerySubShapeFilter()): seq[OverlapHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  position.requireFinite("shape overlap position")
  let capacity = validateQueryCapacity(maxHits)
  let cookedShape = cookConvexQueryShape(shape)
  defer: cookedShape.release()
  var bodyIds = newSeq[uint32](maxHits)
  var depths = newSeq[cfloat](maxHits)
  var points = newSeq[raw.Vec3](maxHits)
  var normals = newSeq[raw.Vec3](maxHits)
  var subShapeIds = newSeq[uint32](maxHits)
  let count = world.physics.overlapConvex(
    cookedShape.native, position.toRaw, rotation.normalized.toRaw,
    addr bodyIds[0], addr depths[0], addr points[0], addr normals[0],
    addr subShapeIds[0], capacity, unsafeAddr layers.layers[0],
    uint32(layers.len), bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
    subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
    subShapeFilter.includesOnly)
  result = newSeqOfCap[OverlapHit](int(count))
  for index in 0 ..< int(count):
    result.add(OverlapHit(
      bodyId: BodyId(bodyIds[index]), subShapeId: subShapeIds[index],
      penetrationDepth: depths[index], contactPoint: fromRaw(points[index]),
      normal: fromRaw(normals[index])))

proc collidePoint*(world: World; point: Vec3; layers: QueryLayerSet;
                   maxHits = 256;
                   bodyFilter = QueryBodyFilter();
                   subShapeFilter = QuerySubShapeFilter()): seq[BodyId] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  world.requireSubShapeFilter(subShapeFilter)
  point.requireFinite("point query position")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.collidePoint(
    point.toRaw, addr bodyIds[0], capacity, unsafeAddr layers.layers[0],
    uint32(layers.len), bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly, subShapeFilter.nativeSubShapeBodyIds,
    subShapeFilter.nativeSubShapeIds, uint32(subShapeFilter.len),
    subShapeFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQueryBox*(world: World; minimum, maximum: Vec3;
                         layers: QueryLayerSet;
                         maxHits = 256;
                         bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  minimum.requireFinite("broad-phase box minimum")
  maximum.requireFinite("broad-phase box maximum")
  if minimum.x > maximum.x or minimum.y > maximum.y or
      minimum.z > maximum.z:
    raise newException(ValueError, "broad-phase box bounds must be ordered")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollideAABox(
    minimum.toRaw, maximum.toRaw, addr bodyIds[0], capacity,
    unsafeAddr layers.layers[0], uint32(layers.len),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQuerySphere*(world: World; center: Vec3; radius: float32;
                            layers: QueryLayerSet;
                            maxHits = 256;
                            bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  center.requireFinite("broad-phase sphere center")
  if not radius.isFinite or radius <= 0:
    raise newException(
      ValueError, "broad-phase sphere radius must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollideSphere(
    center.toRaw, radius, addr bodyIds[0], capacity,
    unsafeAddr layers.layers[0], uint32(layers.len),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQueryPoint*(world: World; point: Vec3;
                           layers: QueryLayerSet;
                           maxHits = 256;
                           bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  point.requireFinite("broad-phase point")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollidePoint(
    point.toRaw, addr bodyIds[0], capacity, unsafeAddr layers.layers[0],
    uint32(layers.len), bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseQueryOrientedBox*(
    world: World; center, halfExtent: Vec3; layers: QueryLayerSet;
    rotation = quatIdentity(); maxHits = 256;
    bodyFilter = QueryBodyFilter()): seq[BodyId] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  center.requireFinite("broad-phase oriented-box center")
  if not halfExtent.isFinite or halfExtent.x <= 0 or
      halfExtent.y <= 0 or halfExtent.z <= 0:
    raise newException(
      ValueError, "broad-phase oriented-box half extents must be positive")
  let capacity = validateQueryCapacity(maxHits)
  var bodyIds = newSeq[uint32](maxHits)
  let count = world.physics.broadPhaseCollideOrientedBox(
    center.toRaw, rotation.normalized.toRaw, halfExtent.toRaw,
    addr bodyIds[0], capacity, unsafeAddr layers.layers[0],
    uint32(layers.len), bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly)
  result = newSeqOfCap[BodyId](int(count))
  for index in 0 ..< int(count):
    result.add(BodyId(bodyIds[index]))

proc broadPhaseCastRay*(world: World; origin, direction: Vec3;
                        maxDistance: float32; layers: QueryLayerSet;
                        maxHits = 256;
                        bodyFilter = QueryBodyFilter()): seq[BroadPhaseCastHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  origin.requireFinite("broad-phase ray origin")
  direction.requireFinite("broad-phase ray direction")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "broad-phase ray direction must be non-zero")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  let count = world.physics.broadPhaseCastRay(
    origin.toRaw, delta.toRaw, addr bodyIds[0], addr fractions[0], capacity,
    unsafeAddr layers.layers[0], uint32(layers.len),
    bodyFilter.nativeBodyIds, uint32(bodyFilter.len), bodyFilter.includesOnly)
  result = newSeqOfCap[BroadPhaseCastHit](int(count))
  for index in 0 ..< int(count):
    result.add(BroadPhaseCastHit(
      bodyId: BodyId(bodyIds[index]), fraction: fractions[index],
      distance: fractions[index] * maxDistance))

proc broadPhaseCastBox*(world: World; center, halfExtent, direction: Vec3;
                        maxDistance: float32; layers: QueryLayerSet;
                        maxHits = 256;
                        bodyFilter = QueryBodyFilter()): seq[BroadPhaseCastHit] =
  world.requireLayers(layers)
  world.requireBodyFilter(bodyFilter)
  center.requireFinite("broad-phase box-cast center")
  direction.requireFinite("broad-phase box-cast direction")
  if not halfExtent.isFinite or halfExtent.x <= 0 or
      halfExtent.y <= 0 or halfExtent.z <= 0:
    raise newException(
      ValueError, "broad-phase box-cast half extents must be positive")
  if not maxDistance.isFinite or maxDistance <= 0:
    raise newException(ValueError, "maxDistance must be finite and positive")
  let capacity = validateQueryCapacity(maxHits)
  let lengthSquared =
    direction.x * direction.x + direction.y * direction.y +
      direction.z * direction.z
  if lengthSquared <= 1.0e-12'f32:
    raise newException(
      ValueError, "broad-phase box-cast direction must be non-zero")
  let scale = maxDistance / sqrt(lengthSquared)
  let delta = Vec3(
    x: direction.x * scale,
    y: direction.y * scale,
    z: direction.z * scale)
  var bodyIds = newSeq[uint32](maxHits)
  var fractions = newSeq[cfloat](maxHits)
  let count = world.physics.broadPhaseCastAABox(
    center.toRaw, halfExtent.toRaw, delta.toRaw, addr bodyIds[0],
    addr fractions[0], capacity, unsafeAddr layers.layers[0],
    uint32(layers.len), bodyFilter.nativeBodyIds, uint32(bodyFilter.len),
    bodyFilter.includesOnly)
  result = newSeqOfCap[BroadPhaseCastHit](int(count))
  for index in 0 ..< int(count):
    result.add(BroadPhaseCastHit(
      bodyId: BodyId(bodyIds[index]), fraction: fractions[index],
      distance: fractions[index] * maxDistance))

proc pendingEventCount*(world: World): uint32 =
  world.requireOpen()
  world.eventBridge.pendingEventCount()

proc pendingSoftBodyContactEventCount*(world: World): uint32 =
  world.requireOpen()
  world.eventBridge.pendingSoftBodyContactEventCount()

proc droppedEventCount*(world: World; reset = false): uint64 =
  world.requireOpen()
  world.eventBridge.droppedEventCount(reset)

proc pollEvent*(world: World): Option[PhysicsEvent] =
  world.requireOpen()
  var kind: uint8
  var body1, body2, subShape1, subShape2: uint32
  var point, normal: raw.Vec3
  if not world.eventBridge.popEvent(
      addr kind,
      addr body1,
      addr body2,
      addr subShape1,
      addr subShape2,
      addr point,
      addr normal
    ):
    return none(PhysicsEvent)

  let eventKind = case kind
    of 0: PhysicsEventKind.ContactAdded
    of 1: PhysicsEventKind.ContactPersisted
    of 2: PhysicsEventKind.ContactRemoved
    of 3: PhysicsEventKind.BodyActivated
    of 4: PhysicsEventKind.BodyDeactivated
    else: raise newException(JoltError, "received an unknown native physics event")
  let hasSecondBody = eventKind in {
    PhysicsEventKind.ContactAdded,
    PhysicsEventKind.ContactPersisted,
    PhysicsEventKind.ContactRemoved
  }
  some(PhysicsEvent(
    kind: eventKind,
    body1: BodyId(body1),
    body2: if hasSecondBody: some(BodyId(body2)) else: none(BodyId),
    subShapeId1: if hasSecondBody: some(subShape1) else: none(uint32),
    subShapeId2: if hasSecondBody: some(subShape2) else: none(uint32),
    contactPoint: fromRaw(point),
    contactNormal: fromRaw(normal),
    hasManifold: eventKind in {
      PhysicsEventKind.ContactAdded,
      PhysicsEventKind.ContactPersisted
    }
  ))

proc pollSoftBodyContactEvent*(world: World): Option[SoftBodyContactEvent] =
  world.requireOpen()
  var softBody, otherBody, vertex: uint32
  var point, normal: raw.Vec3
  var isSensor: bool
  if not world.eventBridge.popSoftBodyContactEvent(
      addr softBody, addr otherBody, addr vertex,
      addr point, addr normal, addr isSensor):
    return none(SoftBodyContactEvent)
  some(SoftBodyContactEvent(
    softBody: BodyId(softBody),
    otherBody: BodyId(otherBody),
    vertex: if vertex == high(uint32): none(uint32) else: some(vertex),
    contactPoint: fromRaw(point),
    contactNormal: fromRaw(normal),
    isSensor: isSensor))

proc material1*(event: PhysicsEvent; world: World): Option[PhysicsMaterial] =
  if event.subShapeId1.isNone:
    return none(PhysicsMaterial)
  world.requireOpen()
  var name: cstring
  var red, green, blue, alpha: uint8
  if not world.physics.bodyMaterial(
      raw.bodyID(uint32(event.body1)), event.subShapeId1.get, addr name,
      addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

proc material2*(event: PhysicsEvent; world: World): Option[PhysicsMaterial] =
  if event.body2.isNone or event.subShapeId2.isNone:
    return none(PhysicsMaterial)
  world.requireOpen()
  var name: cstring
  var red, green, blue, alpha: uint8
  if not world.physics.bodyMaterial(
      raw.bodyID(uint32(event.body2.get)), event.subShapeId2.get, addr name,
      addr red, addr green, addr blue, addr alpha):
    return none(PhysicsMaterial)
  some(PhysicsMaterial(
    name: $name,
    debugColor: MaterialColor(r: red, g: green, b: blue, a: alpha)))

proc drainEvents*(world: World; limit = high(int)): seq[PhysicsEvent] =
  if limit < 0:
    raise newException(ValueError, "event drain limit must be non-negative")
  while result.len < limit:
    let event = world.pollEvent()
    if event.isNone:
      break
    result.add(event.get)

proc drainSoftBodyContactEvents*(world: World;
    limit = high(int)): seq[SoftBodyContactEvent] =
  if limit < 0:
    raise newException(ValueError, "soft body event drain limit must be non-negative")
  while result.len < limit:
    let event = world.pollSoftBodyContactEvent()
    if event.isNone:
      break
    result.add(event.get)

proc involves*(event: PhysicsEvent; body: Body): bool =
  if not body.isAlive:
    return false
  if event.body1 == BodyId(body.rawId):
    return true
  event.body2.isSome and event.body2.get == BodyId(body.rawId)

proc optimizeBroadPhase*(world: World) =
  world.requireOpen()
  world.physics.optimizeBroadPhase()

proc gravity*(world: World): Vec3 =
  world.requireOpen()
  fromRaw(world.physics.gravity())

proc setGravity*(world: World; gravity: Vec3) =
  world.requireOpen()
  gravity.requireFinite("gravity")
  world.physics.setGravity(gravity.toRaw)

proc step*(world: World; deltaTime: float32; collisionSteps = 1): set[UpdateError] =
  world.requireOpen()
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  if collisionSteps <= 0:
    raise newException(ValueError, "collisionSteps must be positive")

  let errors = world.physics.update(
    deltaTime,
    cint(collisionSteps),
    world.allocator,
    world.jobs
  )
  for character in world.rigidCharacters:
    character.postSimulation()
  if (errors and 1'u32) != 0:
    result.incl(UpdateError.ManifoldCacheFull)
  if (errors and 2'u32) != 0:
    result.incl(UpdateError.BodyPairCacheFull)
  if (errors and 4'u32) != 0:
    result.incl(UpdateError.ContactConstraintsFull)

proc position*(body: Body): Vec3 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  fromRaw(body.owner.physics.bodyInterface().position(raw.bodyID(body.rawId)))

proc centerOfMassPosition*(body: Body): Vec3 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  fromRaw(body.owner.physics.bodyInterface().centerOfMassPosition(
    raw.bodyID(body.rawId)))

proc rotation*(body: Body): Quat =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  fromRaw(body.owner.physics.bodyInterface().rotation(raw.bodyID(body.rawId)))

proc shape*(body: Body): Shape =
  if body.isNil:
    raise newException(JoltError, "Jolt body handle is nil")
  body.shapeDesc

proc setShape*(body: Body; shape: Shape; updateMassProperties = true;
               activate = true) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.constraintCount > 0:
    raise newException(
      JoltError, "shape replacement requires an unconstrained body")
  let cooked = cookShape(shape, body.motion)
  defer: cooked.release()
  body.owner.physics.bodyInterface().setShape(
    raw.bodyID(body.rawId),
    cooked.native,
    updateMassProperties,
    if activate: raw.EActivation.Activate else: raw.EActivation.DontActivate
  )
  body.owner.eventBridge.removeBodySubShapeContactPolicies(body.rawId)
  body.shapeDesc = shape

proc requireMutableCompound(body: Body) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.shapeDesc.kind != ShapeKind.MutableCompound:
    raise newException(ValueError, "body does not use a mutable compound shape")
  if body.constraintCount > 0:
    raise newException(
      JoltError,
      "mutable compound changes require an unconstrained body")

proc addMutableChild*(body: Body; child: CompoundChild;
                      activate = true): int =
  body.requireMutableCompound()
  let validated = compoundChild(child.shape, child.position, child.rotation)
  let cooked = cookShape(validated.shape, body.motion)
  defer: cooked.release()
  var nativeIndex: uint32
  if not body.owner.physics.addMutableCompoundShape(
      raw.bodyID(body.rawId),
      validated.position.toRaw,
      validated.rotation.toRaw,
      cooked.native,
      activate,
      addr nativeIndex):
    raise newException(JoltError, "Jolt could not add the mutable compound child")
  body.shapeDesc.children.add(validated)
  result = int(nativeIndex)

proc removeMutableChild*(body: Body; index: int; activate = true) =
  body.requireMutableCompound()
  if index < 0 or index >= body.shapeDesc.children.len:
    raise newException(IndexDefect, "mutable compound child index is out of bounds")
  if body.shapeDesc.children.len == 1:
    raise newException(
      ValueError, "a mutable compound must retain at least one child")
  if not body.owner.physics.removeMutableCompoundShape(
      raw.bodyID(body.rawId), uint32(index), activate):
    raise newException(JoltError, "Jolt could not remove the mutable compound child")
  body.owner.eventBridge.removeBodySubShapeContactPolicies(body.rawId)
  body.shapeDesc.children.delete(index)

proc setMutableChildTransform*(body: Body; index: int; position: Vec3;
                               rotation = quatIdentity();
                               activate = true) =
  body.requireMutableCompound()
  if index < 0 or index >= body.shapeDesc.children.len:
    raise newException(IndexDefect, "mutable compound child index is out of bounds")
  let validated = compoundChild(
    body.shapeDesc.children[index].shape, position, rotation)
  if not body.owner.physics.modifyMutableCompoundShape(
      raw.bodyID(body.rawId),
      uint32(index),
      validated.position.toRaw,
      validated.rotation.toRaw,
      nil,
      false,
      activate):
    raise newException(JoltError, "Jolt could not move the mutable compound child")
  body.shapeDesc.children[index] = validated

proc setMutableChildTransforms*(body: Body; startIndex: int;
                                transforms: openArray[CompoundChildTransform];
                                activate = true) =
  body.requireMutableCompound()
  if transforms.len == 0:
    raise newException(ValueError, "mutable compound transform batch is empty")
  if startIndex < 0 or startIndex >= body.shapeDesc.children.len or
      transforms.len > body.shapeDesc.children.len - startIndex:
    raise newException(
      IndexDefect, "mutable compound child transform range is out of bounds")
  var positions = newSeq[raw.Vec3](transforms.len)
  var rotations = newSeq[raw.Quat](transforms.len)
  var validated = newSeq[CompoundChildTransform](transforms.len)
  for offset, transform in transforms:
    validated[offset] = compoundChildTransform(
      transform.position, transform.rotation)
    positions[offset] = validated[offset].position.toRaw
    rotations[offset] = validated[offset].rotation.toRaw
  if not body.owner.physics.modifyMutableCompoundShapes(
      raw.bodyID(body.rawId), uint32(startIndex), addr positions[0],
      addr rotations[0], uint32(transforms.len), activate):
    raise newException(
      JoltError, "Jolt could not move the mutable compound children")
  for offset, transform in validated:
    body.shapeDesc.children[startIndex + offset].position = transform.position
    body.shapeDesc.children[startIndex + offset].rotation = transform.rotation

proc replaceMutableChild*(body: Body; index: int; child: CompoundChild;
                          activate = true) =
  body.requireMutableCompound()
  if index < 0 or index >= body.shapeDesc.children.len:
    raise newException(IndexDefect, "mutable compound child index is out of bounds")
  let validated = compoundChild(child.shape, child.position, child.rotation)
  let cooked = cookShape(validated.shape, body.motion)
  defer: cooked.release()
  if not body.owner.physics.modifyMutableCompoundShape(
      raw.bodyID(body.rawId),
      uint32(index),
      validated.position.toRaw,
      validated.rotation.toRaw,
      cooked.native,
      true,
      activate):
    raise newException(JoltError, "Jolt could not replace the mutable compound child")
  body.owner.eventBridge.removeBodySubShapeContactPolicies(body.rawId)
  body.shapeDesc.children[index] = validated

proc motionType*(body: Body): MotionType =
  if body.isNil:
    raise newException(JoltError, "Jolt body handle is nil")
  body.motion

proc collisionLayer*(body: Body): CollisionLayer =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().objectLayer(raw.bodyID(body.rawId))

proc setCollisionLayer*(body: Body; layer: CollisionLayer) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.requireLayer(layer)
  body.owner.physics.bodyInterface().setObjectLayer(
    raw.bodyID(body.rawId),
    layer
  )

proc setCollisionGroup*(body: Body; group: BodyCollisionGroup) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if group.filter.isNil:
    raise newException(ValueError, "collision group requires a filter")
  group.filter.validateSubgroup(group.subgroupId)
  body.owner.physics.setCollisionGroup(
    raw.bodyID(body.rawId),
    group.filter.native,
    group.groupId,
    group.subgroupId)
  body.group = some(group)

proc clearCollisionGroup*(body: Body) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.clearCollisionGroup(raw.bodyID(body.rawId))
  body.group = none(BodyCollisionGroup)

proc collisionGroup*(body: Body): Option[BodyCollisionGroup] =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  var groupId, subgroupId: uint32
  if not body.owner.physics.collisionGroup(
      raw.bodyID(body.rawId), addr groupId, addr subgroupId):
    return none(BodyCollisionGroup)
  if body.group.isNone:
    raise newException(
      JoltError, "native collision group has no matching Nim filter handle")
  let stored = body.group.get
  some(BodyCollisionGroup(
    filter: stored.filter,
    groupId: groupId,
    subgroupId: subgroupId))

proc linearVelocity*(body: Body): Vec3 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  fromRaw(body.owner.physics.bodyInterface().linearVelocity(raw.bodyID(body.rawId)))

proc setLinearVelocity*(body: Body; velocity: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  velocity.requireFinite("velocity")
  body.owner.physics.bodyInterface().setLinearVelocity(raw.bodyID(body.rawId), velocity.toRaw)

proc angularVelocity*(body: Body): Vec3 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  fromRaw(body.owner.physics.bodyInterface().angularVelocity(raw.bodyID(body.rawId)))

proc setAngularVelocity*(body: Body; velocity: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  velocity.requireFinite("angular velocity")
  body.owner.physics.bodyInterface().setAngularVelocity(
    raw.bodyID(body.rawId),
    velocity.toRaw
  )

proc velocities*(body: Body): tuple[linear, angular: Vec3] =
  ## Reads linear and angular velocity under one native body lock.
  body.requireMotionProperties("velocities")
  var linear, angular: raw.Vec3
  body.owner.physics.bodyInterface().linearAndAngularVelocity(
    raw.bodyID(body.rawId), addr linear, addr angular)
  (linear.fromRaw, angular.fromRaw)

proc setVelocities*(body: Body; linear, angular: Vec3) =
  ## Replaces both velocities under one native body lock and wakes the body.
  body.requireMotionProperties("setVelocities")
  linear.requireFinite("linear velocity")
  angular.requireFinite("angular velocity")
  body.owner.physics.bodyInterface().setLinearAndAngularVelocity(
    raw.bodyID(body.rawId), linear.toRaw, angular.toRaw)

proc addVelocities*(body: Body; linear, angular: Vec3) =
  ## Adds both velocities under one native body lock and wakes the body.
  body.requireMotionProperties("addVelocities")
  linear.requireFinite("linear velocity delta")
  angular.requireFinite("angular velocity delta")
  body.owner.physics.bodyInterface().addLinearAndAngularVelocity(
    raw.bodyID(body.rawId), linear.toRaw, angular.toRaw)

proc pointVelocity*(body: Body; point: Vec3): Vec3 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  point.requireFinite("velocity point")
  fromRaw(body.owner.physics.bodyInterface().pointVelocity(
    raw.bodyID(body.rawId), point.toRaw))

proc setTransform*(body: Body; position: Vec3; rotation: Quat;
                   activate = true) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  position.requireFinite("position")
  let activation = if activate: raw.EActivation.Activate else: raw.EActivation.DontActivate
  body.owner.physics.bodyInterface().setPositionAndRotation(
    raw.bodyID(body.rawId),
    position.toRaw,
    rotation.normalized.toRaw,
    activation
  )

proc setTransformWhenChanged*(body: Body; position: Vec3; rotation: Quat;
                              activate = true) =
  ## Avoids broad-phase work and activation when the transform is unchanged.
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  position.requireFinite("position")
  let activation =
    if activate: raw.EActivation.Activate
    else: raw.EActivation.DontActivate
  body.owner.physics.bodyInterface().setPositionAndRotationWhenChanged(
    raw.bodyID(body.rawId), position.toRaw, rotation.normalized.toRaw,
    activation)

proc setTransformAndVelocity*(body: Body; position: Vec3; rotation: Quat;
                              linearVelocity, angularVelocity: Vec3) =
  ## Atomically replaces the complete transform and velocity state and wakes
  ## the moving body.
  body.requireMotionProperties("setTransformAndVelocity")
  position.requireFinite("position")
  linearVelocity.requireFinite("linear velocity")
  angularVelocity.requireFinite("angular velocity")
  body.owner.physics.bodyInterface().setPositionRotationAndVelocity(
    raw.bodyID(body.rawId), position.toRaw, rotation.normalized.toRaw,
    linearVelocity.toRaw, angularVelocity.toRaw)

proc moveKinematic*(body: Body; targetPosition: Vec3; targetRotation: Quat;
                    deltaTime: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion != MotionType.Kinematic:
    raise newException(JoltError, "moveKinematic requires a kinematic body")
  targetPosition.requireFinite("target position")
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  body.owner.physics.bodyInterface().moveKinematic(
    raw.bodyID(body.rawId),
    targetPosition.toRaw,
    targetRotation.normalized.toRaw,
    deltaTime
  )

proc addForce*(body: Body; force: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  force.requireFinite("force")
  body.owner.physics.bodyInterface().addForce(
    raw.bodyID(body.rawId),
    force.toRaw,
    raw.EActivation.Activate
  )

proc addForceAtPosition*(body: Body; force, position: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  force.requireFinite("force")
  position.requireFinite("force position")
  body.owner.physics.bodyInterface().addForce(
    raw.bodyID(body.rawId), force.toRaw, position.toRaw,
    raw.EActivation.Activate)

proc addTorque*(body: Body; torque: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  torque.requireFinite("torque")
  body.owner.physics.bodyInterface().addTorque(
    raw.bodyID(body.rawId),
    torque.toRaw,
    raw.EActivation.Activate
  )

proc addImpulse*(body: Body; impulse: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  impulse.requireFinite("impulse")
  body.owner.physics.bodyInterface().addImpulse(
    raw.bodyID(body.rawId),
    impulse.toRaw
  )

proc addImpulseAtPosition*(body: Body; impulse, position: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  impulse.requireFinite("impulse")
  position.requireFinite("impulse position")
  body.owner.physics.bodyInterface().addImpulse(
    raw.bodyID(body.rawId), impulse.toRaw, position.toRaw)

proc addAngularImpulse*(body: Body; impulse: Vec3) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  impulse.requireFinite("angular impulse")
  body.owner.physics.bodyInterface().addAngularImpulse(
    raw.bodyID(body.rawId),
    impulse.toRaw
  )

proc applyBuoyancyImpulse*(body: Body; surfacePosition, surfaceNormal: Vec3;
                           buoyancy, linearDrag, angularDrag: float32;
                           fluidVelocity, gravity: Vec3;
                           deltaTime: float32): bool =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion != MotionType.Dynamic:
    raise newException(JoltError, "buoyancy requires a dynamic body")
  surfacePosition.requireFinite("fluid surface position")
  fluidVelocity.requireFinite("fluid velocity")
  gravity.requireFinite("buoyancy gravity")
  if not surfaceNormal.isFinite:
    raise newException(ValueError, "fluid surface normal must be finite")
  let normalLengthSquared =
    surfaceNormal.x * surfaceNormal.x +
    surfaceNormal.y * surfaceNormal.y +
    surfaceNormal.z * surfaceNormal.z
  if normalLengthSquared <= 1.0e-12'f32:
    raise newException(ValueError, "fluid surface normal must have non-zero length")
  if not buoyancy.isFinite or buoyancy < 0:
    raise newException(ValueError, "buoyancy must be finite and non-negative")
  if not linearDrag.isFinite or linearDrag < 0 or
      not angularDrag.isFinite or angularDrag < 0:
    raise newException(ValueError, "fluid drag must be finite and non-negative")
  if not deltaTime.isFinite or deltaTime <= 0:
    raise newException(ValueError, "deltaTime must be finite and positive")
  let inverseNormalLength = 1.0'f32 / sqrt(normalLengthSquared)
  let normal = Vec3(
    x: surfaceNormal.x * inverseNormalLength,
    y: surfaceNormal.y * inverseNormalLength,
    z: surfaceNormal.z * inverseNormalLength)
  body.owner.physics.bodyInterface().applyBuoyancyImpulse(
    raw.bodyID(body.rawId),
    surfacePosition.toRaw,
    normal.toRaw,
    buoyancy,
    linearDrag,
    angularDrag,
    fluidVelocity.toRaw,
    gravity.toRaw,
    deltaTime
  )

proc damping*(body: Body): tuple[linear, angular: float32] =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion == MotionType.Static:
    raise newException(JoltError, "static bodies do not have damping")
  var linear, angular: cfloat
  if not body.owner.physics.damping(
      raw.bodyID(body.rawId), addr linear, addr angular):
    raise newException(JoltError, "Jolt could not read body damping")
  (float32(linear), float32(angular))

proc setDamping*(body: Body; linear, angular: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if not linear.isFinite or linear < 0 or
      not angular.isFinite or angular < 0:
    raise newException(ValueError, "damping must be finite and non-negative")
  if body.motion == MotionType.Static:
    raise newException(JoltError, "static bodies do not have damping")
  if not body.owner.physics.setDamping(
      raw.bodyID(body.rawId), linear, angular):
    raise newException(JoltError, "Jolt could not update body damping")

proc activate*(body: Body) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().activate(raw.bodyID(body.rawId))

proc deactivate*(body: Body) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion == MotionType.Static:
    raise newException(JoltError, "cannot deactivate a static body")
  body.owner.physics.bodyInterface().deactivate(raw.bodyID(body.rawId))

proc requireMotionProperties(body: Body; operation: string) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion == MotionType.Static:
    raise newException(JoltError, operation & " requires a moving body")

proc mass*(body: Body): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion != MotionType.Dynamic:
    raise newException(JoltError, "mass requires a dynamic body")
  var value: cfloat
  if not body.owner.physics.bodyMass(raw.bodyID(body.rawId), addr value):
    raise newException(
      JoltError, "body mass is unavailable when all translation DOFs are locked")
  float32(value)

proc massProperties*(body: Body): BodyMassProperties =
  ## Returns Jolt's canonical principal-inertia decomposition. Bodies with all
  ## translation or all rotation axes locked do not retain a complete finite
  ## decomposition and therefore reject this operation.
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion != MotionType.Dynamic:
    raise newException(JoltError, "massProperties requires a dynamic body")
  var mass: cfloat
  var inertiaDiagonal: raw.Vec3
  var inertiaRotation: raw.Quat
  if not body.owner.physics.bodyMassProperties(
      raw.bodyID(body.rawId), addr mass, addr inertiaDiagonal,
      addr inertiaRotation):
    raise newException(
      JoltError,
      "complete mass properties require unlocked translation and rotation")
  BodyMassProperties(
    mass: float32(mass),
    inertiaDiagonal: fromRaw(inertiaDiagonal),
    inertiaRotation: fromRaw(inertiaRotation))

proc setMassProperties*(body: Body; properties: BodyMassProperties) =
  ## Atomically replaces mass and the full local-space inertia tensor, then
  ## wakes the body. Principal axes may be oriented independently of its shape.
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion != MotionType.Dynamic:
    raise newException(JoltError, "setMassProperties requires a dynamic body")
  properties.validate()
  if not body.owner.physics.setBodyMassProperties(
      raw.bodyID(body.rawId),
      properties.mass,
      properties.inertiaDiagonal.toRaw,
      properties.inertiaRotation.normalized.toRaw):
    raise newException(JoltError, "Jolt could not update body mass properties")

proc setMass*(body: Body; value: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if body.motion != MotionType.Dynamic:
    raise newException(JoltError, "setMass requires a dynamic body")
  if not value.isFinite or value <= 0:
    raise newException(ValueError, "body mass must be finite and positive")
  if not body.owner.physics.setBodyMass(
      raw.bodyID(body.rawId), value):
    raise newException(
      JoltError, "body mass cannot change when all translation DOFs are locked")

proc allowedDOFs*(body: Body): set[AllowedDOF] =
  body.requireMotionProperties("allowedDOFs")
  var mask: uint8
  if not body.owner.physics.bodyAllowedDOFs(raw.bodyID(body.rawId), addr mask):
    raise newException(JoltError, "Jolt could not read allowed body DOFs")
  if (mask and 0xc0'u8) != 0:
    raise newException(JoltError, "Jolt returned unknown allowed body DOFs")
  allowedDOFsFromMask(mask)

proc creationFlags(body: Body): tuple[
    allowSleeping, collideKinematicVsNonDynamic,
    applyGyroscopicForce, enhancedInternalEdgeRemoval: bool] =
  body.requireMotionProperties("body creation flags")
  if not body.owner.physics.bodyCreationFlags(
      raw.bodyID(body.rawId),
      addr result.allowSleeping,
      addr result.collideKinematicVsNonDynamic,
      addr result.applyGyroscopicForce,
      addr result.enhancedInternalEdgeRemoval):
    raise newException(JoltError, "Jolt could not read body creation flags")

proc allowsSleeping*(body: Body): bool =
  body.creationFlags.allowSleeping

proc collidesKinematicVsNonDynamic*(body: Body): bool =
  body.creationFlags.collideKinematicVsNonDynamic

proc appliesGyroscopicForce*(body: Body): bool =
  body.creationFlags.applyGyroscopicForce

proc usesEnhancedInternalEdgeRemoval*(body: Body): bool =
  body.creationFlags.enhancedInternalEdgeRemoval

proc setCreationFlag(body: Body; flag: uint8; enabled: bool;
                     operation: string) =
  body.requireMotionProperties(operation)
  if not body.owner.physics.setBodyCreationFlag(
      raw.bodyID(body.rawId), flag, enabled):
    raise newException(JoltError, "Jolt could not update " & operation)

proc setAllowSleeping*(body: Body; enabled: bool) =
  body.setCreationFlag(0, enabled, "allowSleeping")

proc setCollideKinematicVsNonDynamic*(body: Body; enabled: bool) =
  body.setCreationFlag(1, enabled, "collideKinematicVsNonDynamic")

proc setApplyGyroscopicForce*(body: Body; enabled: bool) =
  body.setCreationFlag(2, enabled, "applyGyroscopicForce")

proc setEnhancedInternalEdgeRemoval*(body: Body; enabled: bool) =
  body.setCreationFlag(3, enabled, "enhancedInternalEdgeRemoval")

proc solverStepOverrides*(body: Body): tuple[velocity, position: uint32] =
  body.requireMotionProperties("solverStepOverrides")
  if not body.owner.physics.bodySolverStepOverrides(
      raw.bodyID(body.rawId), addr result.velocity, addr result.position):
    raise newException(JoltError, "Jolt could not read solver step overrides")

proc setSolverStepOverrides*(body: Body; velocity, position: uint32) =
  body.requireMotionProperties("setSolverStepOverrides")
  if velocity >= 256 or position >= 256:
    raise newException(ValueError, "body solver step overrides must be below 256")
  if not body.owner.physics.setBodySolverStepOverrides(
      raw.bodyID(body.rawId), velocity, position):
    raise newException(JoltError, "Jolt could not update solver step overrides")

proc resetSleepTimer*(body: Body) =
  body.requireMotionProperties("resetSleepTimer")
  body.owner.physics.bodyInterface().resetSleepTimer(raw.bodyID(body.rawId))

proc motionQuality*(body: Body): MotionQuality =
  body.requireMotionProperties("motionQuality")
  let quality = body.owner.physics.bodyInterface().motionQuality(
    raw.bodyID(body.rawId))
  if quality > uint8(ord(high(MotionQuality))):
    raise newException(JoltError, "Jolt returned an unknown motion quality")
  MotionQuality(quality)

proc setMotionQuality*(body: Body; quality: MotionQuality) =
  body.requireMotionProperties("setMotionQuality")
  body.owner.physics.bodyInterface().setMotionQuality(
    raw.bodyID(body.rawId), uint8(ord(quality)))

proc maxLinearVelocity*(body: Body): float32 =
  body.requireMotionProperties("maxLinearVelocity")
  body.owner.physics.bodyInterface().maxLinearVelocity(raw.bodyID(body.rawId))

proc setMaxLinearVelocity*(body: Body; velocity: float32) =
  body.requireMotionProperties("setMaxLinearVelocity")
  if not velocity.isFinite or velocity <= 0:
    raise newException(
      ValueError, "maximum linear velocity must be finite and positive")
  body.owner.physics.bodyInterface().setMaxLinearVelocity(
    raw.bodyID(body.rawId), velocity)

proc maxAngularVelocity*(body: Body): float32 =
  body.requireMotionProperties("maxAngularVelocity")
  body.owner.physics.bodyInterface().maxAngularVelocity(raw.bodyID(body.rawId))

proc setMaxAngularVelocity*(body: Body; velocity: float32) =
  body.requireMotionProperties("setMaxAngularVelocity")
  if not velocity.isFinite or velocity <= 0:
    raise newException(
      ValueError, "maximum angular velocity must be finite and positive")
  body.owner.physics.bodyInterface().setMaxAngularVelocity(
    raw.bodyID(body.rawId), velocity)

proc useManifoldReduction*(body: Body): bool =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().useManifoldReduction(raw.bodyID(body.rawId))

proc setUseManifoldReduction*(body: Body; enabled: bool) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().setUseManifoldReduction(
    raw.bodyID(body.rawId), enabled)

proc userData*(body: Body): uint64 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().userData(raw.bodyID(body.rawId))

proc setUserData*(body: Body; value: uint64) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().setUserData(raw.bodyID(body.rawId), value)

proc invalidateContactCache*(body: Body) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().invalidateContactCache(raw.bodyID(body.rawId))

proc friction*(body: Body): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().friction(raw.bodyID(body.rawId))

proc setFriction*(body: Body; friction: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if not friction.isFinite or friction < 0:
    raise newException(ValueError, "friction must be finite and non-negative")
  body.owner.physics.bodyInterface().setFriction(raw.bodyID(body.rawId), friction)

proc restitution*(body: Body): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().restitution(raw.bodyID(body.rawId))

proc setRestitution*(body: Body; restitution: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if not restitution.isFinite or restitution < 0 or restitution > 1:
    raise newException(ValueError, "restitution must be between 0 and 1")
  body.owner.physics.bodyInterface().setRestitution(raw.bodyID(body.rawId), restitution)

proc gravityFactor*(body: Body): float32 =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().gravityFactor(raw.bodyID(body.rawId))

proc isSensor*(body: Body): bool =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.sensor

proc setSensor*(body: Body; sensor: bool) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  body.owner.physics.bodyInterface().setSensor(raw.bodyID(body.rawId), sensor)
  body.sensor = sensor

proc setGravityFactor*(body: Body; factor: float32) =
  if not body.isAlive:
    raise newException(JoltError, "Jolt body is no longer alive")
  if not factor.isFinite:
    raise newException(ValueError, "gravity factor must be finite")
  body.owner.physics.bodyInterface().setGravityFactor(raw.bodyID(body.rawId), factor)

proc isActive*(body: Body): bool =
  if not body.isAlive:
    return false
  body.owner.physics.bodyInterface().isActive(raw.bodyID(body.rawId))
