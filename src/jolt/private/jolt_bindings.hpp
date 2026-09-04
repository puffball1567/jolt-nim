#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRendererSimple.h>
#endif
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <limits>
#include <shared_mutex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace joltnim_detail
{
inline std::mutex gLifecycleMutex;
inline std::uint32_t gLifecycleUsers = 0;

inline bool IsFiniteVec3(JPH::Vec3Arg inValue)
{
    return std::isfinite(inValue.GetX()) &&
           std::isfinite(inValue.GetY()) &&
           std::isfinite(inValue.GetZ());
}

inline bool IsFiniteQuat(JPH::QuatArg inValue)
{
    return std::isfinite(inValue.GetX()) &&
           std::isfinite(inValue.GetY()) &&
           std::isfinite(inValue.GetZ()) &&
           std::isfinite(inValue.GetW());
}

inline bool AcquireJolt()
{
    std::lock_guard<std::mutex> lock(gLifecycleMutex);

    if (gLifecycleUsers == 0)
    {
        if (!JPH::VerifyJoltVersionID())
            return false;

        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }

    ++gLifecycleUsers;
    return true;
}

inline void ReleaseJolt()
{
    std::lock_guard<std::mutex> lock(gLifecycleMutex);

    if (gLifecycleUsers == 0 || --gLifecycleUsers != 0)
        return;

    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}

inline JPH::ObjectVsBroadPhaseLayerFilterTable *CreateObjectVsBroadPhaseLayerFilter(
    const JPH::BroadPhaseLayerInterfaceTable *inBroadPhaseLayerInterface,
    JPH::uint inNumBroadPhaseLayers,
    const JPH::ObjectLayerPairFilterTable *inObjectLayerPairFilter,
    JPH::uint inNumObjectLayers)
{
    return new JPH::ObjectVsBroadPhaseLayerFilterTable(
        *inBroadPhaseLayerInterface,
        inNumBroadPhaseLayers,
        *inObjectLayerPairFilter,
        inNumObjectLayers);
}

inline void InitializePhysicsSystem(
    JPH::PhysicsSystem *ioSystem,
    JPH::uint inMaxBodies,
    JPH::uint inNumBodyMutexes,
    JPH::uint inMaxBodyPairs,
    JPH::uint inMaxContactConstraints,
    const JPH::BroadPhaseLayerInterfaceTable *inBroadPhaseLayerInterface,
    const JPH::ObjectVsBroadPhaseLayerFilterTable *inObjectVsBroadPhaseLayerFilter,
    const JPH::ObjectLayerPairFilterTable *inObjectLayerPairFilter)
{
    ioSystem->Init(
        inMaxBodies,
        inNumBodyMutexes,
        inMaxBodyPairs,
        inMaxContactConstraints,
        *inBroadPhaseLayerInterface,
        *inObjectVsBroadPhaseLayerFilter,
        *inObjectLayerPairFilter);
}

inline JPH::BodyInterface *GetBodyInterface(JPH::PhysicsSystem *inSystem)
{
    return &inSystem->GetBodyInterface();
}

inline JPH::GroupFilterTable *CreateGroupFilterTable(std::uint32_t inNumSubGroups)
{
    JPH::GroupFilterTable *filter = new JPH::GroupFilterTable(inNumSubGroups);
    filter->AddRef();
    return filter;
}

inline void ReleaseGroupFilterTable(JPH::GroupFilterTable *inFilter)
{
    if (inFilter != nullptr)
        inFilter->Release();
}

inline void SetBodyCollisionGroup(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    const JPH::GroupFilterTable *inFilter,
    std::uint32_t inGroupID,
    std::uint32_t inSubGroupID)
{
    inSystem->GetBodyInterface().SetCollisionGroup(
        inBodyID, JPH::CollisionGroup(inFilter, inGroupID, inSubGroupID));
    inSystem->GetBodyInterface().ActivateBody(inBodyID);
}

inline void ClearBodyCollisionGroup(
    JPH::PhysicsSystem *inSystem, JPH::BodyID inBodyID)
{
    inSystem->GetBodyInterface().SetCollisionGroup(
        inBodyID, JPH::CollisionGroup());
    inSystem->GetBodyInterface().ActivateBody(inBodyID);
}

inline bool GetBodyCollisionGroup(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t *outGroupID,
    std::uint32_t *outSubGroupID)
{
    const JPH::CollisionGroup &group =
        inSystem->GetBodyInterface().GetCollisionGroup(inBodyID);
    if (group.GetGroupID() == JPH::CollisionGroup::cInvalidGroup)
        return false;
    *outGroupID = group.GetGroupID();
    *outSubGroupID = group.GetSubGroupID();
    return true;
}

inline std::uint8_t GetBodyMotionQuality(
    const JPH::BodyInterface *inBodyInterface, JPH::BodyID inBodyID)
{
    return static_cast<std::uint8_t>(
        inBodyInterface->GetMotionQuality(inBodyID));
}

inline void SetBodyMotionQuality(
    JPH::BodyInterface *ioBodyInterface,
    JPH::BodyID inBodyID,
    std::uint8_t inQuality)
{
    ioBodyInterface->SetMotionQuality(
        inBodyID, static_cast<JPH::EMotionQuality>(inQuality));
}

inline void GetBodyLinearAndAngularVelocity(
    const JPH::BodyInterface *inBodyInterface,
    JPH::BodyID inBodyID,
    JPH::Vec3 *outLinearVelocity,
    JPH::Vec3 *outAngularVelocity)
{
    inBodyInterface->GetLinearAndAngularVelocity(
        inBodyID, *outLinearVelocity, *outAngularVelocity);
}

inline void SetBodyCreationSettingsSensor(
    JPH::BodyCreationSettings &ioSettings, bool inIsSensor)
{
    ioSettings.mIsSensor = inIsSensor;
}

inline JPH::PhysicsMaterial *CreatePhysicsMaterial(
    const char *inName,
    std::uint8_t inRed,
    std::uint8_t inGreen,
    std::uint8_t inBlue,
    std::uint8_t inAlpha)
{
    JPH::PhysicsMaterialSimple *material = new JPH::PhysicsMaterialSimple(
        inName, JPH::Color(inRed, inGreen, inBlue, inAlpha));
    material->AddRef();
    return material;
}

inline void ReleasePhysicsMaterial(JPH::PhysicsMaterial *inMaterial)
{
    if (inMaterial != nullptr)
        inMaterial->Release();
}

inline bool GetBodyMaterial(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inSubShapeID,
    char **outName,
    std::uint8_t *outRed,
    std::uint8_t *outGreen,
    std::uint8_t *outBlue,
    std::uint8_t *outAlpha)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    JPH::SubShapeID sub_shape_id;
    sub_shape_id.SetValue(inSubShapeID);
    const JPH::PhysicsMaterial *material =
        lock.GetBody().GetShape()->GetMaterial(sub_shape_id);
    if (material == nullptr ||
        material == JPH::PhysicsMaterial::sDefault.GetPtr())
        return false;
    const JPH::Color color = material->GetDebugColor();
    *outName = const_cast<char *>(material->GetDebugName());
    *outRed = color.r;
    *outGreen = color.g;
    *outBlue = color.b;
    *outAlpha = color.a;
    return true;
}

inline void ConfigureBodyCreationSettings(
    JPH::BodyCreationSettings &ioSettings,
    std::uint8_t inAllowedDOFs,
    std::uint8_t inMotionQuality,
    float inMass,
    float inInertiaMultiplier,
    JPH::Vec3Arg inLinearVelocity,
    JPH::Vec3Arg inAngularVelocity,
    std::uint64_t inUserData,
    bool inAllowSleeping,
    bool inCollideKinematicVsNonDynamic,
    bool inUseManifoldReduction,
    bool inApplyGyroscopicForce,
    bool inEnhancedInternalEdgeRemoval,
    float inFriction,
    float inRestitution,
    float inLinearDamping,
    float inAngularDamping,
    float inMaxLinearVelocity,
    float inMaxAngularVelocity,
    float inGravityFactor,
    std::uint32_t inNumVelocityStepsOverride,
    std::uint32_t inNumPositionStepsOverride,
    bool inHasCustomMassProperties,
    float inCustomMass,
    JPH::Vec3Arg inCustomInertiaDiagonal,
    JPH::QuatArg inCustomInertiaRotation)
{
    ioSettings.mAllowedDOFs = static_cast<JPH::EAllowedDOFs>(inAllowedDOFs);
    ioSettings.mMotionQuality = static_cast<JPH::EMotionQuality>(inMotionQuality);
    ioSettings.mInertiaMultiplier = inInertiaMultiplier;
    ioSettings.mLinearVelocity = inLinearVelocity;
    ioSettings.mAngularVelocity = inAngularVelocity;
    ioSettings.mUserData = inUserData;
    ioSettings.mAllowSleeping = inAllowSleeping;
    ioSettings.mCollideKinematicVsNonDynamic = inCollideKinematicVsNonDynamic;
    ioSettings.mUseManifoldReduction = inUseManifoldReduction;
    ioSettings.mApplyGyroscopicForce = inApplyGyroscopicForce;
    ioSettings.mEnhancedInternalEdgeRemoval = inEnhancedInternalEdgeRemoval;
    ioSettings.mFriction = inFriction;
    ioSettings.mRestitution = inRestitution;
    ioSettings.mLinearDamping = inLinearDamping;
    ioSettings.mAngularDamping = inAngularDamping;
    ioSettings.mMaxLinearVelocity = inMaxLinearVelocity;
    ioSettings.mMaxAngularVelocity = inMaxAngularVelocity;
    ioSettings.mGravityFactor = inGravityFactor;
    ioSettings.mNumVelocityStepsOverride = inNumVelocityStepsOverride;
    ioSettings.mNumPositionStepsOverride = inNumPositionStepsOverride;

    if (inHasCustomMassProperties)
    {
        ioSettings.mOverrideMassProperties =
            JPH::EOverrideMassProperties::MassAndInertiaProvided;
        ioSettings.mMassPropertiesOverride.mMass = inCustomMass;
        const JPH::Mat44 rotation =
            JPH::Mat44::sRotation(inCustomInertiaRotation);
        ioSettings.mMassPropertiesOverride.mInertia =
            rotation * JPH::Mat44::sScale(inCustomInertiaDiagonal) *
            rotation.Inversed();
    }
    else if (inMass > 0.0f)
    {
        ioSettings.mOverrideMassProperties =
            JPH::EOverrideMassProperties::CalculateInertia;
        ioSettings.mMassPropertiesOverride.mMass = inMass;
    }
    else
    {
        ioSettings.mOverrideMassProperties =
            JPH::EOverrideMassProperties::CalculateMassAndInertia;
    }
}

struct BodySnapshotData
{
    bool mSucceeded = false;
    bool mSoftBody = false;
    std::uint8_t mMotionType = 0;
    std::uint16_t mObjectLayer = 0;
    JPH::Vec3 mPosition = JPH::Vec3::sZero();
    JPH::Vec3 mCenterOfMassPosition = JPH::Vec3::sZero();
    JPH::Quat mRotation = JPH::Quat::sIdentity();
    JPH::Vec3 mLinearVelocity = JPH::Vec3::sZero();
    JPH::Vec3 mAngularVelocity = JPH::Vec3::sZero();
    bool mActive = false;
    bool mSensor = false;
    bool mInBroadPhase = false;
    bool mCollisionCacheInvalid = false;
    bool mUseManifoldReduction = false;
    float mFriction = 0.0f;
    float mRestitution = 0.0f;
    std::uint64_t mUserData = 0;
    bool mHasMotionProperties = false;
    std::uint8_t mMotionQuality = 0;
    std::uint8_t mAllowedDOFs = 0;
    float mLinearDamping = 0.0f;
    float mAngularDamping = 0.0f;
    float mMaxLinearVelocity = 0.0f;
    float mMaxAngularVelocity = 0.0f;
    float mGravityFactor = 0.0f;
    bool mAllowSleeping = false;
    bool mCollideKinematicVsNonDynamic = false;
    bool mApplyGyroscopicForce = false;
    bool mEnhancedInternalEdgeRemoval = false;
    std::uint32_t mNumVelocityStepsOverride = 0;
    std::uint32_t mNumPositionStepsOverride = 0;
    bool mHasMass = false;
    float mMass = 0.0f;
    bool mHasMassProperties = false;
    JPH::Vec3 mInertiaDiagonal = JPH::Vec3::sZero();
    JPH::Quat mInertiaRotation = JPH::Quat::sIdentity();
};

inline void ReadBodySnapshots(
    const JPH::PhysicsSystem *inSystem,
    const std::uint32_t *inBodyIDs,
    std::uint32_t inCount,
    BodySnapshotData *outSnapshots)
{
    if (inCount == 0)
        return;

    std::vector<JPH::BodyID> body_ids;
    body_ids.reserve(inCount);
    for (std::uint32_t index = 0; index < inCount; ++index)
        body_ids.emplace_back(inBodyIDs[index]);

    JPH::BodyLockMultiRead lock(
        inSystem->GetBodyLockInterface(), body_ids.data(),
        static_cast<int>(inCount));
    for (std::uint32_t index = 0; index < inCount; ++index)
    {
        BodySnapshotData &snapshot = outSnapshots[index];
        snapshot = BodySnapshotData{};
        const JPH::Body *body = lock.GetBody(static_cast<int>(index));
        if (body == nullptr)
            continue;

        snapshot.mSucceeded = true;
        snapshot.mSoftBody = !body->IsRigidBody();
        snapshot.mMotionType = static_cast<std::uint8_t>(body->GetMotionType());
        snapshot.mObjectLayer = body->GetObjectLayer();
        snapshot.mPosition = JPH::Vec3(body->GetPosition());
        snapshot.mCenterOfMassPosition =
            JPH::Vec3(body->GetCenterOfMassPosition());
        snapshot.mRotation = body->GetRotation();
        snapshot.mLinearVelocity = body->GetLinearVelocity();
        snapshot.mAngularVelocity = body->GetAngularVelocity();
        snapshot.mActive = body->IsActive();
        snapshot.mSensor = body->IsSensor();
        snapshot.mInBroadPhase = body->IsInBroadPhase();
        snapshot.mCollisionCacheInvalid = body->IsCollisionCacheInvalid();
        snapshot.mUseManifoldReduction = body->GetUseManifoldReduction();
        snapshot.mFriction = body->GetFriction();
        snapshot.mRestitution = body->GetRestitution();
        snapshot.mUserData = body->GetUserData();

        if (body->GetMotionType() == JPH::EMotionType::Static)
            continue;

        snapshot.mHasMotionProperties = true;
        const JPH::MotionProperties *motion = body->GetMotionProperties();
        snapshot.mMotionQuality =
            static_cast<std::uint8_t>(motion->GetMotionQuality());
        snapshot.mAllowedDOFs =
            static_cast<std::uint8_t>(motion->GetAllowedDOFs());
        snapshot.mLinearDamping = motion->GetLinearDamping();
        snapshot.mAngularDamping = motion->GetAngularDamping();
        snapshot.mMaxLinearVelocity = motion->GetMaxLinearVelocity();
        snapshot.mMaxAngularVelocity = motion->GetMaxAngularVelocity();
        snapshot.mGravityFactor = motion->GetGravityFactor();
        snapshot.mAllowSleeping = body->GetAllowSleeping();
        snapshot.mCollideKinematicVsNonDynamic =
            body->GetCollideKinematicVsNonDynamic();
        snapshot.mApplyGyroscopicForce = body->GetApplyGyroscopicForce();
        snapshot.mEnhancedInternalEdgeRemoval =
            body->GetEnhancedInternalEdgeRemoval();
        snapshot.mNumVelocityStepsOverride =
            motion->GetNumVelocityStepsOverride();
        snapshot.mNumPositionStepsOverride =
            motion->GetNumPositionStepsOverride();

        if (body->GetMotionType() != JPH::EMotionType::Dynamic)
            continue;

        const float inverse_mass = motion->GetInverseMassUnchecked();
        if (inverse_mass > 0.0f)
        {
            snapshot.mHasMass = true;
            snapshot.mMass = 1.0f / inverse_mass;
        }
        const JPH::Vec3 inverse_inertia = motion->GetInverseInertiaDiagonal();
        if (snapshot.mHasMass &&
            inverse_inertia.GetX() > 0.0f &&
            inverse_inertia.GetY() > 0.0f &&
            inverse_inertia.GetZ() > 0.0f)
        {
            snapshot.mHasMassProperties = true;
            snapshot.mInertiaDiagonal = inverse_inertia.Reciprocal();
            snapshot.mInertiaRotation = motion->GetInertiaRotation();
        }
    }
}

inline bool GetBodyMass(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    float *outMass)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() != JPH::EMotionType::Dynamic)
        return false;
    const float inverse_mass =
        body.GetMotionProperties()->GetInverseMassUnchecked();
    if (inverse_mass <= 0.0f)
        return false;
    *outMass = 1.0f / inverse_mass;
    return true;
}

inline bool GetBodyMassProperties(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    float *outMass,
    JPH::Vec3 *outInertiaDiagonal,
    JPH::Quat *outInertiaRotation)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() != JPH::EMotionType::Dynamic)
        return false;
    const JPH::MotionProperties *motion = body.GetMotionProperties();
    const float inverse_mass = motion->GetInverseMassUnchecked();
    const JPH::Vec3 inverse_inertia = motion->GetInverseInertiaDiagonal();
    if (inverse_mass <= 0.0f || inverse_inertia.GetX() <= 0.0f ||
        inverse_inertia.GetY() <= 0.0f || inverse_inertia.GetZ() <= 0.0f)
        return false;
    *outMass = 1.0f / inverse_mass;
    *outInertiaDiagonal = inverse_inertia.Reciprocal();
    *outInertiaRotation = motion->GetInertiaRotation();
    return true;
}

inline bool SetBodyMassProperties(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    float inMass,
    JPH::Vec3Arg inInertiaDiagonal,
    JPH::QuatArg inInertiaRotation)
{
    {
        // The caller guarantees this runs between PhysicsSystem updates and
        // the body is no longer published in the broad phase. Not holding its
        // normal mutex avoids violating lock order when CustomUpdate locks
        // rigid collision candidates.
        JPH::BodyLockWrite lock(
            ioSystem->GetBodyLockInterfaceNoLock(), inBodyID);
        if (!lock.Succeeded())
            return false;
        JPH::Body &body = lock.GetBody();
        if (body.GetMotionType() != JPH::EMotionType::Dynamic)
            return false;
        JPH::MassProperties properties;
        properties.mMass = inMass;
        const JPH::Mat44 rotation = JPH::Mat44::sRotation(inInertiaRotation);
        properties.mInertia =
            rotation * JPH::Mat44::sScale(inInertiaDiagonal) *
            rotation.Inversed();
        JPH::MotionProperties *motion = body.GetMotionProperties();
        motion->SetMassProperties(motion->GetAllowedDOFs(), properties);
    }
    ioSystem->GetBodyInterface().ActivateBody(inBodyID);
    return true;
}

inline bool GetBodyAllowedDOFs(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint8_t *outAllowedDOFs)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() == JPH::EMotionType::Static)
        return false;
    *outAllowedDOFs = static_cast<std::uint8_t>(
        body.GetMotionProperties()->GetAllowedDOFs());
    return true;
}

inline bool GetBodyCreationFlags(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    bool *outAllowSleeping,
    bool *outCollideKinematicVsNonDynamic,
    bool *outApplyGyroscopicForce,
    bool *outEnhancedInternalEdgeRemoval)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() == JPH::EMotionType::Static)
        return false;
    *outAllowSleeping = body.GetAllowSleeping();
    *outCollideKinematicVsNonDynamic =
        body.GetCollideKinematicVsNonDynamic();
    *outApplyGyroscopicForce = body.GetApplyGyroscopicForce();
    *outEnhancedInternalEdgeRemoval =
        body.GetEnhancedInternalEdgeRemoval();
    return true;
}

inline bool GetBodySolverStepOverrides(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t *outVelocitySteps,
    std::uint32_t *outPositionSteps)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() == JPH::EMotionType::Static)
        return false;
    const JPH::MotionProperties *motion = body.GetMotionProperties();
    *outVelocitySteps = motion->GetNumVelocityStepsOverride();
    *outPositionSteps = motion->GetNumPositionStepsOverride();
    return true;
}

inline bool SetBodyMass(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    float inMass)
{
    {
        JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return false;
        JPH::Body &body = lock.GetBody();
        if (body.GetMotionType() != JPH::EMotionType::Dynamic)
            return false;
        JPH::MotionProperties *motion = body.GetMotionProperties();
        if (motion->GetInverseMassUnchecked() <= 0.0f)
            return false;
        motion->ScaleToMass(inMass);
    }
    ioSystem->GetBodyInterface().ActivateBody(inBodyID);
    return true;
}

inline bool SetBodyCreationFlag(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    std::uint8_t inFlag,
    bool inEnabled)
{
    {
        JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return false;
        JPH::Body &body = lock.GetBody();
        if (body.GetMotionType() == JPH::EMotionType::Static)
            return false;
        switch (inFlag)
        {
        case 0:
            body.SetAllowSleeping(inEnabled);
            break;
        case 1:
            body.SetCollideKinematicVsNonDynamic(inEnabled);
            break;
        case 2:
            body.SetApplyGyroscopicForce(inEnabled);
            break;
        case 3:
            body.SetEnhancedInternalEdgeRemoval(inEnabled);
            break;
        default:
            return false;
        }
    }
    if (!inEnabled || inFlag != 0)
        ioSystem->GetBodyInterface().ActivateBody(inBodyID);
    return true;
}

inline JPH::BodyID CreateSoftBody(
    JPH::PhysicsSystem *ioSystem,
    const JPH::Vec3 *inPositions,
    const JPH::Vec3 *inVelocities,
    const float *inInverseMasses,
    std::uint32_t inVertexCount,
    const float *inAttributeEdgeCompliances,
    const float *inAttributeShearCompliances,
    const float *inAttributeBendCompliances,
    const std::uint8_t *inAttributeLRATypes,
    const float *inAttributeLRAMultipliers,
    std::uint32_t inVertexAttributeCount,
    const std::uint32_t *inFaceVertices,
    const std::uint32_t *inFaceMaterialIndices,
    std::uint32_t inFaceCount,
    const std::uint32_t *inEdgeVertices,
    const float *inEdgeCompliances,
    std::uint32_t inEdgeCount,
    const std::uint32_t *inDihedralVertices,
    const float *inDihedralCompliances,
    std::uint32_t inDihedralCount,
    const std::uint32_t *inLRAVertices,
    const float *inLRAMaxDistances,
    std::uint32_t inLRACount,
    const std::uint32_t *inVolumeVertices,
    const float *inVolumeCompliances,
    std::uint32_t inVolumeCount,
    const std::uint32_t *inRodVertices,
    const float *inRodCompliances,
    std::uint32_t inRodCount,
    const std::uint32_t *inRodPairs,
    const float *inRodPairCompliances,
    std::uint32_t inRodPairCount,
    std::uint32_t *outRodRemap,
    std::uint32_t *outRodPairRemap,
    const JPH::Vec3 *inSkinBindPositions,
    const JPH::Quat *inSkinBindRotations,
    std::uint32_t inSkinJointCount,
    const std::uint32_t *inSkinVertices,
    const std::uint32_t *inSkinJointIndices,
    const float *inSkinWeights,
    const float *inSkinMaxDistances,
    const float *inSkinBackStopDistances,
    const float *inSkinBackStopRadii,
    std::uint32_t inSkinConstraintCount,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::ObjectLayer inLayer,
    std::uint64_t inUserData,
    std::uint8_t inBendType,
    std::uint8_t inLRAType,
    float inLRAMaxDistanceMultiplier,
    float inEdgeCompliance,
    float inShearCompliance,
    float inBendCompliance,
    float inAngleTolerance,
    std::uint32_t inNumIterations,
    float inLinearDamping,
    float inMaxLinearVelocity,
    float inRestitution,
    float inFriction,
    float inPressure,
    float inGravityFactor,
    float inVertexRadius,
    bool inUpdatePosition,
    bool inMakeRotationIdentity,
    bool inAllowSleeping,
    bool inFacesDoubleSided,
    bool inEnableSkinConstraints,
    float inSkinnedMaxDistanceMultiplier,
    JPH::TempAllocatorImpl *inTempAllocator,
    JPH::PhysicsMaterial *const *inMaterials,
    std::uint32_t inMaterialCount)
{
    if ((inRodCount > 0 && outRodRemap == nullptr) ||
        (inRodPairCount > 0 && outRodPairRemap == nullptr))
        return JPH::BodyID();
    JPH::Ref<JPH::SoftBodySharedSettings> settings =
        new JPH::SoftBodySharedSettings();
    settings->mVertices.reserve(inVertexCount);
    for (std::uint32_t index = 0; index < inVertexCount; ++index)
    {
        JPH::SoftBodySharedSettings::Vertex vertex;
        inPositions[index].StoreFloat3(&vertex.mPosition);
        inVelocities[index].StoreFloat3(&vertex.mVelocity);
        vertex.mInvMass = inInverseMasses[index];
        settings->mVertices.push_back(vertex);
    }

    settings->mFaces.reserve(inFaceCount);
    for (std::uint32_t index = 0; index < inFaceCount; ++index)
    {
        const std::uint32_t *vertices = inFaceVertices + 3 * index;
        settings->AddFace(JPH::SoftBodySharedSettings::Face(
            vertices[0], vertices[1], vertices[2],
            inFaceMaterialIndices[index]));
    }

    if (inMaterialCount > 0)
    {
        settings->mMaterials.clear();
        settings->mMaterials.reserve(inMaterialCount);
        for (std::uint32_t index = 0; index < inMaterialCount; ++index)
            settings->mMaterials.push_back(inMaterials[index]);
    }

    JPH::Array<JPH::SoftBodySharedSettings::VertexAttributes> attributes;
    if (inVertexAttributeCount > 0)
    {
        attributes.reserve(inVertexAttributeCount);
        for (std::uint32_t index = 0; index < inVertexAttributeCount; ++index)
            attributes.emplace_back(
                inAttributeEdgeCompliances[index],
                inAttributeShearCompliances[index],
                inAttributeBendCompliances[index],
                static_cast<JPH::SoftBodySharedSettings::ELRAType>(
                    inAttributeLRATypes[index]),
                inAttributeLRAMultipliers[index]);
    }
    else
        attributes.emplace_back(
            inEdgeCompliance,
            inShearCompliance,
            inBendCompliance,
            static_cast<JPH::SoftBodySharedSettings::ELRAType>(inLRAType),
            inLRAMaxDistanceMultiplier);
    settings->CreateConstraints(
        attributes.data(),
        static_cast<std::uint32_t>(attributes.size()),
        static_cast<JPH::SoftBodySharedSettings::EBendType>(inBendType),
        inAngleTolerance);

    settings->mEdgeConstraints.reserve(
        settings->mEdgeConstraints.size() + inEdgeCount);
    for (std::uint32_t index = 0; index < inEdgeCount; ++index)
    {
        const std::uint32_t *vertices = inEdgeVertices + 2 * index;
        settings->mEdgeConstraints.emplace_back(
            vertices[0], vertices[1], inEdgeCompliances[index]);
    }
    if (inEdgeCount > 0)
        settings->CalculateEdgeLengths();

    settings->mDihedralBendConstraints.reserve(
        settings->mDihedralBendConstraints.size() + inDihedralCount);
    for (std::uint32_t index = 0; index < inDihedralCount; ++index)
    {
        const std::uint32_t *vertices = inDihedralVertices + 4 * index;
        settings->mDihedralBendConstraints.emplace_back(
            vertices[0], vertices[1], vertices[2], vertices[3],
            inDihedralCompliances[index]);
    }
    if (inDihedralCount > 0)
        settings->CalculateBendConstraintConstants();

    settings->mLRAConstraints.reserve(
        settings->mLRAConstraints.size() + inLRACount);
    for (std::uint32_t index = 0; index < inLRACount; ++index)
    {
        const std::uint32_t *vertices = inLRAVertices + 2 * index;
        settings->mLRAConstraints.emplace_back(
            vertices[0], vertices[1], inLRAMaxDistances[index]);
    }

    settings->mVolumeConstraints.reserve(inVolumeCount);
    for (std::uint32_t index = 0; index < inVolumeCount; ++index)
    {
        const std::uint32_t *vertices = inVolumeVertices + 4 * index;
        settings->mVolumeConstraints.emplace_back(
            vertices[0], vertices[1], vertices[2], vertices[3],
            inVolumeCompliances[index]);
    }
    if (inVolumeCount > 0)
        settings->CalculateVolumeConstraintVolumes();

    settings->mRodStretchShearConstraints.reserve(inRodCount);
    for (std::uint32_t index = 0; index < inRodCount; ++index)
    {
        const std::uint32_t *vertices = inRodVertices + 2 * index;
        settings->mRodStretchShearConstraints.emplace_back(
            vertices[0], vertices[1], inRodCompliances[index]);
    }
    settings->mRodBendTwistConstraints.reserve(inRodPairCount);
    for (std::uint32_t index = 0; index < inRodPairCount; ++index)
    {
        const std::uint32_t *rods = inRodPairs + 2 * index;
        settings->mRodBendTwistConstraints.emplace_back(
            rods[0], rods[1], inRodPairCompliances[index]);
    }
    if (inRodCount > 0)
        settings->CalculateRodProperties();

    settings->mInvBindMatrices.reserve(inSkinJointCount);
    JPH::Array<JPH::Mat44> bind_pose;
    bind_pose.reserve(inSkinJointCount);
    for (std::uint32_t index = 0; index < inSkinJointCount; ++index)
    {
        JPH::Mat44 transform = JPH::Mat44::sRotationTranslation(
            inSkinBindRotations[index], inSkinBindPositions[index]);
        bind_pose.push_back(transform);
        settings->mInvBindMatrices.emplace_back(index, transform.Inversed());
    }
    settings->mSkinnedConstraints.reserve(inSkinConstraintCount);
    for (std::uint32_t index = 0; index < inSkinConstraintCount; ++index)
    {
        JPH::SoftBodySharedSettings::Skinned skinned(
            inSkinVertices[index],
            inSkinMaxDistances[index],
            inSkinBackStopDistances[index],
            inSkinBackStopRadii[index]);
        for (std::uint32_t weight = 0; weight < 4; ++weight)
        {
            const std::uint32_t offset = 4 * index + weight;
            skinned.mWeights[weight] =
                JPH::SoftBodySharedSettings::SkinWeight(
                    inSkinJointIndices[offset], inSkinWeights[offset]);
        }
        settings->mSkinnedConstraints.push_back(skinned);
    }
    if (inSkinConstraintCount > 0)
        settings->CalculateSkinnedConstraintNormals();
    JPH::SoftBodySharedSettings::OptimizationResults optimization_results;
    settings->Optimize(optimization_results);
    if (optimization_results.mRodStretchShearConstraintRemap.size() !=
            inRodCount ||
        optimization_results.mRodBendTwistConstraintRemap.size() !=
            inRodPairCount)
        return JPH::BodyID();
    for (std::uint32_t index = 0; index < inRodCount; ++index)
        outRodRemap[index] =
            optimization_results.mRodStretchShearConstraintRemap[index];
    for (std::uint32_t index = 0; index < inRodPairCount; ++index)
        outRodPairRemap[index] =
            optimization_results.mRodBendTwistConstraintRemap[index];

    JPH::SoftBodyCreationSettings creation(
        settings, inPosition, inRotation, inLayer);
    creation.mUserData = inUserData;
    creation.mNumIterations = inNumIterations;
    creation.mLinearDamping = inLinearDamping;
    creation.mMaxLinearVelocity = inMaxLinearVelocity;
    creation.mRestitution = inRestitution;
    creation.mFriction = inFriction;
    creation.mPressure = inPressure;
    creation.mGravityFactor = inGravityFactor;
    creation.mVertexRadius = inVertexRadius;
    creation.mUpdatePosition = inUpdatePosition;
    creation.mMakeRotationIdentity = inMakeRotationIdentity;
    creation.mAllowSleeping = inAllowSleeping;
    creation.mFacesDoubleSided = inFacesDoubleSided;
    JPH::BodyID id = ioSystem->GetBodyInterface().CreateAndAddSoftBody(
        creation, JPH::EActivation::Activate);
    if (id.IsInvalid() || inSkinConstraintCount == 0)
        return id;

    JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), id);
    if (!lock.Succeeded())
        return JPH::BodyID();
    JPH::Body &body = lock.GetBody();
    JPH::SoftBodyMotionProperties *motion =
        static_cast<JPH::SoftBodyMotionProperties *>(body.GetMotionProperties());
    motion->SetEnableSkinConstraints(inEnableSkinConstraints);
    motion->SetSkinnedMaxDistanceMultiplier(inSkinnedMaxDistanceMultiplier);
    motion->SkinVertices(
        body.GetCenterOfMassTransform(), bind_pose.data(), inSkinJointCount,
        true, *inTempAllocator);
    return id;
}

inline const JPH::SoftBodyMotionProperties *GetSoftBodyMotionProperties(
    const JPH::Body &inBody)
{
    return inBody.IsSoftBody()? static_cast<const JPH::SoftBodyMotionProperties *>(
        inBody.GetMotionProperties()) : nullptr;
}

inline JPH::SoftBodyMotionProperties *GetSoftBodyMotionProperties(
    JPH::Body &inBody)
{
    return inBody.IsSoftBody()? static_cast<JPH::SoftBodyMotionProperties *>(
        inBody.GetMotionProperties()) : nullptr;
}

inline std::uint32_t GetSoftBodyVertexCount(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return 0;
    const JPH::SoftBodyMotionProperties *motion =
        GetSoftBodyMotionProperties(lock.GetBody());
    return motion == nullptr? 0 :
        static_cast<std::uint32_t>(motion->GetVertices().size());
}

inline bool GetSoftBodyVertexState(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inVertex,
    JPH::Vec3 *outPosition,
    JPH::Vec3 *outVelocity,
    float *outInverseMass)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::Body &body = lock.GetBody();
    const JPH::SoftBodyMotionProperties *motion =
        GetSoftBodyMotionProperties(body);
    if (motion == nullptr || inVertex >= motion->GetVertices().size())
        return false;
    const JPH::SoftBodyVertex &vertex = motion->GetVertex(inVertex);
    const JPH::RMat44 transform = body.GetCenterOfMassTransform();
    *outPosition = JPH::Vec3(transform * vertex.mPosition);
    *outVelocity = transform.Multiply3x3(vertex.mVelocity);
    *outInverseMass = vertex.mInvMass;
    return true;
}

inline bool SetSoftBodyVertexVelocity(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inVertex,
    JPH::Vec3Arg inWorldVelocity)
{
    {
        JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return false;
        JPH::Body &body = lock.GetBody();
        JPH::SoftBodyMotionProperties *motion =
            GetSoftBodyMotionProperties(body);
        if (motion == nullptr || inVertex >= motion->GetVertices().size())
            return false;
        motion->GetVertex(inVertex).mVelocity =
            body.GetCenterOfMassTransform().Multiply3x3Transposed(
                inWorldVelocity);
    }
    ioSystem->GetBodyInterface().ActivateBody(inBodyID);
    return true;
}

inline bool SetSoftBodyVertexInverseMass(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inVertex,
    float inInverseMass)
{
    {
        JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return false;
        JPH::SoftBodyMotionProperties *motion =
            GetSoftBodyMotionProperties(lock.GetBody());
        if (motion == nullptr || inVertex >= motion->GetVertices().size())
            return false;
        motion->GetVertex(inVertex).mInvMass = inInverseMass;
        motion->CalculateMassAndInertia();
    }
    ioSystem->GetBodyInterface().ActivateBody(inBodyID);
    return true;
}

inline bool GetSoftBodyRuntimeSettings(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t *outNumIterations,
    float *outPressure,
    float *outVertexRadius,
    float *outVolume,
    bool *outUpdatePosition,
    bool *outFacesDoubleSided,
    bool *outEnableSkinConstraints,
    float *outSkinnedMaxDistanceMultiplier)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::SoftBodyMotionProperties *motion =
        GetSoftBodyMotionProperties(lock.GetBody());
    if (motion == nullptr)
        return false;
    *outNumIterations = motion->GetNumIterations();
    *outPressure = motion->GetPressure();
    *outVertexRadius = motion->GetVertexRadius();
    *outVolume = motion->GetVolume();
    *outUpdatePosition = motion->GetUpdatePosition();
    *outFacesDoubleSided = motion->GetFacesDoubleSided();
    *outEnableSkinConstraints = motion->GetEnableSkinConstraints();
    *outSkinnedMaxDistanceMultiplier =
        motion->GetSkinnedMaxDistanceMultiplier();
    return true;
}

inline bool GetSoftBodyConstraintCounts(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t *outEdges,
    std::uint32_t *outDihedralBends,
    std::uint32_t *outVolumes,
    std::uint32_t *outLongRangeAttachments,
    std::uint32_t *outRods,
    std::uint32_t *outRodBendTwists,
    std::uint32_t *outSkinned)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::SoftBodyMotionProperties *motion =
        GetSoftBodyMotionProperties(lock.GetBody());
    if (motion == nullptr || motion->GetSettings() == nullptr)
        return false;
    const JPH::SoftBodySharedSettings *settings = motion->GetSettings();
    *outEdges = static_cast<std::uint32_t>(settings->mEdgeConstraints.size());
    *outDihedralBends = static_cast<std::uint32_t>(
        settings->mDihedralBendConstraints.size());
    *outVolumes = static_cast<std::uint32_t>(
        settings->mVolumeConstraints.size());
    *outLongRangeAttachments = static_cast<std::uint32_t>(
        settings->mLRAConstraints.size());
    *outRods = static_cast<std::uint32_t>(
        settings->mRodStretchShearConstraints.size());
    *outRodBendTwists = static_cast<std::uint32_t>(
        settings->mRodBendTwistConstraints.size());
    *outSkinned = static_cast<std::uint32_t>(
        settings->mSkinnedConstraints.size());
    return true;
}

inline bool GetSoftBodyRodState(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inRod,
    JPH::Quat *outRotation,
    JPH::Vec3 *outAngularVelocity)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::SoftBodyMotionProperties *motion =
        GetSoftBodyMotionProperties(lock.GetBody());
    if (motion == nullptr || motion->GetSettings() == nullptr ||
        inRod >= motion->GetSettings()->mRodStretchShearConstraints.size())
        return false;
    *outRotation = motion->GetRodRotation(inRod);
    *outAngularVelocity = motion->GetRodAngularVelocity(inRod);
    return true;
}

inline bool GetSoftBodyLocalBounds(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    JPH::Vec3 *outMinimum,
    JPH::Vec3 *outMaximum)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::SoftBodyMotionProperties *motion =
        GetSoftBodyMotionProperties(lock.GetBody());
    if (motion == nullptr)
        return false;
    const JPH::AABox &bounds = motion->GetLocalBounds();
    *outMinimum = bounds.mMin;
    *outMaximum = bounds.mMax;
    return true;
}

inline bool CustomUpdateSoftBody(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    float inDeltaTime)
{
    if (ioSystem == nullptr || !std::isfinite(inDeltaTime) ||
        inDeltaTime <= 0.0f)
        return false;
    JPH::BodyInterface &body_interface = ioSystem->GetBodyInterface();
    if (!body_interface.IsAdded(inBodyID))
        return false;

    // CustomUpdate is intended for a body outside the broad phase. Keep its
    // ID allocated so Nim ownership and contact IDs remain stable.
    body_interface.RemoveBody(inBodyID);
    bool updated = false;
    try
    {
        // CustomUpdate locks every broad-phase candidate it finds. The target
        // body has already been removed above, so access it through Jolt's
        // no-lock interface and avoid holding a body mutex while those locks
        // are acquired.
        JPH::BodyLockWrite lock(
            ioSystem->GetBodyLockInterfaceNoLock(), inBodyID);
        if (lock.Succeeded())
        {
            JPH::Body &body = lock.GetBody();
            JPH::SoftBodyMotionProperties *motion =
                GetSoftBodyMotionProperties(body);
            if (motion != nullptr)
            {
                motion->CustomUpdate(inDeltaTime, body, *ioSystem);
                updated = true;
            }
        }
    }
    catch (...)
    {
        updated = false;
    }
    body_interface.AddBody(
        inBodyID,
        updated? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    return updated;
}

inline bool SetSoftBodyRuntimeSettings(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inNumIterations,
    float inPressure,
    float inVertexRadius,
    bool inUpdatePosition,
    bool inFacesDoubleSided,
    bool inEnableSkinConstraints,
    float inSkinnedMaxDistanceMultiplier)
{
    {
        JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return false;
        JPH::SoftBodyMotionProperties *motion =
            GetSoftBodyMotionProperties(lock.GetBody());
        if (motion == nullptr)
            return false;
        motion->SetNumIterations(inNumIterations);
        motion->SetPressure(inPressure);
        motion->SetVertexRadius(inVertexRadius);
        motion->SetUpdatePosition(inUpdatePosition);
        motion->SetFacesDoubleSided(inFacesDoubleSided);
        motion->SetEnableSkinConstraints(inEnableSkinConstraints);
        motion->SetSkinnedMaxDistanceMultiplier(
            inSkinnedMaxDistanceMultiplier);
    }
    ioSystem->GetBodyInterface().ActivateBody(inBodyID);
    return true;
}

inline bool SkinSoftBodyVertices(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    const JPH::Vec3 *inJointPositions,
    const JPH::Quat *inJointRotations,
    std::uint32_t inJointCount,
    bool inHardSkinAll,
    JPH::TempAllocatorImpl *inTempAllocator)
{
    {
        JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return false;
        JPH::Body &body = lock.GetBody();
        JPH::SoftBodyMotionProperties *motion =
            GetSoftBodyMotionProperties(body);
        if (motion == nullptr)
            return false;
        JPH::Array<JPH::Mat44> pose;
        pose.reserve(inJointCount);
        for (std::uint32_t index = 0; index < inJointCount; ++index)
            pose.push_back(JPH::Mat44::sRotationTranslation(
                inJointRotations[index], inJointPositions[index]));
        motion->SkinVertices(
            body.GetCenterOfMassTransform(), pose.data(), inJointCount,
            inHardSkinAll, *inTempAllocator);
    }
    ioSystem->GetBodyInterface().ActivateBody(inBodyID);
    return true;
}

inline bool SetBodySolverStepOverrides(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inVelocitySteps,
    std::uint32_t inPositionSteps)
{
    JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() == JPH::EMotionType::Static)
        return false;
    JPH::MotionProperties *motion = body.GetMotionProperties();
    motion->SetNumVelocityStepsOverride(inVelocitySteps);
    motion->SetNumPositionStepsOverride(inPositionSteps);
    return true;
}

struct RagdollPartData
{
    const JPH::Shape *mShape;
    JPH::Vec3 mPosition;
    JPH::Quat mRotation;
    std::int32_t mParent;
    std::uint8_t mJointType;
    JPH::ObjectLayer mLayer;
    JPH::EMotionType mMotionType;
    JPH::Vec3 mJointPosition;
    JPH::Vec3 mTwistAxis;
    JPH::Vec3 mPlaneAxis;
    std::uint8_t mSixDOFSwingType;
    float mNormalHalfConeAngle;
    float mPlaneHalfConeAngle;
    float mTwistMinAngle;
    float mTwistMaxAngle;
    float mMaxFrictionTorque;
    float mMotorFrequency;
    float mMotorDamping;
    float mMaxMotorTorque;
    JPH::Vec3 mLinearLimitMin;
    JPH::Vec3 mLinearLimitMax;
    JPH::Vec3 mAngularLimitMin;
    JPH::Vec3 mAngularLimitMax;
    JPH::Vec3 mLinearFriction;
    JPH::Vec3 mAngularFriction;
    std::uint8_t mAllowedDOFs;
    std::uint8_t mMotionQuality;
    float mMass;
    float mInertiaMultiplier;
    JPH::Vec3 mLinearVelocity;
    JPH::Vec3 mAngularVelocity;
    std::uint64_t mUserData;
    bool mAllowSleeping;
    bool mCollideKinematicVsNonDynamic;
    bool mUseManifoldReduction;
    bool mApplyGyroscopicForce;
    bool mEnhancedInternalEdgeRemoval;
    float mFriction;
    float mRestitution;
    float mLinearDamping;
    float mAngularDamping;
    float mMaxLinearVelocity;
    float mMaxAngularVelocity;
    float mGravityFactor;
    std::uint32_t mNumVelocityStepsOverride;
    std::uint32_t mNumPositionStepsOverride;
};

inline RagdollPartData MakeRagdollPartData(
    const JPH::Shape *inShape, JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation, std::int32_t inParent,
    std::uint8_t inJointType,
    JPH::ObjectLayer inLayer, std::uint8_t inMotionType,
    JPH::Vec3Arg inJointPosition, JPH::Vec3Arg inTwistAxis,
    JPH::Vec3Arg inPlaneAxis, std::uint8_t inSixDOFSwingType,
    float inNormalHalfConeAngle,
    float inPlaneHalfConeAngle, float inTwistMinAngle,
    float inTwistMaxAngle, float inMaxFrictionTorque,
    float inMotorFrequency, float inMotorDamping, float inMaxMotorTorque,
    JPH::Vec3Arg inLinearLimitMin, JPH::Vec3Arg inLinearLimitMax,
    JPH::Vec3Arg inAngularLimitMin, JPH::Vec3Arg inAngularLimitMax,
    JPH::Vec3Arg inLinearFriction, JPH::Vec3Arg inAngularFriction,
    std::uint8_t inAllowedDOFs, std::uint8_t inMotionQuality,
    float inMass, float inInertiaMultiplier,
    JPH::Vec3Arg inLinearVelocity, JPH::Vec3Arg inAngularVelocity,
    std::uint64_t inUserData, bool inAllowSleeping,
    bool inCollideKinematicVsNonDynamic, bool inUseManifoldReduction,
    bool inApplyGyroscopicForce, bool inEnhancedInternalEdgeRemoval,
    float inFriction, float inRestitution, float inLinearDamping,
    float inAngularDamping, float inMaxLinearVelocity,
    float inMaxAngularVelocity, float inGravityFactor,
    std::uint32_t inNumVelocityStepsOverride,
    std::uint32_t inNumPositionStepsOverride)
{
    return {
        inShape, inPosition, inRotation, inParent, inJointType, inLayer,
        static_cast<JPH::EMotionType>(inMotionType), inJointPosition,
        inTwistAxis, inPlaneAxis, inSixDOFSwingType, inNormalHalfConeAngle,
        inPlaneHalfConeAngle, inTwistMinAngle, inTwistMaxAngle,
        inMaxFrictionTorque, inMotorFrequency, inMotorDamping,
        inMaxMotorTorque, inLinearLimitMin, inLinearLimitMax,
        inAngularLimitMin, inAngularLimitMax, inLinearFriction,
        inAngularFriction, inAllowedDOFs, inMotionQuality, inMass,
        inInertiaMultiplier, inLinearVelocity, inAngularVelocity,
        inUserData, inAllowSleeping, inCollideKinematicVsNonDynamic,
        inUseManifoldReduction, inApplyGyroscopicForce,
        inEnhancedInternalEdgeRemoval, inFriction, inRestitution,
        inLinearDamping, inAngularDamping, inMaxLinearVelocity,
        inMaxAngularVelocity, inGravityFactor, inNumVelocityStepsOverride,
        inNumPositionStepsOverride
    };
}

struct RagdollDistanceConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
    JPH::Vec3 mPoint1;
    JPH::Vec3 mPoint2;
    float mMinDistance;
    float mMaxDistance;
};

inline RagdollDistanceConstraintData MakeRagdollDistanceConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2,
    JPH::Vec3Arg inPoint1, JPH::Vec3Arg inPoint2,
    float inMinDistance, float inMaxDistance)
{
    return {inPart1, inPart2, inPoint1, inPoint2,
            inMinDistance, inMaxDistance};
}

struct RagdollPointConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
    JPH::Vec3 mPoint;
};

inline RagdollPointConstraintData MakeRagdollPointConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2, JPH::Vec3Arg inPoint)
{
    return {inPart1, inPart2, inPoint};
}

struct RagdollFixedConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
};

inline RagdollFixedConstraintData MakeRagdollFixedConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2)
{
    return {inPart1, inPart2};
}

struct RagdollHingeConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
    JPH::Vec3 mPoint;
    JPH::Vec3 mHingeAxis;
    JPH::Vec3 mNormalAxis;
    float mMinAngle;
    float mMaxAngle;
    float mMaxFrictionTorque;
};

inline RagdollHingeConstraintData MakeRagdollHingeConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2,
    JPH::Vec3Arg inPoint, JPH::Vec3Arg inHingeAxis,
    JPH::Vec3Arg inNormalAxis, float inMinAngle, float inMaxAngle,
    float inMaxFrictionTorque)
{
    return {inPart1, inPart2, inPoint, inHingeAxis, inNormalAxis,
            inMinAngle, inMaxAngle, inMaxFrictionTorque};
}

struct RagdollSliderConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
    JPH::Vec3 mPoint;
    JPH::Vec3 mSliderAxis;
    JPH::Vec3 mNormalAxis;
    float mMinPosition;
    float mMaxPosition;
    float mMaxFrictionForce;
};

inline RagdollSliderConstraintData MakeRagdollSliderConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2,
    JPH::Vec3Arg inPoint, JPH::Vec3Arg inSliderAxis,
    JPH::Vec3Arg inNormalAxis, float inMinPosition, float inMaxPosition,
    float inMaxFrictionForce)
{
    return {inPart1, inPart2, inPoint, inSliderAxis, inNormalAxis,
            inMinPosition, inMaxPosition, inMaxFrictionForce};
}

struct RagdollSwingTwistConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
    JPH::Vec3 mPoint;
    JPH::Vec3 mTwistAxis;
    JPH::Vec3 mPlaneAxis;
    float mNormalHalfConeAngle;
    float mPlaneHalfConeAngle;
    float mTwistMinAngle;
    float mTwistMaxAngle;
    float mMaxFrictionTorque;
};

inline RagdollSwingTwistConstraintData MakeRagdollSwingTwistConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2,
    JPH::Vec3Arg inPoint, JPH::Vec3Arg inTwistAxis,
    JPH::Vec3Arg inPlaneAxis, float inNormalHalfConeAngle,
    float inPlaneHalfConeAngle, float inTwistMinAngle,
    float inTwistMaxAngle, float inMaxFrictionTorque)
{
    return {inPart1, inPart2, inPoint, inTwistAxis, inPlaneAxis,
            inNormalHalfConeAngle, inPlaneHalfConeAngle,
            inTwistMinAngle, inTwistMaxAngle, inMaxFrictionTorque};
}

struct RagdollSixDOFConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
    JPH::Vec3 mPoint;
    JPH::Vec3 mAxisX;
    JPH::Vec3 mAxisY;
    std::uint8_t mSwingType;
    JPH::Vec3 mLinearLimitMin;
    JPH::Vec3 mLinearLimitMax;
    JPH::Vec3 mAngularLimitMin;
    JPH::Vec3 mAngularLimitMax;
    JPH::Vec3 mLinearFriction;
    JPH::Vec3 mAngularFriction;
};

inline RagdollSixDOFConstraintData MakeRagdollSixDOFConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2,
    JPH::Vec3Arg inPoint, JPH::Vec3Arg inAxisX, JPH::Vec3Arg inAxisY,
    std::uint8_t inSwingType, JPH::Vec3Arg inLinearLimitMin,
    JPH::Vec3Arg inLinearLimitMax, JPH::Vec3Arg inAngularLimitMin,
    JPH::Vec3Arg inAngularLimitMax, JPH::Vec3Arg inLinearFriction,
    JPH::Vec3Arg inAngularFriction)
{
    return {inPart1, inPart2, inPoint, inAxisX, inAxisY, inSwingType,
            inLinearLimitMin, inLinearLimitMax,
            inAngularLimitMin, inAngularLimitMax,
            inLinearFriction, inAngularFriction};
}

struct RagdollConeConstraintData
{
    std::uint32_t mPart1;
    std::uint32_t mPart2;
    JPH::Vec3 mPoint;
    JPH::Vec3 mTwistAxis1;
    JPH::Vec3 mTwistAxis2;
    float mHalfConeAngle;
};

inline RagdollConeConstraintData MakeRagdollConeConstraintData(
    std::uint32_t inPart1, std::uint32_t inPart2,
    JPH::Vec3Arg inPoint, JPH::Vec3Arg inTwistAxis1,
    JPH::Vec3Arg inTwistAxis2, float inHalfConeAngle)
{
    return {inPart1, inPart2, inPoint, inTwistAxis1,
            inTwistAxis2, inHalfConeAngle};
}

struct RagdollHandle
{
    JPH::Ref<JPH::Ragdoll> mRagdoll;
    JPH::PhysicsSystem *mSystem = nullptr;
    bool mAdded = false;
};

inline RagdollHandle *CreateRagdoll(
    JPH::PhysicsSystem *inSystem, const RagdollPartData *inParts,
    std::uint32_t inPartCount,
    const RagdollDistanceConstraintData *inDistanceConstraints,
    std::uint32_t inDistanceConstraintCount,
    const RagdollPointConstraintData *inPointConstraints,
    std::uint32_t inPointConstraintCount,
    const RagdollFixedConstraintData *inFixedConstraints,
    std::uint32_t inFixedConstraintCount,
    const RagdollHingeConstraintData *inHingeConstraints,
    std::uint32_t inHingeConstraintCount,
    const RagdollSliderConstraintData *inSliderConstraints,
    std::uint32_t inSliderConstraintCount,
    const RagdollSwingTwistConstraintData *inSwingTwistConstraints,
    std::uint32_t inSwingTwistConstraintCount,
    const RagdollSixDOFConstraintData *inSixDOFConstraints,
    std::uint32_t inSixDOFConstraintCount,
    const RagdollConeConstraintData *inConeConstraints,
    std::uint32_t inConeConstraintCount, std::uint32_t inGroupID,
    bool inDisableParentChildCollisions, bool inStabilize,
    bool inCalculatePriorities, bool inActivate)
{
    if (inPartCount == 0 || inParts == nullptr ||
        (inDistanceConstraintCount > 0 && inDistanceConstraints == nullptr) ||
        (inPointConstraintCount > 0 && inPointConstraints == nullptr) ||
        (inFixedConstraintCount > 0 && inFixedConstraints == nullptr) ||
        (inHingeConstraintCount > 0 && inHingeConstraints == nullptr) ||
        (inSliderConstraintCount > 0 && inSliderConstraints == nullptr) ||
        (inSwingTwistConstraintCount > 0 &&
         inSwingTwistConstraints == nullptr) ||
        (inSixDOFConstraintCount > 0 && inSixDOFConstraints == nullptr) ||
        (inConeConstraintCount > 0 && inConeConstraints == nullptr))
        return nullptr;
    for (std::uint32_t index = 0; index < inPartCount; ++index)
    {
        const RagdollPartData &data = inParts[index];
        if (data.mShape == nullptr || data.mJointType > 6 ||
            data.mSixDOFSwingType > 1 ||
            (index == 0 && data.mParent != -1) ||
            (index > 0 &&
             (data.mParent < 0 ||
              static_cast<std::uint32_t>(data.mParent) >= index)))
            return nullptr;
    }
    for (std::uint32_t index = 0;
         index < inDistanceConstraintCount; ++index)
    {
        const RagdollDistanceConstraintData &data =
            inDistanceConstraints[index];
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2 || data.mMinDistance < 0.0f ||
            data.mMinDistance > data.mMaxDistance)
            return nullptr;
    }
    for (std::uint32_t index = 0; index < inPointConstraintCount; ++index)
    {
        const RagdollPointConstraintData &data = inPointConstraints[index];
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2)
            return nullptr;
    }
    for (std::uint32_t index = 0; index < inFixedConstraintCount; ++index)
    {
        const RagdollFixedConstraintData &data = inFixedConstraints[index];
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2)
            return nullptr;
    }
    for (std::uint32_t index = 0; index < inHingeConstraintCount; ++index)
    {
        const RagdollHingeConstraintData &data = inHingeConstraints[index];
        const float hingeLengthSq = data.mHingeAxis.LengthSq();
        const float normalLengthSq = data.mNormalAxis.LengthSq();
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2 ||
            !IsFiniteVec3(data.mPoint) ||
            !IsFiniteVec3(data.mHingeAxis) ||
            !IsFiniteVec3(data.mNormalAxis) || hingeLengthSq <= 1.0e-12f ||
            normalLengthSq <= 1.0e-12f ||
            std::abs(data.mHingeAxis.Dot(data.mNormalAxis)) >
                1.0e-4f * std::sqrt(hingeLengthSq * normalLengthSq) ||
            !std::isfinite(data.mMinAngle) ||
            !std::isfinite(data.mMaxAngle) ||
            !std::isfinite(data.mMaxFrictionTorque) ||
            data.mMinAngle < -JPH::JPH_PI || data.mMinAngle > 0.0f ||
            data.mMaxAngle < 0.0f || data.mMaxAngle > JPH::JPH_PI ||
            data.mMaxFrictionTorque < 0.0f)
            return nullptr;
    }
    for (std::uint32_t index = 0; index < inSliderConstraintCount; ++index)
    {
        const RagdollSliderConstraintData &data = inSliderConstraints[index];
        const float sliderLengthSq = data.mSliderAxis.LengthSq();
        const float normalLengthSq = data.mNormalAxis.LengthSq();
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2 ||
            !IsFiniteVec3(data.mPoint) ||
            !IsFiniteVec3(data.mSliderAxis) ||
            !IsFiniteVec3(data.mNormalAxis) || sliderLengthSq <= 1.0e-12f ||
            normalLengthSq <= 1.0e-12f ||
            std::abs(data.mSliderAxis.Dot(data.mNormalAxis)) >
                1.0e-4f * std::sqrt(sliderLengthSq * normalLengthSq) ||
            !std::isfinite(data.mMinPosition) ||
            !std::isfinite(data.mMaxPosition) ||
            !std::isfinite(data.mMaxFrictionForce) ||
            data.mMinPosition > 0.0f ||
            data.mMaxPosition < 0.0f || data.mMaxFrictionForce < 0.0f)
            return nullptr;
    }
    for (std::uint32_t index = 0;
         index < inSwingTwistConstraintCount; ++index)
    {
        const RagdollSwingTwistConstraintData &data =
            inSwingTwistConstraints[index];
        const float twistLengthSq = data.mTwistAxis.LengthSq();
        const float planeLengthSq = data.mPlaneAxis.LengthSq();
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2 || !IsFiniteVec3(data.mPoint) ||
            !IsFiniteVec3(data.mTwistAxis) ||
            !IsFiniteVec3(data.mPlaneAxis) ||
            twistLengthSq <= 1.0e-12f || planeLengthSq <= 1.0e-12f ||
            std::abs(data.mTwistAxis.Dot(data.mPlaneAxis)) >
                1.0e-4f * std::sqrt(twistLengthSq * planeLengthSq) ||
            !std::isfinite(data.mNormalHalfConeAngle) ||
            !std::isfinite(data.mPlaneHalfConeAngle) ||
            !std::isfinite(data.mTwistMinAngle) ||
            !std::isfinite(data.mTwistMaxAngle) ||
            !std::isfinite(data.mMaxFrictionTorque) ||
            data.mNormalHalfConeAngle < 0.0f ||
            data.mNormalHalfConeAngle > JPH::JPH_PI ||
            data.mPlaneHalfConeAngle < 0.0f ||
            data.mPlaneHalfConeAngle > JPH::JPH_PI ||
            data.mTwistMinAngle < -JPH::JPH_PI ||
            data.mTwistMaxAngle > JPH::JPH_PI ||
            data.mTwistMinAngle > data.mTwistMaxAngle ||
            data.mMaxFrictionTorque < 0.0f)
            return nullptr;
    }
    for (std::uint32_t index = 0; index < inSixDOFConstraintCount; ++index)
    {
        const RagdollSixDOFConstraintData &data = inSixDOFConstraints[index];
        const float axisXLengthSq = data.mAxisX.LengthSq();
        const float axisYLengthSq = data.mAxisY.LengthSq();
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2 || data.mSwingType > 1 ||
            !IsFiniteVec3(data.mPoint) || !IsFiniteVec3(data.mAxisX) ||
            !IsFiniteVec3(data.mAxisY) || axisXLengthSq <= 1.0e-12f ||
            axisYLengthSq <= 1.0e-12f ||
            std::abs(data.mAxisX.Dot(data.mAxisY)) >
                1.0e-4f * std::sqrt(axisXLengthSq * axisYLengthSq) ||
            !IsFiniteVec3(data.mLinearLimitMin) ||
            !IsFiniteVec3(data.mLinearLimitMax) ||
            !IsFiniteVec3(data.mAngularLimitMin) ||
            !IsFiniteVec3(data.mAngularLimitMax) ||
            !IsFiniteVec3(data.mLinearFriction) ||
            !IsFiniteVec3(data.mAngularFriction))
            return nullptr;
        const float limitMin[6] = {
            data.mLinearLimitMin[0], data.mLinearLimitMin[1],
            data.mLinearLimitMin[2], data.mAngularLimitMin[0],
            data.mAngularLimitMin[1], data.mAngularLimitMin[2]};
        const float limitMax[6] = {
            data.mLinearLimitMax[0], data.mLinearLimitMax[1],
            data.mLinearLimitMax[2], data.mAngularLimitMax[0],
            data.mAngularLimitMax[1], data.mAngularLimitMax[2]};
        const float friction[6] = {
            data.mLinearFriction[0], data.mLinearFriction[1],
            data.mLinearFriction[2], data.mAngularFriction[0],
            data.mAngularFriction[1], data.mAngularFriction[2]};
        const float maxFloat = std::numeric_limits<float>::max();
        for (int axis = 0; axis < 6; ++axis)
        {
            const bool free = limitMin[axis] == -maxFloat &&
                              limitMax[axis] == maxFloat;
            const bool fixed = limitMin[axis] == maxFloat &&
                               limitMax[axis] == -maxFloat;
            if (friction[axis] < 0.0f ||
                (!free && !fixed && limitMin[axis] > limitMax[axis]) ||
                (!free && !fixed && axis >= 3 &&
                 (limitMin[axis] < -JPH::JPH_PI ||
                  limitMax[axis] > JPH::JPH_PI)) ||
                (!free && !fixed && data.mSwingType == 0 && axis >= 4 &&
                 (limitMax[axis] < 0.0f ||
                  std::abs(limitMin[axis] + limitMax[axis]) > 1.0e-5f)))
                return nullptr;
        }
    }
    for (std::uint32_t index = 0; index < inConeConstraintCount; ++index)
    {
        const RagdollConeConstraintData &data = inConeConstraints[index];
        if (data.mPart1 >= inPartCount || data.mPart2 >= inPartCount ||
            data.mPart1 == data.mPart2 || data.mHalfConeAngle < 0.0f ||
            data.mHalfConeAngle > JPH::JPH_PI)
            return nullptr;
    }

    JPH::Ref<JPH::RagdollSettings> settings = new JPH::RagdollSettings();
    settings->mSkeleton = new JPH::Skeleton();
    settings->mParts.resize(inPartCount);
    for (std::uint32_t index = 0; index < inPartCount; ++index)
    {
        const RagdollPartData &data = inParts[index];
        settings->mSkeleton->AddJoint(
            std::string("part_") + std::to_string(index), data.mParent);
        JPH::RagdollSettings::Part &part = settings->mParts[index];
        part.SetShape(data.mShape);
        part.mPosition = data.mPosition;
        part.mRotation = data.mRotation;
        part.mMotionType = data.mMotionType;
        part.mObjectLayer = data.mLayer;
        ConfigureBodyCreationSettings(
            part, data.mAllowedDOFs, data.mMotionQuality, data.mMass,
            data.mInertiaMultiplier, data.mLinearVelocity,
            data.mAngularVelocity, data.mUserData, data.mAllowSleeping,
            data.mCollideKinematicVsNonDynamic, data.mUseManifoldReduction,
            data.mApplyGyroscopicForce, data.mEnhancedInternalEdgeRemoval,
            data.mFriction, data.mRestitution, data.mLinearDamping,
            data.mAngularDamping, data.mMaxLinearVelocity,
            data.mMaxAngularVelocity, data.mGravityFactor,
            data.mNumVelocityStepsOverride,
            data.mNumPositionStepsOverride,
            false, 0.0f, JPH::Vec3::sReplicate(1.0f),
            JPH::Quat::sIdentity());

        if (data.mParent >= 0)
        {
            if (data.mJointType == 1)
            {
                JPH::HingeConstraintSettings *constraint =
                    new JPH::HingeConstraintSettings();
                constraint->mPoint1 = constraint->mPoint2 =
                    data.mJointPosition;
                constraint->mHingeAxis1 = constraint->mHingeAxis2 =
                    data.mTwistAxis.Normalized();
                constraint->mNormalAxis1 = constraint->mNormalAxis2 =
                    data.mPlaneAxis.Normalized();
                constraint->mLimitsMin = data.mTwistMinAngle;
                constraint->mLimitsMax = data.mTwistMaxAngle;
                constraint->mMaxFrictionTorque = data.mMaxFrictionTorque;
                constraint->mMotorSettings = JPH::MotorSettings(
                    data.mMotorFrequency, data.mMotorDamping);
                constraint->mMotorSettings.SetTorqueLimit(
                    data.mMaxMotorTorque);
                part.mToParent = constraint;
            }
            else if (data.mJointType == 2)
            {
                JPH::PointConstraintSettings *constraint =
                    new JPH::PointConstraintSettings();
                constraint->mPoint1 = constraint->mPoint2 =
                    data.mJointPosition;
                part.mToParent = constraint;
            }
            else if (data.mJointType == 3)
            {
                JPH::FixedConstraintSettings *constraint =
                    new JPH::FixedConstraintSettings();
                constraint->mPoint1 = constraint->mPoint2 =
                    data.mJointPosition;
                part.mToParent = constraint;
            }
            else if (data.mJointType == 4)
            {
                JPH::ConeConstraintSettings *constraint =
                    new JPH::ConeConstraintSettings();
                constraint->mPoint1 = constraint->mPoint2 =
                    data.mJointPosition;
                constraint->mTwistAxis1 = constraint->mTwistAxis2 =
                    data.mTwistAxis.Normalized();
                constraint->mHalfConeAngle = data.mNormalHalfConeAngle;
                part.mToParent = constraint;
            }
            else if (data.mJointType == 5)
            {
                JPH::SliderConstraintSettings *constraint =
                    new JPH::SliderConstraintSettings();
                constraint->mPoint1 = constraint->mPoint2 =
                    data.mJointPosition;
                constraint->mSliderAxis1 = constraint->mSliderAxis2 =
                    data.mTwistAxis.Normalized();
                constraint->mNormalAxis1 = constraint->mNormalAxis2 =
                    data.mPlaneAxis.Normalized();
                constraint->mLimitsMin = data.mLinearLimitMin[0];
                constraint->mLimitsMax = data.mLinearLimitMax[0];
                constraint->mMaxFrictionForce = data.mLinearFriction[0];
                constraint->mMotorSettings = JPH::MotorSettings(
                    data.mMotorFrequency, data.mMotorDamping);
                constraint->mMotorSettings.SetForceLimit(
                    data.mMaxMotorTorque);
                part.mToParent = constraint;
            }
            else if (data.mJointType == 6)
            {
                JPH::SixDOFConstraintSettings *constraint =
                    new JPH::SixDOFConstraintSettings();
                constraint->mPosition1 = constraint->mPosition2 =
                    data.mJointPosition;
                constraint->mAxisX1 = constraint->mAxisX2 =
                    data.mTwistAxis.Normalized();
                constraint->mAxisY1 = constraint->mAxisY2 =
                    data.mPlaneAxis.Normalized();
                constraint->mSwingType =
                    static_cast<JPH::ESwingType>(data.mSixDOFSwingType);
                for (int axis = 0; axis < 3; ++axis)
                {
                    constraint->mLimitMin[axis] =
                        data.mLinearLimitMin[axis];
                    constraint->mLimitMax[axis] =
                        data.mLinearLimitMax[axis];
                    constraint->mMaxFriction[axis] =
                        data.mLinearFriction[axis];
                    constraint->mLimitMin[axis + 3] =
                        data.mAngularLimitMin[axis];
                    constraint->mLimitMax[axis + 3] =
                        data.mAngularLimitMax[axis];
                    constraint->mMaxFriction[axis + 3] =
                        data.mAngularFriction[axis];
                }
                part.mToParent = constraint;
            }
            else
            {
                JPH::SwingTwistConstraintSettings *constraint =
                    new JPH::SwingTwistConstraintSettings();
                constraint->mPosition1 = constraint->mPosition2 =
                    data.mJointPosition;
                constraint->mTwistAxis1 = constraint->mTwistAxis2 =
                    data.mTwistAxis.Normalized();
                constraint->mPlaneAxis1 = constraint->mPlaneAxis2 =
                    data.mPlaneAxis.Normalized();
                constraint->mNormalHalfConeAngle = data.mNormalHalfConeAngle;
                constraint->mPlaneHalfConeAngle = data.mPlaneHalfConeAngle;
                constraint->mTwistMinAngle = data.mTwistMinAngle;
                constraint->mTwistMaxAngle = data.mTwistMaxAngle;
                constraint->mMaxFrictionTorque = data.mMaxFrictionTorque;
                constraint->mSwingMotorSettings = JPH::MotorSettings(
                    data.mMotorFrequency, data.mMotorDamping);
                constraint->mSwingMotorSettings.SetTorqueLimit(
                    data.mMaxMotorTorque);
                constraint->mTwistMotorSettings =
                    constraint->mSwingMotorSettings;
                part.mToParent = constraint;
            }
        }
    }

    for (std::uint32_t index = 0;
         index < inDistanceConstraintCount; ++index)
    {
        const RagdollDistanceConstraintData &data =
            inDistanceConstraints[index];
        JPH::DistanceConstraintSettings *constraint =
            new JPH::DistanceConstraintSettings();
        constraint->mPoint1 = data.mPoint1;
        constraint->mPoint2 = data.mPoint2;
        constraint->mMinDistance = data.mMinDistance;
        constraint->mMaxDistance = data.mMaxDistance;
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }

    for (std::uint32_t index = 0; index < inPointConstraintCount; ++index)
    {
        const RagdollPointConstraintData &data = inPointConstraints[index];
        JPH::PointConstraintSettings *constraint =
            new JPH::PointConstraintSettings();
        constraint->mPoint1 = constraint->mPoint2 = data.mPoint;
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }
    for (std::uint32_t index = 0; index < inFixedConstraintCount; ++index)
    {
        const RagdollFixedConstraintData &data = inFixedConstraints[index];
        JPH::FixedConstraintSettings *constraint =
            new JPH::FixedConstraintSettings();
        constraint->mAutoDetectPoint = true;
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }
    for (std::uint32_t index = 0; index < inHingeConstraintCount; ++index)
    {
        const RagdollHingeConstraintData &data = inHingeConstraints[index];
        JPH::HingeConstraintSettings *constraint =
            new JPH::HingeConstraintSettings();
        constraint->mPoint1 = constraint->mPoint2 = data.mPoint;
        constraint->mHingeAxis1 = constraint->mHingeAxis2 =
            data.mHingeAxis.Normalized();
        constraint->mNormalAxis1 = constraint->mNormalAxis2 =
            data.mNormalAxis.Normalized();
        constraint->mLimitsMin = data.mMinAngle;
        constraint->mLimitsMax = data.mMaxAngle;
        constraint->mMaxFrictionTorque = data.mMaxFrictionTorque;
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }
    for (std::uint32_t index = 0; index < inSliderConstraintCount; ++index)
    {
        const RagdollSliderConstraintData &data = inSliderConstraints[index];
        JPH::SliderConstraintSettings *constraint =
            new JPH::SliderConstraintSettings();
        constraint->mPoint1 = constraint->mPoint2 = data.mPoint;
        constraint->mSliderAxis1 = constraint->mSliderAxis2 =
            data.mSliderAxis.Normalized();
        constraint->mNormalAxis1 = constraint->mNormalAxis2 =
            data.mNormalAxis.Normalized();
        constraint->mLimitsMin = data.mMinPosition;
        constraint->mLimitsMax = data.mMaxPosition;
        constraint->mMaxFrictionForce = data.mMaxFrictionForce;
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }
    for (std::uint32_t index = 0;
         index < inSwingTwistConstraintCount; ++index)
    {
        const RagdollSwingTwistConstraintData &data =
            inSwingTwistConstraints[index];
        JPH::SwingTwistConstraintSettings *constraint =
            new JPH::SwingTwistConstraintSettings();
        constraint->mPosition1 = constraint->mPosition2 = data.mPoint;
        constraint->mTwistAxis1 = constraint->mTwistAxis2 =
            data.mTwistAxis.Normalized();
        constraint->mPlaneAxis1 = constraint->mPlaneAxis2 =
            data.mPlaneAxis.Normalized();
        constraint->mNormalHalfConeAngle = data.mNormalHalfConeAngle;
        constraint->mPlaneHalfConeAngle = data.mPlaneHalfConeAngle;
        constraint->mTwistMinAngle = data.mTwistMinAngle;
        constraint->mTwistMaxAngle = data.mTwistMaxAngle;
        constraint->mMaxFrictionTorque = data.mMaxFrictionTorque;
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }
    for (std::uint32_t index = 0; index < inSixDOFConstraintCount; ++index)
    {
        const RagdollSixDOFConstraintData &data = inSixDOFConstraints[index];
        JPH::SixDOFConstraintSettings *constraint =
            new JPH::SixDOFConstraintSettings();
        constraint->mPosition1 = constraint->mPosition2 = data.mPoint;
        constraint->mAxisX1 = constraint->mAxisX2 = data.mAxisX.Normalized();
        constraint->mAxisY1 = constraint->mAxisY2 = data.mAxisY.Normalized();
        constraint->mSwingType = static_cast<JPH::ESwingType>(data.mSwingType);
        for (int axis = 0; axis < 3; ++axis)
        {
            constraint->mLimitMin[axis] = data.mLinearLimitMin[axis];
            constraint->mLimitMax[axis] = data.mLinearLimitMax[axis];
            constraint->mMaxFriction[axis] = data.mLinearFriction[axis];
            constraint->mLimitMin[axis + 3] = data.mAngularLimitMin[axis];
            constraint->mLimitMax[axis + 3] = data.mAngularLimitMax[axis];
            constraint->mMaxFriction[axis + 3] = data.mAngularFriction[axis];
        }
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }
    for (std::uint32_t index = 0; index < inConeConstraintCount; ++index)
    {
        const RagdollConeConstraintData &data = inConeConstraints[index];
        JPH::ConeConstraintSettings *constraint =
            new JPH::ConeConstraintSettings();
        constraint->mPoint1 = constraint->mPoint2 = data.mPoint;
        constraint->mTwistAxis1 = data.mTwistAxis1.Normalized();
        constraint->mTwistAxis2 = data.mTwistAxis2.Normalized();
        constraint->mHalfConeAngle = data.mHalfConeAngle;
        settings->mAdditionalConstraints.emplace_back(
            static_cast<int>(data.mPart1), static_cast<int>(data.mPart2),
            constraint);
    }

    if (inStabilize && !settings->Stabilize())
        return nullptr;
    if (inCalculatePriorities)
        settings->CalculateConstraintPriorities();
    if (inDisableParentChildCollisions)
        settings->DisableParentChildCollisions();
    settings->CalculateBodyIndexToConstraintIndex();
    settings->CalculateConstraintIndexToBodyIdxPair();

    JPH::Ragdoll *ragdoll = settings->CreateRagdoll(
        inGroupID, 0, inSystem);
    if (ragdoll == nullptr)
        return nullptr;
    RagdollHandle *handle = new RagdollHandle();
    handle->mRagdoll = ragdoll;
    handle->mSystem = inSystem;
    handle->mRagdoll->AddToPhysicsSystem(
        inActivate ? JPH::EActivation::Activate :
                     JPH::EActivation::DontActivate);
    handle->mAdded = true;
    return handle;
}

inline void DestroyRagdoll(RagdollHandle *inHandle)
{
    if (inHandle == nullptr)
        return;
    if (inHandle->mAdded)
        inHandle->mRagdoll->RemoveFromPhysicsSystem();
    delete inHandle;
}

inline std::uint32_t GetRagdollPartCount(const RagdollHandle *inHandle)
{
    return static_cast<std::uint32_t>(inHandle->mRagdoll->GetBodyCount());
}

inline std::uint32_t GetRagdollConstraintCount(const RagdollHandle *inHandle)
{
    return static_cast<std::uint32_t>(
        inHandle->mRagdoll->GetConstraintCount());
}

inline JPH::Constraint *GetRagdollConstraint(
    RagdollHandle *inHandle, std::uint32_t inIndex)
{
    if (inIndex >= inHandle->mRagdoll->GetConstraintCount())
        return nullptr;
    return inHandle->mRagdoll->GetConstraint(static_cast<int>(inIndex));
}

inline bool GetRagdollConstraintBodyIndices(
    const RagdollHandle *inHandle, std::uint32_t inIndex,
    std::uint32_t *outBody1, std::uint32_t *outBody2)
{
    if (inIndex >= inHandle->mRagdoll->GetConstraintCount())
        return false;
    const JPH::RagdollSettings::BodyIdxPair indices =
        inHandle->mRagdoll->GetRagdollSettings()->
            GetBodyIndicesForConstraintIndex(static_cast<int>(inIndex));
    *outBody1 = static_cast<std::uint32_t>(indices.first);
    *outBody2 = static_cast<std::uint32_t>(indices.second);
    return true;
}

inline std::uint32_t GetRagdollBodyID(
    const RagdollHandle *inHandle, std::uint32_t inIndex)
{
    return inHandle->mRagdoll->GetBodyID(inIndex).GetIndexAndSequenceNumber();
}

inline void SetRagdollPose(
    RagdollHandle *inHandle, const JPH::Vec3 *inPositions,
    const JPH::Quat *inRotations)
{
    const std::uint32_t count = GetRagdollPartCount(inHandle);
    JPH::Array<JPH::Mat44> matrices;
    matrices.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index)
        matrices.push_back(JPH::Mat44::sRotationTranslation(
            inRotations[index], inPositions[index]));
    inHandle->mRagdoll->SetPose(JPH::RVec3::sZero(), matrices.data());
}

inline void GetRagdollPose(
    const RagdollHandle *inHandle, JPH::Vec3 *outPositions,
    JPH::Quat *outRotations)
{
    JPH::BodyInterface &body_interface =
        inHandle->mSystem->GetBodyInterface();
    const std::uint32_t count = GetRagdollPartCount(inHandle);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        JPH::BodyID id = inHandle->mRagdoll->GetBodyID(index);
        outPositions[index] = JPH::Vec3(body_interface.GetPosition(id));
        outRotations[index] = body_interface.GetRotation(id);
    }
}

inline void DriveRagdollKinematic(
    RagdollHandle *inHandle, const JPH::Vec3 *inPositions,
    const JPH::Quat *inRotations, float inDeltaTime)
{
    const std::uint32_t count = GetRagdollPartCount(inHandle);
    JPH::Array<JPH::Mat44> matrices;
    matrices.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index)
        matrices.push_back(JPH::Mat44::sRotationTranslation(
            inRotations[index], inPositions[index]));
    inHandle->mRagdoll->DriveToPoseUsingKinematics(
        JPH::RVec3::sZero(), matrices.data(), inDeltaTime);
}

inline void DriveRagdollMotors(
    RagdollHandle *inHandle, const JPH::Vec3 *inLocalTranslations,
    const JPH::Quat *inLocalRotations)
{
    JPH::SkeletonPose pose;
    pose.SetSkeleton(inHandle->mRagdoll->GetRagdollSettings()->GetSkeleton());
    const std::uint32_t count = GetRagdollPartCount(inHandle);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        pose.GetJoint(index).mTranslation = inLocalTranslations[index];
        pose.GetJoint(index).mRotation = inLocalRotations[index];
    }
    pose.CalculateJointMatrices();
    inHandle->mRagdoll->DriveToPoseUsingMotors(pose);
    inHandle->mRagdoll->Activate();
}

inline void DriveRagdollMotorsWithVelocity(
    RagdollHandle *inHandle,
    const JPH::Vec3 *inPrevLocalTranslations,
    const JPH::Quat *inPrevLocalRotations,
    const JPH::Vec3 *inLocalTranslations,
    const JPH::Quat *inLocalRotations, float inDeltaTime)
{
    const JPH::Skeleton *skeleton =
        inHandle->mRagdoll->GetRagdollSettings()->GetSkeleton();
    JPH::SkeletonPose prev_pose;
    JPH::SkeletonPose pose;
    prev_pose.SetSkeleton(skeleton);
    pose.SetSkeleton(skeleton);
    const std::uint32_t count = GetRagdollPartCount(inHandle);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        prev_pose.GetJoint(index).mTranslation =
            inPrevLocalTranslations[index];
        prev_pose.GetJoint(index).mRotation = inPrevLocalRotations[index];
        pose.GetJoint(index).mTranslation = inLocalTranslations[index];
        pose.GetJoint(index).mRotation = inLocalRotations[index];
    }
    prev_pose.CalculateJointMatrices();
    pose.CalculateJointMatrices();
    inHandle->mRagdoll->DriveToPoseUsingMotors(
        prev_pose, pose, inDeltaTime);
    inHandle->mRagdoll->Activate();
}

inline void ActivateRagdoll(RagdollHandle *inHandle)
{
    inHandle->mRagdoll->Activate();
}

inline bool IsRagdollActive(const RagdollHandle *inHandle)
{
    return inHandle->mRagdoll->IsActive();
}

inline void SetRagdollGroupID(RagdollHandle *inHandle, std::uint32_t inGroupID)
{
    inHandle->mRagdoll->SetGroupID(inGroupID);
}

inline void SetRagdollVelocity(
    RagdollHandle *inHandle, JPH::Vec3Arg inLinear,
    JPH::Vec3Arg inAngular)
{
    inHandle->mRagdoll->SetLinearAndAngularVelocity(inLinear, inAngular);
}

inline void AddRagdollLinearVelocity(
    RagdollHandle *inHandle, JPH::Vec3Arg inVelocity)
{
    inHandle->mRagdoll->AddLinearVelocity(inVelocity);
}

inline void AddRagdollImpulse(
    RagdollHandle *inHandle, JPH::Vec3Arg inImpulse)
{
    inHandle->mRagdoll->AddImpulse(inImpulse);
}

inline void ResetRagdollWarmStart(RagdollHandle *inHandle)
{
    inHandle->mRagdoll->ResetWarmStart();
}

inline void GetRagdollRootTransform(
    const RagdollHandle *inHandle, JPH::Vec3 *outPosition,
    JPH::Quat *outRotation)
{
    JPH::RVec3 position;
    inHandle->mRagdoll->GetRootTransform(position, *outRotation);
    *outPosition = JPH::Vec3(position);
}

inline JPH::AABox GetRagdollBounds(const RagdollHandle *inHandle)
{
    return inHandle->mRagdoll->GetWorldSpaceBounds();
}

struct SkeletonMapperHandle
{
    JPH::Ref<JPH::Skeleton> mSource;
    JPH::Ref<JPH::Skeleton> mTarget;
    JPH::Ref<JPH::SkeletonMapper> mMapper;
};

inline bool IsValidSkeletonInput(
    const char *const *inNames, const std::int32_t *inParents,
    const JPH::Vec3 *inPositions, const JPH::Quat *inRotations,
    std::uint32_t inCount)
{
    if (inCount == 0 || inNames == nullptr || inParents == nullptr ||
        inPositions == nullptr || inRotations == nullptr)
        return false;
    for (std::uint32_t index = 0; index < inCount; ++index)
        if (inNames[index] == nullptr || inNames[index][0] == '\0' ||
            (index == 0 && inParents[index] != -1) ||
            (index > 0 &&
             (inParents[index] < 0 ||
              static_cast<std::uint32_t>(inParents[index]) >= index)))
            return false;
    return true;
}

inline SkeletonMapperHandle *CreateSkeletonMapper(
    const char *const *inSourceNames, const std::int32_t *inSourceParents,
    const JPH::Vec3 *inSourceNeutralPositions,
    const JPH::Quat *inSourceNeutralRotations, std::uint32_t inSourceCount,
    const char *const *inTargetNames, const std::int32_t *inTargetParents,
    const JPH::Vec3 *inTargetNeutralPositions,
    const JPH::Quat *inTargetNeutralRotations, std::uint32_t inTargetCount)
{
    if (!IsValidSkeletonInput(
            inSourceNames, inSourceParents, inSourceNeutralPositions,
            inSourceNeutralRotations, inSourceCount) ||
        !IsValidSkeletonInput(
            inTargetNames, inTargetParents, inTargetNeutralPositions,
            inTargetNeutralRotations, inTargetCount))
        return nullptr;

    SkeletonMapperHandle *handle = new SkeletonMapperHandle();
    handle->mSource = new JPH::Skeleton();
    handle->mTarget = new JPH::Skeleton();
    JPH::Array<JPH::Mat44> source_neutral;
    JPH::Array<JPH::Mat44> target_neutral;
    source_neutral.reserve(inSourceCount);
    target_neutral.reserve(inTargetCount);
    for (std::uint32_t index = 0; index < inSourceCount; ++index)
    {
        handle->mSource->AddJoint(
            inSourceNames[index], inSourceParents[index]);
        source_neutral.push_back(JPH::Mat44::sRotationTranslation(
            inSourceNeutralRotations[index], inSourceNeutralPositions[index]));
    }
    for (std::uint32_t index = 0; index < inTargetCount; ++index)
    {
        handle->mTarget->AddJoint(
            inTargetNames[index], inTargetParents[index]);
        target_neutral.push_back(JPH::Mat44::sRotationTranslation(
            inTargetNeutralRotations[index], inTargetNeutralPositions[index]));
    }
    handle->mMapper = new JPH::SkeletonMapper();
    handle->mMapper->Initialize(
        handle->mSource, source_neutral.data(), handle->mTarget,
        target_neutral.data());
    if (handle->mMapper->GetMappings().size() != inSourceCount)
    {
        delete handle;
        return nullptr;
    }
    return handle;
}

inline void DestroySkeletonMapper(SkeletonMapperHandle *inHandle)
{
    delete inHandle;
}

inline std::uint32_t GetSkeletonMapperMappingCount(
    const SkeletonMapperHandle *inHandle)
{
    return static_cast<std::uint32_t>(
        inHandle->mMapper->GetMappings().size());
}

inline std::uint32_t GetSkeletonMapperChainCount(
    const SkeletonMapperHandle *inHandle)
{
    return static_cast<std::uint32_t>(
        inHandle->mMapper->GetChains().size());
}

inline std::uint32_t GetSkeletonMapperUnmappedCount(
    const SkeletonMapperHandle *inHandle)
{
    return static_cast<std::uint32_t>(
        inHandle->mMapper->GetUnmapped().size());
}

inline std::int32_t GetSkeletonMapperMappedJoint(
    const SkeletonMapperHandle *inHandle, std::int32_t inSourceJoint)
{
    return inHandle->mMapper->GetMappedJointIdx(inSourceJoint);
}

inline bool IsSkeletonMapperTranslationLocked(
    const SkeletonMapperHandle *inHandle, std::int32_t inTargetJoint)
{
    return inHandle->mMapper->IsJointTranslationLocked(inTargetJoint);
}

inline void LockSkeletonMapperTranslations(
    SkeletonMapperHandle *inHandle, const bool *inLocked,
    const JPH::Vec3 *inTargetNeutralPositions,
    const JPH::Quat *inTargetNeutralRotations)
{
    const std::uint32_t count = static_cast<std::uint32_t>(
        inHandle->mTarget->GetJointCount());
    JPH::Array<JPH::Mat44> neutral;
    neutral.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index)
        neutral.push_back(JPH::Mat44::sRotationTranslation(
            inTargetNeutralRotations[index], inTargetNeutralPositions[index]));
    inHandle->mMapper->LockTranslations(
        inHandle->mTarget, inLocked, neutral.data());
}

inline void LockAllSkeletonMapperTranslations(
    SkeletonMapperHandle *inHandle,
    const JPH::Vec3 *inTargetNeutralPositions,
    const JPH::Quat *inTargetNeutralRotations)
{
    const std::uint32_t count = static_cast<std::uint32_t>(
        inHandle->mTarget->GetJointCount());
    JPH::Array<JPH::Mat44> neutral;
    neutral.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index)
        neutral.push_back(JPH::Mat44::sRotationTranslation(
            inTargetNeutralRotations[index], inTargetNeutralPositions[index]));
    inHandle->mMapper->LockAllTranslations(
        inHandle->mTarget, neutral.data());
}

inline void MapSkeletonPose(
    const SkeletonMapperHandle *inHandle,
    const JPH::Vec3 *inSourceModelPositions,
    const JPH::Quat *inSourceModelRotations,
    const JPH::Vec3 *inTargetLocalPositions,
    const JPH::Quat *inTargetLocalRotations,
    JPH::Vec3 *outTargetModelPositions,
    JPH::Quat *outTargetModelRotations)
{
    const std::uint32_t source_count = static_cast<std::uint32_t>(
        inHandle->mSource->GetJointCount());
    const std::uint32_t target_count = static_cast<std::uint32_t>(
        inHandle->mTarget->GetJointCount());
    JPH::Array<JPH::Mat44> source_model;
    JPH::Array<JPH::Mat44> target_local;
    JPH::Array<JPH::Mat44> target_model;
    source_model.reserve(source_count);
    target_local.reserve(target_count);
    target_model.resize(target_count);
    for (std::uint32_t index = 0; index < source_count; ++index)
        source_model.push_back(JPH::Mat44::sRotationTranslation(
            inSourceModelRotations[index], inSourceModelPositions[index]));
    for (std::uint32_t index = 0; index < target_count; ++index)
        target_local.push_back(JPH::Mat44::sRotationTranslation(
            inTargetLocalRotations[index], inTargetLocalPositions[index]));
    inHandle->mMapper->Map(
        source_model.data(), target_local.data(), target_model.data());
    for (std::uint32_t index = 0; index < target_count; ++index)
    {
        outTargetModelPositions[index] = target_model[index].GetTranslation();
        outTargetModelRotations[index] = target_model[index].GetQuaternion();
    }
}

inline void ReverseMapSkeletonPose(
    const SkeletonMapperHandle *inHandle,
    const JPH::Vec3 *inTargetModelPositions,
    const JPH::Quat *inTargetModelRotations,
    JPH::Vec3 *outSourceModelPositions,
    JPH::Quat *outSourceModelRotations)
{
    const std::uint32_t source_count = static_cast<std::uint32_t>(
        inHandle->mSource->GetJointCount());
    const std::uint32_t target_count = static_cast<std::uint32_t>(
        inHandle->mTarget->GetJointCount());
    JPH::Array<JPH::Mat44> target_model;
    JPH::Array<JPH::Mat44> source_model;
    target_model.reserve(target_count);
    source_model.resize(source_count);
    for (std::uint32_t index = 0; index < target_count; ++index)
        target_model.push_back(JPH::Mat44::sRotationTranslation(
            inTargetModelRotations[index], inTargetModelPositions[index]));
    inHandle->mMapper->MapReverse(
        target_model.data(), source_model.data());
    for (std::uint32_t index = 0; index < source_count; ++index)
    {
        outSourceModelPositions[index] = source_model[index].GetTranslation();
        outSourceModelRotations[index] = source_model[index].GetQuaternion();
    }
}

struct SkeletalAnimationHandle
{
    JPH::Ref<JPH::Skeleton> mSkeleton;
    JPH::Ref<JPH::SkeletalAnimation> mAnimation;
    JPH::Array<JPH::SkeletalAnimation::JointState> mNeutralPose;
    std::uint32_t mKeyframeCount = 0;
    std::string mSerialized;
};

inline SkeletalAnimationHandle *CreateSkeletalAnimation(
    const char *const *inJointNames, const std::int32_t *inJointParents,
    const JPH::Vec3 *inNeutralPositions,
    const JPH::Quat *inNeutralRotations, std::uint32_t inJointCount,
    const char *const *inTrackNames, const std::uint32_t *inTrackOffsets,
    std::uint32_t inTrackCount, const float *inTimes,
    const JPH::Vec3 *inTranslations, const JPH::Quat *inRotations,
    std::uint32_t inKeyframeCount, bool inIsLooping)
{
    if (!IsValidSkeletonInput(
            inJointNames, inJointParents, inNeutralPositions,
            inNeutralRotations, inJointCount) ||
        inTrackCount == 0 || inTrackNames == nullptr ||
        inTrackOffsets == nullptr || inTimes == nullptr ||
        inTranslations == nullptr || inRotations == nullptr ||
        inKeyframeCount == 0 || inTrackOffsets[0] != 0 ||
        inTrackOffsets[inTrackCount] != inKeyframeCount)
        return nullptr;

    SkeletalAnimationHandle *handle = new SkeletalAnimationHandle();
    handle->mSkeleton = new JPH::Skeleton();
    handle->mNeutralPose.resize(inJointCount);
    for (std::uint32_t index = 0; index < inJointCount; ++index)
    {
        handle->mSkeleton->AddJoint(inJointNames[index], inJointParents[index]);
        handle->mNeutralPose[index].mTranslation = inNeutralPositions[index];
        handle->mNeutralPose[index].mRotation = inNeutralRotations[index];
    }

    std::uint32_t longest_track = 0;
    float duration = -1.0f;
    for (std::uint32_t track = 0; track < inTrackCount; ++track)
    {
        const std::uint32_t begin = inTrackOffsets[track];
        const std::uint32_t end = inTrackOffsets[track + 1];
        if (inTrackNames[track] == nullptr ||
            handle->mSkeleton->GetJointIndex(inTrackNames[track]) < 0 ||
            begin >= end || end > inKeyframeCount)
        {
            delete handle;
            return nullptr;
        }
        for (std::uint32_t key = begin; key < end; ++key)
            if (inTimes[key] < 0.0f ||
                (key > begin && inTimes[key] <= inTimes[key - 1]))
            {
                delete handle;
                return nullptr;
            }
        if (inTimes[end - 1] > duration)
        {
            duration = inTimes[end - 1];
            longest_track = track;
        }
    }

    handle->mAnimation = new JPH::SkeletalAnimation();
    handle->mAnimation->SetIsLooping(inIsLooping);
    JPH::SkeletalAnimation::AnimatedJointVector &animated_joints =
        handle->mAnimation->GetAnimatedJoints();
    animated_joints.reserve(inTrackCount);
    for (std::uint32_t output_track = 0;
         output_track < inTrackCount; ++output_track)
    {
        const std::uint32_t track = output_track == 0 ? longest_track :
            (output_track <= longest_track ? output_track - 1 : output_track);
        const std::uint32_t begin = inTrackOffsets[track];
        const std::uint32_t end = inTrackOffsets[track + 1];
        JPH::SkeletalAnimation::AnimatedJoint &joint =
            animated_joints.emplace_back();
        joint.mJointName = inTrackNames[track];
        joint.mKeyframes.resize(end - begin);
        for (std::uint32_t key = begin; key < end; ++key)
        {
            JPH::SkeletalAnimation::Keyframe &output =
                joint.mKeyframes[key - begin];
            output.mTime = inTimes[key];
            output.mTranslation = inTranslations[key];
            output.mRotation = inRotations[key];
        }
    }
    handle->mKeyframeCount = inKeyframeCount;
    return handle;
}

inline SkeletalAnimationHandle *RestoreSkeletalAnimation(
    const char *const *inJointNames, const std::int32_t *inJointParents,
    const JPH::Vec3 *inNeutralPositions,
    const JPH::Quat *inNeutralRotations, std::uint32_t inJointCount,
    const std::uint8_t *inData, std::size_t inSize)
{
    if (!IsValidSkeletonInput(
            inJointNames, inJointParents, inNeutralPositions,
            inNeutralRotations, inJointCount) ||
        inData == nullptr || inSize == 0)
        return nullptr;
    try
    {
        const std::string data(
            reinterpret_cast<const char *>(inData), inSize);
        std::istringstream stream(data, std::ios::in | std::ios::binary);
        JPH::StreamInWrapper wrapper(stream);
        JPH::SkeletalAnimation::AnimationResult restored =
            JPH::SkeletalAnimation::sRestoreFromBinaryState(wrapper);
        if (restored.HasError() || wrapper.IsFailed())
            return nullptr;
        JPH::Ref<JPH::SkeletalAnimation> animation = restored.Get();
        if (animation == nullptr || animation->GetAnimatedJoints().empty())
            return nullptr;

        std::unique_ptr<SkeletalAnimationHandle> handle(
            new SkeletalAnimationHandle());
        handle->mSkeleton = new JPH::Skeleton();
        handle->mNeutralPose.resize(inJointCount);
        for (std::uint32_t index = 0; index < inJointCount; ++index)
        {
            handle->mSkeleton->AddJoint(
                inJointNames[index], inJointParents[index]);
            handle->mNeutralPose[index].mTranslation =
                inNeutralPositions[index];
            handle->mNeutralPose[index].mRotation = inNeutralRotations[index];
        }

        std::unordered_set<std::string> track_names;
        std::uint64_t keyframe_count = 0;
        for (const JPH::SkeletalAnimation::AnimatedJoint &joint :
             animation->GetAnimatedJoints())
        {
            const std::string track_name(joint.mJointName.c_str());
            if (joint.mJointName.empty() ||
                handle->mSkeleton->GetJointIndex(joint.mJointName) < 0 ||
                !track_names.insert(track_name).second ||
                joint.mKeyframes.empty())
                return nullptr;
            float previous_time = -1.0f;
            for (const JPH::SkeletalAnimation::Keyframe &keyframe :
                 joint.mKeyframes)
            {
                if (!std::isfinite(keyframe.mTime) ||
                    keyframe.mTime < 0.0f ||
                    keyframe.mTime <= previous_time ||
                    !IsFiniteVec3(keyframe.mTranslation) ||
                    !IsFiniteQuat(keyframe.mRotation) ||
                    keyframe.mRotation.LengthSq() <= 1.0e-12f)
                    return nullptr;
                previous_time = keyframe.mTime;
            }
            keyframe_count += joint.mKeyframes.size();
            if (keyframe_count > std::numeric_limits<std::uint32_t>::max())
                return nullptr;
        }
        handle->mAnimation = animation;
        handle->mKeyframeCount = static_cast<std::uint32_t>(keyframe_count);
        return handle.release();
    }
    catch (...)
    {
        return nullptr;
    }
}

inline bool SerializeSkeletalAnimation(SkeletalAnimationHandle *inHandle)
{
    if (inHandle == nullptr || inHandle->mAnimation == nullptr)
        return false;
    try
    {
        std::ostringstream stream(std::ios::out | std::ios::binary);
        JPH::StreamOutWrapper wrapper(stream);
        inHandle->mAnimation->SaveBinaryState(wrapper);
        if (wrapper.IsFailed())
        {
            inHandle->mSerialized.clear();
            return false;
        }
        inHandle->mSerialized = stream.str();
        return !inHandle->mSerialized.empty();
    }
    catch (...)
    {
        inHandle->mSerialized.clear();
        return false;
    }
}

inline std::size_t GetSkeletalAnimationSerializedSize(
    const SkeletalAnimationHandle *inHandle)
{
    return inHandle->mSerialized.size();
}

inline void CopySkeletalAnimationSerializedData(
    const SkeletalAnimationHandle *inHandle, std::uint8_t *outData)
{
    if (!inHandle->mSerialized.empty())
        std::memcpy(outData, inHandle->mSerialized.data(),
                    inHandle->mSerialized.size());
}

inline void DestroySkeletalAnimation(SkeletalAnimationHandle *inHandle)
{
    delete inHandle;
}

inline float GetSkeletalAnimationDuration(
    const SkeletalAnimationHandle *inHandle)
{
    return inHandle->mAnimation->GetDuration();
}

inline std::uint32_t GetSkeletalAnimationTrackCount(
    const SkeletalAnimationHandle *inHandle)
{
    return static_cast<std::uint32_t>(
        inHandle->mAnimation->GetAnimatedJoints().size());
}

inline std::uint32_t GetSkeletalAnimationKeyframeCount(
    const SkeletalAnimationHandle *inHandle)
{
    return inHandle->mKeyframeCount;
}

inline bool IsSkeletalAnimationLooping(
    const SkeletalAnimationHandle *inHandle)
{
    return inHandle->mAnimation->IsLooping();
}

inline void SetSkeletalAnimationLooping(
    SkeletalAnimationHandle *inHandle, bool inIsLooping)
{
    inHandle->mAnimation->SetIsLooping(inIsLooping);
}

inline void ScaleSkeletalAnimationJoints(
    SkeletalAnimationHandle *inHandle, float inScale)
{
    inHandle->mAnimation->ScaleJoints(inScale);
}

inline void SampleSkeletalAnimation(
    const SkeletalAnimationHandle *inHandle, float inTime, bool inModelSpace,
    JPH::Vec3 *outPositions, JPH::Quat *outRotations)
{
    JPH::SkeletonPose pose;
    pose.SetSkeleton(inHandle->mSkeleton);
    const std::uint32_t count = static_cast<std::uint32_t>(
        inHandle->mSkeleton->GetJointCount());
    for (std::uint32_t index = 0; index < count; ++index)
        pose.GetJoint(index) = inHandle->mNeutralPose[index];
    inHandle->mAnimation->Sample(inTime, pose);
    if (inModelSpace)
    {
        pose.CalculateJointMatrices();
        for (std::uint32_t index = 0; index < count; ++index)
        {
            outPositions[index] = pose.GetJointMatrix(index).GetTranslation();
            outRotations[index] = pose.GetJointMatrix(index).GetQuaternion();
        }
    }
    else
        for (std::uint32_t index = 0; index < count; ++index)
        {
            outPositions[index] = pose.GetJoint(index).mTranslation;
            outRotations[index] = pose.GetJoint(index).mRotation;
        }
}

struct PhysicsSceneHandle
{
    JPH::Ref<JPH::PhysicsScene> mScene;
    std::string mSerialized;
};

struct AuthoredCompoundSettingsHandle
{
    JPH::Ref<JPH::CompoundShapeSettings> mSettings;
};

struct PhysicsSceneInstanceHandle
{
    JPH::PhysicsSystem *mSystem = nullptr;
    JPH::BodyIDVector mBodies;
    JPH::Array<JPH::Constraint *> mConstraints;
    bool mAdded = false;
};

inline PhysicsSceneHandle *CreatePhysicsScene()
{
    try
    {
        std::unique_ptr<PhysicsSceneHandle> handle(
            new PhysicsSceneHandle());
        handle->mScene = new JPH::PhysicsScene();
        return handle.release();
    }
    catch (...)
    {
        return nullptr;
    }
}

inline JPH::BodyCreationSettings CreatePhysicsScenePrimitiveBodySettings(
    std::uint8_t inShapeKind,
    JPH::Vec3Arg inHalfExtent,
    float inHalfHeight,
    float inRadius,
    float inTopRadius,
    float inBottomRadius,
    float inConvexRadius,
    const JPH::Vec3 *inPoints,
    std::uint32_t inPointCount,
    JPH::Vec3Arg inPlaneNormal,
    float inPlaneConstant,
    float inPlaneHalfExtent,
    JPH::Vec3Arg inCenterOfMass,
    const JPH::PhysicsMaterial *inMaterial,
    JPH::RVec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::EMotionType inMotionType,
    JPH::ObjectLayer inObjectLayer)
{
    JPH::Ref<JPH::ShapeSettings> shapeSettings;
    switch (inShapeKind)
    {
    case 0:
        shapeSettings = new JPH::BoxShapeSettings(
            inHalfExtent, inConvexRadius, inMaterial);
        break;
    case 1:
        shapeSettings = new JPH::SphereShapeSettings(inRadius, inMaterial);
        break;
    case 2:
        shapeSettings = new JPH::CapsuleShapeSettings(
            inHalfHeight, inRadius, inMaterial);
        break;
    case 3:
        shapeSettings = new JPH::CylinderShapeSettings(
            inHalfHeight, inRadius, inConvexRadius, inMaterial);
        break;
    case 4:
        shapeSettings = new JPH::TaperedCapsuleShapeSettings(
            inHalfHeight, inTopRadius, inBottomRadius, inMaterial);
        break;
    case 5:
        shapeSettings = new JPH::TaperedCylinderShapeSettings(
            inHalfHeight, inTopRadius, inBottomRadius,
            inConvexRadius, inMaterial);
        break;
    case 6:
        if (inPoints == nullptr || inPointCount != 3)
            return JPH::BodyCreationSettings();
        shapeSettings = new JPH::TriangleShapeSettings(
            inPoints[0], inPoints[1], inPoints[2],
            inConvexRadius, inMaterial);
        break;
    case 7:
        shapeSettings = new JPH::PlaneShapeSettings(
            JPH::Plane(inPlaneNormal, inPlaneConstant),
            inMaterial, inPlaneHalfExtent);
        break;
    case 8:
        shapeSettings = new JPH::EmptyShapeSettings(inCenterOfMass);
        break;
    case 9:
        if (inPoints == nullptr || inPointCount < 4 ||
            inPointCount > static_cast<std::uint32_t>(
                std::numeric_limits<int>::max()))
            return JPH::BodyCreationSettings();
        shapeSettings = new JPH::ConvexHullShapeSettings(
            inPoints, static_cast<int>(inPointCount),
            inConvexRadius, inMaterial);
        break;
    default:
        return JPH::BodyCreationSettings();
    }
    return JPH::BodyCreationSettings(
        shapeSettings.GetPtr(), inPosition, inRotation,
        inMotionType, inObjectLayer);
}

inline JPH::BodyCreationSettings CreatePhysicsSceneMeshBodySettings(
    const JPH::Vec3 *inVertices,
    std::uint32_t inVertexCount,
    const std::uint32_t *inIndices,
    std::uint32_t inTriangleCount,
    JPH::PhysicsMaterial *const *inMaterials,
    std::uint32_t inMaterialCount,
    const std::uint32_t *inMaterialIndices,
    JPH::RVec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::EMotionType inMotionType,
    JPH::ObjectLayer inObjectLayer)
{
    if (inVertices == nullptr || inVertexCount < 3 ||
        inIndices == nullptr || inTriangleCount == 0)
        return JPH::BodyCreationSettings();
    try
    {
        JPH::VertexList vertices;
        vertices.reserve(inVertexCount);
        for (std::uint32_t index = 0; index < inVertexCount; ++index)
            vertices.emplace_back(
                inVertices[index].GetX(),
                inVertices[index].GetY(),
                inVertices[index].GetZ());
        JPH::IndexedTriangleList triangles;
        triangles.reserve(inTriangleCount);
        for (std::uint32_t triangle = 0; triangle < inTriangleCount; ++triangle)
        {
            const std::uint32_t offset = triangle * 3;
            triangles.emplace_back(
                inIndices[offset], inIndices[offset + 1],
                inIndices[offset + 2],
                inMaterialIndices == nullptr? 0 : inMaterialIndices[triangle]);
        }
        JPH::PhysicsMaterialList materials;
        materials.reserve(inMaterialCount);
        for (std::uint32_t index = 0; index < inMaterialCount; ++index)
            materials.emplace_back(inMaterials[index]);
        JPH::Ref<JPH::ShapeSettings> shapeSettings =
            new JPH::MeshShapeSettings(
                std::move(vertices), std::move(triangles),
                std::move(materials));
        return JPH::BodyCreationSettings(
            shapeSettings.GetPtr(), inPosition, inRotation,
            inMotionType, inObjectLayer);
    }
    catch (...)
    {
        return JPH::BodyCreationSettings();
    }
}

inline JPH::BodyCreationSettings CreatePhysicsSceneHeightFieldBodySettings(
    const float *inSamples,
    std::uint32_t inSampleCount,
    JPH::Vec3Arg inOffset,
    JPH::Vec3Arg inScale,
    std::uint32_t inBlockSize,
    std::uint32_t inBitsPerSample,
    const std::uint8_t *inMaterialIndices,
    JPH::PhysicsMaterial *const *inMaterials,
    std::uint32_t inMaterialCount,
    JPH::RVec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::EMotionType inMotionType,
    JPH::ObjectLayer inObjectLayer)
{
    if (inSamples == nullptr || inSampleCount == 0)
        return JPH::BodyCreationSettings();
    try
    {
        JPH::PhysicsMaterialList materials;
        materials.reserve(inMaterialCount);
        for (std::uint32_t index = 0; index < inMaterialCount; ++index)
            materials.emplace_back(inMaterials[index]);
        JPH::Ref<JPH::HeightFieldShapeSettings> shapeSettings =
            new JPH::HeightFieldShapeSettings(
                inSamples, inOffset, inScale, inSampleCount,
                inMaterialIndices, materials);
        shapeSettings->mBlockSize = inBlockSize;
        shapeSettings->mBitsPerSample = inBitsPerSample;
        return JPH::BodyCreationSettings(
            shapeSettings.GetPtr(), inPosition, inRotation,
            inMotionType, inObjectLayer);
    }
    catch (...)
    {
        return JPH::BodyCreationSettings();
    }
}

inline AuthoredCompoundSettingsHandle *CreatePhysicsSceneCompoundSettings(
    bool inMutable)
{
    try
    {
        std::unique_ptr<AuthoredCompoundSettingsHandle> handle(
            new AuthoredCompoundSettingsHandle());
        if (inMutable)
            handle->mSettings = new JPH::MutableCompoundShapeSettings();
        else
            handle->mSettings = new JPH::StaticCompoundShapeSettings();
        return handle.release();
    }
    catch (...)
    {
        return nullptr;
    }
}

inline void DestroyPhysicsSceneCompoundSettings(
    AuthoredCompoundSettingsHandle *inHandle)
{
    delete inHandle;
}

inline bool AddPhysicsSceneCompoundChild(
    AuthoredCompoundSettingsHandle *inHandle,
    const JPH::BodyCreationSettings &inChild,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation)
{
    if (inHandle == nullptr || inHandle->mSettings == nullptr ||
        inChild.GetShapeSettings() == nullptr)
        return false;
    try
    {
        inHandle->mSettings->AddShape(
            inPosition, inRotation, inChild.GetShapeSettings());
        return true;
    }
    catch (...)
    {
        return false;
    }
}

inline JPH::BodyCreationSettings CreatePhysicsSceneCompoundBodySettings(
    const AuthoredCompoundSettingsHandle *inHandle,
    JPH::RVec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::EMotionType inMotionType,
    JPH::ObjectLayer inObjectLayer)
{
    if (inHandle == nullptr || inHandle->mSettings == nullptr)
        return JPH::BodyCreationSettings();
    return JPH::BodyCreationSettings(
        inHandle->mSettings.GetPtr(), inPosition, inRotation,
        inMotionType, inObjectLayer);
}

inline JPH::BodyCreationSettings CreatePhysicsSceneDecoratedBodySettings(
    const JPH::BodyCreationSettings &inChild,
    std::uint8_t inDecoratorKind,
    JPH::Vec3Arg inScale,
    JPH::Vec3Arg inShapePosition,
    JPH::QuatArg inShapeRotation,
    JPH::Vec3Arg inCenterOfMassOffset,
    JPH::RVec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::EMotionType inMotionType,
    JPH::ObjectLayer inObjectLayer)
{
    const JPH::ShapeSettings *child = inChild.GetShapeSettings();
    if (child == nullptr)
        return JPH::BodyCreationSettings();
    JPH::Ref<JPH::ShapeSettings> shapeSettings;
    switch (inDecoratorKind)
    {
    case 0:
        shapeSettings = new JPH::ScaledShapeSettings(child, inScale);
        break;
    case 1:
        shapeSettings = new JPH::RotatedTranslatedShapeSettings(
            inShapePosition, inShapeRotation, child);
        break;
    case 2:
        shapeSettings = new JPH::OffsetCenterOfMassShapeSettings(
            inCenterOfMassOffset, child);
        break;
    default:
        return JPH::BodyCreationSettings();
    }
    return JPH::BodyCreationSettings(
        shapeSettings.GetPtr(), inPosition, inRotation,
        inMotionType, inObjectLayer);
}

inline bool AddPhysicsSceneConstraintSettings(
    PhysicsSceneHandle *inHandle,
    const JPH::TwoBodyConstraintSettings *inSettings,
    std::uint32_t inBody1,
    std::uint32_t inBody2)
{
    const bool body1_valid =
        inBody1 == JPH::PhysicsScene::cFixedToWorld ||
        (inHandle != nullptr && inHandle->mScene != nullptr &&
         inBody1 < inHandle->mScene->GetNumBodies());
    const bool body2_valid =
        inBody2 == JPH::PhysicsScene::cFixedToWorld ||
        (inHandle != nullptr && inHandle->mScene != nullptr &&
         inBody2 < inHandle->mScene->GetNumBodies());
    if (inHandle == nullptr || inHandle->mScene == nullptr ||
        inSettings == nullptr || !body1_valid || !body2_valid ||
        inBody1 == inBody2)
        return false;
    try
    {
        inHandle->mScene->AddConstraint(inSettings, inBody1, inBody2);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

inline bool ConfigurePhysicsSceneConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    bool inEnabled, std::uint32_t inPriority,
    std::uint32_t inNumVelocityStepsOverride,
    std::uint32_t inNumPositionStepsOverride,
    float inDrawConstraintSize, std::uint64_t inUserData)
{
    if (inHandle == nullptr || inHandle->mScene == nullptr ||
        inConstraintIndex >= inHandle->mScene->GetNumConstraints() ||
        inNumVelocityStepsOverride >= 256 ||
        inNumPositionStepsOverride >= 256 ||
        !std::isfinite(inDrawConstraintSize) || inDrawConstraintSize <= 0.0f)
        return false;
    JPH::Array<JPH::PhysicsScene::ConnectedConstraint> &constraints =
        inHandle->mScene->GetConstraints();
    const JPH::TwoBodyConstraintSettings *target =
        constraints[inConstraintIndex].mSettings.GetPtr();
    if (target == nullptr)
        return false;
    for (std::uint32_t index = 0; index < constraints.size(); ++index)
        if (index != inConstraintIndex &&
            constraints[index].mSettings.GetPtr() == target)
            return false;
    JPH::TwoBodyConstraintSettings *settings =
        const_cast<JPH::TwoBodyConstraintSettings *>(target);
    settings->mEnabled = inEnabled;
    settings->mConstraintPriority = inPriority;
    settings->mNumVelocityStepsOverride = inNumVelocityStepsOverride;
    settings->mNumPositionStepsOverride = inNumPositionStepsOverride;
    settings->mDrawConstraintSize = inDrawConstraintSize;
    settings->mUserData = inUserData;
    return true;
}

template <class SettingsType>
inline SettingsType *GetUniquePhysicsSceneConstraintSettings(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    const JPH::RTTI *inExpectedType)
{
    if (inHandle == nullptr || inHandle->mScene == nullptr ||
        inConstraintIndex >= inHandle->mScene->GetNumConstraints())
        return nullptr;
    JPH::Array<JPH::PhysicsScene::ConnectedConstraint> &constraints =
        inHandle->mScene->GetConstraints();
    const JPH::TwoBodyConstraintSettings *target =
        constraints[inConstraintIndex].mSettings.GetPtr();
    if (target == nullptr || !JPH::IsType(target, inExpectedType))
        return nullptr;
    for (std::uint32_t index = 0; index < constraints.size(); ++index)
        if (index != inConstraintIndex &&
            constraints[index].mSettings.GetPtr() == target)
            return nullptr;
    return const_cast<SettingsType *>(
        static_cast<const SettingsType *>(target));
}

inline bool ConfigurePhysicsSceneHingeTuning(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    float inMaxFrictionTorque, std::uint8_t inSpringMode,
    float inSpringValue, float inSpringDamping)
{
    JPH::HingeConstraintSettings *settings =
        GetUniquePhysicsSceneConstraintSettings<JPH::HingeConstraintSettings>(
            inHandle, inConstraintIndex,
            JPH_RTTI(JPH::HingeConstraintSettings));
    if (settings == nullptr)
        return false;
    settings->mMaxFrictionTorque = inMaxFrictionTorque;
    settings->mLimitsSpringSettings = JPH::SpringSettings(
        static_cast<JPH::ESpringMode>(inSpringMode),
        inSpringValue, inSpringDamping);
    return true;
}

inline bool ConfigurePhysicsSceneDistanceSpring(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    std::uint8_t inSpringMode, float inSpringValue,
    float inSpringDamping)
{
    JPH::DistanceConstraintSettings *settings =
        GetUniquePhysicsSceneConstraintSettings<
            JPH::DistanceConstraintSettings>(
                inHandle, inConstraintIndex,
                JPH_RTTI(JPH::DistanceConstraintSettings));
    if (settings == nullptr)
        return false;
    settings->mLimitsSpringSettings = JPH::SpringSettings(
        static_cast<JPH::ESpringMode>(inSpringMode),
        inSpringValue, inSpringDamping);
    return true;
}

inline bool ConfigurePhysicsSceneSliderTuning(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    float inMaxFrictionForce, std::uint8_t inSpringMode,
    float inSpringValue, float inSpringDamping)
{
    JPH::SliderConstraintSettings *settings =
        GetUniquePhysicsSceneConstraintSettings<JPH::SliderConstraintSettings>(
            inHandle, inConstraintIndex,
            JPH_RTTI(JPH::SliderConstraintSettings));
    if (settings == nullptr)
        return false;
    settings->mMaxFrictionForce = inMaxFrictionForce;
    settings->mLimitsSpringSettings = JPH::SpringSettings(
        static_cast<JPH::ESpringMode>(inSpringMode),
        inSpringValue, inSpringDamping);
    return true;
}

inline bool SetPhysicsSceneSwingTwistFriction(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    float inMaxFrictionTorque)
{
    JPH::SwingTwistConstraintSettings *settings =
        GetUniquePhysicsSceneConstraintSettings<
            JPH::SwingTwistConstraintSettings>(
                inHandle, inConstraintIndex,
                JPH_RTTI(JPH::SwingTwistConstraintSettings));
    if (settings == nullptr)
        return false;
    settings->mMaxFrictionTorque = inMaxFrictionTorque;
    return true;
}

inline bool SetPhysicsSceneSixDOFFriction(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    std::uint8_t inAxis, float inMaxFriction)
{
    JPH::SixDOFConstraintSettings *settings =
        GetUniquePhysicsSceneConstraintSettings<JPH::SixDOFConstraintSettings>(
            inHandle, inConstraintIndex,
            JPH_RTTI(JPH::SixDOFConstraintSettings));
    if (settings == nullptr ||
        inAxis >= static_cast<std::uint8_t>(
            JPH::SixDOFConstraintSettings::EAxis::Num))
        return false;
    settings->mMaxFriction[inAxis] = inMaxFriction;
    return true;
}

inline bool SetPhysicsSceneSixDOFTranslationSpring(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    std::uint8_t inAxis, std::uint8_t inSpringMode,
    float inSpringValue, float inSpringDamping)
{
    JPH::SixDOFConstraintSettings *settings =
        GetUniquePhysicsSceneConstraintSettings<JPH::SixDOFConstraintSettings>(
            inHandle, inConstraintIndex,
            JPH_RTTI(JPH::SixDOFConstraintSettings));
    if (settings == nullptr ||
        inAxis >= static_cast<std::uint8_t>(
            JPH::SixDOFConstraintSettings::EAxis::NumTranslation))
        return false;
    settings->mLimitsSpringSettings[inAxis] = JPH::SpringSettings(
        static_cast<JPH::ESpringMode>(inSpringMode),
        inSpringValue, inSpringDamping);
    return true;
}

inline void SetPhysicsSceneMotorSettings(
    JPH::MotorSettings &ioSettings, std::uint8_t inSpringMode,
    float inSpringValue, float inSpringDamping,
    float inMinForce, float inMaxForce,
    float inMinTorque, float inMaxTorque)
{
    ioSettings.mSpringSettings = JPH::SpringSettings(
        static_cast<JPH::ESpringMode>(inSpringMode),
        inSpringValue, inSpringDamping);
    ioSettings.mMinForceLimit = inMinForce;
    ioSettings.mMaxForceLimit = inMaxForce;
    ioSettings.mMinTorqueLimit = inMinTorque;
    ioSettings.mMaxTorqueLimit = inMaxTorque;
}

inline bool ConfigurePhysicsSceneMotor(
    PhysicsSceneHandle *inHandle, std::uint32_t inConstraintIndex,
    std::uint8_t inMotorKind, std::uint8_t inAxis,
    std::uint8_t inSpringMode, float inSpringValue,
    float inSpringDamping, float inMinForce, float inMaxForce,
    float inMinTorque, float inMaxTorque)
{
    JPH::MotorSettings *motor = nullptr;
    switch (inMotorKind)
    {
    case 0:
    {
        auto *settings = GetUniquePhysicsSceneConstraintSettings<
            JPH::HingeConstraintSettings>(
                inHandle, inConstraintIndex,
                JPH_RTTI(JPH::HingeConstraintSettings));
        if (settings != nullptr)
            motor = &settings->mMotorSettings;
        break;
    }
    case 1:
    {
        auto *settings = GetUniquePhysicsSceneConstraintSettings<
            JPH::SliderConstraintSettings>(
                inHandle, inConstraintIndex,
                JPH_RTTI(JPH::SliderConstraintSettings));
        if (settings != nullptr)
            motor = &settings->mMotorSettings;
        break;
    }
    case 2:
    case 3:
    {
        auto *settings = GetUniquePhysicsSceneConstraintSettings<
            JPH::SwingTwistConstraintSettings>(
                inHandle, inConstraintIndex,
                JPH_RTTI(JPH::SwingTwistConstraintSettings));
        if (settings != nullptr)
            motor = inMotorKind == 2?
                &settings->mSwingMotorSettings :
                &settings->mTwistMotorSettings;
        break;
    }
    case 4:
    {
        auto *settings = GetUniquePhysicsSceneConstraintSettings<
            JPH::SixDOFConstraintSettings>(
                inHandle, inConstraintIndex,
                JPH_RTTI(JPH::SixDOFConstraintSettings));
        if (settings != nullptr &&
            inAxis < static_cast<std::uint8_t>(
                JPH::SixDOFConstraintSettings::EAxis::Num))
            motor = &settings->mMotorSettings[inAxis];
        break;
    }
    case 5:
    {
        auto *settings = GetUniquePhysicsSceneConstraintSettings<
            JPH::PathConstraintSettings>(
                inHandle, inConstraintIndex,
                JPH_RTTI(JPH::PathConstraintSettings));
        if (settings != nullptr)
            motor = &settings->mPositionMotorSettings;
        break;
    }
    default:
        break;
    }
    if (motor == nullptr)
        return false;
    SetPhysicsSceneMotorSettings(
        *motor, inSpringMode, inSpringValue, inSpringDamping,
        inMinForce, inMaxForce, inMinTorque, inMaxTorque);
    return true;
}

inline bool AddPhysicsScenePointConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2)
{
    JPH::Ref<JPH::PointConstraintSettings> settings =
        new JPH::PointConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mPoint1 = inPoint1;
    settings->mPoint2 = inPoint2;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneDistanceConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, float inMinDistance, float inMaxDistance)
{
    JPH::Ref<JPH::DistanceConstraintSettings> settings =
        new JPH::DistanceConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mPoint1 = inPoint1;
    settings->mPoint2 = inPoint2;
    settings->mMinDistance = inMinDistance;
    settings->mMaxDistance = inMaxDistance;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneFixedConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2)
{
    JPH::Ref<JPH::FixedConstraintSettings> settings =
        new JPH::FixedConstraintSettings();
    settings->mAutoDetectPoint = true;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneFixedConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, JPH::Vec3Arg inAxisX1,
    JPH::Vec3Arg inAxisY1, JPH::Vec3Arg inAxisX2,
    JPH::Vec3Arg inAxisY2)
{
    JPH::Ref<JPH::FixedConstraintSettings> settings =
        new JPH::FixedConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mAutoDetectPoint = false;
    settings->mPoint1 = inPoint1;
    settings->mPoint2 = inPoint2;
    settings->mAxisX1 = inAxisX1;
    settings->mAxisY1 = inAxisY1;
    settings->mAxisX2 = inAxisX2;
    settings->mAxisY2 = inAxisY2;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneHingeConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, JPH::Vec3Arg inAxis1,
    JPH::Vec3Arg inAxis2, float inMinAngle, float inMaxAngle)
{
    JPH::Ref<JPH::HingeConstraintSettings> settings =
        new JPH::HingeConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mPoint1 = inPoint1;
    settings->mPoint2 = inPoint2;
    settings->mHingeAxis1 = inAxis1;
    settings->mHingeAxis2 = inAxis2;
    settings->mNormalAxis1 = inAxis1.GetNormalizedPerpendicular();
    settings->mNormalAxis2 = inAxis2.GetNormalizedPerpendicular();
    settings->mLimitsMin = inMinAngle;
    settings->mLimitsMax = inMaxAngle;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneSliderConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, JPH::Vec3Arg inAxis,
    float inMinPosition, float inMaxPosition)
{
    JPH::Ref<JPH::SliderConstraintSettings> settings =
        new JPH::SliderConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mPoint1 = inPoint1;
    settings->mPoint2 = inPoint2;
    settings->SetSliderAxis(inAxis);
    settings->mLimitsMin = inMinPosition;
    settings->mLimitsMax = inMaxPosition;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneConeConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, JPH::Vec3Arg inAxis1,
    JPH::Vec3Arg inAxis2, float inHalfConeAngle)
{
    JPH::Ref<JPH::ConeConstraintSettings> settings =
        new JPH::ConeConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mPoint1 = inPoint1;
    settings->mPoint2 = inPoint2;
    settings->mTwistAxis1 = inAxis1;
    settings->mTwistAxis2 = inAxis2;
    settings->mHalfConeAngle = inHalfConeAngle;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneSwingTwistConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, JPH::Vec3Arg inTwistAxis,
    JPH::Vec3Arg inPlaneAxis, float inNormalHalfConeAngle,
    float inPlaneHalfConeAngle, float inTwistMinAngle,
    float inTwistMaxAngle)
{
    JPH::Ref<JPH::SwingTwistConstraintSettings> settings =
        new JPH::SwingTwistConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mPosition1 = inPoint1;
    settings->mPosition2 = inPoint2;
    settings->mTwistAxis1 = settings->mTwistAxis2 = inTwistAxis;
    settings->mPlaneAxis1 = settings->mPlaneAxis2 = inPlaneAxis;
    settings->mNormalHalfConeAngle = inNormalHalfConeAngle;
    settings->mPlaneHalfConeAngle = inPlaneHalfConeAngle;
    settings->mTwistMinAngle = inTwistMinAngle;
    settings->mTwistMaxAngle = inTwistMaxAngle;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneSixDOFConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, JPH::Vec3Arg inAxisX,
    JPH::Vec3Arg inAxisY, std::uint8_t inSwingType,
    const float *inLimitMin,
    const float *inLimitMax)
{
    JPH::Ref<JPH::SixDOFConstraintSettings> settings =
        new JPH::SixDOFConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mPosition1 = inPoint1;
    settings->mPosition2 = inPoint2;
    settings->mAxisX1 = settings->mAxisX2 = inAxisX;
    settings->mAxisY1 = settings->mAxisY2 = inAxisY;
    settings->mSwingType = static_cast<JPH::ESwingType>(inSwingType);
    for (int axis = 0;
         axis < JPH::SixDOFConstraintSettings::EAxis::Num; ++axis)
    {
        settings->mLimitMin[axis] = inLimitMin[axis];
        settings->mLimitMax[axis] = inLimitMax[axis];
    }
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneGearConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::Vec3Arg inAxis1,
    JPH::Vec3Arg inAxis2, float inRatio)
{
    if (inHandle == nullptr || inHandle->mScene == nullptr ||
        inBody1 >= inHandle->mScene->GetNumBodies() ||
        inBody2 >= inHandle->mScene->GetNumBodies() ||
        inHandle->mScene->GetBodies()[inBody1].mMotionType !=
            JPH::EMotionType::Dynamic ||
        inHandle->mScene->GetBodies()[inBody2].mMotionType !=
            JPH::EMotionType::Dynamic)
        return false;
    JPH::Ref<JPH::GearConstraintSettings> settings =
        new JPH::GearConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mHingeAxis1 = inAxis1;
    settings->mHingeAxis2 = inAxis2;
    settings->mRatio = inRatio;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsScenePulleyConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inBody1,
    std::uint32_t inBody2, JPH::RVec3Arg inBodyPoint1,
    JPH::RVec3Arg inFixedPoint1, JPH::RVec3Arg inBodyPoint2,
    JPH::RVec3Arg inFixedPoint2, float inRatio,
    float inMinLength, float inMaxLength)
{
    JPH::Ref<JPH::PulleyConstraintSettings> settings =
        new JPH::PulleyConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::WorldSpace;
    settings->mBodyPoint1 = inBodyPoint1;
    settings->mFixedPoint1 = inFixedPoint1;
    settings->mBodyPoint2 = inBodyPoint2;
    settings->mFixedPoint2 = inFixedPoint2;
    settings->mRatio = inRatio;
    settings->mMinLength = inMinLength;
    settings->mMaxLength = inMaxLength;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inBody1, inBody2);
}

inline bool AddPhysicsSceneRackAndPinionConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inPinionBody,
    std::uint32_t inRackBody, JPH::Vec3Arg inHingeAxis,
    JPH::Vec3Arg inSliderAxis, float inRatio)
{
    if (inHandle == nullptr || inHandle->mScene == nullptr ||
        inPinionBody >= inHandle->mScene->GetNumBodies() ||
        inRackBody >= inHandle->mScene->GetNumBodies() ||
        inHandle->mScene->GetBodies()[inPinionBody].mMotionType !=
            JPH::EMotionType::Dynamic ||
        inHandle->mScene->GetBodies()[inRackBody].mMotionType !=
            JPH::EMotionType::Dynamic)
        return false;
    JPH::Ref<JPH::RackAndPinionConstraintSettings> settings =
        new JPH::RackAndPinionConstraintSettings();
    settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
    settings->mHingeAxis = inHingeAxis;
    settings->mSliderAxis = inSliderAxis;
    settings->mRatio = inRatio;
    return AddPhysicsSceneConstraintSettings(
        inHandle, settings.GetPtr(), inPinionBody, inRackBody);
}

inline bool AddPhysicsScenePathConstraint(
    PhysicsSceneHandle *inHandle, std::uint32_t inPathBody,
    std::uint32_t inMovingBody, const JPH::Vec3 *inPositions,
    const JPH::Vec3 *inTangents, const JPH::Vec3 *inNormals,
    std::uint32_t inPointCount, bool inLooping,
    JPH::Vec3Arg inPathPosition, JPH::QuatArg inPathRotation,
    float inPathFraction, float inMaxFrictionForce,
    std::uint8_t inRotationConstraintType)
{
    if (inPositions == nullptr || inTangents == nullptr ||
        inNormals == nullptr || inPointCount < (inLooping? 3u : 2u))
        return false;
    try
    {
        JPH::Ref<JPH::PathConstraintPathHermite> path =
            new JPH::PathConstraintPathHermite();
        for (std::uint32_t point = 0; point < inPointCount; ++point)
            path->AddPoint(
                inPositions[point], inTangents[point], inNormals[point]);
        path->SetIsLooping(inLooping);

        JPH::Ref<JPH::PathConstraintSettings> settings =
            new JPH::PathConstraintSettings();
        settings->mPath = path;
        settings->mPathPosition = inPathPosition;
        settings->mPathRotation = inPathRotation;
        settings->mPathFraction = inPathFraction;
        settings->mMaxFrictionForce = inMaxFrictionForce;
        settings->mRotationConstraintType =
            static_cast<JPH::EPathRotationConstraintType>(
                inRotationConstraintType);
        return AddPhysicsSceneConstraintSettings(
            inHandle, settings.GetPtr(), inPathBody, inMovingBody);
    }
    catch (...)
    {
        return false;
    }
}

inline std::uint32_t AddPhysicsSceneBody(
    PhysicsSceneHandle *inHandle,
    const JPH::BodyCreationSettings &inSettings)
{
    if (inHandle == nullptr || inHandle->mScene == nullptr ||
        inSettings.GetShapeSettings() == nullptr)
        return std::numeric_limits<std::uint32_t>::max();
    try
    {
        const std::size_t index = inHandle->mScene->GetNumBodies();
        if (index >= std::numeric_limits<std::uint32_t>::max())
            return std::numeric_limits<std::uint32_t>::max();
        inHandle->mScene->AddBody(inSettings);
        return static_cast<std::uint32_t>(index);
    }
    catch (...)
    {
        return std::numeric_limits<std::uint32_t>::max();
    }
}

inline PhysicsSceneHandle *CapturePhysicsScene(
    const JPH::PhysicsSystem *inSystem)
{
    if (inSystem == nullptr)
        return nullptr;
    try
    {
        std::unique_ptr<PhysicsSceneHandle> handle(
            new PhysicsSceneHandle());
        handle->mScene = new JPH::PhysicsScene();
        handle->mScene->FromPhysicsSystem(inSystem);
        return handle.release();
    }
    catch (...)
    {
        return nullptr;
    }
}

inline PhysicsSceneHandle *RestorePhysicsScene(
    const std::uint8_t *inData, std::size_t inSize)
{
    if (inData == nullptr || inSize == 0)
        return nullptr;
    try
    {
        const std::string data(
            reinterpret_cast<const char *>(inData), inSize);
        std::istringstream stream(data, std::ios::in | std::ios::binary);
        JPH::Ref<JPH::PhysicsScene> scene;
        JPH::StreamInWrapper wrapper(stream);
        JPH::PhysicsScene::PhysicsSceneResult restored =
            JPH::PhysicsScene::sRestoreFromBinaryState(wrapper);
        if (restored.HasError() || wrapper.IsFailed())
            return nullptr;
        scene = restored.Get();
        if (scene == nullptr)
            return nullptr;
        std::unordered_set<JPH::SoftBodySharedSettings *> optimized_settings;
        for (JPH::SoftBodyCreationSettings &soft_body : scene->GetSoftBodies())
        {
            JPH::SoftBodySharedSettings *settings =
                const_cast<JPH::SoftBodySharedSettings *>(
                    soft_body.mSettings.GetPtr());
            if (settings != nullptr && optimized_settings.insert(settings).second)
                settings->Optimize();
        }
        PhysicsSceneHandle *handle = new PhysicsSceneHandle();
        handle->mScene = scene;
        return handle;
    }
    catch (...)
    {
        return nullptr;
    }
}

inline PhysicsSceneHandle *RestorePhysicsSceneObjectStream(
    const std::uint8_t *inData, std::size_t inSize)
{
#ifdef JPH_OBJECT_STREAM
    if (inData == nullptr || inSize == 0)
        return nullptr;
    try
    {
        const std::string data(
            reinterpret_cast<const char *>(inData), inSize);
        std::istringstream stream(data, std::ios::in | std::ios::binary);
        JPH::Ref<JPH::PhysicsScene> scene;
        if (!JPH::ObjectStreamIn::sReadObject(stream, scene) ||
            scene == nullptr)
            return nullptr;
        std::unordered_set<JPH::SoftBodySharedSettings *> optimizedSettings;
        for (JPH::SoftBodyCreationSettings &softBody : scene->GetSoftBodies())
        {
            JPH::SoftBodySharedSettings *settings =
                const_cast<JPH::SoftBodySharedSettings *>(
                    softBody.mSettings.GetPtr());
            if (settings != nullptr && optimizedSettings.insert(settings).second)
                settings->Optimize();
        }
        PhysicsSceneHandle *handle = new PhysicsSceneHandle();
        handle->mScene = scene;
        return handle;
    }
    catch (...)
    {
        return nullptr;
    }
#else
    (void)inData;
    (void)inSize;
    return nullptr;
#endif
}

inline void DestroyPhysicsScene(PhysicsSceneHandle *inHandle)
{
    delete inHandle;
}

inline std::uint32_t GetPhysicsSceneBodyCount(
    const PhysicsSceneHandle *inHandle)
{
    return static_cast<std::uint32_t>(inHandle->mScene->GetNumBodies());
}

inline std::uint32_t GetPhysicsSceneConstraintCount(
    const PhysicsSceneHandle *inHandle)
{
    return static_cast<std::uint32_t>(inHandle->mScene->GetNumConstraints());
}

inline std::uint32_t GetPhysicsSceneSoftBodyCount(
    const PhysicsSceneHandle *inHandle)
{
    return static_cast<std::uint32_t>(inHandle->mScene->GetNumSoftBodies());
}

inline bool FixPhysicsSceneInvalidScales(PhysicsSceneHandle *inHandle)
{
    return inHandle->mScene->FixInvalidScales();
}

inline bool SerializePhysicsScene(
    PhysicsSceneHandle *inHandle)
{
    try
    {
        std::ostringstream stream(std::ios::out | std::ios::binary);
        JPH::StreamOutWrapper wrapper(stream);
        inHandle->mScene->SaveBinaryState(wrapper, true, true);
        if (wrapper.IsFailed())
        {
            inHandle->mSerialized.clear();
            return false;
        }
        inHandle->mSerialized = stream.str();
        return !inHandle->mSerialized.empty();
    }
    catch (...)
    {
        inHandle->mSerialized.clear();
        return false;
    }
}

inline bool IsPhysicsSceneObjectStreamSerializable(
    const PhysicsSceneHandle *inHandle)
{
#ifdef JPH_OBJECT_STREAM
    if (inHandle == nullptr || inHandle->mScene == nullptr)
        return false;
    for (const JPH::BodyCreationSettings &body :
         inHandle->mScene->GetBodies())
        if (body.GetShapeSettings() == nullptr)
            return false;
    return true;
#else
    (void)inHandle;
    return false;
#endif
}

inline bool SerializePhysicsSceneObjectStream(
    PhysicsSceneHandle *inHandle, bool inBinary)
{
#ifdef JPH_OBJECT_STREAM
    try
    {
        if (!IsPhysicsSceneObjectStreamSerializable(inHandle))
        {
            inHandle->mSerialized.clear();
            return false;
        }
        std::ostringstream stream(std::ios::out | std::ios::binary);
        const JPH::ObjectStream::EStreamType type = inBinary ?
            JPH::ObjectStream::EStreamType::Binary :
            JPH::ObjectStream::EStreamType::Text;
        if (!JPH::ObjectStreamOut::sWriteObject(
                stream, type, *inHandle->mScene))
        {
            inHandle->mSerialized.clear();
            return false;
        }
        inHandle->mSerialized = stream.str();
        return !inHandle->mSerialized.empty();
    }
    catch (...)
    {
        inHandle->mSerialized.clear();
        return false;
    }
#else
    (void)inHandle;
    (void)inBinary;
    return false;
#endif
}

inline std::size_t GetPhysicsSceneSerializedSize(
    const PhysicsSceneHandle *inHandle)
{
    return inHandle->mSerialized.size();
}

inline void CopyPhysicsSceneSerializedData(
    const PhysicsSceneHandle *inHandle, std::uint8_t *outData)
{
    if (!inHandle->mSerialized.empty())
        std::memcpy(
            outData, inHandle->mSerialized.data(),
            inHandle->mSerialized.size());
}

inline void RemovePhysicsSceneInstanceObjects(
    PhysicsSceneInstanceHandle *inHandle)
{
    if (!inHandle->mAdded || inHandle->mSystem == nullptr)
        return;
    for (JPH::Constraint *constraint : inHandle->mConstraints)
        inHandle->mSystem->RemoveConstraint(constraint);
    JPH::BodyInterface &body_interface =
        inHandle->mSystem->GetBodyInterface();
    for (JPH::BodyID id : inHandle->mBodies)
        body_interface.RemoveBody(id);
    for (JPH::BodyID id : inHandle->mBodies)
        body_interface.DestroyBody(id);
    inHandle->mAdded = false;
}

inline PhysicsSceneInstanceHandle *InstantiatePhysicsScene(
    const PhysicsSceneHandle *inScene, JPH::PhysicsSystem *ioSystem,
    std::uint32_t inLayerCount)
{
    const JPH::Array<JPH::BodyCreationSettings> &scene_bodies =
        inScene->mScene->GetBodies();
    for (const JPH::BodyCreationSettings &body : inScene->mScene->GetBodies())
        if (body.mObjectLayer >= inLayerCount)
            return nullptr;
    for (const JPH::SoftBodyCreationSettings &body :
         inScene->mScene->GetSoftBodies())
        if (body.mObjectLayer >= inLayerCount)
            return nullptr;
    for (const JPH::PhysicsScene::ConnectedConstraint &constraint :
         inScene->mScene->GetConstraints())
    {
        const bool body1_valid =
            constraint.mBody1 == JPH::PhysicsScene::cFixedToWorld ||
            constraint.mBody1 < scene_bodies.size();
        const bool body2_valid =
            constraint.mBody2 == JPH::PhysicsScene::cFixedToWorld ||
            constraint.mBody2 < scene_bodies.size();
        if (!body1_valid || !body2_valid || constraint.mSettings == nullptr)
            return nullptr;
        const bool requires_dynamic_bodies =
            JPH::IsType(
                constraint.mSettings,
                JPH_RTTI(JPH::GearConstraintSettings)) ||
            JPH::IsType(
                constraint.mSettings,
                JPH_RTTI(JPH::RackAndPinionConstraintSettings));
        if (requires_dynamic_bodies &&
            (constraint.mBody1 == JPH::PhysicsScene::cFixedToWorld ||
             constraint.mBody2 == JPH::PhysicsScene::cFixedToWorld ||
             scene_bodies[constraint.mBody1].mMotionType !=
                 JPH::EMotionType::Dynamic ||
             scene_bodies[constraint.mBody2].mMotionType !=
                 JPH::EMotionType::Dynamic))
            return nullptr;
    }

    std::unique_ptr<PhysicsSceneInstanceHandle> instance(
        new PhysicsSceneInstanceHandle());
    instance->mSystem = ioSystem;
    JPH::BodyInterface &body_interface = ioSystem->GetBodyInterface();
    try
    {
        const std::size_t expected_body_count =
            scene_bodies.size() + inScene->mScene->GetNumSoftBodies();
        instance->mBodies.reserve(expected_body_count);
        instance->mConstraints.reserve(
            inScene->mScene->GetNumConstraints());
        for (const JPH::BodyCreationSettings &settings : scene_bodies)
        {
            const JPH::Body *body = body_interface.CreateBody(settings);
            if (body == nullptr)
                break;
            instance->mBodies.push_back(body->GetID());
        }
        for (const JPH::SoftBodyCreationSettings &settings :
             inScene->mScene->GetSoftBodies())
        {
            const JPH::Body *body = body_interface.CreateSoftBody(settings);
            if (body == nullptr)
                break;
            instance->mBodies.push_back(body->GetID());
        }
        if (instance->mBodies.size() != expected_body_count)
        {
            for (JPH::BodyID id : instance->mBodies)
                body_interface.DestroyBody(id);
            return nullptr;
        }

        JPH::BodyIDVector added_bodies = instance->mBodies;
        JPH::BodyInterface::AddState add_state =
            body_interface.AddBodiesPrepare(
                added_bodies.data(), static_cast<int>(added_bodies.size()));
        body_interface.AddBodiesFinalize(
            added_bodies.data(), static_cast<int>(added_bodies.size()),
            add_state, JPH::EActivation::Activate);
        instance->mAdded = true;

        for (const JPH::PhysicsScene::ConnectedConstraint &connected :
             inScene->mScene->GetConstraints())
        {
            const JPH::BodyID body1 =
                connected.mBody1 == JPH::PhysicsScene::cFixedToWorld ?
                JPH::BodyID() : instance->mBodies[connected.mBody1];
            const JPH::BodyID body2 =
                connected.mBody2 == JPH::PhysicsScene::cFixedToWorld ?
                JPH::BodyID() : instance->mBodies[connected.mBody2];
            JPH::Constraint *constraint = body_interface.CreateConstraint(
                connected.mSettings, body1, body2);
            if (constraint == nullptr)
            {
                RemovePhysicsSceneInstanceObjects(instance.get());
                return nullptr;
            }
            ioSystem->AddConstraint(constraint);
            instance->mConstraints.push_back(constraint);
        }
        return instance.release();
    }
    catch (...)
    {
        if (instance->mAdded)
            RemovePhysicsSceneInstanceObjects(instance.get());
        else
            for (JPH::BodyID id : instance->mBodies)
                body_interface.DestroyBody(id);
        return nullptr;
    }
}

inline void DestroyPhysicsSceneInstance(
    PhysicsSceneInstanceHandle *inHandle)
{
    if (inHandle == nullptr)
        return;
    RemovePhysicsSceneInstanceObjects(inHandle);
    delete inHandle;
}

inline void AbandonPhysicsSceneInstance(
    PhysicsSceneInstanceHandle *inHandle)
{
    if (inHandle == nullptr)
        return;
    inHandle->mAdded = false;
    inHandle->mSystem = nullptr;
    delete inHandle;
}

inline std::uint32_t GetPhysicsSceneInstanceBodyCount(
    const PhysicsSceneInstanceHandle *inHandle)
{
    return static_cast<std::uint32_t>(inHandle->mBodies.size());
}

inline std::uint32_t GetPhysicsSceneInstanceConstraintCount(
    const PhysicsSceneInstanceHandle *inHandle)
{
    return static_cast<std::uint32_t>(inHandle->mConstraints.size());
}

inline std::uint32_t GetPhysicsSceneInstanceBodyID(
    const PhysicsSceneInstanceHandle *inHandle, std::uint32_t inIndex)
{
    return inHandle->mBodies[inIndex].GetIndexAndSequenceNumber();
}

inline JPH::Constraint *GetPhysicsSceneInstanceConstraint(
    const PhysicsSceneInstanceHandle *inHandle, std::uint32_t inIndex)
{
    return inHandle->mConstraints[inIndex];
}

inline bool IsPhysicsSceneInstanceSoftBody(
    const PhysicsSceneInstanceHandle *inHandle, std::uint32_t inIndex)
{
    JPH::BodyLockRead lock(
        inHandle->mSystem->GetBodyLockInterface(),
        inHandle->mBodies[inIndex]);
    return lock.Succeeded() && !lock.GetBody().IsRigidBody();
}

inline std::uint8_t GetPhysicsSceneInstanceMotionType(
    const PhysicsSceneInstanceHandle *inHandle, std::uint32_t inIndex)
{
    JPH::BodyLockRead lock(
        inHandle->mSystem->GetBodyLockInterface(),
        inHandle->mBodies[inIndex]);
    return lock.Succeeded() ?
        static_cast<std::uint8_t>(lock.GetBody().GetMotionType()) : 0xff;
}

enum class CharacterContactEventKind : std::uint8_t
{
    BodyAdded,
    BodyPersisted,
    BodyRemoved,
    BodySolved,
    CharacterAdded,
    CharacterPersisted,
    CharacterRemoved,
    CharacterSolved
};

struct CharacterContactEvent
{
    CharacterContactEventKind mKind = CharacterContactEventKind::BodyAdded;
    std::uint32_t mBodyID = JPH::BodyID::cInvalidBodyID;
    std::uint32_t mCharacterID = ~std::uint32_t(0);
    std::uint32_t mSubShapeID = 0;
    JPH::Vec3 mPosition = JPH::Vec3::sZero();
    JPH::Vec3 mNormal = JPH::Vec3::sZero();
    JPH::Vec3 mContactVelocity = JPH::Vec3::sZero();
    JPH::Vec3 mCharacterVelocity = JPH::Vec3::sZero();
    JPH::Vec3 mResultingVelocity = JPH::Vec3::sZero();
    std::uint64_t mUserData = 0;
    bool mIsSensor = false;
    bool mCanPushCharacter = true;
    bool mCanReceiveImpulses = true;
};

class CharacterContactBridge final : public JPH::CharacterContactListener
{
public:
    void Configure(
        std::uint32_t inCapacity, bool inCanPushCharacter,
        bool inCanReceiveImpulses, bool inPreventSliding)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mCapacity = inCapacity == 0 ? 1 : inCapacity;
        mCanPushCharacter = inCanPushCharacter;
        mCanReceiveImpulses = inCanReceiveImpulses;
        mPreventSliding = inPreventSliding;
        while (mEvents.size() > mCapacity)
        {
            mEvents.pop_front();
            ++mDroppedCount;
        }
    }

    void OnContactAdded(
        const JPH::CharacterVirtual *, const JPH::CharacterContact &inContact,
        JPH::CharacterContactSettings &ioSettings) override
    {
        ApplySettings(ioSettings);
        PushContact(CharacterContactEventKind::BodyAdded, inContact, ioSettings);
    }

    void OnContactPersisted(
        const JPH::CharacterVirtual *, const JPH::CharacterContact &inContact,
        JPH::CharacterContactSettings &ioSettings) override
    {
        ApplySettings(ioSettings);
        PushContact(CharacterContactEventKind::BodyPersisted, inContact, ioSettings);
    }

    void OnContactRemoved(
        const JPH::CharacterVirtual *, const JPH::BodyID &inBodyID,
        const JPH::SubShapeID &inSubShapeID) override
    {
        CharacterContactEvent event;
        event.mKind = CharacterContactEventKind::BodyRemoved;
        event.mBodyID = inBodyID.GetIndexAndSequenceNumber();
        event.mSubShapeID = inSubShapeID.GetValue();
        Push(event);
    }

    void OnCharacterContactAdded(
        const JPH::CharacterVirtual *, const JPH::CharacterContact &inContact,
        JPH::CharacterContactSettings &ioSettings) override
    {
        ApplySettings(ioSettings);
        PushContact(CharacterContactEventKind::CharacterAdded, inContact, ioSettings);
    }

    void OnCharacterContactPersisted(
        const JPH::CharacterVirtual *, const JPH::CharacterContact &inContact,
        JPH::CharacterContactSettings &ioSettings) override
    {
        ApplySettings(ioSettings);
        PushContact(
            CharacterContactEventKind::CharacterPersisted, inContact, ioSettings);
    }

    void OnCharacterContactRemoved(
        const JPH::CharacterVirtual *, const JPH::CharacterID &inCharacterID,
        const JPH::SubShapeID &inSubShapeID) override
    {
        CharacterContactEvent event;
        event.mKind = CharacterContactEventKind::CharacterRemoved;
        event.mCharacterID = inCharacterID.GetValue();
        event.mSubShapeID = inSubShapeID.GetValue();
        Push(event);
    }

    void OnContactSolve(
        const JPH::CharacterVirtual *inCharacter,
        const JPH::BodyID &inBodyID,
        const JPH::SubShapeID &inSubShapeID,
        JPH::RVec3Arg inPosition,
        JPH::Vec3Arg inNormal,
        JPH::Vec3Arg inContactVelocity,
        const JPH::PhysicsMaterial *,
        JPH::Vec3Arg inCharacterVelocity,
        JPH::Vec3 &ioNewCharacterVelocity) override
    {
        PreventSliding(inCharacter, inNormal, inContactVelocity,
            inCharacterVelocity, ioNewCharacterVelocity);
        PushSolve(CharacterContactEventKind::BodySolved,
            inBodyID.GetIndexAndSequenceNumber(), ~std::uint32_t(0),
            inSubShapeID.GetValue(), inPosition, inNormal, inContactVelocity,
            inCharacterVelocity, ioNewCharacterVelocity);
    }

    void OnCharacterContactSolve(
        const JPH::CharacterVirtual *inCharacter,
        const JPH::CharacterVirtual *inOtherCharacter,
        const JPH::SubShapeID &inSubShapeID,
        JPH::RVec3Arg inPosition,
        JPH::Vec3Arg inNormal,
        JPH::Vec3Arg inContactVelocity,
        const JPH::PhysicsMaterial *,
        JPH::Vec3Arg inCharacterVelocity,
        JPH::Vec3 &ioNewCharacterVelocity) override
    {
        PreventSliding(inCharacter, inNormal, inContactVelocity,
            inCharacterVelocity, ioNewCharacterVelocity);
        PushSolve(CharacterContactEventKind::CharacterSolved,
            JPH::BodyID::cInvalidBodyID, inOtherCharacter->GetID().GetValue(),
            inSubShapeID.GetValue(), inPosition, inNormal, inContactVelocity,
            inCharacterVelocity, ioNewCharacterVelocity);
    }

    bool Pop(CharacterContactEvent &outEvent)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mEvents.empty())
            return false;
        outEvent = mEvents.front();
        mEvents.pop_front();
        return true;
    }

    std::uint32_t PendingCount() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return static_cast<std::uint32_t>(mEvents.size());
    }

    std::uint64_t DroppedCount(bool inReset)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        const std::uint64_t result = mDroppedCount;
        if (inReset)
            mDroppedCount = 0;
        return result;
    }

    void Clear()
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mEvents.clear();
        mDroppedCount = 0;
    }

private:
    void ApplySettings(JPH::CharacterContactSettings &ioSettings) const
    {
        ioSettings.mCanPushCharacter = mCanPushCharacter.load();
        ioSettings.mCanReceiveImpulses = mCanReceiveImpulses.load();
    }

    void PreventSliding(
        const JPH::CharacterVirtual *inCharacter,
        JPH::Vec3Arg inNormal,
        JPH::Vec3Arg inContactVelocity,
        JPH::Vec3Arg inCharacterVelocity,
        JPH::Vec3 &ioNewCharacterVelocity) const
    {
        if (!mPreventSliding.load() || !inContactVelocity.IsNearZero() ||
            inCharacter->IsSlopeTooSteep(inNormal))
            return;
        const JPH::Vec3 up = inCharacter->GetUp();
        const JPH::Vec3 horizontal_velocity =
            inCharacterVelocity - up * inCharacterVelocity.Dot(up);
        if (horizontal_velocity.IsNearZero())
            ioNewCharacterVelocity = JPH::Vec3::sZero();
    }

    void PushContact(
        CharacterContactEventKind inKind,
        const JPH::CharacterContact &inContact,
        const JPH::CharacterContactSettings &inSettings)
    {
        CharacterContactEvent event;
        event.mKind = inKind;
        event.mBodyID = inContact.mBodyB.GetIndexAndSequenceNumber();
        event.mCharacterID = inContact.mCharacterIDB.GetValue();
        event.mSubShapeID = inContact.mSubShapeIDB.GetValue();
        event.mPosition = JPH::Vec3(inContact.mPosition);
        event.mNormal = inContact.mContactNormal;
        event.mContactVelocity = inContact.mLinearVelocity;
        event.mUserData = inContact.mUserData;
        event.mIsSensor = inContact.mIsSensorB;
        event.mCanPushCharacter = inSettings.mCanPushCharacter;
        event.mCanReceiveImpulses = inSettings.mCanReceiveImpulses;
        Push(event);
    }

    void PushSolve(
        CharacterContactEventKind inKind,
        std::uint32_t inBodyID,
        std::uint32_t inCharacterID,
        std::uint32_t inSubShapeID,
        JPH::RVec3Arg inPosition,
        JPH::Vec3Arg inNormal,
        JPH::Vec3Arg inContactVelocity,
        JPH::Vec3Arg inCharacterVelocity,
        JPH::Vec3Arg inResultingVelocity)
    {
        CharacterContactEvent event;
        event.mKind = inKind;
        event.mBodyID = inBodyID;
        event.mCharacterID = inCharacterID;
        event.mSubShapeID = inSubShapeID;
        event.mPosition = JPH::Vec3(inPosition);
        event.mNormal = inNormal;
        event.mContactVelocity = inContactVelocity;
        event.mCharacterVelocity = inCharacterVelocity;
        event.mResultingVelocity = inResultingVelocity;
        event.mCanPushCharacter = mCanPushCharacter.load();
        event.mCanReceiveImpulses = mCanReceiveImpulses.load();
        Push(event);
    }

    void Push(const CharacterContactEvent &inEvent)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mEvents.size() == mCapacity)
        {
            mEvents.pop_front();
            ++mDroppedCount;
        }
        mEvents.push_back(inEvent);
    }

    std::size_t mCapacity = 1024;
    std::atomic_bool mCanPushCharacter { true };
    std::atomic_bool mCanReceiveImpulses { true };
    std::atomic_bool mPreventSliding { false };
    mutable std::mutex mMutex;
    std::deque<CharacterContactEvent> mEvents;
    std::uint64_t mDroppedCount = 0;
};

struct CharacterBroadPhaseCell
{
    std::int64_t mX;
    std::int64_t mY;
    std::int64_t mZ;

    bool operator==(const CharacterBroadPhaseCell &inOther) const
    {
        return mX == inOther.mX && mY == inOther.mY && mZ == inOther.mZ;
    }
};

struct CharacterBroadPhaseCellHash
{
    std::size_t operator()(const CharacterBroadPhaseCell &inCell) const
    {
        std::size_t hash = std::hash<std::int64_t>()(inCell.mX);
        hash ^= std::hash<std::int64_t>()(inCell.mY) +
            0x9e3779b9U + (hash << 6) + (hash >> 2);
        hash ^= std::hash<std::int64_t>()(inCell.mZ) +
            0x9e3779b9U + (hash << 6) + (hash >> 2);
        return hash;
    }
};

class CharacterBroadPhase : public JPH::CharacterVsCharacterCollision
{
public:
    explicit CharacterBroadPhase(float inCellSize) : mCellSize(inCellSize) { }

    void Add(JPH::CharacterVirtual *inCharacter)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        Entry entry = MakeEntry(inCharacter);
        mEntries.emplace(inCharacter, entry);
        mCells[entry.mCell].push_back(inCharacter);
        mMaxExtent = std::max(mMaxExtent, entry.mExtent);
    }

    void Remove(const JPH::CharacterVirtual *inCharacter)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        auto entry = mEntries.find(inCharacter);
        if (entry == mEntries.end())
            return;
        RemoveFromCell(entry->second.mCell, inCharacter);
        const bool recompute_extent = entry->second.mExtent >= mMaxExtent;
        mEntries.erase(entry);
        if (recompute_extent)
            RecomputeMaxExtent();
    }

    void Update(JPH::CharacterVirtual *inCharacter)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        auto current = mEntries.find(inCharacter);
        if (current == mEntries.end())
        {
            Entry entry = MakeEntry(inCharacter);
            mEntries.emplace(inCharacter, entry);
            mCells[entry.mCell].push_back(inCharacter);
            mMaxExtent = std::max(mMaxExtent, entry.mExtent);
            return;
        }
        Entry updated = MakeEntry(inCharacter);
        if (!(updated.mCell == current->second.mCell))
        {
            RemoveFromCell(current->second.mCell, inCharacter);
            mCells[updated.mCell].push_back(inCharacter);
        }
        const bool recompute_extent =
            current->second.mExtent >= mMaxExtent &&
            updated.mExtent < current->second.mExtent;
        current->second = updated;
        if (recompute_extent)
            RecomputeMaxExtent();
        else
            mMaxExtent = std::max(mMaxExtent, updated.mExtent);
    }

    void CollideCharacter(
        const JPH::CharacterVirtual *inCharacter,
        JPH::RMat44Arg inCenterOfMassTransform,
        const JPH::CollideShapeSettings &inSettings,
        JPH::RVec3Arg inBaseOffset,
        JPH::CollideShapeCollector &ioCollector) const override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        JPH::Mat44 transform1 = inCenterOfMassTransform
            .PostTranslated(-inBaseOffset).ToMat44();
        const JPH::Shape *shape1 = inCharacter->GetShape();
        JPH::AABox query_bounds = shape1->GetWorldSpaceBounds(
            inCenterOfMassTransform.ToMat44(), JPH::Vec3::sOne());
        query_bounds.ExpandBy(JPH::Vec3::sReplicate(
            inSettings.mMaxSeparationDistance));
        JPH::CollideShapeSettings settings = inSettings;

        BeginQuery();
        ForCandidates(query_bounds, inCharacter,
            [&](const JPH::CharacterVirtual *candidate)
            {
                if (ioCollector.ShouldEarlyOut())
                    return false;
                JPH::Mat44 transform2 = candidate->GetCenterOfMassTransform()
                    .PostTranslated(-inBaseOffset).ToMat44();
                settings.mMaxSeparationDistance =
                    inSettings.mMaxSeparationDistance +
                    candidate->GetCharacterPadding();
                const JPH::Shape *shape2 = candidate->GetShape();
                JPH::AABox bounds1 = shape1->GetWorldSpaceBounds(
                    transform1, JPH::Vec3::sOne());
                JPH::AABox bounds2 = shape2->GetWorldSpaceBounds(
                    transform2, JPH::Vec3::sOne());
                bounds2.ExpandBy(JPH::Vec3::sReplicate(
                    settings.mMaxSeparationDistance));
                if (!bounds1.Overlaps(bounds2))
                    return true;
                CountNarrowPhaseTest();
                ioCollector.SetUserData(
                    reinterpret_cast<std::uint64_t>(candidate));
                JPH::CollisionDispatch::sCollideShapeVsShape(
                    shape1, shape2, JPH::Vec3::sOne(), JPH::Vec3::sOne(),
                    transform1, transform2, JPH::SubShapeIDCreator(),
                    JPH::SubShapeIDCreator(), settings, ioCollector);
                return true;
            });
        ioCollector.SetUserData(0);
    }

    void CastCharacter(
        const JPH::CharacterVirtual *inCharacter,
        JPH::RMat44Arg inCenterOfMassTransform,
        JPH::Vec3Arg inDirection,
        const JPH::ShapeCastSettings &inSettings,
        JPH::RVec3Arg inBaseOffset,
        JPH::CastShapeCollector &ioCollector) const override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        JPH::Mat44 transform1 = inCenterOfMassTransform
            .PostTranslated(-inBaseOffset).ToMat44();
        JPH::ShapeCast shape_cast(
            inCharacter->GetShape(), JPH::Vec3::sOne(),
            transform1, inDirection);
        JPH::Vec3 origin = shape_cast.mShapeWorldBounds.GetCenter();
        JPH::Vec3 extents = shape_cast.mShapeWorldBounds.GetExtent() +
            JPH::Vec3::sReplicate(inSettings.mExtraConvexRadius);
        JPH::AABox query_bounds = inCharacter->GetShape()
            ->GetWorldSpaceBounds(
                inCenterOfMassTransform.ToMat44(), JPH::Vec3::sOne());
        query_bounds.Encapsulate(JPH::AABox(
            query_bounds.mMin + inDirection,
            query_bounds.mMax + inDirection));
        query_bounds.ExpandBy(JPH::Vec3::sReplicate(
            inSettings.mExtraConvexRadius));
        JPH::ShapeCastSettings settings = inSettings;

        BeginQuery();
        ForCandidates(query_bounds, inCharacter,
            [&](const JPH::CharacterVirtual *candidate)
            {
                if (ioCollector.ShouldEarlyOut())
                    return false;
                JPH::Mat44 transform2 = candidate->GetCenterOfMassTransform()
                    .PostTranslated(-inBaseOffset).ToMat44();
                settings.mExtraConvexRadius =
                    inSettings.mExtraConvexRadius +
                    candidate->GetCharacterPadding();
                const JPH::Shape *shape2 = candidate->GetShape();
                JPH::AABox bounds2 = shape2->GetWorldSpaceBounds(
                    transform2, JPH::Vec3::sOne());
                bounds2.ExpandBy(extents + JPH::Vec3::sReplicate(
                    candidate->GetCharacterPadding()));
                if (!JPH::RayAABoxHits(
                        origin, inDirection, bounds2.mMin, bounds2.mMax))
                    return true;
                CountNarrowPhaseTest();
                ioCollector.SetUserData(
                    reinterpret_cast<std::uint64_t>(candidate));
                JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
                    shape_cast, settings, shape2, JPH::Vec3::sOne(), { },
                    transform2, JPH::SubShapeIDCreator(),
                    JPH::SubShapeIDCreator(), ioCollector);
                return true;
            });
        ioCollector.SetUserData(0);
    }

    void GetStats(
        std::uint32_t &outCharacterCount,
        std::uint32_t &outOccupiedCellCount,
        std::uint64_t &outQueryCount,
        std::uint64_t &outCandidateCount,
        std::uint64_t &outNarrowPhaseTestCount) const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        outCharacterCount = static_cast<std::uint32_t>(mEntries.size());
        outOccupiedCellCount = static_cast<std::uint32_t>(mCells.size());
        outQueryCount = mQueryCount;
        outCandidateCount = mCandidateCount;
        outNarrowPhaseTestCount = mNarrowPhaseTestCount;
    }

    void ResetStats()
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mQueryCount = 0;
        mCandidateCount = 0;
        mNarrowPhaseTestCount = 0;
    }

private:
    struct Entry
    {
        CharacterBroadPhaseCell mCell;
        float mExtent;
    };

    CharacterBroadPhaseCell PositionToCell(JPH::Vec3Arg inPosition) const
    {
        const auto coordinate = [&](float inValue)
        {
            const double value = std::floor(
                static_cast<double>(inValue) /
                static_cast<double>(mCellSize));
            const double minimum = static_cast<double>(
                std::numeric_limits<std::int64_t>::min() + 1024);
            const double maximum = static_cast<double>(
                std::numeric_limits<std::int64_t>::max() - 1024);
            return static_cast<std::int64_t>(
                std::max(minimum, std::min(maximum, value)));
        };
        return {
            coordinate(inPosition.GetX()),
            coordinate(inPosition.GetY()),
            coordinate(inPosition.GetZ())
        };
    }

    Entry MakeEntry(JPH::CharacterVirtual *inCharacter) const
    {
        JPH::AABox bounds = inCharacter->GetShape()->GetWorldSpaceBounds(
            inCharacter->GetCenterOfMassTransform().ToMat44(),
            JPH::Vec3::sOne());
        bounds.ExpandBy(JPH::Vec3::sReplicate(
            inCharacter->GetCharacterPadding()));
        JPH::Vec3 extent = bounds.GetExtent();
        return {
            PositionToCell(bounds.GetCenter()),
            std::max(extent.GetX(), std::max(extent.GetY(), extent.GetZ()))
        };
    }

    void RemoveFromCell(
        const CharacterBroadPhaseCell &inCell,
        const JPH::CharacterVirtual *inCharacter)
    {
        auto cell = mCells.find(inCell);
        if (cell == mCells.end())
            return;
        auto character = std::find(
            cell->second.begin(), cell->second.end(), inCharacter);
        if (character != cell->second.end())
            cell->second.erase(character);
        if (cell->second.empty())
            mCells.erase(cell);
    }

    void RecomputeMaxExtent()
    {
        mMaxExtent = 0.0f;
        for (const auto &entry : mEntries)
            mMaxExtent = std::max(mMaxExtent, entry.second.mExtent);
    }

    void BeginQuery() const
    {
        ++mQueryCount;
    }

    void CountNarrowPhaseTest() const
    {
        ++mNarrowPhaseTestCount;
    }

    template <class Callback>
    void ForCandidates(
        JPH::AABox inBounds,
        const JPH::CharacterVirtual *inCharacter,
        Callback inCallback) const
    {
        inBounds.ExpandBy(JPH::Vec3::sReplicate(mMaxExtent));
        const CharacterBroadPhaseCell minimum = PositionToCell(inBounds.mMin);
        const CharacterBroadPhaseCell maximum = PositionToCell(inBounds.mMax);
        const long double x_count =
            static_cast<long double>(maximum.mX) - minimum.mX + 1;
        const long double y_count =
            static_cast<long double>(maximum.mY) - minimum.mY + 1;
        const long double z_count =
            static_cast<long double>(maximum.mZ) - minimum.mZ + 1;
        const bool use_all = x_count > 1024 || y_count > 1024 ||
            z_count > 1024 || x_count * y_count * z_count > 1048576;
        if (use_all)
        {
            for (const auto &entry : mEntries)
                if (entry.first != inCharacter)
                {
                    ++mCandidateCount;
                    if (!inCallback(entry.first))
                        return;
                }
            return;
        }
        for (std::int64_t x = minimum.mX; x <= maximum.mX; ++x)
            for (std::int64_t y = minimum.mY; y <= maximum.mY; ++y)
                for (std::int64_t z = minimum.mZ; z <= maximum.mZ; ++z)
                {
                    auto cell = mCells.find({x, y, z});
                    if (cell == mCells.end())
                        continue;
                    for (const JPH::CharacterVirtual *candidate : cell->second)
                        if (candidate != inCharacter)
                        {
                            ++mCandidateCount;
                            if (!inCallback(candidate))
                                return;
                        }
                }
    }

    float mCellSize;
    float mMaxExtent = 0.0f;
    mutable std::mutex mMutex;
    std::unordered_map<
        const JPH::CharacterVirtual *, Entry> mEntries;
    std::unordered_map<
        CharacterBroadPhaseCell,
        std::vector<const JPH::CharacterVirtual *>,
        CharacterBroadPhaseCellHash> mCells;
    mutable std::uint64_t mQueryCount = 0;
    mutable std::uint64_t mCandidateCount = 0;
    mutable std::uint64_t mNarrowPhaseTestCount = 0;
};

inline CharacterBroadPhase *CreateCharacterBroadPhase(float inCellSize)
{
    return new CharacterBroadPhase(inCellSize);
}

inline void DestroyCharacterBroadPhase(CharacterBroadPhase *inBroadPhase)
{
    delete inBroadPhase;
}

inline void GetCharacterBroadPhaseStats(
    const CharacterBroadPhase *inBroadPhase,
    std::uint32_t *outCharacterCount,
    std::uint32_t *outOccupiedCellCount,
    std::uint64_t *outQueryCount,
    std::uint64_t *outCandidateCount,
    std::uint64_t *outNarrowPhaseTestCount)
{
    inBroadPhase->GetStats(
        *outCharacterCount, *outOccupiedCellCount, *outQueryCount,
        *outCandidateCount, *outNarrowPhaseTestCount);
}

inline void ResetCharacterBroadPhaseStats(CharacterBroadPhase *inBroadPhase)
{
    inBroadPhase->ResetStats();
}

struct CharacterHandle
{
    CharacterContactBridge mContactBridge;
    JPH::CharacterVsCharacterCollisionSimple mVsCharacters;
    CharacterBroadPhase *mBroadPhase = nullptr;
    JPH::Ref<JPH::CharacterVirtual> mCharacter;
    JPH::PhysicsSystem *mSystem = nullptr;
    JPH::ObjectLayer mLayer = 0;
};

inline CharacterHandle *CreateCharacter(
    JPH::PhysicsSystem *inSystem,
    const JPH::Shape *inShape,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::ObjectLayer inLayer,
    float inCenterOffset,
    float inSupportingHeight,
    float inMaxSlopeAngle,
    float inMass,
    float inMaxStrength,
    float inPadding,
    float inPredictiveContactDistance,
    std::uint32_t inMaxNumHits,
    float inHitReductionCosMaxAngle,
    float inPenetrationRecoverySpeed,
    bool inEnhancedInternalEdgeRemoval,
    std::uint8_t inBackFaceMode,
    std::uint32_t inMaxCollisionIterations,
    std::uint32_t inMaxConstraintIterations,
    float inMinTimeRemaining,
    float inCollisionTolerance,
    std::uint64_t inUserData,
    const JPH::Shape *inInnerBodyShape,
    JPH::ObjectLayer inInnerBodyLayer,
    std::uint32_t inContactEventCapacity,
    bool inCanPushCharacter,
    bool inCanReceiveImpulses,
    bool inPreventSliding,
    CharacterBroadPhase *inBroadPhase = nullptr)
{
    if (inInnerBodyShape != nullptr &&
        inSystem->GetNumBodies() >= inSystem->GetMaxBodies())
        return nullptr;
    JPH::CharacterVirtualSettings settings;
    settings.mShape = inShape;
    settings.mShapeOffset = JPH::Vec3(0, inCenterOffset, 0);
    settings.mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(), -inSupportingHeight);
    settings.mMaxSlopeAngle = inMaxSlopeAngle;
    settings.mMass = inMass;
    settings.mMaxStrength = inMaxStrength;
    settings.mCharacterPadding = inPadding;
    settings.mPredictiveContactDistance = inPredictiveContactDistance;
    settings.mMaxNumHits = inMaxNumHits;
    settings.mHitReductionCosMaxAngle = inHitReductionCosMaxAngle;
    settings.mPenetrationRecoverySpeed = inPenetrationRecoverySpeed;
    settings.mEnhancedInternalEdgeRemoval = inEnhancedInternalEdgeRemoval;
    settings.mBackFaceMode = static_cast<JPH::EBackFaceMode>(inBackFaceMode);
    settings.mMaxCollisionIterations = inMaxCollisionIterations;
    settings.mMaxConstraintIterations = inMaxConstraintIterations;
    settings.mMinTimeRemaining = inMinTimeRemaining;
    settings.mCollisionTolerance = inCollisionTolerance;
    settings.mInnerBodyShape = inInnerBodyShape;
    settings.mInnerBodyLayer = inInnerBodyLayer;

    CharacterHandle *handle = new CharacterHandle();
    handle->mContactBridge.Configure(
        inContactEventCapacity, inCanPushCharacter,
        inCanReceiveImpulses, inPreventSliding);
    handle->mSystem = inSystem;
    handle->mLayer = inLayer;
    handle->mBroadPhase = inBroadPhase;
    handle->mCharacter = new JPH::CharacterVirtual(
        &settings, inPosition, inRotation, inUserData, inSystem);
    if (inInnerBodyShape != nullptr &&
        handle->mCharacter->GetInnerBodyID().IsInvalid())
    {
        delete handle;
        return nullptr;
    }
    if (inBroadPhase != nullptr)
    {
        inBroadPhase->Add(handle->mCharacter);
        handle->mCharacter->SetCharacterVsCharacterCollision(inBroadPhase);
    }
    else
        handle->mCharacter->SetCharacterVsCharacterCollision(
            &handle->mVsCharacters);
    handle->mCharacter->SetListener(&handle->mContactBridge);
    return handle;
}

inline void DestroyCharacter(CharacterHandle *inHandle)
{
    if (inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Remove(inHandle->mCharacter);
    delete inHandle;
}

inline void UpdateCharacter(
    CharacterHandle *inHandle,
    float inDeltaTime,
    JPH::Vec3Arg inGravity,
    JPH::Vec3Arg inStepUp,
    JPH::Vec3Arg inStepDown,
    JPH::TempAllocatorImpl *inAllocator)
{
    JPH::CharacterVirtual::ExtendedUpdateSettings settings;
    settings.mWalkStairsStepUp = inStepUp;
    settings.mStickToFloorStepDown = inStepDown;
    inHandle->mCharacter->ExtendedUpdate(
        inDeltaTime,
        inGravity,
        settings,
        inHandle->mSystem->GetDefaultBroadPhaseLayerFilter(inHandle->mLayer),
        inHandle->mSystem->GetDefaultLayerFilter(inHandle->mLayer),
        { },
        { },
        *inAllocator);
    if (inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Update(inHandle->mCharacter);
}

inline void RefreshCharacterContacts(
    CharacterHandle *inHandle,
    JPH::TempAllocatorImpl *inAllocator)
{
    inHandle->mCharacter->RefreshContacts(
        inHandle->mSystem->GetDefaultBroadPhaseLayerFilter(inHandle->mLayer),
        inHandle->mSystem->GetDefaultLayerFilter(inHandle->mLayer),
        { },
        { },
        *inAllocator);
}

inline JPH::Vec3 GetCharacterPosition(const CharacterHandle *inHandle)
{
    return JPH::Vec3(inHandle->mCharacter->GetPosition());
}

inline void SetCharacterPosition(CharacterHandle *inHandle, JPH::Vec3Arg inPosition)
{
    inHandle->mCharacter->SetPosition(inPosition);
    if (inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Update(inHandle->mCharacter);
}

inline JPH::Quat GetCharacterRotation(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetRotation();
}

inline void SetCharacterRotation(CharacterHandle *inHandle, JPH::QuatArg inRotation)
{
    inHandle->mCharacter->SetRotation(inRotation);
    if (inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Update(inHandle->mCharacter);
}

inline JPH::Vec3 GetCharacterVelocity(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetLinearVelocity();
}

inline void SetCharacterVelocity(CharacterHandle *inHandle, JPH::Vec3Arg inVelocity)
{
    inHandle->mCharacter->SetLinearVelocity(inVelocity);
}

inline std::uint8_t GetCharacterGroundState(const CharacterHandle *inHandle)
{
    return static_cast<std::uint8_t>(inHandle->mCharacter->GetGroundState());
}

inline bool IsCharacterSupported(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->IsSupported();
}

inline JPH::Vec3 GetCharacterGroundPosition(const CharacterHandle *inHandle)
{
    return JPH::Vec3(inHandle->mCharacter->GetGroundPosition());
}

inline JPH::Vec3 GetCharacterGroundNormal(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundNormal();
}

inline JPH::Vec3 GetCharacterGroundVelocity(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundVelocity();
}

inline std::uint32_t GetCharacterGroundBodyID(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundBodyID().GetIndexAndSequenceNumber();
}

inline std::uint32_t GetCharacterActiveContactCount(const CharacterHandle *inHandle)
{
    std::uint32_t count = 0;
    for (const JPH::CharacterContact &contact : inHandle->mCharacter->GetActiveContacts())
        if (contact.mHadCollision)
            ++count;
    return count;
}

inline void UpdateCharacterGroundVelocity(CharacterHandle *inHandle)
{
    inHandle->mCharacter->UpdateGroundVelocity();
}

inline std::uint32_t GetCharacterMaxNumHits(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetMaxNumHits();
}

inline void SetCharacterMaxNumHits(CharacterHandle *inHandle, std::uint32_t inMaxHits)
{
    inHandle->mCharacter->SetMaxNumHits(inMaxHits);
}

inline float GetCharacterHitReductionCosMaxAngle(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetHitReductionCosMaxAngle();
}

inline void SetCharacterHitReductionCosMaxAngle(
    CharacterHandle *inHandle, float inCosMaxAngle)
{
    inHandle->mCharacter->SetHitReductionCosMaxAngle(inCosMaxAngle);
}

inline float GetCharacterPenetrationRecoverySpeed(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetPenetrationRecoverySpeed();
}

inline void SetCharacterPenetrationRecoverySpeed(
    CharacterHandle *inHandle, float inSpeed)
{
    inHandle->mCharacter->SetPenetrationRecoverySpeed(inSpeed);
}

inline bool GetCharacterMaxHitsExceeded(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetMaxHitsExceeded();
}

inline void AddCharacterPeer(
    CharacterHandle *inHandle, CharacterHandle *inPeer)
{
    if (inHandle->mBroadPhase == nullptr)
        inHandle->mVsCharacters.Add(inPeer->mCharacter);
}

inline void RemoveCharacterPeer(
    CharacterHandle *inHandle, CharacterHandle *inPeer)
{
    if (inHandle->mBroadPhase == nullptr)
        inHandle->mVsCharacters.Remove(inPeer->mCharacter);
}

inline std::uint32_t GetCharacterID(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetID().GetValue();
}

inline std::uint32_t GetCharacterInnerBodyID(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetInnerBodyID().GetIndexAndSequenceNumber();
}

inline bool CharacterHasCollidedWithBody(
    const CharacterHandle *inHandle, std::uint32_t inBodyID)
{
    return inHandle->mCharacter->HasCollidedWith(JPH::BodyID(inBodyID));
}

inline bool CharacterHasCollidedWithCharacter(
    const CharacterHandle *inHandle, const CharacterHandle *inOther)
{
    return inHandle->mCharacter->HasCollidedWith(inOther->mCharacter);
}

inline std::uint32_t GetCharacterContactCount(const CharacterHandle *inHandle)
{
    return static_cast<std::uint32_t>(
        inHandle->mCharacter->GetActiveContacts().size());
}

inline void GetCharacterContact(
    const CharacterHandle *inHandle,
    std::uint32_t inIndex,
    std::uint32_t *outBodyID,
    std::uint32_t *outCharacterID,
    std::uint32_t *outSubShapeID,
    JPH::Vec3 *outPosition,
    JPH::Vec3 *outLinearVelocity,
    JPH::Vec3 *outContactNormal,
    JPH::Vec3 *outSurfaceNormal,
    float *outDistance,
    float *outFraction,
    std::uint8_t *outMotionType,
    bool *outIsSensor,
    std::uint64_t *outUserData,
    bool *outHadCollision,
    bool *outWasDiscarded,
    bool *outCanPushCharacter,
    bool *outIsBackFacing)
{
    const JPH::CharacterContact &contact =
        inHandle->mCharacter->GetActiveContacts()[inIndex];
    *outBodyID = contact.mBodyB.GetIndexAndSequenceNumber();
    *outCharacterID = contact.mCharacterIDB.GetValue();
    *outSubShapeID = contact.mSubShapeIDB.GetValue();
    *outPosition = JPH::Vec3(contact.mPosition);
    *outLinearVelocity = contact.mLinearVelocity;
    *outContactNormal = contact.mContactNormal;
    *outSurfaceNormal = contact.mSurfaceNormal;
    *outDistance = contact.mDistance;
    *outFraction = contact.mFraction;
    *outMotionType = static_cast<std::uint8_t>(contact.mMotionTypeB);
    *outIsSensor = contact.mIsSensorB;
    *outUserData = contact.mUserData;
    *outHadCollision = contact.mHadCollision;
    *outWasDiscarded = contact.mWasDiscarded;
    *outCanPushCharacter = contact.mCanPushCharacter;
    *outIsBackFacing = contact.mIsBackFacingContact;
}

inline JPH::Vec3 CancelCharacterVelocityTowardsSteepSlopes(
    const CharacterHandle *inHandle, JPH::Vec3Arg inVelocity)
{
    return inHandle->mCharacter->CancelVelocityTowardsSteepSlopes(inVelocity);
}

inline bool CanCharacterWalkStairs(
    const CharacterHandle *inHandle, JPH::Vec3Arg inVelocity)
{
    return inHandle->mCharacter->CanWalkStairs(inVelocity);
}

inline bool WalkCharacterStairs(
    CharacterHandle *inHandle,
    float inDeltaTime,
    JPH::Vec3Arg inStepUp,
    JPH::Vec3Arg inStepForward,
    JPH::Vec3Arg inStepForwardTest,
    JPH::Vec3Arg inStepDownExtra,
    JPH::TempAllocatorImpl *inAllocator)
{
    const bool moved = inHandle->mCharacter->WalkStairs(
        inDeltaTime,
        inStepUp,
        inStepForward,
        inStepForwardTest,
        inStepDownExtra,
        inHandle->mSystem->GetDefaultBroadPhaseLayerFilter(inHandle->mLayer),
        inHandle->mSystem->GetDefaultLayerFilter(inHandle->mLayer),
        { },
        { },
        *inAllocator);
    if (moved && inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Update(inHandle->mCharacter);
    return moved;
}

inline bool StickCharacterToFloor(
    CharacterHandle *inHandle,
    JPH::Vec3Arg inStepDown,
    JPH::TempAllocatorImpl *inAllocator)
{
    const bool moved = inHandle->mCharacter->StickToFloor(
        inStepDown,
        inHandle->mSystem->GetDefaultBroadPhaseLayerFilter(inHandle->mLayer),
        inHandle->mSystem->GetDefaultLayerFilter(inHandle->mLayer),
        { },
        { },
        *inAllocator);
    if (moved && inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Update(inHandle->mCharacter);
    return moved;
}

inline bool SetCharacterShape(
    CharacterHandle *inHandle,
    const JPH::Shape *inShape,
    float inMaxPenetrationDepth,
    JPH::TempAllocatorImpl *inAllocator)
{
    JPH::RefConst<JPH::Shape> retained_shape = inShape;
    const bool changed = inHandle->mCharacter->SetShape(
        retained_shape,
        inMaxPenetrationDepth,
        inHandle->mSystem->GetDefaultBroadPhaseLayerFilter(inHandle->mLayer),
        inHandle->mSystem->GetDefaultLayerFilter(inHandle->mLayer),
        { },
        { },
        *inAllocator);
    if (changed && inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Update(inHandle->mCharacter);
    return changed;
}

inline void SetCharacterInnerBodyShape(
    CharacterHandle *inHandle, const JPH::Shape *inShape)
{
    JPH::RefConst<JPH::Shape> retained_shape = inShape;
    inHandle->mCharacter->SetInnerBodyShape(retained_shape);
}

inline void SetCharacterShapeOffset(
    CharacterHandle *inHandle, JPH::Vec3Arg inOffset)
{
    inHandle->mCharacter->SetShapeOffset(inOffset);
    if (inHandle->mBroadPhase != nullptr)
        inHandle->mBroadPhase->Update(inHandle->mCharacter);
}

inline float GetCharacterMass(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetMass();
}

inline void SetCharacterMass(CharacterHandle *inHandle, float inMass)
{
    inHandle->mCharacter->SetMass(inMass);
}

inline float GetCharacterMaxStrength(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetMaxStrength();
}

inline void SetCharacterMaxStrength(CharacterHandle *inHandle, float inStrength)
{
    inHandle->mCharacter->SetMaxStrength(inStrength);
}

inline bool GetCharacterEnhancedInternalEdgeRemoval(
    const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetEnhancedInternalEdgeRemoval();
}

inline void SetCharacterEnhancedInternalEdgeRemoval(
    CharacterHandle *inHandle, bool inEnabled)
{
    inHandle->mCharacter->SetEnhancedInternalEdgeRemoval(inEnabled);
}

inline std::uint64_t GetCharacterUserData(const CharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetUserData();
}

inline void SetCharacterUserData(
    CharacterHandle *inHandle, std::uint64_t inUserData)
{
    inHandle->mCharacter->SetUserData(inUserData);
}

inline std::uint32_t GetPendingCharacterContactEventCount(
    const CharacterHandle *inHandle)
{
    return inHandle->mContactBridge.PendingCount();
}

inline bool PopCharacterContactEvent(
    CharacterHandle *inHandle,
    std::uint8_t *outKind,
    std::uint32_t *outBodyID,
    std::uint32_t *outCharacterID,
    std::uint32_t *outSubShapeID,
    JPH::Vec3 *outPosition,
    JPH::Vec3 *outNormal,
    JPH::Vec3 *outContactVelocity,
    JPH::Vec3 *outCharacterVelocity,
    JPH::Vec3 *outResultingVelocity,
    std::uint64_t *outUserData,
    bool *outIsSensor,
    bool *outCanPushCharacter,
    bool *outCanReceiveImpulses)
{
    CharacterContactEvent event;
    if (!inHandle->mContactBridge.Pop(event))
        return false;
    *outKind = static_cast<std::uint8_t>(event.mKind);
    *outBodyID = event.mBodyID;
    *outCharacterID = event.mCharacterID;
    *outSubShapeID = event.mSubShapeID;
    *outPosition = event.mPosition;
    *outNormal = event.mNormal;
    *outContactVelocity = event.mContactVelocity;
    *outCharacterVelocity = event.mCharacterVelocity;
    *outResultingVelocity = event.mResultingVelocity;
    *outUserData = event.mUserData;
    *outIsSensor = event.mIsSensor;
    *outCanPushCharacter = event.mCanPushCharacter;
    *outCanReceiveImpulses = event.mCanReceiveImpulses;
    return true;
}

inline std::uint64_t GetDroppedCharacterContactEventCount(
    CharacterHandle *inHandle, bool inReset)
{
    return inHandle->mContactBridge.DroppedCount(inReset);
}

inline void SetCharacterContactResponse(
    CharacterHandle *inHandle,
    std::uint32_t inContactEventCapacity,
    bool inCanPushCharacter,
    bool inCanReceiveImpulses,
    bool inPreventSliding)
{
    inHandle->mContactBridge.Configure(
        inContactEventCapacity, inCanPushCharacter,
        inCanReceiveImpulses, inPreventSliding);
}

struct RigidCharacterHandle
{
    JPH::Ref<JPH::Character> mCharacter;
    JPH::PhysicsSystem *mSystem = nullptr;
    float mMaxSeparationDistance = 0.05f;
    bool mAdded = false;
};

inline RigidCharacterHandle *CreateRigidCharacter(
    JPH::PhysicsSystem *inSystem,
    const JPH::Shape *inShape,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation,
    JPH::ObjectLayer inLayer,
    JPH::Vec3Arg inUp,
    float inSupportingHeight,
    float inMaxSlopeAngle,
    float inMass,
    float inFriction,
    float inGravityFactor,
    std::uint8_t inAllowedDOFs,
    bool inEnhancedInternalEdgeRemoval,
    std::uint64_t inUserData,
    float inMaxSeparationDistance,
    bool inActivate)
{
    if (inSystem == nullptr || inShape == nullptr)
        return nullptr;
    // JPH::Character always destroys its BodyID from its destructor, but its
    // constructor can leave that ID invalid when BodyManager is full. Avoid
    // constructing it in that state because destroying an invalid ID is not a
    // supported Jolt operation.
    if (inSystem->GetNumBodies() >= inSystem->GetMaxBodies())
        return nullptr;
    try
    {
        JPH::CharacterSettings settings;
        settings.mShape = inShape;
        settings.mUp = inUp;
        settings.mSupportingVolume = JPH::Plane(inUp, -inSupportingHeight);
        settings.mMaxSlopeAngle = inMaxSlopeAngle;
        settings.mEnhancedInternalEdgeRemoval = inEnhancedInternalEdgeRemoval;
        settings.mLayer = inLayer;
        settings.mMass = inMass;
        settings.mFriction = inFriction;
        settings.mGravityFactor = inGravityFactor;
        settings.mAllowedDOFs = static_cast<JPH::EAllowedDOFs>(inAllowedDOFs);

        std::unique_ptr<RigidCharacterHandle> handle(new RigidCharacterHandle());
        handle->mSystem = inSystem;
        handle->mMaxSeparationDistance = inMaxSeparationDistance;
        handle->mCharacter = new JPH::Character(
            &settings, inPosition, inRotation, inUserData, inSystem);
        if (handle->mCharacter->GetBodyID().IsInvalid())
            return nullptr;
        handle->mCharacter->AddToPhysicsSystem(
            inActivate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
        handle->mAdded = true;
        return handle.release();
    }
    catch (...)
    {
        return nullptr;
    }
}

inline void DestroyRigidCharacter(RigidCharacterHandle *inHandle)
{
    if (inHandle == nullptr)
        return;
    if (inHandle->mAdded)
        inHandle->mCharacter->RemoveFromPhysicsSystem();
    delete inHandle;
}

inline void PostSimulateRigidCharacter(RigidCharacterHandle *inHandle)
{
    inHandle->mCharacter->PostSimulation(inHandle->mMaxSeparationDistance);
}

inline void RefreshRigidCharacter(
    RigidCharacterHandle *inHandle, float inMaxSeparationDistance)
{
    inHandle->mMaxSeparationDistance = inMaxSeparationDistance;
    inHandle->mCharacter->PostSimulation(inMaxSeparationDistance);
}

inline std::uint32_t GetRigidCharacterBodyID(const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetBodyID().GetIndexAndSequenceNumber();
}

inline JPH::Vec3 GetRigidCharacterPosition(const RigidCharacterHandle *inHandle)
{
    return JPH::Vec3(inHandle->mCharacter->GetPosition());
}

inline void SetRigidCharacterPosition(
    RigidCharacterHandle *inHandle, JPH::Vec3Arg inPosition, bool inActivate)
{
    inHandle->mCharacter->SetPosition(
        inPosition,
        inActivate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
}

inline JPH::Quat GetRigidCharacterRotation(const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetRotation();
}

inline void SetRigidCharacterRotation(
    RigidCharacterHandle *inHandle, JPH::QuatArg inRotation, bool inActivate)
{
    inHandle->mCharacter->SetRotation(
        inRotation,
        inActivate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
}

inline JPH::Vec3 GetRigidCharacterCenterOfMassPosition(
    const RigidCharacterHandle *inHandle)
{
    return JPH::Vec3(inHandle->mCharacter->GetCenterOfMassPosition());
}

inline JPH::Vec3 GetRigidCharacterLinearVelocity(
    const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetLinearVelocity();
}

inline void SetRigidCharacterLinearVelocity(
    RigidCharacterHandle *inHandle, JPH::Vec3Arg inVelocity)
{
    inHandle->mCharacter->SetLinearVelocity(inVelocity);
}

inline void AddRigidCharacterLinearVelocity(
    RigidCharacterHandle *inHandle, JPH::Vec3Arg inVelocity)
{
    inHandle->mCharacter->AddLinearVelocity(inVelocity);
}

inline void AddRigidCharacterImpulse(
    RigidCharacterHandle *inHandle, JPH::Vec3Arg inImpulse)
{
    inHandle->mCharacter->AddImpulse(inImpulse);
}

inline void ActivateRigidCharacter(RigidCharacterHandle *inHandle)
{
    inHandle->mCharacter->Activate();
}

inline std::uint16_t GetRigidCharacterLayer(
    const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetLayer();
}

inline void SetRigidCharacterLayer(
    RigidCharacterHandle *inHandle, JPH::ObjectLayer inLayer)
{
    inHandle->mCharacter->SetLayer(inLayer);
}

inline bool SetRigidCharacterShape(
    RigidCharacterHandle *inHandle,
    const JPH::Shape *inShape,
    float inMaxPenetrationDepth)
{
    return inHandle->mCharacter->SetShape(inShape, inMaxPenetrationDepth);
}

inline std::uint8_t GetRigidCharacterGroundState(
    const RigidCharacterHandle *inHandle)
{
    return static_cast<std::uint8_t>(inHandle->mCharacter->GetGroundState());
}

inline bool IsRigidCharacterSupported(const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->IsSupported();
}

inline JPH::Vec3 GetRigidCharacterGroundPosition(
    const RigidCharacterHandle *inHandle)
{
    return JPH::Vec3(inHandle->mCharacter->GetGroundPosition());
}

inline JPH::Vec3 GetRigidCharacterGroundNormal(
    const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundNormal();
}

inline JPH::Vec3 GetRigidCharacterGroundVelocity(
    const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundVelocity();
}

inline std::uint32_t GetRigidCharacterGroundBodyID(
    const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundBodyID().GetIndexAndSequenceNumber();
}

inline std::uint32_t GetRigidCharacterGroundSubShapeID(
    const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundSubShapeID().GetValue();
}

inline std::uint64_t GetRigidCharacterGroundUserData(
    const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetGroundUserData();
}

inline float GetRigidCharacterMaxSlopeAngle(
    const RigidCharacterHandle *inHandle)
{
    return std::acos(inHandle->mCharacter->GetCosMaxSlopeAngle());
}

inline void SetRigidCharacterMaxSlopeAngle(
    RigidCharacterHandle *inHandle, float inAngle)
{
    inHandle->mCharacter->SetMaxSlopeAngle(inAngle);
}

inline JPH::Vec3 GetRigidCharacterUp(const RigidCharacterHandle *inHandle)
{
    return inHandle->mCharacter->GetUp();
}

inline void SetRigidCharacterUp(
    RigidCharacterHandle *inHandle, JPH::Vec3Arg inUp)
{
    inHandle->mCharacter->SetUp(inUp);
}

inline float GetRigidCharacterSupportingHeight(
    const RigidCharacterHandle *inHandle)
{
    return -inHandle->mCharacter->GetSupportingVolume().GetConstant();
}

inline void SetRigidCharacterSupportingHeight(
    RigidCharacterHandle *inHandle, float inHeight)
{
    inHandle->mCharacter->SetSupportingVolume(
        JPH::Plane(inHandle->mCharacter->GetUp(), -inHeight));
}

inline bool IsRigidCharacterSlopeTooSteep(
    const RigidCharacterHandle *inHandle, JPH::Vec3Arg inNormal)
{
    return inHandle->mCharacter->IsSlopeTooSteep(inNormal);
}

struct VehicleWheelConfigData
{
    JPH::Vec3 mPosition;
    JPH::Vec3 mSuspensionForcePoint;
    JPH::Vec3 mSuspensionDirection;
    JPH::Vec3 mSteeringAxis;
    JPH::Vec3 mWheelUp;
    JPH::Vec3 mWheelForward;
    float mSuspensionMinLength;
    float mSuspensionMaxLength;
    float mSuspensionPreloadLength;
    float mSuspensionFrequency;
    float mSuspensionDamping;
    float mRadius;
    float mWidth;
    bool mEnableSuspensionForcePoint;
    float mInertia;
    float mAngularDamping;
    float mMaxSteerAngle;
    float mMaxBrakeTorque;
    float mMaxHandBrakeTorque;
    float mLongitudinalImpulseMultiplier;
    float mLateralImpulseMultiplier;
    const float *mLongitudinalFrictionSlips;
    const float *mLongitudinalFrictionValues;
    std::uint32_t mLongitudinalFrictionCount;
    const float *mLateralFrictionSlips;
    const float *mLateralFrictionValues;
    std::uint32_t mLateralFrictionCount;
};

inline VehicleWheelConfigData MakeVehicleWheelConfigData(
    JPH::Vec3Arg inPosition,
    JPH::Vec3Arg inSuspensionForcePoint,
    JPH::Vec3Arg inSuspensionDirection,
    JPH::Vec3Arg inSteeringAxis,
    JPH::Vec3Arg inWheelUp,
    JPH::Vec3Arg inWheelForward,
    float inSuspensionMinLength,
    float inSuspensionMaxLength,
    float inSuspensionPreloadLength,
    float inSuspensionFrequency,
    float inSuspensionDamping,
    float inRadius,
    float inWidth,
    bool inEnableSuspensionForcePoint,
    float inInertia,
    float inAngularDamping,
    float inMaxSteerAngle,
    float inMaxBrakeTorque,
    float inMaxHandBrakeTorque,
    float inLongitudinalImpulseMultiplier,
    float inLateralImpulseMultiplier,
    const float *inLongitudinalFrictionSlips,
    const float *inLongitudinalFrictionValues,
    std::uint32_t inLongitudinalFrictionCount,
    const float *inLateralFrictionSlips,
    const float *inLateralFrictionValues,
    std::uint32_t inLateralFrictionCount)
{
    return {
        inPosition, inSuspensionForcePoint, inSuspensionDirection,
        inSteeringAxis, inWheelUp, inWheelForward,
        inSuspensionMinLength, inSuspensionMaxLength,
        inSuspensionPreloadLength, inSuspensionFrequency,
        inSuspensionDamping, inRadius, inWidth,
        inEnableSuspensionForcePoint, inInertia, inAngularDamping,
        inMaxSteerAngle, inMaxBrakeTorque, inMaxHandBrakeTorque,
        inLongitudinalImpulseMultiplier, inLateralImpulseMultiplier,
        inLongitudinalFrictionSlips, inLongitudinalFrictionValues,
        inLongitudinalFrictionCount, inLateralFrictionSlips,
        inLateralFrictionValues, inLateralFrictionCount
    };
}

struct TrackedVehicleWheelConfigData
{
    JPH::Vec3 mPosition;
    JPH::Vec3 mSuspensionForcePoint;
    JPH::Vec3 mSuspensionDirection;
    JPH::Vec3 mSteeringAxis;
    JPH::Vec3 mWheelUp;
    JPH::Vec3 mWheelForward;
    float mSuspensionMinLength;
    float mSuspensionMaxLength;
    float mSuspensionPreloadLength;
    float mSuspensionFrequency;
    float mSuspensionDamping;
    float mRadius;
    float mWidth;
    bool mEnableSuspensionForcePoint;
    float mLongitudinalFriction;
    float mLateralFriction;
};

inline TrackedVehicleWheelConfigData MakeTrackedVehicleWheelConfigData(
    JPH::Vec3Arg inPosition,
    JPH::Vec3Arg inSuspensionForcePoint,
    JPH::Vec3Arg inSuspensionDirection,
    JPH::Vec3Arg inSteeringAxis,
    JPH::Vec3Arg inWheelUp,
    JPH::Vec3Arg inWheelForward,
    float inSuspensionMinLength,
    float inSuspensionMaxLength,
    float inSuspensionPreloadLength,
    float inSuspensionFrequency,
    float inSuspensionDamping,
    float inRadius,
    float inWidth,
    bool inEnableSuspensionForcePoint,
    float inLongitudinalFriction,
    float inLateralFriction)
{
    return {
        inPosition, inSuspensionForcePoint, inSuspensionDirection,
        inSteeringAxis, inWheelUp, inWheelForward,
        inSuspensionMinLength, inSuspensionMaxLength,
        inSuspensionPreloadLength, inSuspensionFrequency,
        inSuspensionDamping, inRadius, inWidth,
        inEnableSuspensionForcePoint, inLongitudinalFriction,
        inLateralFriction
    };
}

struct VehicleDifferentialConfigData
{
    std::int32_t mLeftWheel;
    std::int32_t mRightWheel;
    float mDifferentialRatio;
    float mLeftRightSplit;
    float mLimitedSlipRatio;
    float mEngineTorqueRatio;
};

inline VehicleDifferentialConfigData MakeVehicleDifferentialConfigData(
    std::int32_t inLeftWheel,
    std::int32_t inRightWheel,
    float inDifferentialRatio,
    float inLeftRightSplit,
    float inLimitedSlipRatio,
    float inEngineTorqueRatio)
{
    return {
        inLeftWheel, inRightWheel, inDifferentialRatio, inLeftRightSplit,
        inLimitedSlipRatio, inEngineTorqueRatio
    };
}

struct VehicleAntiRollBarConfigData
{
    std::int32_t mLeftWheel;
    std::int32_t mRightWheel;
    float mStiffness;
};

inline VehicleAntiRollBarConfigData MakeVehicleAntiRollBarConfigData(
    std::int32_t inLeftWheel,
    std::int32_t inRightWheel,
    float inStiffness)
{
    return { inLeftWheel, inRightWheel, inStiffness };
}

struct VehicleHandle
{
    JPH::Ref<JPH::VehicleConstraint> mVehicle;
    JPH::PhysicsSystem *mSystem = nullptr;
    JPH::BodyID mBodyID;
};

inline VehicleHandle *CreateVehicle(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    float inHalfWidth,
    float inHalfHeight,
    float inHalfLength,
    float inWheelRadius,
    float inWheelWidth,
    float inSuspensionMinLength,
    float inSuspensionMaxLength,
    float inSuspensionFrequency,
    float inSuspensionDamping,
    float inMaxSteerAngle,
    float inMaxPitchRollAngle,
    float inEngineMaxTorque,
    float inEngineMinRPM,
    float inEngineMaxRPM,
    float inEngineInertia,
    float inEngineAngularDamping,
    const float *inTorqueCurveRPMFractions,
    const float *inTorqueCurveFractions,
    std::uint32_t inTorqueCurveCount,
    std::uint8_t inTransmissionMode,
    const float *inGearRatios,
    std::uint32_t inGearRatioCount,
    const float *inReverseGearRatios,
    std::uint32_t inReverseGearRatioCount,
    float inTransmissionSwitchTime,
    float inClutchReleaseTime,
    float inTransmissionSwitchLatency,
    float inShiftUpRPM,
    float inShiftDownRPM,
    float inClutchStrength,
    bool inFourWheelDrive,
    bool inFrontWheelDrive,
    float inFrontTorqueRatio,
    float inDifferentialRatio,
    float inDifferentialLeftRightSplit,
    float inDifferentialLimitedSlipRatio,
    float inCenterDifferentialLimitedSlipRatio,
    float inWheelTrack,
    float inFrontAxleOffset,
    float inRearAxleOffset,
    float inSuspensionAttachmentHeightRatio,
    float inRearMaxSteerAngle,
    float inFrontBrakeTorque,
    float inRearBrakeTorque,
    float inRearHandBrakeTorque,
    float inAntiRollBarStiffness,
    float inWheelInertia,
    float inWheelAngularDamping,
    float inTireLongitudinalImpulseMultiplier,
    float inTireLateralImpulseMultiplier,
    const VehicleWheelConfigData *inCustomWheels,
    std::uint32_t inCustomWheelCount,
    const VehicleDifferentialConfigData *inCustomDifferentials,
    std::uint32_t inCustomDifferentialCount,
    const VehicleAntiRollBarConfigData *inCustomAntiRollBars,
    std::uint32_t inCustomAntiRollBarCount,
    std::uint8_t inWheelCollisionMode,
    JPH::Vec3Arg inWheelCollisionUp,
    float inWheelCollisionMaxSlopeAngle,
    float inWheelSphereCastRadius,
    float inWheelCylinderConvexRadiusFraction,
    JPH::ObjectLayer inWheelCollisionLayer,
    std::uint8_t inControllerKind,
    float inMaxLeanAngle,
    float inLeanSpringConstant,
    float inLeanSpringDamping,
    float inLeanSpringIntegrationCoefficient,
    float inLeanSpringIntegrationCoefficientDecay,
    float inLeanSmoothingFactor,
    bool inEnableLeanController,
    bool inEnableLeanSteeringLimit)
{
    JPH::Ref<JPH::VehicleConstraint> vehicle;
    {
        JPH::BodyLockWrite lock(inSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return nullptr;
        JPH::Body &body = lock.GetBody();
        if (body.GetMotionType() != JPH::EMotionType::Dynamic)
            return nullptr;

        JPH::VehicleConstraintSettings settings;
        settings.mMaxPitchRollAngle = inMaxPitchRollAngle;
        JPH::Array<float> longitudinal_multipliers;
        JPH::Array<float> lateral_multipliers;
        if (inCustomWheelCount > 0)
        {
            longitudinal_multipliers.reserve(inCustomWheelCount);
            lateral_multipliers.reserve(inCustomWheelCount);
            for (std::uint32_t index = 0; index < inCustomWheelCount; ++index)
            {
                const VehicleWheelConfigData &data = inCustomWheels[index];
                JPH::WheelSettingsWV *wheel = new JPH::WheelSettingsWV();
                wheel->mPosition = data.mPosition;
                wheel->mSuspensionForcePoint = data.mSuspensionForcePoint;
                wheel->mSuspensionDirection = data.mSuspensionDirection;
                wheel->mSteeringAxis = data.mSteeringAxis;
                wheel->mWheelUp = data.mWheelUp;
                wheel->mWheelForward = data.mWheelForward;
                wheel->mSuspensionMinLength = data.mSuspensionMinLength;
                wheel->mSuspensionMaxLength = data.mSuspensionMaxLength;
                wheel->mSuspensionPreloadLength =
                    data.mSuspensionPreloadLength;
                wheel->mSuspensionSpring.mFrequency =
                    data.mSuspensionFrequency;
                wheel->mSuspensionSpring.mDamping = data.mSuspensionDamping;
                wheel->mRadius = data.mRadius;
                wheel->mWidth = data.mWidth;
                wheel->mEnableSuspensionForcePoint =
                    data.mEnableSuspensionForcePoint;
                wheel->mInertia = data.mInertia;
                wheel->mAngularDamping = data.mAngularDamping;
                wheel->mMaxSteerAngle = data.mMaxSteerAngle;
                wheel->mMaxBrakeTorque = data.mMaxBrakeTorque;
                wheel->mMaxHandBrakeTorque = data.mMaxHandBrakeTorque;
                if (data.mLongitudinalFrictionCount > 0)
                {
                    wheel->mLongitudinalFriction.Clear();
                    wheel->mLongitudinalFriction.Reserve(
                        data.mLongitudinalFrictionCount);
                    for (std::uint32_t point = 0;
                         point < data.mLongitudinalFrictionCount; ++point)
                        wheel->mLongitudinalFriction.AddPoint(
                            data.mLongitudinalFrictionSlips[point],
                            data.mLongitudinalFrictionValues[point]);
                }
                if (data.mLateralFrictionCount > 0)
                {
                    wheel->mLateralFriction.Clear();
                    wheel->mLateralFriction.Reserve(
                        data.mLateralFrictionCount);
                    for (std::uint32_t point = 0;
                         point < data.mLateralFrictionCount; ++point)
                        wheel->mLateralFriction.AddPoint(
                            data.mLateralFrictionSlips[point],
                            data.mLateralFrictionValues[point]);
                }
                settings.mWheels.push_back(wheel);
                longitudinal_multipliers.push_back(
                    data.mLongitudinalImpulseMultiplier);
                lateral_multipliers.push_back(data.mLateralImpulseMultiplier);
            }
        }
        else
        {
            const float wheel_x = inWheelTrack > 0.0f?
                0.5f * inWheelTrack : inHalfWidth;
            const float default_wheel_z = inHalfLength - 2.0f * inWheelRadius;
            const float front_wheel_z = inFrontAxleOffset > 0.0f?
                inFrontAxleOffset : default_wheel_z;
            const float rear_wheel_z = inRearAxleOffset > 0.0f?
                inRearAxleOffset : default_wheel_z;
            const float wheel_y =
                inSuspensionAttachmentHeightRatio * inHalfHeight;
            longitudinal_multipliers.resize(
                4, inTireLongitudinalImpulseMultiplier);
            lateral_multipliers.resize(4, inTireLateralImpulseMultiplier);
            for (int index = 0; index < 4; ++index)
            {
                JPH::WheelSettingsWV *wheel = new JPH::WheelSettingsWV();
                const bool left = (index & 1) == 0;
                const bool front = index < 2;
                wheel->mPosition = JPH::Vec3(
                    left? wheel_x : -wheel_x,
                    wheel_y,
                    front? front_wheel_z : -rear_wheel_z);
                wheel->mSuspensionMinLength = inSuspensionMinLength;
                wheel->mSuspensionMaxLength = inSuspensionMaxLength;
                wheel->mSuspensionSpring.mFrequency = inSuspensionFrequency;
                wheel->mSuspensionSpring.mDamping = inSuspensionDamping;
                wheel->mRadius = inWheelRadius;
                wheel->mWidth = inWheelWidth;
                wheel->mInertia = inWheelInertia;
                wheel->mAngularDamping = inWheelAngularDamping;
                wheel->mMaxSteerAngle = front?
                    inMaxSteerAngle : inRearMaxSteerAngle;
                wheel->mMaxBrakeTorque = front?
                    inFrontBrakeTorque : inRearBrakeTorque;
                wheel->mMaxHandBrakeTorque =
                    front? 0.0f : inRearHandBrakeTorque;
                settings.mWheels.push_back(wheel);
            }
        }

        JPH::WheeledVehicleControllerSettings *controller;
        if (inControllerKind == 1)
        {
            JPH::MotorcycleControllerSettings *motorcycle =
                new JPH::MotorcycleControllerSettings();
            motorcycle->mMaxLeanAngle = inMaxLeanAngle;
            motorcycle->mLeanSpringConstant = inLeanSpringConstant;
            motorcycle->mLeanSpringDamping = inLeanSpringDamping;
            motorcycle->mLeanSpringIntegrationCoefficient =
                inLeanSpringIntegrationCoefficient;
            motorcycle->mLeanSpringIntegrationCoefficientDecay =
                inLeanSpringIntegrationCoefficientDecay;
            motorcycle->mLeanSmoothingFactor = inLeanSmoothingFactor;
            controller = motorcycle;
        }
        else
            controller = new JPH::WheeledVehicleControllerSettings();
        controller->mEngine.mMaxTorque = inEngineMaxTorque;
        controller->mEngine.mMinRPM = inEngineMinRPM;
        controller->mEngine.mMaxRPM = inEngineMaxRPM;
        controller->mEngine.mInertia = inEngineInertia;
        controller->mEngine.mAngularDamping = inEngineAngularDamping;
        controller->mEngine.mNormalizedTorque.Clear();
        controller->mEngine.mNormalizedTorque.Reserve(inTorqueCurveCount);
        for (std::uint32_t index = 0; index < inTorqueCurveCount; ++index)
            controller->mEngine.mNormalizedTorque.AddPoint(
                inTorqueCurveRPMFractions[index],
                inTorqueCurveFractions[index]);
        controller->mTransmission.mMode =
            static_cast<JPH::ETransmissionMode>(inTransmissionMode);
        controller->mTransmission.mGearRatios.assign(
            inGearRatios, inGearRatios + inGearRatioCount);
        controller->mTransmission.mReverseGearRatios.assign(
            inReverseGearRatios,
            inReverseGearRatios + inReverseGearRatioCount);
        controller->mTransmission.mSwitchTime = inTransmissionSwitchTime;
        controller->mTransmission.mClutchReleaseTime = inClutchReleaseTime;
        controller->mTransmission.mSwitchLatency = inTransmissionSwitchLatency;
        controller->mTransmission.mShiftUpRPM = inShiftUpRPM;
        controller->mTransmission.mShiftDownRPM = inShiftDownRPM;
        controller->mTransmission.mClutchStrength = inClutchStrength;
        if (inCustomDifferentialCount > 0)
        {
            controller->mDifferentials.resize(inCustomDifferentialCount);
            for (std::uint32_t index = 0;
                 index < inCustomDifferentialCount; ++index)
            {
                const VehicleDifferentialConfigData &data =
                    inCustomDifferentials[index];
                JPH::VehicleDifferentialSettings &differential =
                    controller->mDifferentials[index];
                differential.mLeftWheel = data.mLeftWheel;
                differential.mRightWheel = data.mRightWheel;
                differential.mDifferentialRatio = data.mDifferentialRatio;
                differential.mLeftRightSplit = data.mLeftRightSplit;
                differential.mLimitedSlipRatio = data.mLimitedSlipRatio;
                differential.mEngineTorqueRatio = data.mEngineTorqueRatio;
            }
        }
        else
        {
            controller->mDifferentials.resize(inFourWheelDrive? 2 : 1);
            if (inFourWheelDrive)
            {
                controller->mDifferentials[0].mLeftWheel = 0;
                controller->mDifferentials[0].mRightWheel = 1;
                controller->mDifferentials[1].mLeftWheel = 2;
                controller->mDifferentials[1].mRightWheel = 3;
                controller->mDifferentials[0].mEngineTorqueRatio =
                    inFrontTorqueRatio;
                controller->mDifferentials[1].mEngineTorqueRatio =
                    1.0f - inFrontTorqueRatio;
            }
            else
            {
                controller->mDifferentials[0].mLeftWheel =
                    inFrontWheelDrive? 0 : 2;
                controller->mDifferentials[0].mRightWheel =
                    inFrontWheelDrive? 1 : 3;
            }
            for (JPH::VehicleDifferentialSettings &differential :
                 controller->mDifferentials)
            {
                differential.mDifferentialRatio = inDifferentialRatio;
                differential.mLeftRightSplit = inDifferentialLeftRightSplit;
                differential.mLimitedSlipRatio =
                    inDifferentialLimitedSlipRatio;
            }
        }
        controller->mDifferentialLimitedSlipRatio =
            inCenterDifferentialLimitedSlipRatio;
        settings.mController = controller;

        if (inCustomAntiRollBarCount > 0)
        {
            settings.mAntiRollBars.resize(inCustomAntiRollBarCount);
            for (std::uint32_t index = 0;
                 index < inCustomAntiRollBarCount; ++index)
            {
                settings.mAntiRollBars[index].mLeftWheel =
                    inCustomAntiRollBars[index].mLeftWheel;
                settings.mAntiRollBars[index].mRightWheel =
                    inCustomAntiRollBars[index].mRightWheel;
                settings.mAntiRollBars[index].mStiffness =
                    inCustomAntiRollBars[index].mStiffness;
            }
        }
        else if (inCustomWheelCount == 0 && inAntiRollBarStiffness > 0.0f)
        {
            settings.mAntiRollBars.resize(2);
            settings.mAntiRollBars[0].mLeftWheel = 0;
            settings.mAntiRollBars[0].mRightWheel = 1;
            settings.mAntiRollBars[0].mStiffness = inAntiRollBarStiffness;
            settings.mAntiRollBars[1].mLeftWheel = 2;
            settings.mAntiRollBars[1].mRightWheel = 3;
            settings.mAntiRollBars[1].mStiffness = inAntiRollBarStiffness;
        }

        vehicle = new JPH::VehicleConstraint(body, settings);
        if (inControllerKind == 1)
        {
            JPH::MotorcycleController *motorcycle =
                static_cast<JPH::MotorcycleController *>(
                    vehicle->GetController());
            motorcycle->EnableLeanController(inEnableLeanController);
            motorcycle->EnableLeanSteeringLimit(inEnableLeanSteeringLimit);
        }
        switch (inWheelCollisionMode)
        {
        case 1:
            vehicle->SetVehicleCollisionTester(
                new JPH::VehicleCollisionTesterCastSphere(
                    inWheelCollisionLayer, inWheelSphereCastRadius,
                    inWheelCollisionUp, inWheelCollisionMaxSlopeAngle));
            break;
        case 2:
            vehicle->SetVehicleCollisionTester(
                new JPH::VehicleCollisionTesterCastCylinder(
                    inWheelCollisionLayer,
                    inWheelCylinderConvexRadiusFraction));
            break;
        default:
            vehicle->SetVehicleCollisionTester(
                new JPH::VehicleCollisionTesterRay(
                    inWheelCollisionLayer, inWheelCollisionUp,
                    inWheelCollisionMaxSlopeAngle));
            break;
        }
        static_cast<JPH::WheeledVehicleController *>(vehicle->GetController())
            ->SetTireMaxImpulseCallback(
                [longitudinal_multipliers, lateral_multipliers]
                (JPH::uint inWheelIndex, float &outLongitudinalImpulse,
                   float &outLateralImpulse, float inSuspensionImpulse,
                   float inLongitudinalFriction, float inLateralFriction,
                   float, float, float)
                {
                    outLongitudinalImpulse =
                        longitudinal_multipliers[inWheelIndex] *
                        inLongitudinalFriction *
                        inSuspensionImpulse;
                    outLateralImpulse =
                        lateral_multipliers[inWheelIndex] *
                        inLateralFriction *
                        inSuspensionImpulse;
                });
    }

    inSystem->AddConstraint(vehicle);
    inSystem->AddStepListener(vehicle);
    VehicleHandle *handle = new VehicleHandle();
    handle->mVehicle = vehicle;
    handle->mSystem = inSystem;
    handle->mBodyID = inBodyID;
    return handle;
}

inline VehicleHandle *CreateTrackedVehicle(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    float inMaxPitchRollAngle,
    float inEngineMaxTorque,
    float inEngineMinRPM,
    float inEngineMaxRPM,
    float inEngineInertia,
    float inEngineAngularDamping,
    const float *inTorqueCurveRPMFractions,
    const float *inTorqueCurveFractions,
    std::uint32_t inTorqueCurveCount,
    std::uint8_t inTransmissionMode,
    const float *inGearRatios,
    std::uint32_t inGearRatioCount,
    const float *inReverseGearRatios,
    std::uint32_t inReverseGearRatioCount,
    float inTransmissionSwitchTime,
    float inClutchReleaseTime,
    float inTransmissionSwitchLatency,
    float inShiftUpRPM,
    float inShiftDownRPM,
    float inClutchStrength,
    const TrackedVehicleWheelConfigData *inWheels,
    std::uint32_t inWheelCount,
    const std::uint32_t *inLeftWheels,
    std::uint32_t inLeftWheelCount,
    std::uint32_t inLeftDrivenWheel,
    float inLeftInertia,
    float inLeftAngularDamping,
    float inLeftMaxBrakeTorque,
    float inLeftDifferentialRatio,
    const std::uint32_t *inRightWheels,
    std::uint32_t inRightWheelCount,
    std::uint32_t inRightDrivenWheel,
    float inRightInertia,
    float inRightAngularDamping,
    float inRightMaxBrakeTorque,
    float inRightDifferentialRatio,
    std::uint8_t inWheelCollisionMode,
    JPH::Vec3Arg inWheelCollisionUp,
    float inWheelCollisionMaxSlopeAngle,
    float inWheelSphereCastRadius,
    float inWheelCylinderConvexRadiusFraction,
    JPH::ObjectLayer inWheelCollisionLayer)
{
    JPH::Ref<JPH::VehicleConstraint> vehicle;
    {
        JPH::BodyLockWrite lock(inSystem->GetBodyLockInterface(), inBodyID);
        if (!lock.Succeeded())
            return nullptr;
        JPH::Body &body = lock.GetBody();
        if (body.GetMotionType() != JPH::EMotionType::Dynamic)
            return nullptr;

        JPH::VehicleConstraintSettings settings;
        settings.mMaxPitchRollAngle = inMaxPitchRollAngle;
        settings.mWheels.reserve(inWheelCount);
        for (std::uint32_t index = 0; index < inWheelCount; ++index)
        {
            const TrackedVehicleWheelConfigData &data = inWheels[index];
            JPH::WheelSettingsTV *wheel = new JPH::WheelSettingsTV();
            wheel->mPosition = data.mPosition;
            wheel->mSuspensionForcePoint = data.mSuspensionForcePoint;
            wheel->mSuspensionDirection = data.mSuspensionDirection;
            wheel->mSteeringAxis = data.mSteeringAxis;
            wheel->mWheelUp = data.mWheelUp;
            wheel->mWheelForward = data.mWheelForward;
            wheel->mSuspensionMinLength = data.mSuspensionMinLength;
            wheel->mSuspensionMaxLength = data.mSuspensionMaxLength;
            wheel->mSuspensionPreloadLength =
                data.mSuspensionPreloadLength;
            wheel->mSuspensionSpring.mFrequency =
                data.mSuspensionFrequency;
            wheel->mSuspensionSpring.mDamping = data.mSuspensionDamping;
            wheel->mRadius = data.mRadius;
            wheel->mWidth = data.mWidth;
            wheel->mEnableSuspensionForcePoint =
                data.mEnableSuspensionForcePoint;
            wheel->mLongitudinalFriction = data.mLongitudinalFriction;
            wheel->mLateralFriction = data.mLateralFriction;
            settings.mWheels.push_back(wheel);
        }

        JPH::TrackedVehicleControllerSettings *controller =
            new JPH::TrackedVehicleControllerSettings();
        controller->mEngine.mMaxTorque = inEngineMaxTorque;
        controller->mEngine.mMinRPM = inEngineMinRPM;
        controller->mEngine.mMaxRPM = inEngineMaxRPM;
        controller->mEngine.mInertia = inEngineInertia;
        controller->mEngine.mAngularDamping = inEngineAngularDamping;
        controller->mEngine.mNormalizedTorque.Clear();
        controller->mEngine.mNormalizedTorque.Reserve(inTorqueCurveCount);
        for (std::uint32_t index = 0; index < inTorqueCurveCount; ++index)
            controller->mEngine.mNormalizedTorque.AddPoint(
                inTorqueCurveRPMFractions[index],
                inTorqueCurveFractions[index]);
        controller->mTransmission.mMode =
            static_cast<JPH::ETransmissionMode>(inTransmissionMode);
        controller->mTransmission.mGearRatios.assign(
            inGearRatios, inGearRatios + inGearRatioCount);
        controller->mTransmission.mReverseGearRatios.assign(
            inReverseGearRatios,
            inReverseGearRatios + inReverseGearRatioCount);
        controller->mTransmission.mSwitchTime = inTransmissionSwitchTime;
        controller->mTransmission.mClutchReleaseTime = inClutchReleaseTime;
        controller->mTransmission.mSwitchLatency = inTransmissionSwitchLatency;
        controller->mTransmission.mShiftUpRPM = inShiftUpRPM;
        controller->mTransmission.mShiftDownRPM = inShiftDownRPM;
        controller->mTransmission.mClutchStrength = inClutchStrength;

        JPH::VehicleTrackSettings &left =
            controller->mTracks[static_cast<int>(JPH::ETrackSide::Left)];
        left.mWheels.assign(inLeftWheels, inLeftWheels + inLeftWheelCount);
        left.mDrivenWheel = inLeftDrivenWheel;
        left.mInertia = inLeftInertia;
        left.mAngularDamping = inLeftAngularDamping;
        left.mMaxBrakeTorque = inLeftMaxBrakeTorque;
        left.mDifferentialRatio = inLeftDifferentialRatio;
        JPH::VehicleTrackSettings &right =
            controller->mTracks[static_cast<int>(JPH::ETrackSide::Right)];
        right.mWheels.assign(inRightWheels, inRightWheels + inRightWheelCount);
        right.mDrivenWheel = inRightDrivenWheel;
        right.mInertia = inRightInertia;
        right.mAngularDamping = inRightAngularDamping;
        right.mMaxBrakeTorque = inRightMaxBrakeTorque;
        right.mDifferentialRatio = inRightDifferentialRatio;
        settings.mController = controller;

        vehicle = new JPH::VehicleConstraint(body, settings);
        switch (inWheelCollisionMode)
        {
        case 1:
            vehicle->SetVehicleCollisionTester(
                new JPH::VehicleCollisionTesterCastSphere(
                    inWheelCollisionLayer, inWheelSphereCastRadius,
                    inWheelCollisionUp, inWheelCollisionMaxSlopeAngle));
            break;
        case 2:
            vehicle->SetVehicleCollisionTester(
                new JPH::VehicleCollisionTesterCastCylinder(
                    inWheelCollisionLayer,
                    inWheelCylinderConvexRadiusFraction));
            break;
        default:
            vehicle->SetVehicleCollisionTester(
                new JPH::VehicleCollisionTesterRay(
                    inWheelCollisionLayer, inWheelCollisionUp,
                    inWheelCollisionMaxSlopeAngle));
            break;
        }
    }

    inSystem->AddConstraint(vehicle);
    inSystem->AddStepListener(vehicle);
    VehicleHandle *handle = new VehicleHandle();
    handle->mVehicle = vehicle;
    handle->mSystem = inSystem;
    handle->mBodyID = inBodyID;
    return handle;
}

inline void DestroyVehicle(VehicleHandle *inHandle)
{
    if (inHandle == nullptr)
        return;
    inHandle->mSystem->RemoveStepListener(inHandle->mVehicle);
    inHandle->mSystem->RemoveConstraint(inHandle->mVehicle);
    delete inHandle;
}

inline void SetVehicleInput(
    VehicleHandle *inHandle,
    float inForward,
    float inRight,
    float inBrake,
    float inHandBrake)
{
    static_cast<JPH::WheeledVehicleController *>(
        inHandle->mVehicle->GetController())
        ->SetDriverInput(inForward, inRight, inBrake, inHandBrake);
    inHandle->mSystem->GetBodyInterface().ActivateBody(inHandle->mBodyID);
}

inline void GetMotorcycleControllerState(
    const VehicleHandle *inHandle,
    float *outWheelBase,
    bool *outLeanControllerEnabled,
    bool *outLeanSteeringLimitEnabled,
    float *outLeanSpringConstant,
    float *outLeanSpringDamping,
    float *outLeanSpringIntegrationCoefficient,
    float *outLeanSpringIntegrationCoefficientDecay,
    float *outLeanSmoothingFactor)
{
    const JPH::MotorcycleController *controller =
        static_cast<const JPH::MotorcycleController *>(
            inHandle->mVehicle->GetController());
    *outWheelBase = controller->GetWheelBase();
    *outLeanControllerEnabled = controller->IsLeanControllerEnabled();
    *outLeanSteeringLimitEnabled =
        controller->IsLeanSteeringLimitEnabled();
    *outLeanSpringConstant = controller->GetLeanSpringConstant();
    *outLeanSpringDamping = controller->GetLeanSpringDamping();
    *outLeanSpringIntegrationCoefficient =
        controller->GetLeanSpringIntegrationCoefficient();
    *outLeanSpringIntegrationCoefficientDecay =
        controller->GetLeanSpringIntegrationCoefficientDecay();
    *outLeanSmoothingFactor = controller->GetLeanSmoothingFactor();
}

inline void ConfigureMotorcycleController(
    VehicleHandle *inHandle,
    bool inEnableLeanController,
    bool inEnableLeanSteeringLimit,
    float inLeanSpringConstant,
    float inLeanSpringDamping,
    float inLeanSpringIntegrationCoefficient,
    float inLeanSpringIntegrationCoefficientDecay,
    float inLeanSmoothingFactor)
{
    JPH::MotorcycleController *controller =
        static_cast<JPH::MotorcycleController *>(
            inHandle->mVehicle->GetController());
    controller->EnableLeanController(inEnableLeanController);
    controller->EnableLeanSteeringLimit(inEnableLeanSteeringLimit);
    controller->SetLeanSpringConstant(inLeanSpringConstant);
    controller->SetLeanSpringDamping(inLeanSpringDamping);
    controller->SetLeanSpringIntegrationCoefficient(
        inLeanSpringIntegrationCoefficient);
    controller->SetLeanSpringIntegrationCoefficientDecay(
        inLeanSpringIntegrationCoefficientDecay);
    controller->SetLeanSmoothingFactor(inLeanSmoothingFactor);
    inHandle->mSystem->GetBodyInterface().ActivateBody(inHandle->mBodyID);
}

inline void SetTrackedVehicleInput(
    VehicleHandle *inHandle,
    float inForward,
    float inLeftRatio,
    float inRightRatio,
    float inBrake)
{
    static_cast<JPH::TrackedVehicleController *>(
        inHandle->mVehicle->GetController())
        ->SetDriverInput(inForward, inLeftRatio, inRightRatio, inBrake);
    inHandle->mSystem->GetBodyInterface().ActivateBody(inHandle->mBodyID);
}

inline void GetTrackedVehiclePowertrainState(
    const VehicleHandle *inHandle,
    float *outEngineRPM,
    std::int32_t *outCurrentGear,
    float *outClutchFriction,
    bool *outSwitchingGear,
    float *outTransmissionRatio)
{
    const JPH::TrackedVehicleController *controller =
        static_cast<const JPH::TrackedVehicleController *>(
            inHandle->mVehicle->GetController());
    const JPH::VehicleTransmission &transmission =
        controller->GetTransmission();
    *outEngineRPM = controller->GetEngine().GetCurrentRPM();
    *outCurrentGear = transmission.GetCurrentGear();
    *outClutchFriction = transmission.GetClutchFriction();
    *outSwitchingGear = transmission.IsSwitchingGear();
    *outTransmissionRatio = transmission.GetCurrentRatio();
}

inline void SetTrackedVehicleEngineRPM(VehicleHandle *inHandle, float inRPM)
{
    JPH::TrackedVehicleController *controller =
        static_cast<JPH::TrackedVehicleController *>(
            inHandle->mVehicle->GetController());
    controller->GetEngine().SetCurrentRPM(inRPM);
    inHandle->mSystem->GetBodyInterface().ActivateBody(inHandle->mBodyID);
}

inline void SetTrackedVehicleTransmission(
    VehicleHandle *inHandle, std::int32_t inGear, float inClutchFriction)
{
    JPH::TrackedVehicleController *controller =
        static_cast<JPH::TrackedVehicleController *>(
            inHandle->mVehicle->GetController());
    controller->GetTransmission().Set(inGear, inClutchFriction);
    inHandle->mSystem->GetBodyInterface().ActivateBody(inHandle->mBodyID);
}

inline void GetTrackedVehicleTrackState(
    const VehicleHandle *inHandle,
    std::uint32_t inTrack,
    std::uint32_t *outDrivenWheel,
    float *outInertia,
    float *outAngularDamping,
    float *outMaxBrakeTorque,
    float *outDifferentialRatio,
    float *outAngularVelocity)
{
    const JPH::TrackedVehicleController *controller =
        static_cast<const JPH::TrackedVehicleController *>(
            inHandle->mVehicle->GetController());
    const JPH::VehicleTrack &track = controller->GetTracks()[inTrack];
    *outDrivenWheel = track.mDrivenWheel;
    *outInertia = track.mInertia;
    *outAngularDamping = track.mAngularDamping;
    *outMaxBrakeTorque = track.mMaxBrakeTorque;
    *outDifferentialRatio = track.mDifferentialRatio;
    *outAngularVelocity = track.mAngularVelocity;
}

inline void GetTrackedVehicleWheelDynamics(
    const VehicleHandle *inHandle,
    std::uint32_t inWheel,
    float *outCombinedLongitudinalFriction,
    float *outCombinedLateralFriction)
{
    const JPH::WheelTV *wheel = static_cast<const JPH::WheelTV *>(
        inHandle->mVehicle->GetWheel(inWheel));
    *outCombinedLongitudinalFriction =
        wheel->mCombinedLongitudinalFriction;
    *outCombinedLateralFriction = wheel->mCombinedLateralFriction;
}

inline void GetVehiclePowertrainState(
    const VehicleHandle *inHandle,
    float *outEngineRPM,
    std::int32_t *outCurrentGear,
    float *outClutchFriction,
    bool *outSwitchingGear,
    float *outTransmissionRatio,
    float *outWheelSpeedAtClutch)
{
    const JPH::WheeledVehicleController *controller =
        static_cast<const JPH::WheeledVehicleController *>(
            inHandle->mVehicle->GetController());
    const JPH::VehicleTransmission &transmission =
        controller->GetTransmission();
    *outEngineRPM = controller->GetEngine().GetCurrentRPM();
    *outCurrentGear = transmission.GetCurrentGear();
    *outClutchFriction = transmission.GetClutchFriction();
    *outSwitchingGear = transmission.IsSwitchingGear();
    *outTransmissionRatio = transmission.GetCurrentRatio();
    *outWheelSpeedAtClutch = controller->GetWheelSpeedAtClutch();
}

inline void SetVehicleEngineRPM(VehicleHandle *inHandle, float inRPM)
{
    JPH::WheeledVehicleController *controller =
        static_cast<JPH::WheeledVehicleController *>(
            inHandle->mVehicle->GetController());
    controller->GetEngine().SetCurrentRPM(inRPM);
    inHandle->mSystem->GetBodyInterface().ActivateBody(inHandle->mBodyID);
}

inline void SetVehicleTransmission(
    VehicleHandle *inHandle, std::int32_t inGear, float inClutchFriction)
{
    JPH::WheeledVehicleController *controller =
        static_cast<JPH::WheeledVehicleController *>(
            inHandle->mVehicle->GetController());
    controller->GetTransmission().Set(inGear, inClutchFriction);
    inHandle->mSystem->GetBodyInterface().ActivateBody(inHandle->mBodyID);
}

inline std::uint32_t GetVehicleDifferentialCount(
    const VehicleHandle *inHandle)
{
    const JPH::WheeledVehicleController *controller =
        static_cast<const JPH::WheeledVehicleController *>(
            inHandle->mVehicle->GetController());
    return static_cast<std::uint32_t>(controller->GetDifferentials().size());
}

inline void GetVehicleDifferentialState(
    const VehicleHandle *inHandle,
    std::uint32_t inDifferential,
    std::int32_t *outLeftWheel,
    std::int32_t *outRightWheel,
    float *outDifferentialRatio,
    float *outLeftRightSplit,
    float *outLimitedSlipRatio,
    float *outEngineTorqueRatio)
{
    const JPH::WheeledVehicleController *controller =
        static_cast<const JPH::WheeledVehicleController *>(
            inHandle->mVehicle->GetController());
    const JPH::VehicleDifferentialSettings &differential =
        controller->GetDifferentials()[inDifferential];
    *outLeftWheel = differential.mLeftWheel;
    *outRightWheel = differential.mRightWheel;
    *outDifferentialRatio = differential.mDifferentialRatio;
    *outLeftRightSplit = differential.mLeftRightSplit;
    *outLimitedSlipRatio = differential.mLimitedSlipRatio;
    *outEngineTorqueRatio = differential.mEngineTorqueRatio;
}

inline void GetVehicleWheelDynamics(
    const VehicleHandle *inHandle,
    std::uint32_t inWheel,
    float *outLongitudinalSlip,
    float *outLateralSlip,
    float *outCombinedLongitudinalFriction,
    float *outCombinedLateralFriction)
{
    const JPH::WheelWV *wheel = static_cast<const JPH::WheelWV *>(
        inHandle->mVehicle->GetWheel(inWheel));
    *outLongitudinalSlip = wheel->mLongitudinalSlip;
    *outLateralSlip = wheel->mLateralSlip;
    *outCombinedLongitudinalFriction =
        wheel->mCombinedLongitudinalFriction;
    *outCombinedLateralFriction = wheel->mCombinedLateralFriction;
}

inline std::uint32_t GetVehicleAntiRollBarCount(
    const VehicleHandle *inHandle)
{
    return static_cast<std::uint32_t>(
        inHandle->mVehicle->GetAntiRollBars().size());
}

inline void GetVehicleAntiRollBarState(
    const VehicleHandle *inHandle,
    std::uint32_t inAntiRollBar,
    std::int32_t *outLeftWheel,
    std::int32_t *outRightWheel,
    float *outStiffness)
{
    const JPH::VehicleAntiRollBar &anti_roll_bar =
        inHandle->mVehicle->GetAntiRollBars()[inAntiRollBar];
    *outLeftWheel = anti_roll_bar.mLeftWheel;
    *outRightWheel = anti_roll_bar.mRightWheel;
    *outStiffness = anti_roll_bar.mStiffness;
}

inline std::uint32_t GetVehicleWheelCount(const VehicleHandle *inHandle)
{
    return static_cast<std::uint32_t>(inHandle->mVehicle->GetWheels().size());
}

inline bool VehicleWheelHasContact(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    return inHandle->mVehicle->GetWheel(inWheel)->HasContact();
}

inline float GetVehicleWheelSuspensionLength(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    return inHandle->mVehicle->GetWheel(inWheel)->GetSuspensionLength();
}

inline float GetVehicleWheelAngularVelocity(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    return inHandle->mVehicle->GetWheel(inWheel)->GetAngularVelocity();
}

inline float GetVehicleWheelSteerAngle(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    return inHandle->mVehicle->GetWheel(inWheel)->GetSteerAngle();
}

inline JPH::Vec3 GetVehicleWheelPosition(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    const JPH::RMat44 transform = inHandle->mVehicle->GetWheelWorldTransform(
        inWheel, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX());
    return JPH::Vec3(transform.GetTranslation());
}

inline JPH::Quat GetVehicleWheelRotation(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    return inHandle->mVehicle->GetWheelWorldTransform(
        inWheel, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX()).GetQuaternion();
}

inline JPH::Vec3 GetVehicleWheelContactPosition(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    const JPH::Wheel *wheel = inHandle->mVehicle->GetWheel(inWheel);
    return wheel->HasContact()? JPH::Vec3(wheel->GetContactPosition()) :
        JPH::Vec3::sZero();
}

inline JPH::Vec3 GetVehicleWheelContactNormal(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    const JPH::Wheel *wheel = inHandle->mVehicle->GetWheel(inWheel);
    return wheel->HasContact()? wheel->GetContactNormal() : JPH::Vec3::sZero();
}

inline std::uint32_t GetVehicleWheelContactBodyID(
    const VehicleHandle *inHandle, std::uint32_t inWheel)
{
    return inHandle->mVehicle->GetWheel(inWheel)->GetContactBodyID()
        .GetIndexAndSequenceNumber();
}

inline void GetVehicleWheelConstraintState(
    const VehicleHandle *inHandle,
    std::uint32_t inWheel,
    bool *outHitHardPoint,
    float *outSuspensionImpulse,
    float *outLongitudinalImpulse,
    float *outLateralImpulse)
{
    const JPH::Wheel *wheel = inHandle->mVehicle->GetWheel(inWheel);
    *outHitHardPoint = wheel->HasHitHardPoint();
    *outSuspensionImpulse = wheel->GetSuspensionLambda();
    *outLongitudinalImpulse = wheel->GetLongitudinalLambda();
    *outLateralImpulse = wheel->GetLateralLambda();
}

inline void GetVehicleWheelContactDetails(
    const VehicleHandle *inHandle,
    std::uint32_t inWheel,
    std::uint32_t *outSubShapeID,
    JPH::Vec3 *outPointVelocity,
    JPH::Vec3 *outLongitudinal,
    JPH::Vec3 *outLateral)
{
    const JPH::Wheel *wheel = inHandle->mVehicle->GetWheel(inWheel);
    *outSubShapeID = wheel->GetContactSubShapeID().GetValue();
    *outPointVelocity = wheel->GetContactPointVelocity();
    *outLongitudinal = wheel->GetContactLongitudinal();
    *outLateral = wheel->GetContactLateral();
}

inline JPH::Shape *CreateConvexHullShape(
    const JPH::Vec3 *inPoints,
    std::uint32_t inPointCount,
    float inMaxConvexRadius,
    const JPH::PhysicsMaterial *inMaterial)
{
    JPH::ConvexHullShapeSettings settings(
        inPoints, static_cast<int>(inPointCount), inMaxConvexRadius);
    settings.mMaterial = inMaterial;
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;

    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateTaperedCapsuleShape(
    float inHalfHeight, float inTopRadius, float inBottomRadius,
    const JPH::PhysicsMaterial *inMaterial)
{
    JPH::TaperedCapsuleShapeSettings settings(
        inHalfHeight, inTopRadius, inBottomRadius);
    settings.mMaterial = inMaterial;
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;
    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateTaperedCylinderShape(
    float inHalfHeight, float inTopRadius, float inBottomRadius,
    float inConvexRadius, const JPH::PhysicsMaterial *inMaterial)
{
    JPH::TaperedCylinderShapeSettings settings(
        inHalfHeight, inTopRadius, inBottomRadius, inConvexRadius);
    settings.mMaterial = inMaterial;
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;
    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateTriangleShape(
    JPH::Vec3Arg inV1, JPH::Vec3Arg inV2, JPH::Vec3Arg inV3,
    float inConvexRadius, const JPH::PhysicsMaterial *inMaterial)
{
    JPH::TriangleShapeSettings settings(inV1, inV2, inV3, inConvexRadius);
    settings.mMaterial = inMaterial;
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;
    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreatePlaneShape(
    JPH::Vec3Arg inNormal, float inConstant, float inHalfExtent,
    const JPH::PhysicsMaterial *inMaterial)
{
    JPH::PlaneShapeSettings settings(
        JPH::Plane(inNormal, inConstant), inMaterial, inHalfExtent);
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;
    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateEmptyShape(JPH::Vec3Arg inCenterOfMass)
{
    JPH::EmptyShape *shape = new JPH::EmptyShape(inCenterOfMass);
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateTriangleMeshShape(
    const JPH::Vec3 *inVertices,
    std::uint32_t inVertexCount,
    const std::uint32_t *inIndices,
    std::uint32_t inTriangleCount,
    JPH::PhysicsMaterial *const *inMaterials,
    std::uint32_t inMaterialCount,
    const std::uint32_t *inMaterialIndices)
{
    JPH::VertexList vertices;
    vertices.reserve(inVertexCount);
    for (std::uint32_t index = 0; index < inVertexCount; ++index)
        vertices.emplace_back(
            inVertices[index].GetX(),
            inVertices[index].GetY(),
            inVertices[index].GetZ());

    JPH::IndexedTriangleList triangles;
    triangles.reserve(inTriangleCount);
    for (std::uint32_t triangle = 0; triangle < inTriangleCount; ++triangle)
    {
        const std::uint32_t offset = triangle * 3;
        triangles.emplace_back(
            inIndices[offset],
            inIndices[offset + 1],
            inIndices[offset + 2],
            inMaterialIndices == nullptr? 0 : inMaterialIndices[triangle]);
    }
    JPH::PhysicsMaterialList materials;
    materials.reserve(inMaterialCount);
    for (std::uint32_t index = 0; index < inMaterialCount; ++index)
        materials.emplace_back(inMaterials[index]);

    JPH::MeshShapeSettings settings(
        std::move(vertices), std::move(triangles), std::move(materials));
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;

    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateHeightFieldShape(
    const float *inSamples,
    std::uint32_t inSampleCount,
    JPH::Vec3Arg inOffset,
    JPH::Vec3Arg inScale,
    std::uint32_t inBlockSize,
    std::uint32_t inBitsPerSample,
    const std::uint8_t *inMaterialIndices,
    JPH::PhysicsMaterial *const *inMaterials,
    std::uint32_t inMaterialCount)
{
    JPH::PhysicsMaterialList materials;
    materials.reserve(inMaterialCount);
    for (std::uint32_t index = 0; index < inMaterialCount; ++index)
        materials.emplace_back(inMaterials[index]);
    JPH::HeightFieldShapeSettings settings(
        inSamples, inOffset, inScale, inSampleCount,
        inMaterialIndices, materials);
    settings.mBlockSize = inBlockSize;
    settings.mBitsPerSample = inBitsPerSample;
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;

    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateStaticCompoundShape(
    JPH::Shape *const *inShapes,
    const JPH::Vec3 *inPositions,
    const JPH::Quat *inRotations,
    std::uint32_t inShapeCount)
{
    JPH::StaticCompoundShapeSettings settings;
    for (std::uint32_t index = 0; index < inShapeCount; ++index)
        settings.AddShape(
            inPositions[index], inRotations[index], inShapes[index]);

    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;

    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateMutableCompoundShape(
    JPH::Shape *const *inShapes,
    const JPH::Vec3 *inPositions,
    const JPH::Quat *inRotations,
    std::uint32_t inShapeCount)
{
    JPH::MutableCompoundShapeSettings settings;
    for (std::uint32_t index = 0; index < inShapeCount; ++index)
        settings.AddShape(
            inPositions[index], inRotations[index], inShapes[index]);

    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;

    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateScaledShape(
    const JPH::Shape *inInnerShape, JPH::Vec3Arg inScale)
{
    JPH::ScaledShapeSettings settings(inInnerShape, inScale);
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;
    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateRotatedTranslatedShape(
    const JPH::Shape *inInnerShape,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation)
{
    JPH::RotatedTranslatedShapeSettings settings(
        inPosition, inRotation, inInnerShape);
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;
    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::Shape *CreateOffsetCenterOfMassShape(
    const JPH::Shape *inInnerShape, JPH::Vec3Arg inOffset)
{
    JPH::OffsetCenterOfMassShapeSettings settings(inOffset, inInnerShape);
    JPH::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError())
        return nullptr;
    JPH::Shape *shape = result.Get().GetPtr();
    shape->AddRef();
    return shape;
}

inline JPH::MutableCompoundShape *GetMutableCompoundShape(JPH::Body &inBody)
{
    const JPH::Shape *shape = inBody.GetShape();
    if (shape->GetSubType() != JPH::EShapeSubType::MutableCompound)
        return nullptr;
    return static_cast<JPH::MutableCompoundShape *>(
        const_cast<JPH::Shape *>(shape));
}

inline void FinishMutableCompoundChange(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    JPH::MutableCompoundShape *inShape,
    bool inActivate)
{
    inShape->AdjustCenterOfMass();
    inSystem->GetBodyInterfaceNoLock().SetShape(
        inBodyID,
        inShape,
        true,
        inActivate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
}

inline bool AddMutableCompoundShape(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation,
    const JPH::Shape *inChildShape,
    bool inActivate,
    std::uint32_t *outIndex)
{
    JPH::BodyLockWrite lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    JPH::MutableCompoundShape *shape = GetMutableCompoundShape(lock.GetBody());
    if (shape == nullptr)
        return false;
    JPH::Ref<JPH::MutableCompoundShape> clone = shape->Clone();
    *outIndex = clone->AddShape(inPosition, inRotation, inChildShape);
    FinishMutableCompoundChange(
        inSystem, inBodyID, clone.GetPtr(), inActivate);
    return true;
}

inline bool RemoveMutableCompoundShape(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inIndex,
    bool inActivate)
{
    JPH::BodyLockWrite lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    JPH::MutableCompoundShape *shape = GetMutableCompoundShape(lock.GetBody());
    if (shape == nullptr || inIndex >= shape->GetNumSubShapes())
        return false;
    JPH::Ref<JPH::MutableCompoundShape> clone = shape->Clone();
    clone->RemoveShape(inIndex);
    FinishMutableCompoundChange(
        inSystem, inBodyID, clone.GetPtr(), inActivate);
    return true;
}

inline bool ModifyMutableCompoundShape(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inIndex,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation,
    const JPH::Shape *inChildShape,
    bool inReplaceShape,
    bool inActivate)
{
    JPH::BodyLockWrite lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    JPH::MutableCompoundShape *shape = GetMutableCompoundShape(lock.GetBody());
    if (shape == nullptr || inIndex >= shape->GetNumSubShapes())
        return false;
    JPH::Ref<JPH::MutableCompoundShape> clone = shape->Clone();
    if (inReplaceShape)
        clone->ModifyShape(inIndex, inPosition, inRotation, inChildShape);
    else
        clone->ModifyShape(inIndex, inPosition, inRotation);
    FinishMutableCompoundChange(
        inSystem, inBodyID, clone.GetPtr(), inActivate);
    return true;
}

inline bool ModifyMutableCompoundShapes(
    JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    std::uint32_t inStartIndex,
    const JPH::Vec3 *inPositions,
    const JPH::Quat *inRotations,
    std::uint32_t inCount,
    bool inActivate)
{
    JPH::BodyLockWrite lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    JPH::MutableCompoundShape *shape = GetMutableCompoundShape(lock.GetBody());
    if (shape == nullptr || inCount == 0 ||
        inStartIndex > shape->GetNumSubShapes() ||
        inCount > shape->GetNumSubShapes() - inStartIndex)
        return false;
    JPH::Ref<JPH::MutableCompoundShape> clone = shape->Clone();
    clone->ModifyShapes(
        inStartIndex, inCount, inPositions, inRotations,
        sizeof(JPH::Vec3), sizeof(JPH::Quat));
    FinishMutableCompoundChange(
        inSystem, inBodyID, clone.GetPtr(), inActivate);
    return true;
}

enum class EventKind : std::uint8_t
{
    ContactAdded,
    ContactPersisted,
    ContactRemoved,
    BodyActivated,
    BodyDeactivated
};

struct PhysicsEvent
{
    EventKind mKind;
    std::uint32_t mBody1;
    std::uint32_t mBody2;
    std::uint32_t mSubShape1;
    std::uint32_t mSubShape2;
    JPH::Vec3 mPoint;
    JPH::Vec3 mNormal;
};

struct SoftBodyContactEvent
{
    std::uint32_t mSoftBody;
    std::uint32_t mOtherBody;
    std::uint32_t mVertex;
    JPH::Vec3 mPoint;
    JPH::Vec3 mNormal;
    bool mIsSensor;
};

struct ContactPolicy
{
    JPH::ObjectLayer mLayer1;
    JPH::ObjectLayer mLayer2;
    std::uint8_t mResponse;
    float mFriction;
    float mRestitution;
    float mInvMassScale1;
    float mInvInertiaScale1;
    float mInvMassScale2;
    float mInvInertiaScale2;
    JPH::Vec3 mLinearSurfaceVelocity;
    JPH::Vec3 mAngularSurfaceVelocity;
};

struct BodyPairContactPolicy
{
    std::uint32_t mFirstBody;
    ContactPolicy mPolicy;
};

struct SubShapePairKey
{
    std::uint32_t mBody1;
    std::uint32_t mSubShape1;
    std::uint32_t mBody2;
    std::uint32_t mSubShape2;

    bool operator==(const SubShapePairKey &inOther) const
    {
        return mBody1 == inOther.mBody1 &&
            mSubShape1 == inOther.mSubShape1 &&
            mBody2 == inOther.mBody2 &&
            mSubShape2 == inOther.mSubShape2;
    }
};

struct SubShapePairKeyHash
{
    std::size_t operator()(const SubShapePairKey &inKey) const
    {
        std::size_t hash = inKey.mBody1;
        hash ^= static_cast<std::size_t>(inKey.mSubShape1) +
            0x9e3779b9U + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(inKey.mBody2) +
            0x9e3779b9U + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(inKey.mSubShape2) +
            0x9e3779b9U + (hash << 6) + (hash >> 2);
        return hash;
    }
};

class EventBridge final : public JPH::ContactListener,
                          public JPH::BodyActivationListener,
                          public JPH::SoftBodyContactListener
{
public:
    EventBridge(
        std::uint32_t inCapacity,
        const JPH::ObjectLayer *inLayer1,
        const JPH::ObjectLayer *inLayer2,
        const std::uint8_t *inResponses,
        const float *inFrictions,
        const float *inRestitutions,
        const float *inInvMassScales1,
        const float *inInvInertiaScales1,
        const float *inInvMassScales2,
        const float *inInvInertiaScales2,
        const JPH::Vec3 *inLinearSurfaceVelocities,
        const JPH::Vec3 *inAngularSurfaceVelocities,
        std::uint32_t inPolicyCount) : mCapacity(inCapacity)
    {
        mPolicies.reserve(inPolicyCount);
        for (std::uint32_t index = 0; index < inPolicyCount; ++index)
            mPolicies.push_back({
                inLayer1[index], inLayer2[index], inResponses[index],
                inFrictions[index], inRestitutions[index],
                inInvMassScales1[index], inInvInertiaScales1[index],
                inInvMassScales2[index], inInvInertiaScales2[index],
                inLinearSurfaceVelocities[index],
                inAngularSurfaceVelocities[index]
            });
    }

    JPH::ValidateResult OnContactValidate(
        const JPH::Body &inBody1,
        const JPH::Body &inBody2,
        JPH::RVec3Arg,
        const JPH::CollideShapeResult &inCollisionResult) override
    {
        ContactPolicy policy;
        bool forward;
        bool hasSubShapeRules;
        const bool found = FindPolicy(
            inBody1, inCollisionResult.mSubShapeID1.GetValue(),
            inBody2, inCollisionResult.mSubShapeID2.GetValue(),
            true, policy, forward, hasSubShapeRules);
        if (found && policy.mResponse == 2)
            return hasSubShapeRules ? JPH::ValidateResult::RejectContact :
                JPH::ValidateResult::RejectAllContactsForThisBodyPair;
        return hasSubShapeRules ? JPH::ValidateResult::AcceptContact :
            JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    void OnContactAdded(
        const JPH::Body &inBody1,
        const JPH::Body &inBody2,
        const JPH::ContactManifold &inManifold,
        JPH::ContactSettings &ioSettings) override
    {
        ApplyPolicy(inBody1, inBody2, inManifold, ioSettings);
        PushContact(EventKind::ContactAdded, inBody1, inBody2, inManifold);
    }

    void OnContactPersisted(
        const JPH::Body &inBody1,
        const JPH::Body &inBody2,
        const JPH::ContactManifold &inManifold,
        JPH::ContactSettings &ioSettings) override
    {
        ApplyPolicy(inBody1, inBody2, inManifold, ioSettings);
        PushContact(EventKind::ContactPersisted, inBody1, inBody2, inManifold);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair &inPair) override
    {
        Push({
            EventKind::ContactRemoved,
            inPair.GetBody1ID().GetIndexAndSequenceNumber(),
            inPair.GetBody2ID().GetIndexAndSequenceNumber(),
            inPair.GetSubShapeID1().GetValue(),
            inPair.GetSubShapeID2().GetValue(),
            JPH::Vec3::sZero(),
            JPH::Vec3::sZero()
        });
    }

    void OnBodyActivated(const JPH::BodyID &inBodyID, JPH::uint64) override
    {
        PushActivation(EventKind::BodyActivated, inBodyID);
    }

    void OnBodyDeactivated(const JPH::BodyID &inBodyID, JPH::uint64) override
    {
        PushActivation(EventKind::BodyDeactivated, inBodyID);
    }

    JPH::SoftBodyValidateResult OnSoftBodyContactValidate(
        const JPH::Body &inSoftBody,
        const JPH::Body &inOtherBody,
        JPH::SoftBodyContactSettings &ioSettings) override
    {
        ContactPolicy policy;
        bool forward;
        bool hasSubShapeRules;
        if (!FindPolicy(
                inSoftBody, 0, inOtherBody, 0, false,
                policy, forward, hasSubShapeRules))
            return JPH::SoftBodyValidateResult::AcceptContact;
        if (policy.mResponse == 2)
            return JPH::SoftBodyValidateResult::RejectContact;
        if (policy.mResponse == 1)
            ioSettings.mIsSensor = true;
        if (forward)
        {
            ioSettings.mInvMassScale1 = policy.mInvMassScale1;
            ioSettings.mInvMassScale2 = policy.mInvMassScale2;
            ioSettings.mInvInertiaScale2 = policy.mInvInertiaScale2;
        }
        else
        {
            ioSettings.mInvMassScale1 = policy.mInvMassScale2;
            ioSettings.mInvMassScale2 = policy.mInvMassScale1;
            ioSettings.mInvInertiaScale2 = policy.mInvInertiaScale1;
        }
        return JPH::SoftBodyValidateResult::AcceptContact;
    }

    void OnSoftBodyContactAdded(
        const JPH::Body &inSoftBody,
        const JPH::SoftBodyManifold &inManifold) override
    {
        const JPH::RMat44 transform = inSoftBody.GetCenterOfMassTransform();
        const JPH::Array<JPH::SoftBodyVertex> &vertices =
            inManifold.GetVertices();
        for (std::uint32_t index = 0; index < vertices.size(); ++index)
        {
            const JPH::SoftBodyVertex &vertex = vertices[index];
            if (!inManifold.HasContact(vertex))
                continue;
            PushSoft({
                inSoftBody.GetID().GetIndexAndSequenceNumber(),
                inManifold.GetContactBodyID(vertex).GetIndexAndSequenceNumber(),
                index,
                JPH::Vec3(transform * inManifold.GetLocalContactPoint(vertex)),
                inManifold.GetContactNormal(vertex),
                false
            });
        }
        for (std::uint32_t index = 0;
             index < inManifold.GetNumSensorContacts(); ++index)
            PushSoft({
                inSoftBody.GetID().GetIndexAndSequenceNumber(),
                inManifold.GetSensorContactBodyID(index).GetIndexAndSequenceNumber(),
                ~std::uint32_t(0),
                JPH::Vec3::sZero(),
                JPH::Vec3::sZero(),
                true
            });
    }

    bool Pop(PhysicsEvent &outEvent)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mEvents.empty())
            return false;
        outEvent = mEvents.front();
        mEvents.pop_front();
        return true;
    }

    bool PopSoft(SoftBodyContactEvent &outEvent)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mSoftEvents.empty())
            return false;
        outEvent = mSoftEvents.front();
        mSoftEvents.pop_front();
        return true;
    }

    std::uint32_t PendingCount() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return static_cast<std::uint32_t>(mEvents.size());
    }

    std::uint32_t PendingSoftCount() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return static_cast<std::uint32_t>(mSoftEvents.size());
    }

    std::uint64_t DroppedCount(bool inReset)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        const std::uint64_t result = mDroppedCount;
        if (inReset)
            mDroppedCount = 0;
        return result;
    }

    void Clear()
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mEvents.clear();
        mSoftEvents.clear();
        mDroppedCount = 0;
    }

    void SetBodyPairPolicy(
        std::uint32_t inBody1,
        std::uint32_t inBody2,
        std::uint8_t inResponse,
        float inFriction,
        float inRestitution,
        float inInvMassScale1,
        float inInvInertiaScale1,
        float inInvMassScale2,
        float inInvInertiaScale2,
        JPH::Vec3Arg inLinearSurfaceVelocity,
        JPH::Vec3Arg inAngularSurfaceVelocity)
    {
        std::unique_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        mBodyPolicies[BodyPairKey(inBody1, inBody2)] = {
            inBody1,
            {
                0, 0, inResponse, inFriction, inRestitution,
                inInvMassScale1, inInvInertiaScale1,
                inInvMassScale2, inInvInertiaScale2,
                inLinearSurfaceVelocity, inAngularSurfaceVelocity
            }
        };
    }

    bool RemoveBodyPairPolicy(std::uint32_t inBody1, std::uint32_t inBody2)
    {
        std::unique_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        return mBodyPolicies.erase(BodyPairKey(inBody1, inBody2)) != 0;
    }

    bool HasBodyPairPolicy(std::uint32_t inBody1, std::uint32_t inBody2) const
    {
        std::shared_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        return mBodyPolicies.find(BodyPairKey(inBody1, inBody2)) !=
            mBodyPolicies.end();
    }

    std::uint32_t BodyPairPolicyCount() const
    {
        std::shared_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        return static_cast<std::uint32_t>(mBodyPolicies.size());
    }

    void SetSubShapePairPolicy(
        std::uint32_t inBody1,
        std::uint32_t inSubShape1,
        std::uint32_t inBody2,
        std::uint32_t inSubShape2,
        std::uint8_t inResponse,
        float inFriction,
        float inRestitution,
        float inInvMassScale1,
        float inInvInertiaScale1,
        float inInvMassScale2,
        float inInvInertiaScale2,
        JPH::Vec3Arg inLinearSurfaceVelocity,
        JPH::Vec3Arg inAngularSurfaceVelocity)
    {
        std::unique_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        const SubShapePairKey key = MakeSubShapePairKey(
            inBody1, inSubShape1, inBody2, inSubShape2);
        const auto existing = mSubShapePolicies.find(key);
        if (existing == mSubShapePolicies.end())
            ++mSubShapePolicyPairCounts[BodyPairKey(inBody1, inBody2)];
        mSubShapePolicies[key] = {
            inBody1,
            {
                0, 0, inResponse, inFriction, inRestitution,
                inInvMassScale1, inInvInertiaScale1,
                inInvMassScale2, inInvInertiaScale2,
                inLinearSurfaceVelocity, inAngularSurfaceVelocity
            }
        };
    }

    bool RemoveSubShapePairPolicy(
        std::uint32_t inBody1,
        std::uint32_t inSubShape1,
        std::uint32_t inBody2,
        std::uint32_t inSubShape2)
    {
        std::unique_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        const SubShapePairKey key = MakeSubShapePairKey(
            inBody1, inSubShape1, inBody2, inSubShape2);
        if (mSubShapePolicies.erase(key) == 0)
            return false;
        DecrementSubShapePairCount(BodyPairKey(inBody1, inBody2));
        return true;
    }

    bool HasSubShapePairPolicy(
        std::uint32_t inBody1,
        std::uint32_t inSubShape1,
        std::uint32_t inBody2,
        std::uint32_t inSubShape2) const
    {
        std::shared_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        return mSubShapePolicies.find(MakeSubShapePairKey(
            inBody1, inSubShape1, inBody2, inSubShape2)) !=
            mSubShapePolicies.end();
    }

    std::uint32_t SubShapePairPolicyCount() const
    {
        std::shared_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        return static_cast<std::uint32_t>(mSubShapePolicies.size());
    }

    void RemoveBodySubShapePolicies(std::uint32_t inBody)
    {
        std::unique_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        RemoveBodySubShapePoliciesLocked(inBody);
    }

    void RemoveBodyPolicies(std::uint32_t inBody)
    {
        std::unique_lock<std::shared_mutex> lock(mBodyPolicyMutex);
        for (auto policy = mBodyPolicies.begin(); policy != mBodyPolicies.end();)
        {
            const std::uint32_t body1 =
                static_cast<std::uint32_t>(policy->first >> 32);
            const std::uint32_t body2 =
                static_cast<std::uint32_t>(policy->first);
            if (body1 == inBody || body2 == inBody)
                policy = mBodyPolicies.erase(policy);
            else
                ++policy;
        }
        RemoveBodySubShapePoliciesLocked(inBody);
    }

private:
    void RemoveBodySubShapePoliciesLocked(std::uint32_t inBody)
    {
        for (auto policy = mSubShapePolicies.begin();
             policy != mSubShapePolicies.end();)
        {
            if (policy->first.mBody1 == inBody ||
                policy->first.mBody2 == inBody)
            {
                DecrementSubShapePairCount(BodyPairKey(
                    policy->first.mBody1, policy->first.mBody2));
                policy = mSubShapePolicies.erase(policy);
            }
            else
                ++policy;
        }
    }

    static std::uint64_t BodyPairKey(
        std::uint32_t inBody1, std::uint32_t inBody2)
    {
        const std::uint32_t lower = std::min(inBody1, inBody2);
        const std::uint32_t upper = std::max(inBody1, inBody2);
        return (static_cast<std::uint64_t>(lower) << 32) | upper;
    }

    static SubShapePairKey MakeSubShapePairKey(
        std::uint32_t inBody1,
        std::uint32_t inSubShape1,
        std::uint32_t inBody2,
        std::uint32_t inSubShape2)
    {
        if (inBody1 < inBody2)
            return { inBody1, inSubShape1, inBody2, inSubShape2 };
        return { inBody2, inSubShape2, inBody1, inSubShape1 };
    }

    void DecrementSubShapePairCount(std::uint64_t inPair)
    {
        const auto count = mSubShapePolicyPairCounts.find(inPair);
        if (count == mSubShapePolicyPairCounts.end())
            return;
        if (--count->second == 0)
            mSubShapePolicyPairCounts.erase(count);
    }

    bool FindPolicy(
        const JPH::Body &inBody1,
        std::uint32_t inSubShape1,
        const JPH::Body &inBody2,
        std::uint32_t inSubShape2,
        bool inCheckSubShapes,
        ContactPolicy &outPolicy,
        bool &outForward,
        bool &outHasSubShapeRules) const
    {
        const std::uint32_t body1 =
            inBody1.GetID().GetIndexAndSequenceNumber();
        const std::uint32_t body2 =
            inBody2.GetID().GetIndexAndSequenceNumber();
        {
            std::shared_lock<std::shared_mutex> lock(mBodyPolicyMutex);
            const std::uint64_t bodyPair = BodyPairKey(body1, body2);
            outHasSubShapeRules =
                mSubShapePolicyPairCounts.find(bodyPair) !=
                mSubShapePolicyPairCounts.end();
            if (inCheckSubShapes)
            {
                const auto subShape = mSubShapePolicies.find(
                    MakeSubShapePairKey(
                        body1, inSubShape1, body2, inSubShape2));
                if (subShape != mSubShapePolicies.end())
                {
                    outPolicy = subShape->second.mPolicy;
                    outForward = subShape->second.mFirstBody == body1;
                    return true;
                }
            }
            const auto exact = mBodyPolicies.find(BodyPairKey(body1, body2));
            if (exact != mBodyPolicies.end())
            {
                outPolicy = exact->second.mPolicy;
                outForward = exact->second.mFirstBody == body1;
                return true;
            }
        }

        const JPH::ObjectLayer layer1 = inBody1.GetObjectLayer();
        const JPH::ObjectLayer layer2 = inBody2.GetObjectLayer();
        for (const ContactPolicy &policy : mPolicies)
        {
            if (policy.mLayer1 == layer1 && policy.mLayer2 == layer2)
            {
                outForward = true;
                outPolicy = policy;
                return true;
            }
            if (policy.mLayer1 == layer2 && policy.mLayer2 == layer1)
            {
                outForward = false;
                outPolicy = policy;
                return true;
            }
        }
        return false;
    }

    void ApplyPolicy(
        const JPH::Body &inBody1,
        const JPH::Body &inBody2,
        const JPH::ContactManifold &inManifold,
        JPH::ContactSettings &ioSettings) const
    {
        ContactPolicy policy;
        bool forward;
        bool hasSubShapeRules;
        if (!FindPolicy(
                inBody1, inManifold.mSubShapeID1.GetValue(),
                inBody2, inManifold.mSubShapeID2.GetValue(),
                true, policy, forward, hasSubShapeRules))
            return;
        if (policy.mResponse == 1)
            ioSettings.mIsSensor = true;
        if (policy.mFriction >= 0.0f)
            ioSettings.mCombinedFriction = policy.mFriction;
        if (policy.mRestitution >= 0.0f)
            ioSettings.mCombinedRestitution = policy.mRestitution;
        if (forward)
        {
            ioSettings.mInvMassScale1 = policy.mInvMassScale1;
            ioSettings.mInvInertiaScale1 = policy.mInvInertiaScale1;
            ioSettings.mInvMassScale2 = policy.mInvMassScale2;
            ioSettings.mInvInertiaScale2 = policy.mInvInertiaScale2;
            ioSettings.mRelativeLinearSurfaceVelocity =
                policy.mLinearSurfaceVelocity;
            ioSettings.mRelativeAngularSurfaceVelocity =
                policy.mAngularSurfaceVelocity;
        }
        else
        {
            ioSettings.mInvMassScale1 = policy.mInvMassScale2;
            ioSettings.mInvInertiaScale1 = policy.mInvInertiaScale2;
            ioSettings.mInvMassScale2 = policy.mInvMassScale1;
            ioSettings.mInvInertiaScale2 = policy.mInvInertiaScale1;
            ioSettings.mRelativeLinearSurfaceVelocity =
                -policy.mLinearSurfaceVelocity;
            ioSettings.mRelativeAngularSurfaceVelocity =
                -policy.mAngularSurfaceVelocity;
        }
    }

    void PushContact(
        EventKind inKind,
        const JPH::Body &inBody1,
        const JPH::Body &inBody2,
        const JPH::ContactManifold &inManifold)
    {
        JPH::Vec3 point = JPH::Vec3::sZero();
        if (!inManifold.mRelativeContactPointsOn1.empty())
        {
            const JPH::Vec3 point1(inManifold.GetWorldSpaceContactPointOn1(0));
            const JPH::Vec3 point2(inManifold.GetWorldSpaceContactPointOn2(0));
            point = 0.5f * (point1 + point2);
        }
        Push({
            inKind,
            inBody1.GetID().GetIndexAndSequenceNumber(),
            inBody2.GetID().GetIndexAndSequenceNumber(),
            inManifold.mSubShapeID1.GetValue(),
            inManifold.mSubShapeID2.GetValue(),
            point,
            inManifold.mWorldSpaceNormal
        });
    }

    void PushActivation(EventKind inKind, const JPH::BodyID &inBodyID)
    {
        Push({
            inKind,
            inBodyID.GetIndexAndSequenceNumber(),
            JPH::BodyID::cInvalidBodyID,
            JPH::SubShapeID().GetValue(),
            JPH::SubShapeID().GetValue(),
            JPH::Vec3::sZero(),
            JPH::Vec3::sZero()
        });
    }

    void Push(PhysicsEvent inEvent)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mEvents.size() == mCapacity)
        {
            mEvents.pop_front();
            ++mDroppedCount;
        }
        mEvents.push_back(inEvent);
    }

    void PushSoft(SoftBodyContactEvent inEvent)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mSoftEvents.size() == mCapacity)
        {
            mSoftEvents.pop_front();
            ++mDroppedCount;
        }
        mSoftEvents.push_back(inEvent);
    }

    const std::size_t mCapacity;
    std::vector<ContactPolicy> mPolicies;
    mutable std::shared_mutex mBodyPolicyMutex;
    std::unordered_map<std::uint64_t, BodyPairContactPolicy> mBodyPolicies;
    std::unordered_map<
        SubShapePairKey, BodyPairContactPolicy,
        SubShapePairKeyHash> mSubShapePolicies;
    std::unordered_map<std::uint64_t, std::uint32_t>
        mSubShapePolicyPairCounts;
    mutable std::mutex mMutex;
    std::deque<PhysicsEvent> mEvents;
    std::deque<SoftBodyContactEvent> mSoftEvents;
    std::uint64_t mDroppedCount = 0;
};

inline EventBridge *CreateEventBridge(
    JPH::PhysicsSystem *ioSystem,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inLayer1,
    const JPH::ObjectLayer *inLayer2,
    const std::uint8_t *inResponses,
    const float *inFrictions,
    const float *inRestitutions,
    const float *inInvMassScales1,
    const float *inInvInertiaScales1,
    const float *inInvMassScales2,
    const float *inInvInertiaScales2,
    const JPH::Vec3 *inLinearSurfaceVelocities,
    const JPH::Vec3 *inAngularSurfaceVelocities,
    std::uint32_t inPolicyCount)
{
    EventBridge *bridge = new EventBridge(
        inCapacity, inLayer1, inLayer2, inResponses, inFrictions,
        inRestitutions, inInvMassScales1, inInvInertiaScales1,
        inInvMassScales2, inInvInertiaScales2,
        inLinearSurfaceVelocities, inAngularSurfaceVelocities,
        inPolicyCount);
    ioSystem->SetContactListener(bridge);
    ioSystem->SetBodyActivationListener(bridge);
    ioSystem->SetSoftBodyContactListener(bridge);
    return bridge;
}

inline void DestroyEventBridge(JPH::PhysicsSystem *ioSystem, EventBridge *inBridge)
{
    ioSystem->SetContactListener(nullptr);
    ioSystem->SetBodyActivationListener(nullptr);
    ioSystem->SetSoftBodyContactListener(nullptr);
    delete inBridge;
}

inline bool PopEvent(
    EventBridge *inBridge,
    std::uint8_t *outKind,
    std::uint32_t *outBody1,
    std::uint32_t *outBody2,
    std::uint32_t *outSubShape1,
    std::uint32_t *outSubShape2,
    JPH::Vec3 *outPoint,
    JPH::Vec3 *outNormal)
{
    PhysicsEvent event;
    if (!inBridge->Pop(event))
        return false;
    *outKind = static_cast<std::uint8_t>(event.mKind);
    *outBody1 = event.mBody1;
    *outBody2 = event.mBody2;
    *outSubShape1 = event.mSubShape1;
    *outSubShape2 = event.mSubShape2;
    *outPoint = event.mPoint;
    *outNormal = event.mNormal;
    return true;
}

inline std::uint32_t PendingEventCount(const EventBridge *inBridge)
{
    return inBridge->PendingCount();
}

inline bool PopSoftBodyContactEvent(
    EventBridge *inBridge,
    std::uint32_t *outSoftBody,
    std::uint32_t *outOtherBody,
    std::uint32_t *outVertex,
    JPH::Vec3 *outPoint,
    JPH::Vec3 *outNormal,
    bool *outIsSensor)
{
    SoftBodyContactEvent event;
    if (!inBridge->PopSoft(event))
        return false;
    *outSoftBody = event.mSoftBody;
    *outOtherBody = event.mOtherBody;
    *outVertex = event.mVertex;
    *outPoint = event.mPoint;
    *outNormal = event.mNormal;
    *outIsSensor = event.mIsSensor;
    return true;
}

inline std::uint32_t PendingSoftBodyContactEventCount(
    const EventBridge *inBridge)
{
    return inBridge->PendingSoftCount();
}

inline std::uint64_t DroppedEventCount(EventBridge *inBridge, bool inReset)
{
    return inBridge->DroppedCount(inReset);
}

inline void SetBodyPairContactPolicy(
    JPH::PhysicsSystem *ioSystem,
    EventBridge *inBridge,
    std::uint32_t inBody1,
    std::uint32_t inBody2,
    std::uint8_t inResponse,
    float inFriction,
    float inRestitution,
    float inInvMassScale1,
    float inInvInertiaScale1,
    float inInvMassScale2,
    float inInvInertiaScale2,
    JPH::Vec3Arg inLinearSurfaceVelocity,
    JPH::Vec3Arg inAngularSurfaceVelocity)
{
    inBridge->SetBodyPairPolicy(
        inBody1, inBody2, inResponse, inFriction, inRestitution,
        inInvMassScale1, inInvInertiaScale1,
        inInvMassScale2, inInvInertiaScale2,
        inLinearSurfaceVelocity, inAngularSurfaceVelocity);
    JPH::BodyInterface &bodies = ioSystem->GetBodyInterface();
    bodies.InvalidateContactCache(JPH::BodyID(inBody1));
    bodies.InvalidateContactCache(JPH::BodyID(inBody2));
}

inline bool RemoveBodyPairContactPolicy(
    JPH::PhysicsSystem *ioSystem,
    EventBridge *inBridge,
    std::uint32_t inBody1,
    std::uint32_t inBody2)
{
    const bool removed = inBridge->RemoveBodyPairPolicy(inBody1, inBody2);
    if (removed)
    {
        JPH::BodyInterface &bodies = ioSystem->GetBodyInterface();
        bodies.InvalidateContactCache(JPH::BodyID(inBody1));
        bodies.InvalidateContactCache(JPH::BodyID(inBody2));
    }
    return removed;
}

inline bool HasBodyPairContactPolicy(
    const EventBridge *inBridge,
    std::uint32_t inBody1,
    std::uint32_t inBody2)
{
    return inBridge->HasBodyPairPolicy(inBody1, inBody2);
}

inline std::uint32_t BodyPairContactPolicyCount(const EventBridge *inBridge)
{
    return inBridge->BodyPairPolicyCount();
}

inline void SetSubShapePairContactPolicy(
    JPH::PhysicsSystem *ioSystem,
    EventBridge *inBridge,
    std::uint32_t inBody1,
    std::uint32_t inSubShape1,
    std::uint32_t inBody2,
    std::uint32_t inSubShape2,
    std::uint8_t inResponse,
    float inFriction,
    float inRestitution,
    float inInvMassScale1,
    float inInvInertiaScale1,
    float inInvMassScale2,
    float inInvInertiaScale2,
    JPH::Vec3Arg inLinearSurfaceVelocity,
    JPH::Vec3Arg inAngularSurfaceVelocity)
{
    inBridge->SetSubShapePairPolicy(
        inBody1, inSubShape1, inBody2, inSubShape2,
        inResponse, inFriction, inRestitution,
        inInvMassScale1, inInvInertiaScale1,
        inInvMassScale2, inInvInertiaScale2,
        inLinearSurfaceVelocity, inAngularSurfaceVelocity);
    JPH::BodyInterface &bodies = ioSystem->GetBodyInterface();
    bodies.InvalidateContactCache(JPH::BodyID(inBody1));
    bodies.InvalidateContactCache(JPH::BodyID(inBody2));
}

inline bool RemoveSubShapePairContactPolicy(
    JPH::PhysicsSystem *ioSystem,
    EventBridge *inBridge,
    std::uint32_t inBody1,
    std::uint32_t inSubShape1,
    std::uint32_t inBody2,
    std::uint32_t inSubShape2)
{
    const bool removed = inBridge->RemoveSubShapePairPolicy(
        inBody1, inSubShape1, inBody2, inSubShape2);
    if (removed)
    {
        JPH::BodyInterface &bodies = ioSystem->GetBodyInterface();
        bodies.InvalidateContactCache(JPH::BodyID(inBody1));
        bodies.InvalidateContactCache(JPH::BodyID(inBody2));
    }
    return removed;
}

inline bool HasSubShapePairContactPolicy(
    const EventBridge *inBridge,
    std::uint32_t inBody1,
    std::uint32_t inSubShape1,
    std::uint32_t inBody2,
    std::uint32_t inSubShape2)
{
    return inBridge->HasSubShapePairPolicy(
        inBody1, inSubShape1, inBody2, inSubShape2);
}

inline std::uint32_t SubShapePairContactPolicyCount(
    const EventBridge *inBridge)
{
    return inBridge->SubShapePairPolicyCount();
}

inline void RemoveBodySubShapeContactPolicies(
    EventBridge *inBridge, std::uint32_t inBody)
{
    inBridge->RemoveBodySubShapePolicies(inBody);
}

inline void RemoveBodyContactPolicies(
    EventBridge *inBridge, std::uint32_t inBody)
{
    inBridge->RemoveBodyPolicies(inBody);
}

struct WorldStateHandle
{
    JPH::StateRecorderImpl mRecorder;
};

inline WorldStateHandle *SaveWorldState(
    JPH::PhysicsSystem *inSystem,
    CharacterHandle *const *inCharacters,
    std::uint32_t inCharacterCount)
{
    WorldStateHandle *state = new WorldStateHandle();
    for (std::uint32_t index = 0; index < inCharacterCount; ++index)
        inCharacters[index]->mCharacter->SaveState(state->mRecorder);
    inSystem->SaveState(
        state->mRecorder,
        JPH::EStateRecorderState::All);
    return state;
}

inline bool RestoreWorldState(
    JPH::PhysicsSystem *ioSystem,
    WorldStateHandle *inState,
    CharacterHandle *const *ioCharacters,
    std::uint32_t inCharacterCount,
    EventBridge *ioEventBridge)
{
    inState->mRecorder.Rewind();
    for (std::uint32_t index = 0; index < inCharacterCount; ++index)
        ioCharacters[index]->mCharacter->RestoreState(inState->mRecorder);
    const bool restored = ioSystem->RestoreState(inState->mRecorder);
    if (restored && !inState->mRecorder.IsFailed())
    {
        ioEventBridge->Clear();
        for (std::uint32_t index = 0; index < inCharacterCount; ++index)
        {
            ioCharacters[index]->mContactBridge.Clear();
            if (ioCharacters[index]->mBroadPhase != nullptr)
                ioCharacters[index]->mBroadPhase->Update(
                    ioCharacters[index]->mCharacter);
        }
        return true;
    }
    return false;
}

inline std::size_t WorldStateSize(WorldStateHandle *inState)
{
    return inState->mRecorder.GetDataSize();
}

inline JPH::Constraint *CreatePointConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::PointConstraintSettings settings;
    settings.mPoint1 = inPoint1;
    settings.mPoint2 = inPoint2;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateDistanceConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2,
    float inMinDistance,
    float inMaxDistance)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::DistanceConstraintSettings settings;
    settings.mPoint1 = inPoint1;
    settings.mPoint2 = inPoint2;
    settings.mMinDistance = inMinDistance;
    settings.mMaxDistance = inMaxDistance;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateFixedConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::FixedConstraintSettings settings;
    settings.mAutoDetectPoint = true;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateFixedConstraint(
    JPH::PhysicsSystem *ioSystem, JPH::BodyID inBody1,
    JPH::BodyID inBody2, JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2, JPH::Vec3Arg inAxisX1,
    JPH::Vec3Arg inAxisY1, JPH::Vec3Arg inAxisX2,
    JPH::Vec3Arg inAxisY2)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::FixedConstraintSettings settings;
    settings.mAutoDetectPoint = false;
    settings.mPoint1 = inPoint1;
    settings.mPoint2 = inPoint2;
    settings.mAxisX1 = inAxisX1;
    settings.mAxisY1 = inAxisY1;
    settings.mAxisX2 = inAxisX2;
    settings.mAxisY2 = inAxisY2;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateHingeConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2,
    JPH::Vec3Arg inHingeAxis,
    float inMinAngle,
    float inMaxAngle)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    const JPH::Vec3 hinge_axis = inHingeAxis.Normalized();
    const JPH::Vec3 normal_axis = hinge_axis.GetNormalizedPerpendicular();
    JPH::HingeConstraintSettings settings;
    settings.mPoint1 = inPoint1;
    settings.mPoint2 = inPoint2;
    settings.mHingeAxis1 = hinge_axis;
    settings.mHingeAxis2 = hinge_axis;
    settings.mNormalAxis1 = normal_axis;
    settings.mNormalAxis2 = normal_axis;
    settings.mLimitsMin = inMinAngle;
    settings.mLimitsMax = inMaxAngle;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateSliderConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2,
    JPH::Vec3Arg inSliderAxis,
    float inMinPosition,
    float inMaxPosition)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::SliderConstraintSettings settings;
    settings.mPoint1 = inPoint1;
    settings.mPoint2 = inPoint2;
    settings.SetSliderAxis(inSliderAxis.Normalized());
    settings.mLimitsMin = inMinPosition;
    settings.mLimitsMax = inMaxPosition;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::SpringSettings MakeSpringSettings(
    std::uint8_t inMode,
    float inValue,
    float inDamping)
{
    return JPH::SpringSettings(
        static_cast<JPH::ESpringMode>(inMode), inValue, inDamping);
}

inline void ApplyMotorSettings(
    JPH::MotorSettings &ioSettings,
    std::uint8_t inSpringMode,
    float inSpringValue,
    float inDamping,
    float inMinForce,
    float inMaxForce,
    float inMinTorque,
    float inMaxTorque)
{
    ioSettings.mSpringSettings = MakeSpringSettings(
        inSpringMode, inSpringValue, inDamping);
    ioSettings.SetForceLimits(inMinForce, inMaxForce);
    ioSettings.SetTorqueLimits(inMinTorque, inMaxTorque);
}

inline JPH::Constraint *CreateConeConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2,
    JPH::Vec3Arg inTwistAxis1,
    JPH::Vec3Arg inTwistAxis2,
    float inHalfConeAngle)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::ConeConstraintSettings settings;
    settings.mPoint1 = inPoint1;
    settings.mPoint2 = inPoint2;
    settings.mTwistAxis1 = inTwistAxis1.Normalized();
    settings.mTwistAxis2 = inTwistAxis2.Normalized();
    settings.mHalfConeAngle = inHalfConeAngle;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateSwingTwistConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2,
    JPH::Vec3Arg inTwistAxis,
    JPH::Vec3Arg inPlaneAxis,
    float inNormalHalfConeAngle,
    float inPlaneHalfConeAngle,
    float inTwistMinAngle,
    float inTwistMaxAngle)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::SwingTwistConstraintSettings settings;
    settings.mPosition1 = inPoint1;
    settings.mPosition2 = inPoint2;
    settings.mTwistAxis1 = settings.mTwistAxis2 = inTwistAxis.Normalized();
    settings.mPlaneAxis1 = settings.mPlaneAxis2 = inPlaneAxis.Normalized();
    settings.mNormalHalfConeAngle = inNormalHalfConeAngle;
    settings.mPlaneHalfConeAngle = inPlaneHalfConeAngle;
    settings.mTwistMinAngle = inTwistMinAngle;
    settings.mTwistMaxAngle = inTwistMaxAngle;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateSixDOFConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inPoint1,
    JPH::Vec3Arg inPoint2,
    JPH::Vec3Arg inAxisX,
    JPH::Vec3Arg inAxisY,
    std::uint8_t inSwingType,
    const float *inLimitMin,
    const float *inLimitMax)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::SixDOFConstraintSettings settings;
    settings.mPosition1 = inPoint1;
    settings.mPosition2 = inPoint2;
    settings.mAxisX1 = settings.mAxisX2 = inAxisX.Normalized();
    settings.mAxisY1 = settings.mAxisY2 = inAxisY.Normalized();
    settings.mSwingType = static_cast<JPH::ESwingType>(inSwingType);
    for (int axis = 0; axis < JPH::SixDOFConstraintSettings::EAxis::Num; ++axis)
    {
        settings.mLimitMin[axis] = inLimitMin[axis];
        settings.mLimitMax[axis] = inLimitMax[axis];
    }
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateGearConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inAxis1,
    JPH::Vec3Arg inAxis2,
    float inRatio,
    JPH::Constraint *inHinge1,
    JPH::Constraint *inHinge2)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::GearConstraintSettings settings;
    settings.mHingeAxis1 = inAxis1.Normalized();
    settings.mHingeAxis2 = inAxis2.Normalized();
    settings.mRatio = inRatio;
    auto *constraint = static_cast<JPH::GearConstraint *>(
        settings.Create(*body1, *body2));
    constraint->SetConstraints(inHinge1, inHinge2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreatePulleyConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBody1,
    JPH::BodyID inBody2,
    JPH::Vec3Arg inBodyPoint1,
    JPH::Vec3Arg inFixedPoint1,
    JPH::Vec3Arg inBodyPoint2,
    JPH::Vec3Arg inFixedPoint2,
    float inRatio,
    float inMinLength,
    float inMaxLength)
{
    const JPH::BodyID body_ids[] = { inBody1, inBody2 };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *body1 = lock.GetBody(0);
    JPH::Body *body2 = lock.GetBody(1);
    if (body1 == nullptr || body2 == nullptr)
        return nullptr;

    JPH::PulleyConstraintSettings settings;
    settings.mBodyPoint1 = inBodyPoint1;
    settings.mFixedPoint1 = inFixedPoint1;
    settings.mBodyPoint2 = inBodyPoint2;
    settings.mFixedPoint2 = inFixedPoint2;
    settings.mRatio = inRatio;
    settings.mMinLength = inMinLength;
    settings.mMaxLength = inMaxLength;
    JPH::Constraint *constraint = settings.Create(*body1, *body2);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreateRackAndPinionConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inPinionBody,
    JPH::BodyID inRackBody,
    JPH::Vec3Arg inHingeAxis,
    JPH::Vec3Arg inSliderAxis,
    float inRatio,
    JPH::Constraint *inHinge,
    JPH::Constraint *inSlider)
{
    const JPH::BodyID body_ids[] = { inPinionBody, inRackBody };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *pinion = lock.GetBody(0);
    JPH::Body *rack = lock.GetBody(1);
    if (pinion == nullptr || rack == nullptr)
        return nullptr;

    JPH::RackAndPinionConstraintSettings settings;
    settings.mHingeAxis = inHingeAxis.Normalized();
    settings.mSliderAxis = inSliderAxis.Normalized();
    settings.mRatio = inRatio;
    auto *constraint = static_cast<JPH::RackAndPinionConstraint *>(
        settings.Create(*pinion, *rack));
    constraint->SetConstraints(inHinge, inSlider);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline JPH::Constraint *CreatePathConstraint(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inPathBody,
    JPH::BodyID inMovingBody,
    const JPH::Vec3 *inPositions,
    const JPH::Vec3 *inTangents,
    const JPH::Vec3 *inNormals,
    std::uint32_t inPointCount,
    bool inLooping,
    JPH::Vec3Arg inPathPosition,
    JPH::QuatArg inPathRotation,
    float inPathFraction,
    float inMaxFrictionForce,
    std::uint8_t inRotationConstraintType)
{
    const JPH::BodyID body_ids[] = { inPathBody, inMovingBody };
    JPH::BodyLockMultiWrite lock(ioSystem->GetBodyLockInterface(), body_ids, 2);
    JPH::Body *path_body = lock.GetBody(0);
    JPH::Body *moving_body = lock.GetBody(1);
    if (path_body == nullptr || moving_body == nullptr)
        return nullptr;

    JPH::Ref<JPH::PathConstraintPathHermite> path =
        new JPH::PathConstraintPathHermite();
    for (std::uint32_t point = 0; point < inPointCount; ++point)
        path->AddPoint(inPositions[point], inTangents[point], inNormals[point]);
    path->SetIsLooping(inLooping);

    JPH::PathConstraintSettings settings;
    settings.mPath = path;
    settings.mPathPosition = inPathPosition;
    settings.mPathRotation = inPathRotation.Normalized();
    settings.mPathFraction = inPathFraction;
    settings.mMaxFrictionForce = inMaxFrictionForce;
    settings.mRotationConstraintType =
        static_cast<JPH::EPathRotationConstraintType>(inRotationConstraintType);
    JPH::Constraint *constraint = settings.Create(*path_body, *moving_body);
    ioSystem->AddConstraint(constraint);
    return constraint;
}

inline bool GetConstraintSolverImpulse(
    const JPH::Constraint *inConstraint,
    JPH::Vec3 *outPosition,
    JPH::Vec3 *outRotation,
    float *outLimit,
    JPH::Vec3 *outMotorTranslation,
    JPH::Vec3 *outMotorRotation)
{
    *outPosition = JPH::Vec3::sZero();
    *outRotation = JPH::Vec3::sZero();
    *outLimit = 0.0f;
    *outMotorTranslation = JPH::Vec3::sZero();
    *outMotorRotation = JPH::Vec3::sZero();
    const auto vec2 = [](const JPH::Vector<2> &inValue) {
        return JPH::Vec3(inValue[0], inValue[1], 0.0f);
    };
    switch (inConstraint->GetSubType())
    {
    case JPH::EConstraintSubType::Fixed:
    {
        const auto *constraint = static_cast<const JPH::FixedConstraint *>(inConstraint);
        *outPosition = constraint->GetTotalLambdaPosition();
        *outRotation = constraint->GetTotalLambdaRotation();
        return true;
    }
    case JPH::EConstraintSubType::Point:
        *outPosition = static_cast<const JPH::PointConstraint *>(
            inConstraint)->GetTotalLambdaPosition();
        return true;
    case JPH::EConstraintSubType::Hinge:
    {
        const auto *constraint = static_cast<const JPH::HingeConstraint *>(inConstraint);
        *outPosition = constraint->GetTotalLambdaPosition();
        *outRotation = vec2(constraint->GetTotalLambdaRotation());
        *outLimit = constraint->GetTotalLambdaRotationLimits();
        outMotorRotation->SetX(constraint->GetTotalLambdaMotor());
        return true;
    }
    case JPH::EConstraintSubType::Slider:
    {
        const auto *constraint = static_cast<const JPH::SliderConstraint *>(inConstraint);
        *outPosition = vec2(constraint->GetTotalLambdaPosition());
        *outRotation = constraint->GetTotalLambdaRotation();
        *outLimit = constraint->GetTotalLambdaPositionLimits();
        outMotorTranslation->SetX(constraint->GetTotalLambdaMotor());
        return true;
    }
    case JPH::EConstraintSubType::Distance:
        outPosition->SetX(static_cast<const JPH::DistanceConstraint *>(
            inConstraint)->GetTotalLambdaPosition());
        return true;
    case JPH::EConstraintSubType::Cone:
    {
        const auto *constraint = static_cast<const JPH::ConeConstraint *>(inConstraint);
        *outPosition = constraint->GetTotalLambdaPosition();
        outRotation->SetX(constraint->GetTotalLambdaRotation());
        return true;
    }
    case JPH::EConstraintSubType::SwingTwist:
    {
        const auto *constraint = static_cast<const JPH::SwingTwistConstraint *>(inConstraint);
        *outPosition = constraint->GetTotalLambdaPosition();
        *outRotation = JPH::Vec3(
            constraint->GetTotalLambdaTwist(),
            constraint->GetTotalLambdaSwingY(),
            constraint->GetTotalLambdaSwingZ());
        *outMotorRotation = constraint->GetTotalLambdaMotor();
        return true;
    }
    case JPH::EConstraintSubType::SixDOF:
    {
        const auto *constraint = static_cast<const JPH::SixDOFConstraint *>(inConstraint);
        *outPosition = constraint->GetTotalLambdaPosition();
        *outRotation = constraint->GetTotalLambdaRotation();
        *outMotorTranslation = constraint->GetTotalLambdaMotorTranslation();
        *outMotorRotation = constraint->GetTotalLambdaMotorRotation();
        return true;
    }
    case JPH::EConstraintSubType::Path:
    {
        const auto *constraint = static_cast<const JPH::PathConstraint *>(inConstraint);
        *outPosition = vec2(constraint->GetTotalLambdaPosition());
        *outLimit = constraint->GetTotalLambdaPositionLimits();
        *outRotation = constraint->GetTotalLambdaRotation() +
            vec2(constraint->GetTotalLambdaRotationHinge());
        outMotorTranslation->SetX(constraint->GetTotalLambdaMotor());
        return true;
    }
    case JPH::EConstraintSubType::RackAndPinion:
        outPosition->SetX(static_cast<const JPH::RackAndPinionConstraint *>(
            inConstraint)->GetTotalLambda());
        return true;
    case JPH::EConstraintSubType::Gear:
        outRotation->SetX(static_cast<const JPH::GearConstraint *>(
            inConstraint)->GetTotalLambda());
        return true;
    case JPH::EConstraintSubType::Pulley:
        outPosition->SetX(static_cast<const JPH::PulleyConstraint *>(
            inConstraint)->GetTotalLambdaPosition());
        return true;
    default:
        return false;
    }
}

inline bool GetConstraintFriction(
    const JPH::Constraint *inConstraint, std::uint8_t inAxis,
    float *outFriction)
{
    if (inConstraint == nullptr || outFriction == nullptr)
        return false;
    switch (inConstraint->GetSubType())
    {
    case JPH::EConstraintSubType::Hinge:
        *outFriction = static_cast<const JPH::HingeConstraint *>(
            inConstraint)->GetMaxFrictionTorque();
        return true;
    case JPH::EConstraintSubType::Slider:
        *outFriction = static_cast<const JPH::SliderConstraint *>(
            inConstraint)->GetMaxFrictionForce();
        return true;
    case JPH::EConstraintSubType::SwingTwist:
        *outFriction = static_cast<const JPH::SwingTwistConstraint *>(
            inConstraint)->GetMaxFrictionTorque();
        return true;
    case JPH::EConstraintSubType::SixDOF:
        if (inAxis >= static_cast<std::uint8_t>(
                JPH::SixDOFConstraint::EAxis::Num))
            return false;
        *outFriction = static_cast<const JPH::SixDOFConstraint *>(
            inConstraint)->GetMaxFriction(
                static_cast<JPH::SixDOFConstraint::EAxis>(inAxis));
        return true;
    case JPH::EConstraintSubType::Path:
        *outFriction = static_cast<const JPH::PathConstraint *>(
            inConstraint)->GetMaxFrictionForce();
        return true;
    default:
        return false;
    }
}

inline bool GetConstraintLimitSpring(
    const JPH::Constraint *inConstraint, std::uint8_t inAxis,
    std::uint8_t *outMode, float *outValue, float *outDamping)
{
    if (inConstraint == nullptr || outMode == nullptr ||
        outValue == nullptr || outDamping == nullptr)
        return false;
    const JPH::SpringSettings *settings = nullptr;
    switch (inConstraint->GetSubType())
    {
    case JPH::EConstraintSubType::Distance:
        settings = &static_cast<const JPH::DistanceConstraint *>(
            inConstraint)->GetLimitsSpringSettings();
        break;
    case JPH::EConstraintSubType::Hinge:
        settings = &static_cast<const JPH::HingeConstraint *>(
            inConstraint)->GetLimitsSpringSettings();
        break;
    case JPH::EConstraintSubType::Slider:
        settings = &static_cast<const JPH::SliderConstraint *>(
            inConstraint)->GetLimitsSpringSettings();
        break;
    case JPH::EConstraintSubType::SixDOF:
        if (inAxis >= static_cast<std::uint8_t>(
                JPH::SixDOFConstraint::EAxis::NumTranslation))
            return false;
        settings = &static_cast<const JPH::SixDOFConstraint *>(
            inConstraint)->GetLimitsSpringSettings(
                static_cast<JPH::SixDOFConstraint::EAxis>(inAxis));
        break;
    default:
        return false;
    }
    *outMode = static_cast<std::uint8_t>(settings->mMode);
    *outValue = settings->mFrequency;
    *outDamping = settings->mDamping;
    return true;
}

inline bool GetConstraintMotorSettings(
    const JPH::Constraint *inConstraint, std::uint8_t inMotorIndex,
    std::uint8_t *outSpringMode, float *outSpringValue,
    float *outSpringDamping, float *outMinForce, float *outMaxForce,
    float *outMinTorque, float *outMaxTorque)
{
    if (inConstraint == nullptr || outSpringMode == nullptr ||
        outSpringValue == nullptr || outSpringDamping == nullptr ||
        outMinForce == nullptr || outMaxForce == nullptr ||
        outMinTorque == nullptr || outMaxTorque == nullptr)
        return false;
    const JPH::MotorSettings *motor = nullptr;
    switch (inConstraint->GetSubType())
    {
    case JPH::EConstraintSubType::Hinge:
        motor = &static_cast<const JPH::HingeConstraint *>(
            inConstraint)->GetMotorSettings();
        break;
    case JPH::EConstraintSubType::Slider:
        motor = &static_cast<const JPH::SliderConstraint *>(
            inConstraint)->GetMotorSettings();
        break;
    case JPH::EConstraintSubType::SwingTwist:
        if (inMotorIndex > 1)
            return false;
        motor = inMotorIndex == 0?
            &static_cast<const JPH::SwingTwistConstraint *>(
                inConstraint)->GetSwingMotorSettings() :
            &static_cast<const JPH::SwingTwistConstraint *>(
                inConstraint)->GetTwistMotorSettings();
        break;
    case JPH::EConstraintSubType::SixDOF:
        if (inMotorIndex >= static_cast<std::uint8_t>(
                JPH::SixDOFConstraint::EAxis::Num))
            return false;
        motor = &static_cast<const JPH::SixDOFConstraint *>(
            inConstraint)->GetMotorSettings(
                static_cast<JPH::SixDOFConstraint::EAxis>(inMotorIndex));
        break;
    case JPH::EConstraintSubType::Path:
        motor = &static_cast<const JPH::PathConstraint *>(
            inConstraint)->GetPositionMotorSettings();
        break;
    default:
        return false;
    }
    *outSpringMode = static_cast<std::uint8_t>(motor->mSpringSettings.mMode);
    *outSpringValue = motor->mSpringSettings.mFrequency;
    *outSpringDamping = motor->mSpringSettings.mDamping;
    *outMinForce = motor->mMinForceLimit;
    *outMaxForce = motor->mMaxForceLimit;
    *outMinTorque = motor->mMinTorqueLimit;
    *outMaxTorque = motor->mMaxTorqueLimit;
    return true;
}

inline bool GetSwingTwistMotorState(
    const JPH::Constraint *inConstraint, std::uint8_t *outSwingState,
    std::uint8_t *outTwistState, JPH::Vec3 *outAngularVelocity,
    JPH::Quat *outOrientation)
{
    if (inConstraint == nullptr ||
        inConstraint->GetSubType() != JPH::EConstraintSubType::SwingTwist)
        return false;
    const auto *constraint = static_cast<const JPH::SwingTwistConstraint *>(
        inConstraint);
    *outSwingState = static_cast<std::uint8_t>(
        constraint->GetSwingMotorState());
    *outTwistState = static_cast<std::uint8_t>(
        constraint->GetTwistMotorState());
    *outAngularVelocity = constraint->GetTargetAngularVelocityCS();
    *outOrientation = constraint->GetTargetOrientationCS();
    return true;
}

inline bool GetSixDOFMotorState(
    const JPH::Constraint *inConstraint, std::uint8_t inAxis,
    std::uint8_t *outState, JPH::Vec3 *outVelocity,
    JPH::Vec3 *outAngularVelocity, JPH::Vec3 *outPosition,
    JPH::Quat *outOrientation)
{
    if (inConstraint == nullptr ||
        inConstraint->GetSubType() != JPH::EConstraintSubType::SixDOF ||
        inAxis >= static_cast<std::uint8_t>(
            JPH::SixDOFConstraint::EAxis::Num))
        return false;
    const auto *constraint = static_cast<const JPH::SixDOFConstraint *>(
        inConstraint);
    *outState = static_cast<std::uint8_t>(constraint->GetMotorState(
        static_cast<JPH::SixDOFConstraint::EAxis>(inAxis)));
    *outVelocity = constraint->GetTargetVelocityCS();
    *outAngularVelocity = constraint->GetTargetAngularVelocityCS();
    *outPosition = constraint->GetTargetPositionCS();
    *outOrientation = constraint->GetTargetOrientationCS();
    return true;
}

inline std::uint8_t GetConstraintSubTypeValue(
    const JPH::Constraint *inConstraint)
{
    return static_cast<std::uint8_t>(inConstraint->GetSubType());
}

inline bool GetTwoBodyConstraintBodyIDs(
    const JPH::Constraint *inConstraint,
    std::uint32_t *outBody1, bool *outBody1IsFixed,
    std::uint32_t *outBody2, bool *outBody2IsFixed)
{
    if (inConstraint == nullptr ||
        inConstraint->GetType() != JPH::EConstraintType::TwoBodyConstraint)
        return false;
    const auto *constraint =
        static_cast<const JPH::TwoBodyConstraint *>(inConstraint);
    const JPH::BodyID body1 = constraint->GetBody1()->GetID();
    const JPH::BodyID body2 = constraint->GetBody2()->GetID();
    *outBody1IsFixed = body1.IsInvalid();
    *outBody2IsFixed = body2.IsInvalid();
    *outBody1 = body1.GetIndexAndSequenceNumber();
    *outBody2 = body2.GetIndexAndSequenceNumber();
    return true;
}

inline float GetGearTotalLambda(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::GearConstraint *>(inConstraint)->GetTotalLambda();
}

inline float GetRackAndPinionTotalLambda(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::RackAndPinionConstraint *>(inConstraint)
        ->GetTotalLambda();
}

inline float GetPulleyCurrentLength(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::PulleyConstraint *>(inConstraint)
        ->GetCurrentLength();
}

inline void GetPulleyLengths(
    const JPH::Constraint *inConstraint,
    float *outMinimum,
    float *outMaximum)
{
    const auto *constraint =
        static_cast<const JPH::PulleyConstraint *>(inConstraint);
    *outMinimum = constraint->GetMinLength();
    *outMaximum = constraint->GetMaxLength();
}

inline void SetPulleyLengths(
    JPH::Constraint *inConstraint,
    float inMinimum,
    float inMaximum)
{
    static_cast<JPH::PulleyConstraint *>(inConstraint)
        ->SetLength(inMinimum, inMaximum);
}

inline float GetPathFraction(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::PathConstraint *>(inConstraint)
        ->GetPathFraction();
}

inline float GetPathMaxFraction(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::PathConstraint *>(inConstraint)
        ->GetPath()->GetPathMaxFraction();
}

inline void SetPathFriction(JPH::Constraint *inConstraint, float inForce)
{
    static_cast<JPH::PathConstraint *>(inConstraint)
        ->SetMaxFrictionForce(inForce);
}

inline void ConfigurePathMotor(
    JPH::Constraint *inConstraint,
    std::uint8_t inSpringMode,
    float inSpringValue,
    float inDamping,
    float inMinForce,
    float inMaxForce)
{
    JPH::MotorSettings &motor =
        static_cast<JPH::PathConstraint *>(inConstraint)
            ->GetPositionMotorSettings();
    ApplyMotorSettings(
        motor, inSpringMode, inSpringValue, inDamping,
        inMinForce, inMaxForce, 0.0f, 0.0f);
}

inline void SetPathMotorState(JPH::Constraint *inConstraint, std::uint8_t inState)
{
    static_cast<JPH::PathConstraint *>(inConstraint)->SetPositionMotorState(
        static_cast<JPH::EMotorState>(inState));
}

inline void SetPathMotorTargets(
    JPH::Constraint *inConstraint,
    float inVelocity,
    float inFraction)
{
    JPH::PathConstraint *constraint =
        static_cast<JPH::PathConstraint *>(inConstraint);
    constraint->SetTargetVelocity(inVelocity);
    constraint->SetTargetPathFraction(inFraction);
}

inline void GetPathMotor(
    const JPH::Constraint *inConstraint,
    std::uint8_t *outState,
    float *outVelocity,
    float *outFraction)
{
    const JPH::PathConstraint *constraint =
        static_cast<const JPH::PathConstraint *>(inConstraint);
    *outState = static_cast<std::uint8_t>(constraint->GetPositionMotorState());
    *outVelocity = constraint->GetTargetVelocity();
    *outFraction = constraint->GetTargetPathFraction();
}

inline float GetHingeAngle(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::HingeConstraint *>(inConstraint)->GetCurrentAngle();
}

inline float GetSliderPosition(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::SliderConstraint *>(inConstraint)->GetCurrentPosition();
}

inline void SetHingeLimits(JPH::Constraint *inConstraint, float inMinimum, float inMaximum)
{
    static_cast<JPH::HingeConstraint *>(inConstraint)->SetLimits(inMinimum, inMaximum);
}

inline void SetSliderLimits(JPH::Constraint *inConstraint, float inMinimum, float inMaximum)
{
    static_cast<JPH::SliderConstraint *>(inConstraint)->SetLimits(inMinimum, inMaximum);
}

inline void SetHingeFriction(JPH::Constraint *inConstraint, float inTorque)
{
    static_cast<JPH::HingeConstraint *>(inConstraint)->SetMaxFrictionTorque(inTorque);
}

inline void SetSliderFriction(JPH::Constraint *inConstraint, float inForce)
{
    static_cast<JPH::SliderConstraint *>(inConstraint)->SetMaxFrictionForce(inForce);
}

inline void ConfigureHingeMotor(
    JPH::Constraint *inConstraint,
    std::uint8_t inSpringMode,
    float inSpringValue,
    float inDamping,
    float inMinTorque,
    float inMaxTorque)
{
    JPH::MotorSettings &settings =
        static_cast<JPH::HingeConstraint *>(inConstraint)->GetMotorSettings();
    ApplyMotorSettings(
        settings, inSpringMode, inSpringValue, inDamping,
        -FLT_MAX, FLT_MAX, inMinTorque, inMaxTorque);
}

inline void ConfigureSliderMotor(
    JPH::Constraint *inConstraint,
    std::uint8_t inSpringMode,
    float inSpringValue,
    float inDamping,
    float inMinForce,
    float inMaxForce)
{
    JPH::MotorSettings &settings =
        static_cast<JPH::SliderConstraint *>(inConstraint)->GetMotorSettings();
    ApplyMotorSettings(
        settings, inSpringMode, inSpringValue, inDamping,
        inMinForce, inMaxForce, -FLT_MAX, FLT_MAX);
}

inline void SetHingeMotorState(JPH::Constraint *inConstraint, std::uint8_t inState)
{
    static_cast<JPH::HingeConstraint *>(inConstraint)->SetMotorState(
        static_cast<JPH::EMotorState>(inState));
}

inline void SetSliderMotorState(JPH::Constraint *inConstraint, std::uint8_t inState)
{
    static_cast<JPH::SliderConstraint *>(inConstraint)->SetMotorState(
        static_cast<JPH::EMotorState>(inState));
}

inline void SetHingeMotorTarget(
    JPH::Constraint *inConstraint, float inVelocity, float inPosition)
{
    JPH::HingeConstraint *constraint =
        static_cast<JPH::HingeConstraint *>(inConstraint);
    constraint->SetTargetAngularVelocity(inVelocity);
    constraint->SetTargetAngle(inPosition);
}

inline void SetSliderMotorTarget(
    JPH::Constraint *inConstraint, float inVelocity, float inPosition)
{
    JPH::SliderConstraint *constraint =
        static_cast<JPH::SliderConstraint *>(inConstraint);
    constraint->SetTargetVelocity(inVelocity);
    constraint->SetTargetPosition(inPosition);
}

inline void GetHingeMotor(
    const JPH::Constraint *inConstraint,
    std::uint8_t *outState,
    float *outVelocity,
    float *outPosition)
{
    const JPH::HingeConstraint *constraint =
        static_cast<const JPH::HingeConstraint *>(inConstraint);
    *outState = static_cast<std::uint8_t>(constraint->GetMotorState());
    *outVelocity = constraint->GetTargetAngularVelocity();
    *outPosition = constraint->GetTargetAngle();
}

inline void GetSliderMotor(
    const JPH::Constraint *inConstraint,
    std::uint8_t *outState,
    float *outVelocity,
    float *outPosition)
{
    const JPH::SliderConstraint *constraint =
        static_cast<const JPH::SliderConstraint *>(inConstraint);
    *outState = static_cast<std::uint8_t>(constraint->GetMotorState());
    *outVelocity = constraint->GetTargetVelocity();
    *outPosition = constraint->GetTargetPosition();
}

inline void GetDistanceLimits(
    const JPH::Constraint *inConstraint,
    float *outMinimum, float *outMaximum)
{
    const auto *constraint = static_cast<const JPH::DistanceConstraint *>(
        inConstraint);
    *outMinimum = constraint->GetMinDistance();
    *outMaximum = constraint->GetMaxDistance();
}

inline void SetDistanceLimits(
    JPH::Constraint *inConstraint, float inMinimum, float inMaximum)
{
    static_cast<JPH::DistanceConstraint *>(inConstraint)->SetDistance(
        inMinimum, inMaximum);
}

inline void SetDistanceLimitSpring(
    JPH::Constraint *inConstraint, std::uint8_t inMode,
    float inValue, float inDamping)
{
    static_cast<JPH::DistanceConstraint *>(inConstraint)
        ->SetLimitsSpringSettings(MakeSpringSettings(inMode, inValue, inDamping));
}

inline void SetHingeLimitSpring(
    JPH::Constraint *inConstraint,
    std::uint8_t inMode,
    float inValue,
    float inDamping)
{
    static_cast<JPH::HingeConstraint *>(inConstraint)
        ->SetLimitsSpringSettings(MakeSpringSettings(inMode, inValue, inDamping));
}

inline void SetSliderLimitSpring(
    JPH::Constraint *inConstraint,
    std::uint8_t inMode,
    float inValue,
    float inDamping)
{
    static_cast<JPH::SliderConstraint *>(inConstraint)
        ->SetLimitsSpringSettings(MakeSpringSettings(inMode, inValue, inDamping));
}

inline float GetConeHalfAngle(const JPH::Constraint *inConstraint)
{
    return std::acos(static_cast<const JPH::ConeConstraint *>(inConstraint)
        ->GetCosHalfConeAngle());
}

inline void SetConeHalfAngle(JPH::Constraint *inConstraint, float inAngle)
{
    static_cast<JPH::ConeConstraint *>(inConstraint)->SetHalfConeAngle(inAngle);
}

inline void SetSwingTwistLimits(
    JPH::Constraint *inConstraint,
    float inNormalHalfConeAngle,
    float inPlaneHalfConeAngle,
    float inTwistMinAngle,
    float inTwistMaxAngle)
{
    JPH::SwingTwistConstraint *constraint =
        static_cast<JPH::SwingTwistConstraint *>(inConstraint);
    constraint->SetNormalHalfConeAngle(inNormalHalfConeAngle);
    constraint->SetPlaneHalfConeAngle(inPlaneHalfConeAngle);
    constraint->SetTwistMinAngle(inTwistMinAngle);
    constraint->SetTwistMaxAngle(inTwistMaxAngle);
}

inline JPH::Quat GetSwingTwistRotation(const JPH::Constraint *inConstraint)
{
    return static_cast<const JPH::SwingTwistConstraint *>(inConstraint)
        ->GetRotationInConstraintSpace();
}

inline void SetSwingTwistFriction(JPH::Constraint *inConstraint, float inTorque)
{
    static_cast<JPH::SwingTwistConstraint *>(inConstraint)
        ->SetMaxFrictionTorque(inTorque);
}

inline void ConfigureSwingTwistMotor(
    JPH::Constraint *inConstraint,
    bool inSwing,
    std::uint8_t inSpringMode,
    float inSpringValue,
    float inDamping,
    float inMinTorque,
    float inMaxTorque)
{
    JPH::SwingTwistConstraint *constraint =
        static_cast<JPH::SwingTwistConstraint *>(inConstraint);
    JPH::MotorSettings &settings = inSwing?
        constraint->GetSwingMotorSettings() : constraint->GetTwistMotorSettings();
    ApplyMotorSettings(
        settings, inSpringMode, inSpringValue, inDamping,
        -FLT_MAX, FLT_MAX, inMinTorque, inMaxTorque);
}

inline void SetSwingTwistMotorState(
    JPH::Constraint *inConstraint,
    bool inSwing,
    std::uint8_t inState)
{
    JPH::SwingTwistConstraint *constraint =
        static_cast<JPH::SwingTwistConstraint *>(inConstraint);
    if (inSwing)
        constraint->SetSwingMotorState(static_cast<JPH::EMotorState>(inState));
    else
        constraint->SetTwistMotorState(static_cast<JPH::EMotorState>(inState));
}

inline void SetSwingTwistMotorTargets(
    JPH::Constraint *inConstraint,
    JPH::Vec3Arg inAngularVelocity,
    JPH::QuatArg inOrientation)
{
    JPH::SwingTwistConstraint *constraint =
        static_cast<JPH::SwingTwistConstraint *>(inConstraint);
    constraint->SetTargetAngularVelocityCS(inAngularVelocity);
    constraint->SetTargetOrientationCS(inOrientation);
}

inline void GetSixDOFAxisLimit(
    const JPH::Constraint *inConstraint,
    std::uint8_t inAxis,
    float *outMinimum,
    float *outMaximum,
    std::uint8_t *outMode)
{
    const JPH::SixDOFConstraint *constraint =
        static_cast<const JPH::SixDOFConstraint *>(inConstraint);
    const auto axis = static_cast<JPH::SixDOFConstraint::EAxis>(inAxis);
    *outMinimum = constraint->GetLimitsMin(axis);
    *outMaximum = constraint->GetLimitsMax(axis);
    *outMode = constraint->IsFreeAxis(axis)? 0 : constraint->IsFixedAxis(axis)? 1 : 2;
}

inline std::uint8_t GetSixDOFSwingType(
    const JPH::Constraint *inConstraint)
{
    const JPH::Ref<JPH::ConstraintSettings> base_settings =
        inConstraint->GetConstraintSettings();
    const JPH::SixDOFConstraintSettings *settings =
        static_cast<const JPH::SixDOFConstraintSettings *>(
            base_settings.GetPtr());
    return static_cast<std::uint8_t>(settings->mSwingType);
}

inline void SetSixDOFAxisLimit(
    JPH::Constraint *inConstraint,
    std::uint8_t inAxis,
    std::uint8_t inMode,
    float inMinimum,
    float inMaximum)
{
    JPH::SixDOFConstraint *constraint =
        static_cast<JPH::SixDOFConstraint *>(inConstraint);
    const int axis = static_cast<int>(inAxis);
    const float minimum = inMode == 0? -FLT_MAX : inMode == 1? FLT_MAX : inMinimum;
    const float maximum = inMode == 0? FLT_MAX : inMode == 1? -FLT_MAX : inMaximum;
    if (axis < 3)
    {
        JPH::Vec3 limits_min = constraint->GetTranslationLimitsMin();
        JPH::Vec3 limits_max = constraint->GetTranslationLimitsMax();
        limits_min.SetComponent(axis, minimum);
        limits_max.SetComponent(axis, maximum);
        constraint->SetTranslationLimits(limits_min, limits_max);
    }
    else
    {
        JPH::Vec3 limits_min = constraint->GetRotationLimitsMin();
        JPH::Vec3 limits_max = constraint->GetRotationLimitsMax();
        limits_min.SetComponent(axis - 3, minimum);
        limits_max.SetComponent(axis - 3, maximum);
        constraint->SetRotationLimits(limits_min, limits_max);
    }
}

inline void SetSixDOFFriction(
    JPH::Constraint *inConstraint, std::uint8_t inAxis, float inFriction)
{
    static_cast<JPH::SixDOFConstraint *>(inConstraint)->SetMaxFriction(
        static_cast<JPH::SixDOFConstraint::EAxis>(inAxis), inFriction);
}

inline void SetSixDOFLimitSpring(
    JPH::Constraint *inConstraint, std::uint8_t inAxis,
    std::uint8_t inSpringMode, float inSpringValue, float inDamping)
{
    static_cast<JPH::SixDOFConstraint *>(inConstraint)->
        SetLimitsSpringSettings(
            static_cast<JPH::SixDOFConstraint::EAxis>(inAxis),
            JPH::SpringSettings(
                static_cast<JPH::ESpringMode>(inSpringMode),
                inSpringValue, inDamping));
}

inline void ConfigureSixDOFMotor(
    JPH::Constraint *inConstraint,
    std::uint8_t inAxis,
    std::uint8_t inSpringMode,
    float inSpringValue,
    float inDamping,
    float inMinimum,
    float inMaximum)
{
    JPH::SixDOFConstraint *constraint =
        static_cast<JPH::SixDOFConstraint *>(inConstraint);
    const auto axis = static_cast<JPH::SixDOFConstraint::EAxis>(inAxis);
    JPH::MotorSettings &settings = constraint->GetMotorSettings(axis);
    if (inAxis < 3)
        ApplyMotorSettings(
            settings, inSpringMode, inSpringValue, inDamping,
            inMinimum, inMaximum, -FLT_MAX, FLT_MAX);
    else
        ApplyMotorSettings(
            settings, inSpringMode, inSpringValue, inDamping,
            -FLT_MAX, FLT_MAX, inMinimum, inMaximum);
}

inline void SetSixDOFMotorState(
    JPH::Constraint *inConstraint, std::uint8_t inAxis, std::uint8_t inState)
{
    static_cast<JPH::SixDOFConstraint *>(inConstraint)->SetMotorState(
        static_cast<JPH::SixDOFConstraint::EAxis>(inAxis),
        static_cast<JPH::EMotorState>(inState));
}

inline void SetSixDOFMotorTargets(
    JPH::Constraint *inConstraint,
    JPH::Vec3Arg inVelocity,
    JPH::Vec3Arg inAngularVelocity,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inOrientation)
{
    JPH::SixDOFConstraint *constraint =
        static_cast<JPH::SixDOFConstraint *>(inConstraint);
    constraint->SetTargetVelocityCS(inVelocity);
    constraint->SetTargetAngularVelocityCS(inAngularVelocity);
    constraint->SetTargetPositionCS(inPosition);
    constraint->SetTargetOrientationCS(inOrientation);
}

inline bool SetBodyDamping(
    JPH::PhysicsSystem *ioSystem,
    JPH::BodyID inBodyID,
    float inLinearDamping,
    float inAngularDamping)
{
    JPH::BodyLockWrite lock(ioSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() == JPH::EMotionType::Static)
        return false;
    JPH::MotionProperties *motion = body.GetMotionProperties();
    motion->SetLinearDamping(inLinearDamping);
    motion->SetAngularDamping(inAngularDamping);
    return true;
}

inline bool GetBodyDamping(
    const JPH::PhysicsSystem *inSystem,
    JPH::BodyID inBodyID,
    float *outLinearDamping,
    float *outAngularDamping)
{
    JPH::BodyLockRead lock(inSystem->GetBodyLockInterface(), inBodyID);
    if (!lock.Succeeded())
        return false;
    const JPH::Body &body = lock.GetBody();
    if (body.GetMotionType() == JPH::EMotionType::Static)
        return false;
    const JPH::MotionProperties *motion = body.GetMotionProperties();
    *outLinearDamping = motion->GetLinearDamping();
    *outAngularDamping = motion->GetAngularDamping();
    return true;
}

inline void RemoveConstraint(JPH::PhysicsSystem *ioSystem, JPH::Constraint *inConstraint)
{
    ioSystem->RemoveConstraint(inConstraint);
}

class ObjectLayerSetFilter final : public JPH::ObjectLayerFilter
{
public:
    ObjectLayerSetFilter(
        const JPH::ObjectLayer *inLayers,
        std::uint32_t inLayerCount) :
        mLayers(inLayers),
        mLayerCount(inLayerCount)
    {
    }

    virtual bool ShouldCollide(JPH::ObjectLayer inLayer) const override
    {
        if (mLayerCount == 0)
            return true;
        for (std::uint32_t index = 0; index < mLayerCount; ++index)
            if (mLayers[index] == inLayer)
                return true;
        return false;
    }

private:
    const JPH::ObjectLayer *mLayers;
    std::uint32_t mLayerCount;
};

class BodyIDSetFilter final : public JPH::BodyFilter
{
public:
    BodyIDSetFilter(
        const std::uint32_t *inBodyIDs,
        std::uint32_t inBodyIDCount,
        bool inIncludeBodies) :
        mBodyIDs(inBodyIDs),
        mBodyIDCount(inBodyIDCount),
        mIncludeBodies(inIncludeBodies)
    {
    }

    virtual bool ShouldCollide(const JPH::BodyID &inBodyID) const override
    {
        if (mBodyIDCount == 0)
            return !mIncludeBodies;
        const std::uint32_t id = inBodyID.GetIndexAndSequenceNumber();
        for (std::uint32_t index = 0; index < mBodyIDCount; ++index)
            if (mBodyIDs[index] == id)
                return mIncludeBodies;
        return !mIncludeBodies;
    }

private:
    const std::uint32_t *mBodyIDs;
    std::uint32_t mBodyIDCount;
    bool mIncludeBodies;
};

class BodySubShapeSetFilter
{
public:
    BodySubShapeSetFilter(
        const std::uint32_t *inBodyIDs,
        const std::uint32_t *inSubShapeIDs,
        std::uint32_t inCount,
        bool inIncludeSubShapes) :
        mBodyIDs(inBodyIDs),
        mSubShapeIDs(inSubShapeIDs),
        mCount(inCount),
        mIncludeSubShapes(inIncludeSubShapes)
    {
    }

    bool ShouldInclude(const JPH::BodyID &inBodyID, const JPH::SubShapeID &inSubShapeID) const
    {
        if (mCount == 0)
            return true;
        const std::uint32_t body_id = inBodyID.GetIndexAndSequenceNumber();
        const std::uint32_t sub_shape_id = inSubShapeID.GetValue();
        for (std::uint32_t index = 0; index < mCount; ++index)
            if (mBodyIDs[index] == body_id && mSubShapeIDs[index] == sub_shape_id)
                return mIncludeSubShapes;
        return !mIncludeSubShapes;
    }

private:
    const std::uint32_t *mBodyIDs;
    const std::uint32_t *mSubShapeIDs;
    std::uint32_t mCount;
    bool mIncludeSubShapes;
};

inline const JPH::BodyID &GetResultBodyID(const JPH::CollidePointResult &inResult)
{
    return inResult.mBodyID;
}

inline const JPH::BodyID &GetResultBodyID(const JPH::RayCastResult &inResult)
{
    return inResult.mBodyID;
}

inline const JPH::BodyID &GetResultBodyID(const JPH::ShapeCastResult &inResult)
{
    return inResult.mBodyID2;
}

inline const JPH::BodyID &GetResultBodyID(const JPH::CollideShapeResult &inResult)
{
    return inResult.mBodyID2;
}

template <class CollectorType>
class SubShapeFilteredCollector final : public CollectorType
{
public:
    using ResultType = typename CollectorType::ResultType;

    SubShapeFilteredCollector(
        const std::uint32_t *inBodyIDs,
        const std::uint32_t *inSubShapeIDs,
        std::uint32_t inCount,
        bool inIncludeSubShapes) :
        mFilter(inBodyIDs, inSubShapeIDs, inCount, inIncludeSubShapes)
    {
    }

    virtual void AddHit(const ResultType &inResult) override
    {
        if (mFilter.ShouldInclude(GetResultBodyID(inResult), inResult.mSubShapeID2))
            CollectorType::AddHit(inResult);
    }

private:
    BodySubShapeSetFilter mFilter;
};

inline std::uint32_t CollidePoint(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inPoint,
    std::uint32_t *outBodyIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    SubShapeFilteredCollector<
        JPH::ClosestHitPerBodyCollisionCollector<JPH::CollidePointCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CollidePoint(
        inPoint, collector, {}, layer_filter, body_filter);
    const std::uint32_t count = static_cast<std::uint32_t>(collector.mHits.size());
    const std::uint32_t copied = count < inCapacity? count : inCapacity;
    for (std::uint32_t index = 0; index < copied; ++index)
        outBodyIDs[index] = collector.mHits[index].mBodyID.GetIndexAndSequenceNumber();
    return copied;
}

inline void CopyBroadPhaseBodyHits(
    const JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> &inCollector,
    std::uint32_t *outBodyIDs,
    std::uint32_t inCapacity,
    std::uint32_t *outCopied,
    const BodyIDSetFilter &inBodyFilter)
{
    *outCopied = 0;
    for (const JPH::BodyID &body_id : inCollector.mHits)
    {
        if (*outCopied == inCapacity)
            break;
        if (inBodyFilter.ShouldCollide(body_id))
            outBodyIDs[(*outCopied)++] = body_id.GetIndexAndSequenceNumber();
    }
}

inline std::uint32_t BroadPhaseCollideAABox(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inMinimum,
    JPH::Vec3Arg inMaximum,
    std::uint32_t *outBodyIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false)
{
    JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
    const JPH::AABox box(inMinimum, inMaximum);
    const ObjectLayerSetFilter filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetBroadPhaseQuery().CollideAABox(box, collector, {}, filter);
    std::uint32_t copied;
    CopyBroadPhaseBodyHits(collector, outBodyIDs, inCapacity, &copied, body_filter);
    return copied;
}

inline std::uint32_t BroadPhaseCollideSphere(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inCenter,
    float inRadius,
    std::uint32_t *outBodyIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false)
{
    JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
    const ObjectLayerSetFilter filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetBroadPhaseQuery().CollideSphere(
        inCenter, inRadius, collector, {}, filter);
    std::uint32_t copied;
    CopyBroadPhaseBodyHits(collector, outBodyIDs, inCapacity, &copied, body_filter);
    return copied;
}

inline std::uint32_t BroadPhaseCollidePoint(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inPoint,
    std::uint32_t *outBodyIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false)
{
    JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
    const ObjectLayerSetFilter filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetBroadPhaseQuery().CollidePoint(
        inPoint, collector, {}, filter);
    std::uint32_t copied;
    CopyBroadPhaseBodyHits(collector, outBodyIDs, inCapacity, &copied, body_filter);
    return copied;
}

inline std::uint32_t BroadPhaseCollideOrientedBox(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inCenter,
    JPH::QuatArg inRotation,
    JPH::Vec3Arg inHalfExtent,
    std::uint32_t *outBodyIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false)
{
    JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
    const JPH::OrientedBox box(
        JPH::Mat44::sRotationTranslation(inRotation, inCenter), inHalfExtent);
    const ObjectLayerSetFilter filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetBroadPhaseQuery().CollideOrientedBox(
        box, collector, {}, filter);
    std::uint32_t copied;
    CopyBroadPhaseBodyHits(collector, outBodyIDs, inCapacity, &copied, body_filter);
    return copied;
}

inline std::uint32_t BroadPhaseCastRay(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inOrigin,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyIDs,
    float *outFractions,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false)
{
    JPH::AllHitCollisionCollector<JPH::RayCastBodyCollector> collector;
    const JPH::RayCast ray(inOrigin, inDirectionAndLength);
    const ObjectLayerSetFilter filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetBroadPhaseQuery().CastRay(ray, collector, {}, filter);
    collector.Sort();
    std::uint32_t copied = 0;
    for (const JPH::BroadPhaseCastResult &hit : collector.mHits)
    {
        if (copied == inCapacity)
            break;
        if (!body_filter.ShouldCollide(hit.mBodyID))
            continue;
        outBodyIDs[copied] = hit.mBodyID.GetIndexAndSequenceNumber();
        outFractions[copied] = hit.mFraction;
        ++copied;
    }
    return copied;
}

inline std::uint32_t BroadPhaseCastAABox(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inCenter,
    JPH::Vec3Arg inHalfExtent,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyIDs,
    float *outFractions,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false)
{
    JPH::AllHitCollisionCollector<JPH::CastShapeBodyCollector> collector;
    JPH::AABoxCast cast;
    cast.mBox = JPH::AABox(inCenter - inHalfExtent, inCenter + inHalfExtent);
    cast.mDirection = inDirectionAndLength;
    const ObjectLayerSetFilter filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetBroadPhaseQuery().CastAABox(cast, collector, {}, filter);
    collector.Sort();
    std::uint32_t copied = 0;
    for (const JPH::BroadPhaseCastResult &hit : collector.mHits)
    {
        if (copied == inCapacity)
            break;
        if (!body_filter.ShouldCollide(hit.mBodyID))
            continue;
        outBodyIDs[copied] = hit.mBodyID.GetIndexAndSequenceNumber();
        outFractions[copied] = hit.mFraction;
        ++copied;
    }
    return copied;
}

inline void GetBroadPhaseBounds(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3 *outMinimum,
    JPH::Vec3 *outMaximum)
{
    const JPH::AABox bounds = inSystem->GetBroadPhaseQuery().GetBounds();
    *outMinimum = bounds.mMin;
    *outMaximum = bounds.mMax;
}

inline bool CastRay(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inOrigin,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyID,
    float *outFraction,
    std::uint32_t *outSubShapeID,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    SubShapeFilteredCollector<
        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    JPH::RayCastSettings settings;
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CastRay(
        JPH::RRayCast(inOrigin, inDirectionAndLength), settings, collector,
        {}, layer_filter, body_filter);
    if (!collector.HadHit())
        return false;
    const JPH::RayCastResult &hit = collector.mHit;
    *outBodyID = hit.mBodyID.GetIndexAndSequenceNumber();
    *outFraction = hit.mFraction;
    *outSubShapeID = hit.mSubShapeID2.GetValue();
    return true;
}

inline std::uint32_t CastRayAll(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inOrigin,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyIDs,
    float *outFractions,
    std::uint32_t *outSubShapeIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    SubShapeFilteredCollector<
        JPH::ClosestHitPerBodyCollisionCollector<JPH::CastRayCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    JPH::RayCastSettings settings;
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CastRay(
        JPH::RRayCast(inOrigin, inDirectionAndLength), settings, collector,
        {}, layer_filter, body_filter);
    collector.Sort();

    const std::uint32_t count = static_cast<std::uint32_t>(collector.mHits.size());
    const std::uint32_t copied = count < inCapacity? count : inCapacity;
    for (std::uint32_t index = 0; index < copied; ++index)
    {
        const JPH::RayCastResult &hit = collector.mHits[index];
        outBodyIDs[index] = hit.mBodyID.GetIndexAndSequenceNumber();
        outFractions[index] = hit.mFraction;
        outSubShapeIDs[index] = hit.mSubShapeID2.GetValue();
    }
    return copied;
}

inline bool CastSphere(
    const JPH::PhysicsSystem *inSystem,
    float inRadius,
    JPH::Vec3Arg inOrigin,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyID,
    float *outFraction,
    JPH::Vec3 *outContactPoint,
    JPH::Vec3 *outNormal,
    std::uint32_t *outSubShapeID,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    JPH::SphereShape sphere(inRadius);
    JPH::RShapeCast shape_cast {
        &sphere,
        JPH::Vec3::sOne(),
        JPH::RMat44::sTranslation(inOrigin),
        inDirectionAndLength
    };
    JPH::ShapeCastSettings settings;
    settings.mReturnDeepestPoint = true;
    SubShapeFilteredCollector<
        JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CastShape(
        shape_cast, settings, JPH::RVec3::sZero(), collector,
        {}, layer_filter, body_filter);
    if (!collector.HadHit())
        return false;
    const JPH::ShapeCastResult &hit = collector.mHit;
    *outBodyID = hit.mBodyID2.GetIndexAndSequenceNumber();
    *outFraction = hit.mFraction;
    *outContactPoint = hit.mContactPointOn2;
    *outNormal = -hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());
    *outSubShapeID = hit.mSubShapeID2.GetValue();
    return true;
}

inline std::uint32_t CastSphereAll(
    const JPH::PhysicsSystem *inSystem,
    float inRadius,
    JPH::Vec3Arg inOrigin,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyIDs,
    float *outFractions,
    JPH::Vec3 *outContactPoints,
    JPH::Vec3 *outNormals,
    std::uint32_t *outSubShapeIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    JPH::SphereShape sphere(inRadius);
    JPH::RShapeCast shape_cast {
        &sphere,
        JPH::Vec3::sOne(),
        JPH::RMat44::sTranslation(inOrigin),
        inDirectionAndLength
    };
    JPH::ShapeCastSettings settings;
    settings.mReturnDeepestPoint = true;
    SubShapeFilteredCollector<
        JPH::ClosestHitPerBodyCollisionCollector<JPH::CastShapeCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CastShape(
        shape_cast, settings, JPH::RVec3::sZero(), collector,
        {}, layer_filter, body_filter);
    collector.Sort();

    const std::uint32_t count = static_cast<std::uint32_t>(collector.mHits.size());
    const std::uint32_t copied = count < inCapacity? count : inCapacity;
    for (std::uint32_t index = 0; index < copied; ++index)
    {
        const JPH::ShapeCastResult &hit = collector.mHits[index];
        outBodyIDs[index] = hit.mBodyID2.GetIndexAndSequenceNumber();
        outFractions[index] = hit.mFraction;
        outContactPoints[index] = hit.mContactPointOn2;
        outNormals[index] =
            -hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());
        outSubShapeIDs[index] = hit.mSubShapeID2.GetValue();
    }
    return copied;
}

inline std::uint32_t OverlapSphere(
    const JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inCenter,
    float inRadius,
    std::uint32_t *outBodyIDs,
    float *outPenetrationDepths,
    JPH::Vec3 *outContactPoints,
    JPH::Vec3 *outNormals,
    std::uint32_t *outSubShapeIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    JPH::SphereShape sphere(inRadius);
    JPH::CollideShapeSettings settings;
    SubShapeFilteredCollector<
        JPH::ClosestHitPerBodyCollisionCollector<JPH::CollideShapeCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CollideShape(
        &sphere,
        JPH::Vec3::sReplicate(1.0f),
        JPH::RMat44::sTranslation(inCenter),
        settings,
        JPH::RVec3::sZero(),
        collector,
        {},
        layer_filter,
        body_filter);
    collector.Sort();

    const std::uint32_t count = static_cast<std::uint32_t>(collector.mHits.size());
    const std::uint32_t copied = count < inCapacity? count : inCapacity;
    for (std::uint32_t index = 0; index < copied; ++index)
    {
        const JPH::CollideShapeResult &hit = collector.mHits[index];
        outBodyIDs[index] = hit.mBodyID2.GetIndexAndSequenceNumber();
        outPenetrationDepths[index] = hit.mPenetrationDepth;
        outContactPoints[index] = hit.mContactPointOn2;
        outNormals[index] = -hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());
        outSubShapeIDs[index] = hit.mSubShapeID2.GetValue();
    }
    return copied;
}

inline bool CastConvex(
    const JPH::PhysicsSystem *inSystem,
    const JPH::Shape *inShape,
    JPH::Vec3Arg inOrigin,
    JPH::QuatArg inRotation,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyID,
    float *outFraction,
    JPH::Vec3 *outContactPoint,
    JPH::Vec3 *outNormal,
    std::uint32_t *outSubShapeID,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    const JPH::RShapeCast shape_cast = JPH::RShapeCast::sFromWorldTransform(
        inShape,
        JPH::Vec3::sOne(),
        JPH::RMat44::sRotationTranslation(inRotation, inOrigin),
        inDirectionAndLength);
    JPH::ShapeCastSettings settings;
    settings.mReturnDeepestPoint = true;
    SubShapeFilteredCollector<
        JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CastShape(
        shape_cast, settings, JPH::RVec3::sZero(), collector,
        {}, layer_filter, body_filter);
    if (!collector.HadHit())
        return false;
    const JPH::ShapeCastResult &hit = collector.mHit;
    *outBodyID = hit.mBodyID2.GetIndexAndSequenceNumber();
    *outFraction = hit.mFraction;
    *outContactPoint = hit.mContactPointOn2;
    *outNormal = -hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());
    *outSubShapeID = hit.mSubShapeID2.GetValue();
    return true;
}

inline std::uint32_t CastConvexAll(
    const JPH::PhysicsSystem *inSystem,
    const JPH::Shape *inShape,
    JPH::Vec3Arg inOrigin,
    JPH::QuatArg inRotation,
    JPH::Vec3Arg inDirectionAndLength,
    std::uint32_t *outBodyIDs,
    float *outFractions,
    JPH::Vec3 *outContactPoints,
    JPH::Vec3 *outNormals,
    std::uint32_t *outSubShapeIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    const JPH::RShapeCast shape_cast = JPH::RShapeCast::sFromWorldTransform(
        inShape,
        JPH::Vec3::sOne(),
        JPH::RMat44::sRotationTranslation(inRotation, inOrigin),
        inDirectionAndLength);
    JPH::ShapeCastSettings settings;
    settings.mReturnDeepestPoint = true;
    SubShapeFilteredCollector<
        JPH::ClosestHitPerBodyCollisionCollector<JPH::CastShapeCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CastShape(
        shape_cast, settings, JPH::RVec3::sZero(), collector,
        {}, layer_filter, body_filter);
    collector.Sort();

    const std::uint32_t count = static_cast<std::uint32_t>(collector.mHits.size());
    const std::uint32_t copied = count < inCapacity? count : inCapacity;
    for (std::uint32_t index = 0; index < copied; ++index)
    {
        const JPH::ShapeCastResult &hit = collector.mHits[index];
        outBodyIDs[index] = hit.mBodyID2.GetIndexAndSequenceNumber();
        outFractions[index] = hit.mFraction;
        outContactPoints[index] = hit.mContactPointOn2;
        outNormals[index] =
            -hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());
        outSubShapeIDs[index] = hit.mSubShapeID2.GetValue();
    }
    return copied;
}

inline std::uint32_t OverlapConvex(
    const JPH::PhysicsSystem *inSystem,
    const JPH::Shape *inShape,
    JPH::Vec3Arg inPosition,
    JPH::QuatArg inRotation,
    std::uint32_t *outBodyIDs,
    float *outPenetrationDepths,
    JPH::Vec3 *outContactPoints,
    JPH::Vec3 *outNormals,
    std::uint32_t *outSubShapeIDs,
    std::uint32_t inCapacity,
    const JPH::ObjectLayer *inObjectLayers,
    std::uint32_t inObjectLayerCount,
    const std::uint32_t *inBodyIDs = nullptr,
    std::uint32_t inBodyIDCount = 0,
    bool inIncludeBodies = false,
    const std::uint32_t *inSubShapeBodyIDs = nullptr,
    const std::uint32_t *inSubShapeIDs = nullptr,
    std::uint32_t inSubShapeCount = 0,
    bool inIncludeSubShapes = false)
{
    const JPH::RMat44 center_of_mass_transform =
        JPH::RMat44::sRotationTranslation(inRotation, inPosition)
            .PreTranslated(inShape->GetCenterOfMass());
    JPH::CollideShapeSettings settings;
    SubShapeFilteredCollector<
        JPH::ClosestHitPerBodyCollisionCollector<JPH::CollideShapeCollector>> collector(
            inSubShapeBodyIDs, inSubShapeIDs, inSubShapeCount, inIncludeSubShapes);
    const ObjectLayerSetFilter layer_filter(inObjectLayers, inObjectLayerCount);
    const BodyIDSetFilter body_filter(
        inBodyIDs, inBodyIDCount, inIncludeBodies);
    inSystem->GetNarrowPhaseQuery().CollideShape(
        inShape,
        JPH::Vec3::sOne(),
        center_of_mass_transform,
        settings,
        JPH::RVec3::sZero(),
        collector,
        {},
        layer_filter,
        body_filter);
    collector.Sort();

    const std::uint32_t count = static_cast<std::uint32_t>(collector.mHits.size());
    const std::uint32_t copied = count < inCapacity? count : inCapacity;
    for (std::uint32_t index = 0; index < copied; ++index)
    {
        const JPH::CollideShapeResult &hit = collector.mHits[index];
        outBodyIDs[index] = hit.mBodyID2.GetIndexAndSequenceNumber();
        outPenetrationDepths[index] = hit.mPenetrationDepth;
        outContactPoints[index] = hit.mContactPointOn2;
        outNormals[index] =
            -hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sAxisY());
        outSubShapeIDs[index] = hit.mSubShapeID2.GetValue();
    }
    return copied;
}

#ifdef JPH_DEBUG_RENDERER
struct DebugPointData
{
    float mX = 0.0f;
    float mY = 0.0f;
    float mZ = 0.0f;
};

struct DebugColorData
{
    std::uint8_t mR = 0;
    std::uint8_t mG = 0;
    std::uint8_t mB = 0;
    std::uint8_t mA = 0;
};

struct DebugLineData
{
    DebugPointData mFrom;
    DebugPointData mTo;
    DebugColorData mColor;
};

struct DebugTriangleData
{
    DebugPointData mV1;
    DebugPointData mV2;
    DebugPointData mV3;
    DebugColorData mColor;
    bool mCastsShadow = false;
};

struct DebugTextData
{
    DebugPointData mPosition;
    DebugColorData mColor;
    float mHeight = 0.0f;
    const char *mText = nullptr;
    std::size_t mTextLength = 0;
};

struct DebugBodyDrawSettingsData
{
    bool mDrawGetSupportFunction = false;
    bool mDrawSupportDirection = false;
    bool mDrawGetSupportingFace = false;
    bool mDrawShape = true;
    bool mDrawShapeWireframe = false;
    std::uint8_t mDrawShapeColor = 2;
    bool mDrawBoundingBox = false;
    bool mDrawCenterOfMassTransform = false;
    bool mDrawWorldTransform = false;
    bool mDrawVelocity = false;
    bool mDrawMassAndInertia = false;
    bool mDrawSleepStats = false;
    bool mDrawSoftBodyVertices = false;
    bool mDrawSoftBodyVertexVelocities = false;
    bool mDrawSoftBodyEdgeConstraints = false;
    bool mDrawSoftBodyBendConstraints = false;
    bool mDrawSoftBodyVolumeConstraints = false;
    bool mDrawSoftBodySkinConstraints = false;
    bool mDrawSoftBodyLRAConstraints = false;
    bool mDrawSoftBodyRods = false;
    bool mDrawSoftBodyRodStates = false;
    bool mDrawSoftBodyRodBendTwistConstraints = false;
    bool mDrawSoftBodyPredictedBounds = false;
    std::uint8_t mDrawSoftBodyConstraintColor = 0;
};

inline DebugPointData ToDebugPoint(JPH::RVec3Arg inValue)
{
    return {
        static_cast<float>(inValue.GetX()),
        static_cast<float>(inValue.GetY()),
        static_cast<float>(inValue.GetZ())};
}

inline DebugColorData ToDebugColor(JPH::ColorArg inValue)
{
    return {inValue.r, inValue.g, inValue.b, inValue.a};
}

class DebugDrawCollector final : public JPH::DebugRendererSimple
{
public:
    DebugDrawCollector(
        std::uint32_t inMaxLines,
        std::uint32_t inMaxTriangles,
        std::uint32_t inMaxTexts,
        std::size_t inMaxTextBytes)
        : mMaxLines(inMaxLines),
          mMaxTriangles(inMaxTriangles),
          mMaxTexts(inMaxTexts),
          mMaxTextBytes(inMaxTextBytes)
    {
        Initialize();
    }

    void DrawLine(
        JPH::RVec3Arg inFrom,
        JPH::RVec3Arg inTo,
        JPH::ColorArg inColor) override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mLines.size() >= mMaxLines)
        {
            ++mDroppedLines;
            return;
        }
        mLines.push_back({
            ToDebugPoint(inFrom), ToDebugPoint(inTo), ToDebugColor(inColor)});
    }

    void DrawTriangle(
        JPH::RVec3Arg inV1,
        JPH::RVec3Arg inV2,
        JPH::RVec3Arg inV3,
        JPH::ColorArg inColor,
        JPH::DebugRenderer::ECastShadow inCastShadow) override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mTriangles.size() >= mMaxTriangles)
        {
            ++mDroppedTriangles;
            return;
        }
        mTriangles.push_back({
            ToDebugPoint(inV1),
            ToDebugPoint(inV2),
            ToDebugPoint(inV3),
            ToDebugColor(inColor),
            inCastShadow == JPH::DebugRenderer::ECastShadow::On});
    }

    void DrawText3D(
        JPH::RVec3Arg inPosition,
        const std::string_view &inString,
        JPH::ColorArg inColor,
        float inHeight) override
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mTexts.size() >= mMaxTexts ||
            inString.size() > mMaxTextBytes - std::min(mTextBytes, mMaxTextBytes))
        {
            ++mDroppedTexts;
            return;
        }
        mTexts.push_back({
            ToDebugPoint(inPosition),
            ToDebugColor(inColor),
            inHeight,
            std::string(inString)});
        mTextBytes += inString.size();
    }

    struct StoredText
    {
        DebugPointData mPosition;
        DebugColorData mColor;
        float mHeight;
        std::string mText;
    };

    std::vector<DebugLineData> mLines;
    std::vector<DebugTriangleData> mTriangles;
    std::vector<StoredText> mTexts;
    std::uint64_t mDroppedLines = 0;
    std::uint64_t mDroppedTriangles = 0;
    std::uint64_t mDroppedTexts = 0;

private:
    std::mutex mMutex;
    std::size_t mMaxLines;
    std::size_t mMaxTriangles;
    std::size_t mMaxTexts;
    std::size_t mMaxTextBytes;
    std::size_t mTextBytes = 0;
};

class DebugBodyIDFilter final : public JPH::BodyDrawFilter
{
public:
    DebugBodyIDFilter(
        const std::uint32_t *inBodyIDs,
        std::uint32_t inBodyIDCount,
        bool inIncludeBodies)
        : mBodyIDs(inBodyIDs),
          mBodyIDCount(inBodyIDCount),
          mIncludeBodies(inIncludeBodies) { }

    bool ShouldDraw(const JPH::Body &inBody) const override
    {
        const std::uint32_t id = inBody.GetID().GetIndexAndSequenceNumber();
        for (std::uint32_t index = 0; index < mBodyIDCount; ++index)
            if (mBodyIDs[index] == id)
                return mIncludeBodies;
        return !mIncludeBodies;
    }

private:
    const std::uint32_t *mBodyIDs;
    std::uint32_t mBodyIDCount;
    bool mIncludeBodies;
};

inline JPH::BodyManager::DrawSettings ToBodyDrawSettings(
    const DebugBodyDrawSettingsData &inSettings)
{
    JPH::BodyManager::DrawSettings result;
    result.mDrawGetSupportFunction = inSettings.mDrawGetSupportFunction;
    result.mDrawSupportDirection = inSettings.mDrawSupportDirection;
    result.mDrawGetSupportingFace = inSettings.mDrawGetSupportingFace;
    result.mDrawShape = inSettings.mDrawShape;
    result.mDrawShapeWireframe = inSettings.mDrawShapeWireframe;
    result.mDrawShapeColor = static_cast<JPH::BodyManager::EShapeColor>(
        inSettings.mDrawShapeColor);
    result.mDrawBoundingBox = inSettings.mDrawBoundingBox;
    result.mDrawCenterOfMassTransform = inSettings.mDrawCenterOfMassTransform;
    result.mDrawWorldTransform = inSettings.mDrawWorldTransform;
    result.mDrawVelocity = inSettings.mDrawVelocity;
    result.mDrawMassAndInertia = inSettings.mDrawMassAndInertia;
    result.mDrawSleepStats = inSettings.mDrawSleepStats;
    result.mDrawSoftBodyVertices = inSettings.mDrawSoftBodyVertices;
    result.mDrawSoftBodyVertexVelocities = inSettings.mDrawSoftBodyVertexVelocities;
    result.mDrawSoftBodyEdgeConstraints = inSettings.mDrawSoftBodyEdgeConstraints;
    result.mDrawSoftBodyBendConstraints = inSettings.mDrawSoftBodyBendConstraints;
    result.mDrawSoftBodyVolumeConstraints = inSettings.mDrawSoftBodyVolumeConstraints;
    result.mDrawSoftBodySkinConstraints = inSettings.mDrawSoftBodySkinConstraints;
    result.mDrawSoftBodyLRAConstraints = inSettings.mDrawSoftBodyLRAConstraints;
    result.mDrawSoftBodyRods = inSettings.mDrawSoftBodyRods;
    result.mDrawSoftBodyRodStates = inSettings.mDrawSoftBodyRodStates;
    result.mDrawSoftBodyRodBendTwistConstraints =
        inSettings.mDrawSoftBodyRodBendTwistConstraints;
    result.mDrawSoftBodyPredictedBounds = inSettings.mDrawSoftBodyPredictedBounds;
    result.mDrawSoftBodyConstraintColor =
        static_cast<JPH::ESoftBodyConstraintColor>(
            inSettings.mDrawSoftBodyConstraintColor);
    return result;
}

inline DebugDrawCollector *CaptureDebugDraw(
    JPH::PhysicsSystem *inSystem,
    JPH::Vec3Arg inCameraPosition,
    const DebugBodyDrawSettingsData &inSettings,
    const std::uint32_t *inBodyIDs,
    std::uint32_t inBodyIDCount,
    bool inBodyFilterEnabled,
    bool inIncludeBodies,
    bool inDrawBodies,
    bool inDrawConstraints,
    bool inDrawConstraintLimits,
    bool inDrawConstraintReferenceFrames,
    std::uint32_t inMaxLines,
    std::uint32_t inMaxTriangles,
    std::uint32_t inMaxTexts,
    std::size_t inMaxTextBytes)
{
    auto collector = std::make_unique<DebugDrawCollector>(
        inMaxLines, inMaxTriangles, inMaxTexts, inMaxTextBytes);
    collector->SetCameraPos(inCameraPosition);
    DebugBodyIDFilter filter(inBodyIDs, inBodyIDCount, inIncludeBodies);
    if (inDrawBodies)
        inSystem->DrawBodies(
            ToBodyDrawSettings(inSettings),
            collector.get(),
            inBodyFilterEnabled? &filter : nullptr);
    if (inDrawConstraints)
        inSystem->DrawConstraints(collector.get());
    if (inDrawConstraintLimits)
        inSystem->DrawConstraintLimits(collector.get());
    if (inDrawConstraintReferenceFrames)
        inSystem->DrawConstraintReferenceFrame(collector.get());
    return collector.release();
}

inline bool GetDebugLine(
    const DebugDrawCollector *inCollector,
    std::uint32_t inIndex,
    DebugLineData *outLine)
{
    if (inIndex >= inCollector->mLines.size())
        return false;
    *outLine = inCollector->mLines[inIndex];
    return true;
}

inline bool GetDebugTriangle(
    const DebugDrawCollector *inCollector,
    std::uint32_t inIndex,
    DebugTriangleData *outTriangle)
{
    if (inIndex >= inCollector->mTriangles.size())
        return false;
    *outTriangle = inCollector->mTriangles[inIndex];
    return true;
}

inline bool GetDebugText(
    const DebugDrawCollector *inCollector,
    std::uint32_t inIndex,
    DebugTextData *outText)
{
    if (inIndex >= inCollector->mTexts.size())
        return false;
    const DebugDrawCollector::StoredText &text = inCollector->mTexts[inIndex];
    outText->mPosition = text.mPosition;
    outText->mColor = text.mColor;
    outText->mHeight = text.mHeight;
    outText->mText = text.mText.data();
    outText->mTextLength = text.mText.size();
    return true;
}

inline std::uint32_t DebugLineCount(const DebugDrawCollector *inCollector)
{
    return static_cast<std::uint32_t>(inCollector->mLines.size());
}

inline std::uint32_t DebugTriangleCount(const DebugDrawCollector *inCollector)
{
    return static_cast<std::uint32_t>(inCollector->mTriangles.size());
}

inline std::uint32_t DebugTextCount(const DebugDrawCollector *inCollector)
{
    return static_cast<std::uint32_t>(inCollector->mTexts.size());
}

inline std::uint64_t DroppedDebugLineCount(const DebugDrawCollector *inCollector)
{
    return inCollector->mDroppedLines;
}

inline std::uint64_t DroppedDebugTriangleCount(const DebugDrawCollector *inCollector)
{
    return inCollector->mDroppedTriangles;
}

inline std::uint64_t DroppedDebugTextCount(const DebugDrawCollector *inCollector)
{
    return inCollector->mDroppedTexts;
}
#endif

inline std::uint32_t Update(
    JPH::PhysicsSystem *ioSystem,
    float inDeltaTime,
    int inCollisionSteps,
    JPH::TempAllocatorImpl *inTempAllocator,
    JPH::JobSystemThreadPool *inJobSystem)
{
    return static_cast<std::uint32_t>(ioSystem->Update(
        inDeltaTime,
        inCollisionSteps,
        inTempAllocator,
        inJobSystem));
}

inline void RemoveAndDestroyBody(JPH::BodyInterface *ioBodyInterface, JPH::BodyID inBodyID)
{
    if (ioBodyInterface->IsAdded(inBodyID))
        ioBodyInterface->RemoveBody(inBodyID);
    ioBodyInterface->DestroyBody(inBodyID);
}

inline void AddBodies(
    JPH::BodyInterface *ioBodyInterface,
    JPH::BodyID *ioBodyIDs,
    std::uint32_t inCount,
    JPH::EActivation inActivation)
{
    if (inCount == 0)
        return;
    JPH::BodyInterface::AddState state =
        ioBodyInterface->AddBodiesPrepare(ioBodyIDs, static_cast<int>(inCount));
    ioBodyInterface->AddBodiesFinalize(
        ioBodyIDs, static_cast<int>(inCount), state, inActivation);
}

inline void DestroyBodies(
    JPH::BodyInterface *ioBodyInterface,
    JPH::BodyID *inBodyIDs,
    std::uint32_t inCount)
{
    if (inCount != 0)
        ioBodyInterface->DestroyBodies(
            inBodyIDs, static_cast<int>(inCount));
}

inline void RemoveAndDestroyBodies(
    JPH::BodyInterface *ioBodyInterface,
    JPH::BodyID *ioBodyIDs,
    std::uint32_t inCount)
{
    if (inCount == 0)
        return;
    ioBodyInterface->RemoveBodies(ioBodyIDs, static_cast<int>(inCount));
    ioBodyInterface->DestroyBodies(ioBodyIDs, static_cast<int>(inCount));
}

}
