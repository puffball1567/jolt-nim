#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/StateRecorder.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhase.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterMask.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyShape.h>
#include <Jolt/Physics/Hair/Hair.h>
#include <Jolt/Physics/Hair/HairSettings.h>
#include <Jolt/Physics/Hair/HairShaders.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleController.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/Wheel.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Physics/Collision/ActiveEdges.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseBruteForce.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuadTree.h>
#include <Jolt/Physics/Collision/BroadPhase/QuadTree.h>
#include <Jolt/Physics/Collision/CastConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CastSphereVsTriangles.h>
#include <Jolt/Physics/Collision/CollideConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShapeVsShapePerLeaf.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVerticesVsTriangles.h>
#include <Jolt/Physics/Collision/CollideSphereVsTriangles.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/InternalEdgeRemovingCollector.h>
#include <Jolt/Physics/Collision/NarrowPhaseStats.h>
#include <Jolt/Physics/Collision/Shape/CompoundShapeVisitors.h>
#include <Jolt/Physics/Collision/Shape/GetTrianglesContext.h>
#include <Jolt/Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Collision/SimShapeFilterWrapper.h>
#include <Jolt/Physics/Constraints/CalculateSolverSteps.h>
#include <Jolt/Physics/Constraints/ConstraintPart/RotationQuatConstraintPart.h>
#include <Jolt/Physics/Hair/RegisterHair.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>

// Public utility surfaces outside Jolt/Physics. Some of these are reached
// transitively today; keeping them explicit makes the raw inventory stable
// when upstream include relationships change.
#include <Jolt/Core/BinaryHeap.h>
#include <Jolt/Core/FPFlushDenormals.h>
#include <Jolt/Core/IncludeWindows.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/LSANSuppressions.h>
#include <Jolt/Core/Prefetch.h>
#include <Jolt/Core/ScopeExit.h>
#include <Jolt/Core/StreamUtils.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/ConfigurationString.h>
#include <Jolt/Math/DynMatrix.h>
#include <Jolt/Math/EigenValueSymmetric.h>
#include <Jolt/Math/FindRoot.h>
#include <Jolt/Geometry/ClipPoly.h>
#include <Jolt/Geometry/ConvexHullBuilder.h>
#include <Jolt/Geometry/ConvexHullBuilder2D.h>
#include <Jolt/Geometry/ConvexSupport.h>
#include <Jolt/Geometry/EPAConvexHullBuilder.h>
#include <Jolt/Geometry/EPAPenetrationDepth.h>
#include <Jolt/Geometry/GJKClosestPoint.h>
#include <Jolt/Geometry/Indexify.h>
#include <Jolt/Geometry/MortonCode.h>
#include <Jolt/Geometry/RayCapsule.h>
#include <Jolt/Geometry/RayCylinder.h>
#include <Jolt/Geometry/RaySphere.h>
#include <Jolt/Geometry/RayTriangle.h>
#include <Jolt/ObjectStream/GetPrimitiveTypeOfType.h>
#include <Jolt/ObjectStream/ObjectStreamBinaryIn.h>
#include <Jolt/ObjectStream/ObjectStreamBinaryOut.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/ObjectStream/ObjectStreamTextIn.h>
#include <Jolt/ObjectStream/ObjectStreamTextOut.h>
#include <Jolt/ObjectStream/SerializableAttributeEnum.h>
#include <Jolt/ObjectStream/SerializableAttributeTyped.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/AABBTree/AABBTreeBuilder.h>
#include <Jolt/AABBTree/AABBTreeToBuffer.h>
#include <Jolt/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h>
#include <Jolt/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h>
#include <Jolt/TriangleSplitter/TriangleSplitter.h>
#include <Jolt/TriangleSplitter/TriangleSplitterBinning.h>
#include <Jolt/TriangleSplitter/TriangleSplitterMean.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <Jolt/Compute/CPU/ComputeBufferCPU.h>
#include <Jolt/Compute/CPU/ComputeQueueCPU.h>
#include <Jolt/Compute/CPU/ComputeShaderCPU.h>
#include <Jolt/Compute/CPU/ComputeSystemCPU.h>
#include <Jolt/Compute/CPU/HLSLToCPP.h>
#include <Jolt/Compute/CPU/ShaderWrapper.h>
#include <Jolt/Shaders/HairWrapper.h>
#include <Jolt/Compute/DX12/ComputeSystemDX12.h>
#include <Jolt/Compute/MTL/ComputeSystemMTL.h>
#include <Jolt/Compute/VK/ComputeSystemVK.h>
#include <Jolt/Compute/VK/ComputeSystemVKWithAllocator.h>

#ifdef JPH_USE_DX12
#include <Jolt/Compute/DX12/ComputeBufferDX12.h>
#include <Jolt/Compute/DX12/ComputeQueueDX12.h>
#include <Jolt/Compute/DX12/ComputeShaderDX12.h>
#include <Jolt/Compute/DX12/ComputeSystemDX12Impl.h>
#endif

#ifdef JPH_USE_MTL
#include <Jolt/Compute/MTL/ComputeBufferMTL.h>
#include <Jolt/Compute/MTL/ComputeQueueMTL.h>
#include <Jolt/Compute/MTL/ComputeShaderMTL.h>
#include <Jolt/Compute/MTL/ComputeSystemMTLImpl.h>
#endif

#ifdef JPH_USE_VK
#include <Jolt/Compute/VK/ComputeBufferVK.h>
#include <Jolt/Compute/VK/ComputeQueueVK.h>
#include <Jolt/Compute/VK/ComputeShaderVK.h>
#include <Jolt/Compute/VK/ComputeSystemVKImpl.h>
#endif

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRenderer.h>
#include <Jolt/Renderer/DebugRendererPlayback.h>
#include <Jolt/Renderer/DebugRendererRecorder.h>
#include <Jolt/Renderer/DebugRendererSimple.h>
#endif
#include <atomic>
#include <mutex>
#include <sstream>

namespace joltnim_raw_detail
{
#ifdef JPH_DEBUG_RENDERER
using DebugLineCallback = void (*)(void *, const JPH::RVec3 *, const JPH::RVec3 *, const JPH::Color *);
using DebugTriangleCallback = void (*)(void *, const JPH::RVec3 *, const JPH::RVec3 *, const JPH::RVec3 *, const JPH::Color *, JPH::DebugRenderer::ECastShadow);
using DebugTextCallback = void (*)(void *, const JPH::RVec3 *, const char *, size_t, const JPH::Color *, float);
using BodyDrawCallback = bool (*)(void *, const JPH::Body *);

class DebugRendererSimpleAdapter final : public JPH::DebugRendererSimple
{
public:
    DebugRendererSimpleAdapter(DebugLineCallback inLine, DebugTriangleCallback inTriangle, DebugTextCallback inText, void *inUserData)
        : mLine(inLine), mTriangle(inTriangle), mText(inText), mUserData(inUserData) { }

    void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override
    {
        if (mLine != nullptr)
            mLine(mUserData, &inFrom, &inTo, &inColor);
    }

    void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, JPH::DebugRenderer::ECastShadow inCastShadow) override
    {
        if (mTriangle != nullptr)
            mTriangle(mUserData, &inV1, &inV2, &inV3, &inColor, inCastShadow);
        else
            JPH::DebugRendererSimple::DrawTriangle(inV1, inV2, inV3, inColor, inCastShadow);
    }

    void DrawText3D(JPH::RVec3Arg inPosition, const std::string_view &inString, JPH::ColorArg inColor, float inHeight) override
    {
        if (mText != nullptr)
            mText(mUserData, &inPosition, inString.data(), inString.size(), &inColor, inHeight);
    }

private:
    DebugLineCallback mLine;
    DebugTriangleCallback mTriangle;
    DebugTextCallback mText;
    void *mUserData;
};

class BodyDrawFilterAdapter final : public JPH::BodyDrawFilter
{
public:
    BodyDrawFilterAdapter(BodyDrawCallback inCallback, void *inUserData)
        : mCallback(inCallback), mUserData(inUserData) { }

    bool ShouldDraw(const JPH::Body &inBody) const override
    {
        return mCallback == nullptr || mCallback(mUserData, &inBody);
    }

private:
    BodyDrawCallback mCallback;
    void *mUserData;
};

inline JPH::DebugRenderer *AsDebugRenderer(DebugRendererSimpleAdapter *inAdapter) { return inAdapter; }
inline JPH::DebugRendererSimple *AsDebugRendererSimple(DebugRendererSimpleAdapter *inAdapter) { return inAdapter; }
inline JPH::BodyDrawFilter *AsBodyDrawFilter(BodyDrawFilterAdapter *inAdapter) { return inAdapter; }
#endif

inline void RewindStringStream(std::stringstream &ioStream)
{
    ioStream.clear();
    // The concrete ObjectStream constructors emit an eight-byte TOS/BOS
    // version header. ObjectStreamIn::Open normally consumes it before
    // constructing the concrete reader; direct low-level construction needs
    // to position the stream equivalently.
    ioStream.seekg(8);
}

inline void RewindStringStreamToStart(std::stringstream &ioStream)
{
    ioStream.clear();
    ioStream.seekg(0);
}

inline JPH::StreamIn *AsStreamIn(JPH::StreamInWrapper *inStream) { return inStream; }
inline JPH::StreamOut *AsStreamOut(JPH::StreamOutWrapper *inStream) { return inStream; }

template <class T>
using RestoreBinaryStateMember = void (T::*)(JPH::StreamIn &);

template <class T>
constexpr RestoreBinaryStateMember<T> GetRestoreBinaryStateMember()
{
    return &T::RestoreBinaryState;
}

using JobCallback = void (*)(void *);
using ThreadInitExitCallback = void (*)(void *, int);
using SimCollideCallback = void (*)(void *, const JPH::Body *, const JPH::Body *, const JPH::Mat44 *, const JPH::Mat44 *, JPH::CollideShapeSettings *, JPH::CollideShapeCollector *, const JPH::ShapeFilter *);
using ShaderLoaderCallback = bool (*)(void *, const char *, JPH::Array<JPH::uint8> *, JPH::String *);
using HairRenderCallback = void (*)(void *, JPH::ComputeBuffer *, JPH::Float3 *, JPH::uint);
using VehicleCombineCallback = void (*)(void *, JPH::uint, float *, float *, const JPH::Body *, const JPH::SubShapeID *);
using VehicleStepCallback = void (*)(void *, JPH::VehicleConstraint *, const JPH::PhysicsStepListenerContext *);
using TireMaxImpulseCallback = void (*)(void *, JPH::uint, float *, float *, float, float, float, float, float, float);
using BodyActivationCallback = void (*)(void *, const JPH::BodyID *, JPH::uint64);
using PhysicsStepCallback = void (*)(void *, const JPH::PhysicsStepListenerContext *);
using ContactValidateCallback = JPH::ValidateResult (*)(void *, const JPH::Body *, const JPH::Body *, const JPH::RVec3 *, const JPH::CollideShapeResult *);
using ContactCallback = void (*)(void *, const JPH::Body *, const JPH::Body *, const JPH::ContactManifold *, JPH::ContactSettings *);
using ContactRemovedCallback = void (*)(void *, const JPH::SubShapeIDPair *);
using SoftBodyValidateCallback = JPH::SoftBodyValidateResult (*)(void *, const JPH::Body *, const JPH::Body *, JPH::SoftBodyContactSettings *);
using SoftBodyContactAddedCallback = void (*)(void *, const JPH::Body *, const JPH::SoftBodyManifold *);
using SimShapeCallback = bool (*)(void *, const JPH::Body *, const JPH::Shape *, const JPH::SubShapeID *, const JPH::Body *, const JPH::Shape *, const JPH::SubShapeID *);
using ContactCombineCallback = float (*)(void *, const JPH::Body *, const JPH::SubShapeID *, const JPH::Body *, const JPH::SubShapeID *);
using BroadPhaseLayerFilterCallback = bool (*)(void *, const JPH::BroadPhaseLayer *);
using ObjectLayerFilterCallback = bool (*)(void *, JPH::ObjectLayer);
using ObjectLayerPairFilterCallback = bool (*)(void *, JPH::ObjectLayer, JPH::ObjectLayer);
using ObjectVsBroadPhaseLayerFilterCallback = bool (*)(void *, JPH::ObjectLayer, const JPH::BroadPhaseLayer *);
using BodyIDFilterCallback = bool (*)(void *, const JPH::BodyID *);
using BodyFilterLockedCallback = bool (*)(void *, const JPH::Body *);
using ShapeFilterSingleCallback = bool (*)(void *, const JPH::Shape *, const JPH::SubShapeID *);
using ShapeFilterPairCallback = bool (*)(void *, const JPH::Shape *, const JPH::SubShapeID *, const JPH::Shape *, const JPH::SubShapeID *);
using StateRecorderBodyCallback = bool (*)(void *, const JPH::Body *);
using StateRecorderConstraintCallback = bool (*)(void *, const JPH::Constraint *);
using StateRecorderContactCallback = bool (*)(void *, const JPH::BodyID *, const JPH::BodyID *);
using BroadPhaseLayerCountCallback = JPH::uint (*)(void *);
using BroadPhaseLayerMapCallback = JPH::uint8 (*)(void *, JPH::ObjectLayer);
using BroadPhaseLayerNameCallback = const char *(*)(void *, const JPH::BroadPhaseLayer *);
using CharacterAdjustVelocityCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::Body *, JPH::Vec3 *, JPH::Vec3 *);
using CharacterContactValidateCallback = bool (*)(void *, const JPH::CharacterVirtual *, const JPH::CharacterContact *);
using CharacterContactCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::CharacterContact *, JPH::CharacterContactSettings *);
using CharacterContactRemovedCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::BodyID *, const JPH::SubShapeID *);
using CharacterCharacterRemovedCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::CharacterID *, const JPH::SubShapeID *);
using CharacterBodySolveCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::BodyID *, const JPH::SubShapeID *, const JPH::RVec3 *, const JPH::Vec3 *, const JPH::Vec3 *, const JPH::PhysicsMaterial *, const JPH::Vec3 *, JPH::Vec3 *);
using CharacterCharacterSolveCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::CharacterVirtual *, const JPH::SubShapeID *, const JPH::RVec3 *, const JPH::Vec3 *, const JPH::Vec3 *, const JPH::PhysicsMaterial *, const JPH::Vec3 *, JPH::Vec3 *);
using CharacterCollideCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::RMat44 *, const JPH::CollideShapeSettings *, const JPH::RVec3 *, JPH::CollideShapeCollector *);
using CharacterCastCallback = void (*)(void *, const JPH::CharacterVirtual *, const JPH::RMat44 *, const JPH::Vec3 *, const JPH::ShapeCastSettings *, const JPH::RVec3 *, JPH::CastShapeCollector *);
using VehicleCollideCallback = bool (*)(void *, JPH::PhysicsSystem *, const JPH::VehicleConstraint *, JPH::uint, const JPH::RVec3 *, const JPH::Vec3 *, const JPH::BodyID *, JPH::Body **, JPH::SubShapeID *, JPH::RVec3 *, JPH::Vec3 *, float *);
using VehiclePredictCallback = void (*)(void *, JPH::PhysicsSystem *, const JPH::VehicleConstraint *, JPH::uint, const JPH::RVec3 *, const JPH::Vec3 *, const JPH::BodyID *, JPH::Body **, JPH::SubShapeID *, JPH::RVec3 *, JPH::Vec3 *, float *);

struct ContactCombineSlot
{
    std::atomic<ContactCombineCallback> mCallback { nullptr };
    std::atomic<void *> mUserData { nullptr };
};

inline ContactCombineSlot gContactCombineSlots[16];
inline std::mutex gContactCombineSlotMutex;

template <size_t Slot>
inline float ContactCombineThunk(const JPH::Body &inBody1, const JPH::SubShapeID &inSubShapeID1, const JPH::Body &inBody2, const JPH::SubShapeID &inSubShapeID2)
{
    ContactCombineCallback callback = gContactCombineSlots[Slot].mCallback.load(std::memory_order_acquire);
    return callback(gContactCombineSlots[Slot].mUserData.load(std::memory_order_relaxed), &inBody1, &inSubShapeID1, &inBody2, &inSubShapeID2);
}

inline constexpr JPH::ContactConstraintManager::CombineFunction gContactCombineThunks[16] = {
    ContactCombineThunk<0>, ContactCombineThunk<1>, ContactCombineThunk<2>, ContactCombineThunk<3>,
    ContactCombineThunk<4>, ContactCombineThunk<5>, ContactCombineThunk<6>, ContactCombineThunk<7>,
    ContactCombineThunk<8>, ContactCombineThunk<9>, ContactCombineThunk<10>, ContactCombineThunk<11>,
    ContactCombineThunk<12>, ContactCombineThunk<13>, ContactCombineThunk<14>, ContactCombineThunk<15>
};

class ContactCombineFunctionAdapter
{
public:
    ContactCombineFunctionAdapter(ContactCombineCallback inCallback, void *inUserData)
    {
        if (inCallback == nullptr)
            return;
        std::lock_guard<std::mutex> lock(gContactCombineSlotMutex);
        for (int slot = 0; slot < 16; ++slot)
        {
            if (gContactCombineSlots[slot].mCallback.load(std::memory_order_relaxed) == nullptr)
            {
                gContactCombineSlots[slot].mUserData.store(inUserData, std::memory_order_relaxed);
                gContactCombineSlots[slot].mCallback.store(inCallback, std::memory_order_release);
                mSlot = slot;
                return;
            }
        }
    }

    ~ContactCombineFunctionAdapter()
    {
        if (mSlot >= 0)
        {
            std::lock_guard<std::mutex> lock(gContactCombineSlotMutex);
            gContactCombineSlots[mSlot].mCallback.store(nullptr, std::memory_order_release);
            gContactCombineSlots[mSlot].mUserData.store(nullptr, std::memory_order_relaxed);
        }
    }

    bool IsValid() const { return mSlot >= 0; }
    JPH::ContactConstraintManager::CombineFunction Get() const { return mSlot < 0? nullptr : gContactCombineThunks[mSlot]; }

private:
    int mSlot = -1;
};

class BodyActivationListenerAdapter final : public JPH::BodyActivationListener
{
public:
    BodyActivationListenerAdapter(BodyActivationCallback inActivated, BodyActivationCallback inDeactivated, void *inUserData) :
        mActivated(inActivated), mDeactivated(inDeactivated), mUserData(inUserData) { }

    void OnBodyActivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
    {
        if (mActivated != nullptr)
            mActivated(mUserData, &inBodyID, inBodyUserData);
    }

    void OnBodyDeactivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
    {
        if (mDeactivated != nullptr)
            mDeactivated(mUserData, &inBodyID, inBodyUserData);
    }

private:
    BodyActivationCallback mActivated;
    BodyActivationCallback mDeactivated;
    void *mUserData;
};

class PhysicsStepListenerAdapter final : public JPH::PhysicsStepListener
{
public:
    PhysicsStepListenerAdapter(PhysicsStepCallback inCallback, void *inUserData) : mCallback(inCallback), mUserData(inUserData) { }
    void OnStep(const JPH::PhysicsStepListenerContext &inContext) override { mCallback(mUserData, &inContext); }

private:
    PhysicsStepCallback mCallback;
    void *mUserData;
};

class ContactListenerAdapter final : public JPH::ContactListener
{
public:
    ContactListenerAdapter(ContactValidateCallback inValidate, ContactCallback inAdded, ContactCallback inPersisted, ContactRemovedCallback inRemoved, void *inUserData) :
        mValidate(inValidate), mAdded(inAdded), mPersisted(inPersisted), mRemoved(inRemoved), mUserData(inUserData) { }

    JPH::ValidateResult OnContactValidate(const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult &inResult) override
    {
        return mValidate == nullptr? JPH::ValidateResult::AcceptAllContactsForThisBodyPair : mValidate(mUserData, &inBody1, &inBody2, &inBaseOffset, &inResult);
    }

    void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
    {
        if (mAdded != nullptr)
            mAdded(mUserData, &inBody1, &inBody2, &inManifold, &ioSettings);
    }

    void OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
    {
        if (mPersisted != nullptr)
            mPersisted(mUserData, &inBody1, &inBody2, &inManifold, &ioSettings);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair &inPair) override
    {
        if (mRemoved != nullptr)
            mRemoved(mUserData, &inPair);
    }

private:
    ContactValidateCallback mValidate;
    ContactCallback mAdded;
    ContactCallback mPersisted;
    ContactRemovedCallback mRemoved;
    void *mUserData;
};

class SoftBodyContactListenerAdapter final : public JPH::SoftBodyContactListener
{
public:
    SoftBodyContactListenerAdapter(SoftBodyValidateCallback inValidate, SoftBodyContactAddedCallback inAdded, void *inUserData) :
        mValidate(inValidate), mAdded(inAdded), mUserData(inUserData) { }

    JPH::SoftBodyValidateResult OnSoftBodyContactValidate(const JPH::Body &inSoftBody, const JPH::Body &inOtherBody, JPH::SoftBodyContactSettings &ioSettings) override
    {
        return mValidate == nullptr? JPH::SoftBodyValidateResult::AcceptContact : mValidate(mUserData, &inSoftBody, &inOtherBody, &ioSettings);
    }

    void OnSoftBodyContactAdded(const JPH::Body &inSoftBody, const JPH::SoftBodyManifold &inManifold) override
    {
        if (mAdded != nullptr)
            mAdded(mUserData, &inSoftBody, &inManifold);
    }

private:
    SoftBodyValidateCallback mValidate;
    SoftBodyContactAddedCallback mAdded;
    void *mUserData;
};

class SimShapeFilterAdapter final : public JPH::SimShapeFilter
{
public:
    SimShapeFilterAdapter(SimShapeCallback inCallback, void *inUserData) : mCallback(inCallback), mUserData(inUserData) { }

    bool ShouldCollide(const JPH::Body &inBody1, const JPH::Shape *inShape1, const JPH::SubShapeID &inSubShapeID1,
                       const JPH::Body &inBody2, const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeID2) const override
    {
        return mCallback(mUserData, &inBody1, inShape1, &inSubShapeID1, &inBody2, inShape2, &inSubShapeID2);
    }

private:
    SimShapeCallback mCallback;
    void *mUserData;
};

class BroadPhaseLayerFilterAdapter final : public JPH::BroadPhaseLayerFilter
{
public:
    BroadPhaseLayerFilterAdapter(BroadPhaseLayerFilterCallback inCallback, void *inUserData) : mCallback(inCallback), mUserData(inUserData) { }
    bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override { return mCallback == nullptr || mCallback(mUserData, &inLayer); }

private:
    BroadPhaseLayerFilterCallback mCallback;
    void *mUserData;
};

class ObjectLayerFilterAdapter final : public JPH::ObjectLayerFilter
{
public:
    ObjectLayerFilterAdapter(ObjectLayerFilterCallback inCallback, void *inUserData) : mCallback(inCallback), mUserData(inUserData) { }
    bool ShouldCollide(JPH::ObjectLayer inLayer) const override { return mCallback == nullptr || mCallback(mUserData, inLayer); }

private:
    ObjectLayerFilterCallback mCallback;
    void *mUserData;
};

class ObjectLayerPairFilterAdapter final : public JPH::ObjectLayerPairFilter
{
public:
    ObjectLayerPairFilterAdapter(ObjectLayerPairFilterCallback inCallback, void *inUserData) : mCallback(inCallback), mUserData(inUserData) { }
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override { return mCallback == nullptr || mCallback(mUserData, inLayer1, inLayer2); }

private:
    ObjectLayerPairFilterCallback mCallback;
    void *mUserData;
};

class ObjectVsBroadPhaseLayerFilterAdapter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    ObjectVsBroadPhaseLayerFilterAdapter(ObjectVsBroadPhaseLayerFilterCallback inCallback, void *inUserData) : mCallback(inCallback), mUserData(inUserData) { }
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override { return mCallback == nullptr || mCallback(mUserData, inLayer1, &inLayer2); }

private:
    ObjectVsBroadPhaseLayerFilterCallback mCallback;
    void *mUserData;
};

class BodyFilterAdapter final : public JPH::BodyFilter
{
public:
    BodyFilterAdapter(BodyIDFilterCallback inIDCallback, BodyFilterLockedCallback inLockedCallback, void *inUserData) :
        mIDCallback(inIDCallback), mLockedCallback(inLockedCallback), mUserData(inUserData) { }
    bool ShouldCollide(const JPH::BodyID &inBodyID) const override { return mIDCallback == nullptr || mIDCallback(mUserData, &inBodyID); }
    bool ShouldCollideLocked(const JPH::Body &inBody) const override { return mLockedCallback == nullptr || mLockedCallback(mUserData, &inBody); }

private:
    BodyIDFilterCallback mIDCallback;
    BodyFilterLockedCallback mLockedCallback;
    void *mUserData;
};

class ShapeFilterAdapter final : public JPH::ShapeFilter
{
public:
    ShapeFilterAdapter(ShapeFilterSingleCallback inSingleCallback, ShapeFilterPairCallback inPairCallback, void *inUserData) :
        mSingleCallback(inSingleCallback), mPairCallback(inPairCallback), mUserData(inUserData) { }
    bool ShouldCollide(const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeID2) const override
    {
        return mSingleCallback == nullptr || mSingleCallback(mUserData, inShape2, &inSubShapeID2);
    }
    bool ShouldCollide(const JPH::Shape *inShape1, const JPH::SubShapeID &inSubShapeID1, const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeID2) const override
    {
        return mPairCallback == nullptr || mPairCallback(mUserData, inShape1, &inSubShapeID1, inShape2, &inSubShapeID2);
    }

private:
    ShapeFilterSingleCallback mSingleCallback;
    ShapeFilterPairCallback mPairCallback;
    void *mUserData;
};

class StateRecorderFilterAdapter final : public JPH::StateRecorderFilter
{
public:
    StateRecorderFilterAdapter(StateRecorderBodyCallback inBodyCallback, StateRecorderConstraintCallback inConstraintCallback,
                               StateRecorderContactCallback inSaveContactCallback, StateRecorderContactCallback inRestoreContactCallback,
                               void *inUserData) :
        mBodyCallback(inBodyCallback), mConstraintCallback(inConstraintCallback), mSaveContactCallback(inSaveContactCallback),
        mRestoreContactCallback(inRestoreContactCallback), mUserData(inUserData) { }
    bool ShouldSaveBody(const JPH::Body &inBody) const override { return mBodyCallback == nullptr || mBodyCallback(mUserData, &inBody); }
    bool ShouldSaveConstraint(const JPH::Constraint &inConstraint) const override { return mConstraintCallback == nullptr || mConstraintCallback(mUserData, &inConstraint); }
    bool ShouldSaveContact(const JPH::BodyID &inBody1, const JPH::BodyID &inBody2) const override { return mSaveContactCallback == nullptr || mSaveContactCallback(mUserData, &inBody1, &inBody2); }
    bool ShouldRestoreContact(const JPH::BodyID &inBody1, const JPH::BodyID &inBody2) const override { return mRestoreContactCallback == nullptr || mRestoreContactCallback(mUserData, &inBody1, &inBody2); }

private:
    StateRecorderBodyCallback mBodyCallback;
    StateRecorderConstraintCallback mConstraintCallback;
    StateRecorderContactCallback mSaveContactCallback;
    StateRecorderContactCallback mRestoreContactCallback;
    void *mUserData;
};

class BroadPhaseLayerInterfaceAdapter final : public JPH::BroadPhaseLayerInterface
{
public:
    BroadPhaseLayerInterfaceAdapter(BroadPhaseLayerCountCallback inCountCallback, BroadPhaseLayerMapCallback inMapCallback,
                                    BroadPhaseLayerNameCallback inNameCallback, void *inUserData) :
        mCountCallback(inCountCallback), mMapCallback(inMapCallback), mNameCallback(inNameCallback), mUserData(inUserData) { }
    JPH::uint GetNumBroadPhaseLayers() const override { return mCountCallback(mUserData); }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override { return JPH::BroadPhaseLayer(mMapCallback(mUserData, inLayer)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
    {
        return mNameCallback == nullptr? "Unnamed" : mNameCallback(mUserData, &inLayer);
    }
#endif

private:
    BroadPhaseLayerCountCallback mCountCallback;
    BroadPhaseLayerMapCallback mMapCallback;
    BroadPhaseLayerNameCallback mNameCallback;
    void *mUserData;
};

class CharacterContactListenerAdapter final : public JPH::CharacterContactListener
{
public:
    CharacterContactListenerAdapter(CharacterAdjustVelocityCallback inAdjust, CharacterContactValidateCallback inValidate,
        CharacterContactCallback inAdded, CharacterContactCallback inPersisted, CharacterContactRemovedCallback inRemoved,
        CharacterContactValidateCallback inCharacterValidate, CharacterContactCallback inCharacterAdded,
        CharacterContactCallback inCharacterPersisted, CharacterCharacterRemovedCallback inCharacterRemoved,
        CharacterBodySolveCallback inBodySolve, CharacterCharacterSolveCallback inCharacterSolve, void *inUserData) :
        mAdjust(inAdjust), mValidate(inValidate), mAdded(inAdded), mPersisted(inPersisted), mRemoved(inRemoved),
        mCharacterValidate(inCharacterValidate), mCharacterAdded(inCharacterAdded), mCharacterPersisted(inCharacterPersisted),
        mCharacterRemoved(inCharacterRemoved), mBodySolve(inBodySolve), mCharacterSolve(inCharacterSolve), mUserData(inUserData) { }

    void OnAdjustBodyVelocity(const JPH::CharacterVirtual *inCharacter, const JPH::Body &inBody2, JPH::Vec3 &ioLinearVelocity, JPH::Vec3 &ioAngularVelocity) override
    {
        if (mAdjust != nullptr) mAdjust(mUserData, inCharacter, &inBody2, &ioLinearVelocity, &ioAngularVelocity);
    }
    bool OnContactValidate(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterContact &inContact) override
    {
        return mValidate == nullptr || mValidate(mUserData, inCharacter, &inContact);
    }
    void OnContactAdded(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterContact &inContact, JPH::CharacterContactSettings &ioSettings) override
    {
        if (mAdded != nullptr) mAdded(mUserData, inCharacter, &inContact, &ioSettings);
    }
    void OnContactPersisted(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterContact &inContact, JPH::CharacterContactSettings &ioSettings) override
    {
        if (mPersisted != nullptr) mPersisted(mUserData, inCharacter, &inContact, &ioSettings);
    }
    void OnContactRemoved(const JPH::CharacterVirtual *inCharacter, const JPH::BodyID &inBodyID2, const JPH::SubShapeID &inSubShapeID2) override
    {
        if (mRemoved != nullptr) mRemoved(mUserData, inCharacter, &inBodyID2, &inSubShapeID2);
    }
    bool OnCharacterContactValidate(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterContact &inContact) override
    {
        return mCharacterValidate == nullptr || mCharacterValidate(mUserData, inCharacter, &inContact);
    }
    void OnCharacterContactAdded(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterContact &inContact, JPH::CharacterContactSettings &ioSettings) override
    {
        if (mCharacterAdded != nullptr) mCharacterAdded(mUserData, inCharacter, &inContact, &ioSettings);
    }
    void OnCharacterContactPersisted(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterContact &inContact, JPH::CharacterContactSettings &ioSettings) override
    {
        if (mCharacterPersisted != nullptr) mCharacterPersisted(mUserData, inCharacter, &inContact, &ioSettings);
    }
    void OnCharacterContactRemoved(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterID &inOtherCharacterID, const JPH::SubShapeID &inSubShapeID2) override
    {
        if (mCharacterRemoved != nullptr) mCharacterRemoved(mUserData, inCharacter, &inOtherCharacterID, &inSubShapeID2);
    }
    void OnContactSolve(const JPH::CharacterVirtual *inCharacter, const JPH::BodyID &inBodyID2, const JPH::SubShapeID &inSubShapeID2,
        JPH::RVec3Arg inContactPosition, JPH::Vec3Arg inContactNormal, JPH::Vec3Arg inContactVelocity, const JPH::PhysicsMaterial *inContactMaterial,
        JPH::Vec3Arg inCharacterVelocity, JPH::Vec3 &ioNewCharacterVelocity) override
    {
        if (mBodySolve != nullptr) mBodySolve(mUserData, inCharacter, &inBodyID2, &inSubShapeID2, &inContactPosition, &inContactNormal, &inContactVelocity, inContactMaterial, &inCharacterVelocity, &ioNewCharacterVelocity);
    }
    void OnCharacterContactSolve(const JPH::CharacterVirtual *inCharacter, const JPH::CharacterVirtual *inOtherCharacter, const JPH::SubShapeID &inSubShapeID2,
        JPH::RVec3Arg inContactPosition, JPH::Vec3Arg inContactNormal, JPH::Vec3Arg inContactVelocity, const JPH::PhysicsMaterial *inContactMaterial,
        JPH::Vec3Arg inCharacterVelocity, JPH::Vec3 &ioNewCharacterVelocity) override
    {
        if (mCharacterSolve != nullptr) mCharacterSolve(mUserData, inCharacter, inOtherCharacter, &inSubShapeID2, &inContactPosition, &inContactNormal, &inContactVelocity, inContactMaterial, &inCharacterVelocity, &ioNewCharacterVelocity);
    }

private:
    CharacterAdjustVelocityCallback mAdjust;
    CharacterContactValidateCallback mValidate;
    CharacterContactCallback mAdded;
    CharacterContactCallback mPersisted;
    CharacterContactRemovedCallback mRemoved;
    CharacterContactValidateCallback mCharacterValidate;
    CharacterContactCallback mCharacterAdded;
    CharacterContactCallback mCharacterPersisted;
    CharacterCharacterRemovedCallback mCharacterRemoved;
    CharacterBodySolveCallback mBodySolve;
    CharacterCharacterSolveCallback mCharacterSolve;
    void *mUserData;
};

class CharacterVsCharacterCollisionAdapter final : public JPH::CharacterVsCharacterCollision
{
public:
    CharacterVsCharacterCollisionAdapter(CharacterCollideCallback inCollide, CharacterCastCallback inCast, void *inUserData) :
        mCollide(inCollide), mCast(inCast), mUserData(inUserData) { }
    void CollideCharacter(const JPH::CharacterVirtual *inCharacter, JPH::RMat44Arg inCenterOfMassTransform,
        const JPH::CollideShapeSettings &inSettings, JPH::RVec3Arg inBaseOffset, JPH::CollideShapeCollector &ioCollector) const override
    {
        mCollide(mUserData, inCharacter, &inCenterOfMassTransform, &inSettings, &inBaseOffset, &ioCollector);
    }
    void CastCharacter(const JPH::CharacterVirtual *inCharacter, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inDirection,
        const JPH::ShapeCastSettings &inSettings, JPH::RVec3Arg inBaseOffset, JPH::CastShapeCollector &ioCollector) const override
    {
        mCast(mUserData, inCharacter, &inCenterOfMassTransform, &inDirection, &inSettings, &inBaseOffset, &ioCollector);
    }

private:
    CharacterCollideCallback mCollide;
    CharacterCastCallback mCast;
    void *mUserData;
};

class VehicleCollisionTesterAdapter final : public JPH::VehicleCollisionTester
{
public:
    VehicleCollisionTesterAdapter(JPH::ObjectLayer inObjectLayer, VehicleCollideCallback inCollide, VehiclePredictCallback inPredict, void *inUserData) :
        JPH::VehicleCollisionTester(inObjectLayer), mCollide(inCollide), mPredict(inPredict), mUserData(inUserData) { }
    bool Collide(JPH::PhysicsSystem &inPhysicsSystem, const JPH::VehicleConstraint &inVehicleConstraint, JPH::uint inWheelIndex,
        JPH::RVec3Arg inOrigin, JPH::Vec3Arg inDirection, const JPH::BodyID &inVehicleBodyID, JPH::Body *&outBody,
        JPH::SubShapeID &outSubShapeID, JPH::RVec3 &outContactPosition, JPH::Vec3 &outContactNormal, float &outSuspensionLength) const override
    {
        return mCollide(mUserData, &inPhysicsSystem, &inVehicleConstraint, inWheelIndex, &inOrigin, &inDirection, &inVehicleBodyID,
            &outBody, &outSubShapeID, &outContactPosition, &outContactNormal, &outSuspensionLength);
    }
    void PredictContactProperties(JPH::PhysicsSystem &inPhysicsSystem, const JPH::VehicleConstraint &inVehicleConstraint, JPH::uint inWheelIndex,
        JPH::RVec3Arg inOrigin, JPH::Vec3Arg inDirection, const JPH::BodyID &inVehicleBodyID, JPH::Body *&ioBody,
        JPH::SubShapeID &ioSubShapeID, JPH::RVec3 &ioContactPosition, JPH::Vec3 &ioContactNormal, float &ioSuspensionLength) const override
    {
        if (mPredict != nullptr)
            mPredict(mUserData, &inPhysicsSystem, &inVehicleConstraint, inWheelIndex, &inOrigin, &inDirection, &inVehicleBodyID,
                &ioBody, &ioSubShapeID, &ioContactPosition, &ioContactNormal, &ioSuspensionLength);
    }

private:
    VehicleCollideCallback mCollide;
    VehiclePredictCallback mPredict;
    void *mUserData;
};

template <class Collector, class Result>
class CollisionCollectorAdapter final : public Collector
{
public:
    using Callback = void (*)(void *, const Result *);

    CollisionCollectorAdapter(Callback inCallback, void *inUserData) : mCallback(inCallback), mUserData(inUserData) { }
    void AddHit(const Result &inResult) override { mCallback(mUserData, &inResult); }

private:
    Callback mCallback;
    void *mUserData;
};

using CastRayCollectorAdapter = CollisionCollectorAdapter<JPH::CastRayCollector, JPH::RayCastResult>;
using CastShapeCollectorAdapter = CollisionCollectorAdapter<JPH::CastShapeCollector, JPH::ShapeCastResult>;
using CollidePointCollectorAdapter = CollisionCollectorAdapter<JPH::CollidePointCollector, JPH::CollidePointResult>;
using CollideShapeCollectorAdapter = CollisionCollectorAdapter<JPH::CollideShapeCollector, JPH::CollideShapeResult>;
using TransformedShapeCollectorAdapter = CollisionCollectorAdapter<JPH::TransformedShapeCollector, JPH::TransformedShape>;
using RayCastBodyCollectorAdapter = CollisionCollectorAdapter<JPH::RayCastBodyCollector, JPH::BroadPhaseCastResult>;
using CastShapeBodyCollectorAdapter = CollisionCollectorAdapter<JPH::CastShapeBodyCollector, JPH::BroadPhaseCastResult>;
using CollideShapeBodyCollectorAdapter = CollisionCollectorAdapter<JPH::CollideShapeBodyCollector, JPH::BodyID>;
using BodyPairCollectorAdapter = CollisionCollectorAdapter<JPH::BodyPairCollector, JPH::BodyPair>;

inline JPH::BodyActivationListener *AsBodyActivationListener(BodyActivationListenerAdapter *inListener) { return inListener; }
inline JPH::PhysicsStepListener *AsPhysicsStepListener(PhysicsStepListenerAdapter *inListener) { return inListener; }
inline JPH::ContactListener *AsContactListener(ContactListenerAdapter *inListener) { return inListener; }
inline JPH::SoftBodyContactListener *AsSoftBodyContactListener(SoftBodyContactListenerAdapter *inListener) { return inListener; }
inline JPH::SimShapeFilter *AsSimShapeFilter(SimShapeFilterAdapter *inFilter) { return inFilter; }
inline JPH::BroadPhaseLayerFilter *AsBroadPhaseLayerFilter(BroadPhaseLayerFilterAdapter *inFilter) { return inFilter; }
inline JPH::ObjectLayerFilter *AsObjectLayerFilter(ObjectLayerFilterAdapter *inFilter) { return inFilter; }
inline JPH::ObjectLayerPairFilter *AsObjectLayerPairFilter(ObjectLayerPairFilterAdapter *inFilter) { return inFilter; }
inline JPH::ObjectVsBroadPhaseLayerFilter *AsObjectVsBroadPhaseLayerFilter(ObjectVsBroadPhaseLayerFilterAdapter *inFilter) { return inFilter; }
inline JPH::BodyFilter *AsBodyFilter(BodyFilterAdapter *inFilter) { return inFilter; }
inline JPH::ShapeFilter *AsShapeFilter(ShapeFilterAdapter *inFilter) { return inFilter; }
inline JPH::StateRecorderFilter *AsStateRecorderFilter(StateRecorderFilterAdapter *inFilter) { return inFilter; }
inline JPH::BroadPhaseLayerInterface *AsBroadPhaseLayerInterface(BroadPhaseLayerInterfaceAdapter *inInterface) { return inInterface; }
inline JPH::CharacterContactListener *AsCharacterContactListener(CharacterContactListenerAdapter *inListener) { return inListener; }
inline JPH::CharacterVsCharacterCollision *AsCharacterVsCharacterCollision(CharacterVsCharacterCollisionAdapter *inCollision) { return inCollision; }
inline JPH::VehicleCollisionTester *AsVehicleCollisionTester(VehicleCollisionTesterAdapter *inTester) { return inTester; }
inline JPH::StateRecorder *AsStateRecorder(JPH::StateRecorderImpl *inRecorder) { return inRecorder; }
inline JPH::CastRayCollector *AsCastRayCollector(CastRayCollectorAdapter *inCollector) { return inCollector; }
inline JPH::CastShapeCollector *AsCastShapeCollector(CastShapeCollectorAdapter *inCollector) { return inCollector; }
inline JPH::CollidePointCollector *AsCollidePointCollector(CollidePointCollectorAdapter *inCollector) { return inCollector; }
inline JPH::CollideShapeCollector *AsCollideShapeCollector(CollideShapeCollectorAdapter *inCollector) { return inCollector; }
inline JPH::TransformedShapeCollector *AsTransformedShapeCollector(TransformedShapeCollectorAdapter *inCollector) { return inCollector; }
inline JPH::RayCastBodyCollector *AsRayCastBodyCollector(RayCastBodyCollectorAdapter *inCollector) { return inCollector; }
inline JPH::CastShapeBodyCollector *AsCastShapeBodyCollector(CastShapeBodyCollectorAdapter *inCollector) { return inCollector; }
inline JPH::CollideShapeBodyCollector *AsCollideShapeBodyCollector(CollideShapeBodyCollectorAdapter *inCollector) { return inCollector; }
inline JPH::BodyPairCollector *AsBodyPairCollector(BodyPairCollectorAdapter *inCollector) { return inCollector; }

inline void InvokeBodyActivated(BodyActivationListenerAdapter *inListener, const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) { inListener->OnBodyActivated(inBodyID, inBodyUserData); }
inline void InvokePhysicsStep(PhysicsStepListenerAdapter *inListener, const JPH::PhysicsStepListenerContext &inContext) { inListener->OnStep(inContext); }

inline JPH::JobSystem::JobFunction MakeJobFunction(JobCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData]() { inCallback(inUserData); };
}

inline JPH::JobSystemThreadPool::InitExitFunction MakeThreadInitExitFunction(ThreadInitExitCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData](int inThreadIndex) { inCallback(inUserData, inThreadIndex); };
}

inline JPH::JobSystem *AsJobSystem(JPH::JobSystemThreadPool *inJobSystem)
{
    return inJobSystem;
}

inline JPH::TempAllocator *AsTempAllocator(JPH::TempAllocatorImpl *inAllocator)
{
    return inAllocator;
}

template <class ShapeType>
inline const JPH::Shape *AsShape(const ShapeType *inShape)
{
    return inShape;
}

template <class SettingsType>
inline const JPH::ShapeSettings *AsShapeSettings(const SettingsType *inSettings)
{
    return inSettings;
}

template <class ShapeType>
inline const JPH::ConvexShape *AsConvexShape(const ShapeType *inShape)
{
    return inShape;
}

template <class ShapeType>
inline const JPH::CompoundShape *AsCompoundShape(const ShapeType *inShape)
{
    return inShape;
}

template <class ShapeType>
inline const JPH::DecoratedShape *AsDecoratedShape(const ShapeType *inShape)
{
    return inShape;
}

template <class SettingsType>
inline const JPH::TwoBodyConstraintSettings *AsTwoBodyConstraintSettings(const SettingsType *inSettings)
{
    return inSettings;
}

inline const JPH::PathConstraintPath *AsPathConstraintPath(const JPH::PathConstraintPathHermite *inPath)
{
    return inPath;
}

template <class SettingsType>
inline JPH::WheelSettings *AsWheelSettings(SettingsType *inSettings)
{
    return inSettings;
}

template <class SettingsType>
inline JPH::VehicleControllerSettings *AsVehicleControllerSettings(SettingsType *inSettings)
{
    return inSettings;
}

inline JPH::WheeledVehicleControllerSettings *AsWheeledVehicleControllerSettings(JPH::MotorcycleControllerSettings *inSettings)
{
    return inSettings;
}

inline JPH::CharacterBaseSettings *AsCharacterBaseSettings(JPH::CharacterSettings *inSettings)
{
    return inSettings;
}

inline JPH::CharacterBaseSettings *AsCharacterBaseSettings(JPH::CharacterVirtualSettings *inSettings)
{
    return inSettings;
}

inline void SetCharacterSettingsShape(JPH::CharacterBaseSettings *inSettings, const JPH::Shape *inShape)
{
    inSettings->mShape = inShape;
}

inline const JPH::Shape *GetCharacterSettingsShape(const JPH::CharacterBaseSettings *inSettings)
{
    return inSettings->mShape;
}

inline void SetCharacterVirtualInnerBodyShape(JPH::CharacterVirtualSettings *inSettings, const JPH::Shape *inShape)
{
    inSettings->mInnerBodyShape = inShape;
}

inline const JPH::Shape *GetCharacterVirtualInnerBodyShape(const JPH::CharacterVirtualSettings *inSettings)
{
    return inSettings->mInnerBodyShape;
}

inline JPH::BodyCreationSettings *AsBodyCreationSettings(JPH::RagdollSettings::Part *inPart)
{
    return inPart;
}

inline void SetRagdollSkeleton(JPH::RagdollSettings *inSettings, JPH::Skeleton *inSkeleton)
{
    inSettings->mSkeleton = inSkeleton;
}

inline void SetRagdollPartToParent(JPH::RagdollSettings::Part *inPart, JPH::TwoBodyConstraintSettings *inConstraint)
{
    inPart->mToParent = inConstraint;
}

inline JPH::TwoBodyConstraintSettings *GetRagdollPartToParent(JPH::RagdollSettings::Part *inPart)
{
    return inPart->mToParent;
}

inline void AddRagdollPart(JPH::RagdollSettings *inSettings, const JPH::RagdollSettings::Part &inPart)
{
    inSettings->mParts.push_back(inPart);
}

inline JPH::uint GetRagdollPartCount(const JPH::RagdollSettings *inSettings)
{
    return static_cast<JPH::uint>(inSettings->mParts.size());
}

inline JPH::RagdollSettings::Part *GetRagdollPart(JPH::RagdollSettings *inSettings, JPH::uint inIndex)
{
    return &inSettings->mParts[inIndex];
}

inline void ClearRagdollParts(JPH::RagdollSettings *inSettings)
{
    inSettings->mParts.clear();
}

inline void AddRagdollAdditionalConstraint(JPH::RagdollSettings *inSettings, int inBodyIndex1, int inBodyIndex2, JPH::TwoBodyConstraintSettings *inConstraint)
{
    inSettings->mAdditionalConstraints.emplace_back(inBodyIndex1, inBodyIndex2, inConstraint);
}

inline JPH::uint GetRagdollAdditionalConstraintCount(const JPH::RagdollSettings *inSettings)
{
    return static_cast<JPH::uint>(inSettings->mAdditionalConstraints.size());
}

inline JPH::RagdollSettings::AdditionalConstraint *GetRagdollAdditionalConstraint(JPH::RagdollSettings *inSettings, JPH::uint inIndex)
{
    return &inSettings->mAdditionalConstraints[inIndex];
}

inline int GetRagdollAdditionalConstraintBodyIndex(const JPH::RagdollSettings::AdditionalConstraint *inConstraint, JPH::uint inSide)
{
    return inConstraint->mBodyIdx[inSide];
}

inline JPH::TwoBodyConstraintSettings *GetRagdollAdditionalConstraintSettings(JPH::RagdollSettings::AdditionalConstraint *inConstraint)
{
    return inConstraint->mConstraint;
}

inline void ClearRagdollAdditionalConstraints(JPH::RagdollSettings *inSettings)
{
    inSettings->mAdditionalConstraints.clear();
}

inline JPH::BodyCreationSettings *GetPhysicsSceneBody(JPH::PhysicsScene *inScene, JPH::uint inIndex)
{
    return &inScene->GetBodies()[inIndex];
}

inline JPH::PhysicsScene::ConnectedConstraint *GetPhysicsSceneConstraint(JPH::PhysicsScene *inScene, JPH::uint inIndex)
{
    return &inScene->GetConstraints()[inIndex];
}

inline JPH::TwoBodyConstraintSettings *GetPhysicsSceneConstraintSettings(JPH::PhysicsScene::ConnectedConstraint *inConstraint)
{
    return const_cast<JPH::TwoBodyConstraintSettings *>(static_cast<const JPH::TwoBodyConstraintSettings *>(inConstraint->mSettings));
}

inline JPH::SoftBodyCreationSettings *GetPhysicsSceneSoftBody(JPH::PhysicsScene *inScene, JPH::uint inIndex)
{
    return &inScene->GetSoftBodies()[inIndex];
}

inline JPH::uint32 GetPhysicsSceneFixedToWorld()
{
    return JPH::PhysicsScene::cFixedToWorld;
}

inline JPH::SkeletalAnimation::AnimatedJoint *AddSkeletalAnimationJoint(JPH::SkeletalAnimation *inAnimation, const char *inName)
{
    JPH::SkeletalAnimation::AnimatedJointVector &joints = inAnimation->GetAnimatedJoints();
    joints.emplace_back();
    joints.back().mJointName = inName;
    return &joints.back();
}

inline JPH::uint GetSkeletalAnimationJointCount(const JPH::SkeletalAnimation *inAnimation)
{
    return static_cast<JPH::uint>(inAnimation->GetAnimatedJoints().size());
}

inline JPH::SkeletalAnimation::AnimatedJoint *GetSkeletalAnimationJoint(JPH::SkeletalAnimation *inAnimation, JPH::uint inIndex)
{
    return &inAnimation->GetAnimatedJoints()[inIndex];
}

inline const char *GetSkeletalAnimationJointName(const JPH::SkeletalAnimation::AnimatedJoint *inJoint)
{
    return inJoint->mJointName.c_str();
}

inline void ClearSkeletalAnimationJoints(JPH::SkeletalAnimation *inAnimation)
{
    inAnimation->GetAnimatedJoints().clear();
}

inline JPH::SkeletalAnimation::Keyframe *AddSkeletalAnimationKeyframe(JPH::SkeletalAnimation::AnimatedJoint *inJoint, float inTime, const JPH::Quat &inRotation, const JPH::Vec3 &inTranslation)
{
    inJoint->mKeyframes.emplace_back();
    JPH::SkeletalAnimation::Keyframe &keyframe = inJoint->mKeyframes.back();
    keyframe.mTime = inTime;
    keyframe.mRotation = inRotation;
    keyframe.mTranslation = inTranslation;
    return &keyframe;
}

inline JPH::uint GetSkeletalAnimationKeyframeCount(const JPH::SkeletalAnimation::AnimatedJoint *inJoint)
{
    return static_cast<JPH::uint>(inJoint->mKeyframes.size());
}

inline JPH::SkeletalAnimation::Keyframe *GetSkeletalAnimationKeyframe(JPH::SkeletalAnimation::AnimatedJoint *inJoint, JPH::uint inIndex)
{
    return &inJoint->mKeyframes[inIndex];
}

inline void ClearSkeletalAnimationKeyframes(JPH::SkeletalAnimation::AnimatedJoint *inJoint)
{
    inJoint->mKeyframes.clear();
}

inline JPH::SkeletalAnimation::JointState *AsSkeletalAnimationJointState(JPH::SkeletalAnimation::Keyframe *inKeyframe)
{
    return inKeyframe;
}

inline JPH::Array<JPH::HairSettings::SVertex> *GetHairSimVertices(JPH::HairSettings *inSettings)
{
    return &inSettings->mSimVertices;
}

inline JPH::Array<JPH::HairSettings::SStrand> *GetHairSimStrands(JPH::HairSettings *inSettings)
{
    return &inSettings->mSimStrands;
}

inline JPH::Array<JPH::HairSettings::RVertex> *GetHairRenderVertices(JPH::HairSettings *inSettings)
{
    return &inSettings->mRenderVertices;
}

inline JPH::Array<JPH::HairSettings::RStrand> *GetHairRenderStrands(JPH::HairSettings *inSettings)
{
    return &inSettings->mRenderStrands;
}

inline JPH::Array<JPH::Float3> *GetHairScalpVertices(JPH::HairSettings *inSettings)
{
    return &inSettings->mScalpVertices;
}

inline JPH::Array<JPH::IndexedTriangleNoMaterial> *GetHairScalpTriangles(JPH::HairSettings *inSettings)
{
    return &inSettings->mScalpTriangles;
}

inline JPH::Array<JPH::Mat44> *GetHairScalpInverseBindPose(JPH::HairSettings *inSettings)
{
    return &inSettings->mScalpInverseBindPose;
}

inline JPH::Array<JPH::HairSettings::SkinWeight> *GetHairScalpSkinWeights(JPH::HairSettings *inSettings)
{
    return &inSettings->mScalpSkinWeights;
}

inline JPH::Array<JPH::HairSettings::Material> *GetHairMaterials(JPH::HairSettings *inSettings)
{
    return &inSettings->mMaterials;
}

inline JPH::Array<JPH::HairSettings::SkinPoint> *GetHairSkinPoints(JPH::HairSettings *inSettings)
{
    return &inSettings->mSkinPoints;
}

inline JPH::Array<float> *GetHairNeutralDensity(JPH::HairSettings *inSettings)
{
    return &inSettings->mNeutralDensity;
}

inline JPH::HairSettings::RStrand *AsHairRenderStrand(JPH::HairSettings::SStrand *inStrand)
{
    return inStrand;
}

inline JPH::HairSettings::SVertexInfluence *GetHairRenderVertexInfluence(JPH::HairSettings::RVertex *inVertex, JPH::uint inIndex)
{
    return &inVertex->mInfluences[inIndex];
}

inline JPH::uint32 GetHairNoInfluence()
{
    return JPH::HairSettings::cNoInfluence;
}

inline JPH::uint32 GetHairNumSVertexInfluences()
{
    return static_cast<JPH::uint32>(cHairNumSVertexInfluences);
}

inline void SetPathConstraintSettingsPath(JPH::PathConstraintSettings *inSettings, const JPH::PathConstraintPath *inPath)
{
    inSettings->mPath = inPath;
}

inline const JPH::PathConstraintPath *GetPathConstraintSettingsPath(const JPH::PathConstraintSettings *inSettings)
{
    return inSettings->mPath;
}

inline void AddVehicleWheelSettings(JPH::VehicleConstraintSettings *inSettings, JPH::WheelSettings *inWheel)
{
    inSettings->mWheels.push_back(inWheel);
}

inline JPH::uint GetVehicleWheelSettingsCount(const JPH::VehicleConstraintSettings *inSettings)
{
    return static_cast<JPH::uint>(inSettings->mWheels.size());
}

inline JPH::WheelSettings *GetVehicleWheelSettings(JPH::VehicleConstraintSettings *inSettings, JPH::uint inIndex)
{
    return inSettings->mWheels[inIndex];
}

inline void ClearVehicleWheelSettings(JPH::VehicleConstraintSettings *inSettings)
{
    inSettings->mWheels.clear();
}

inline void SetVehicleControllerSettings(JPH::VehicleConstraintSettings *inSettings, JPH::VehicleControllerSettings *inController)
{
    inSettings->mController = inController;
}

inline JPH::VehicleControllerSettings *GetVehicleControllerSettings(JPH::VehicleConstraintSettings *inSettings)
{
    return inSettings->mController;
}

inline void AddVehicleDifferentialSettings(JPH::WheeledVehicleControllerSettings *inSettings, const JPH::VehicleDifferentialSettings &inDifferential)
{
    inSettings->mDifferentials.push_back(inDifferential);
}

inline JPH::uint GetVehicleDifferentialSettingsCount(const JPH::WheeledVehicleControllerSettings *inSettings)
{
    return static_cast<JPH::uint>(inSettings->mDifferentials.size());
}

inline JPH::VehicleDifferentialSettings *GetVehicleDifferentialSettings(JPH::WheeledVehicleControllerSettings *inSettings, JPH::uint inIndex)
{
    return &inSettings->mDifferentials[inIndex];
}

inline void ClearVehicleDifferentialSettings(JPH::WheeledVehicleControllerSettings *inSettings)
{
    inSettings->mDifferentials.clear();
}

inline JPH::VehicleTrackSettings *GetTrackedVehicleTrackSettings(JPH::TrackedVehicleControllerSettings *inSettings, JPH::uint inIndex)
{
    return &inSettings->mTracks[inIndex];
}

inline void AddVehicleTrackWheel(JPH::VehicleTrackSettings *inSettings, JPH::uint inWheelIndex)
{
    inSettings->mWheels.push_back(inWheelIndex);
}

inline JPH::uint GetVehicleTrackWheelCount(const JPH::VehicleTrackSettings *inSettings)
{
    return static_cast<JPH::uint>(inSettings->mWheels.size());
}

inline JPH::uint GetVehicleTrackWheel(const JPH::VehicleTrackSettings *inSettings, JPH::uint inIndex)
{
    return inSettings->mWheels[inIndex];
}

inline void ClearVehicleTrackWheels(JPH::VehicleTrackSettings *inSettings)
{
    inSettings->mWheels.clear();
}

inline JPH::VehicleDifferentialSettings *GetRuntimeVehicleDifferential(JPH::WheeledVehicleController *inController, JPH::uint inIndex)
{
    return &inController->GetDifferentials()[inIndex];
}

inline JPH::VehicleTrack *GetRuntimeVehicleTrack(JPH::TrackedVehicleController *inController, JPH::uint inIndex)
{
    return &inController->GetTracks()[inIndex];
}

template <class TesterType>
inline JPH::VehicleCollisionTester *AsVehicleCollisionTester(TesterType *inTester)
{
    return inTester;
}

inline JPH::CharacterVsCharacterCollision *AsCharacterVsCharacterCollision(JPH::CharacterVsCharacterCollisionSimple *inCollision)
{
    return inCollision;
}

inline JPH::GroupFilter *AsGroupFilter(JPH::GroupFilterTable *inFilter)
{
    return inFilter;
}

template <class InterfaceType>
inline JPH::BodyLockInterface *AsBodyLockInterface(InterfaceType *inInterface)
{
    return inInterface;
}

inline void InvokeJobFunction(const JPH::JobSystem::JobFunction &inFunction)
{
    inFunction();
}

inline JPH::PhysicsSystem::SimCollideBodyVsBody MakeSimCollideBodyVsBody(SimCollideCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData](const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::Mat44Arg inTransform1, JPH::Mat44Arg inTransform2, JPH::CollideShapeSettings &ioSettings, JPH::CollideShapeCollector &ioCollector, const JPH::ShapeFilter &inFilter)
    {
        inCallback(inUserData, &inBody1, &inBody2, &inTransform1, &inTransform2, &ioSettings, &ioCollector, &inFilter);
    };
}

inline JPH::ComputeSystem::ShaderLoader MakeShaderLoader(ShaderLoaderCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData](const char *inName, JPH::Array<JPH::uint8> &outData, JPH::String &outError)
    {
        return inCallback(inUserData, inName, &outData, &outError);
    };
}

inline bool InvokeShaderLoader(const JPH::ComputeSystem::ShaderLoader &inLoader, const char *inName, JPH::Array<JPH::uint8> *outData, JPH::String *outError)
{
    return inLoader(inName, *outData, *outError);
}

inline void SetString(JPH::String *outString, const char *inValue)
{
    *outString = inValue;
}

inline JPH::Hair::RenderPositionsToFloat3 MakeHairRenderPositions(HairRenderCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData](JPH::ComputeBuffer *inBuffer, JPH::Float3 *outPositions, JPH::uint inCount)
    {
        inCallback(inUserData, inBuffer, outPositions, inCount);
    };
}

inline JPH::VehicleConstraint::CombineFunction MakeVehicleCombineFriction(VehicleCombineCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData](JPH::uint inWheelIndex, float &ioLongitudinal, float &ioLateral, const JPH::Body &inBody, const JPH::SubShapeID &inSubShapeID)
    {
        inCallback(inUserData, inWheelIndex, &ioLongitudinal, &ioLateral, &inBody, &inSubShapeID);
    };
}

inline JPH::VehicleConstraint::StepCallback MakeVehicleStepCallback(VehicleStepCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData](JPH::VehicleConstraint &inVehicle, const JPH::PhysicsStepListenerContext &inContext)
    {
        inCallback(inUserData, &inVehicle, &inContext);
    };
}

inline JPH::WheeledVehicleController::TireMaxImpulseCallback MakeTireMaxImpulseCallback(TireMaxImpulseCallback inCallback, void *inUserData)
{
    return [inCallback, inUserData](JPH::uint inWheelIndex, float &outLongitudinal, float &outLateral, float inSuspension, float inLongitudinalFriction, float inLateralFriction, float inLongitudinalSlip, float inLateralSlip, float inDeltaTime)
    {
        inCallback(inUserData, inWheelIndex, &outLongitudinal, &outLateral, inSuspension, inLongitudinalFriction, inLateralFriction, inLongitudinalSlip, inLateralSlip, inDeltaTime);
    };
}
}
