import std/math
import jolt/raw as api

proc closeEnough(a, b: cfloat): bool =
  abs(a - b) < 1.0e-5'f32

proc testHlslToCpp() =
  var a = api.constructHLSLToCPPFloat3(1.0'f32, 2.0'f32, 3.0'f32)
  let b = api.constructHLSLToCPPFloat3(4.0'f32, 5.0'f32, 6.0'f32)
  let sum = api.hlslAdd(api.JoltApi, a, b)
  doAssert sum.x == 5.0'f32
  doAssert sum.y == 7.0'f32
  doAssert sum.z == 9.0'f32
  doAssert api.hlslDotFloat3(api.JoltApi, a, b) == 32.0'f32

  let scaled = api.hlslMulRight(api.JoltApi, a, 2.0'f32)
  doAssert scaled == api.constructHLSLToCPPFloat3(2.0'f32, 4.0'f32, 6.0'f32)
  let reversedScale = api.hlslMulLeft(api.JoltApi, 3.0'f32, a)
  doAssert reversedScale.z == 9.0'f32
  let divided = api.hlslDivScalar(api.JoltApi, reversedScale, 3.0'f32)
  doAssert divided == a

  discard a.addAssign(api.constructHLSLToCPPFloat3(1.0'f32))
  doAssert a == api.constructHLSLToCPPFloat3(2.0'f32, 3.0'f32, 4.0'f32)
  a.hlslIndexMutable(api.uint(1))[] = 8.0'f32
  doAssert a[api.uint(1)] == 8.0'f32
  doAssert a.swizzle_zyx() == api.constructHLSLToCPPFloat3(4.0'f32, 8.0'f32, 2.0'f32)

  let cross = api.cross(
    api.JoltApi,
    api.constructHLSLToCPPFloat3(1.0'f32, 0.0'f32, 0.0'f32),
    api.constructHLSLToCPPFloat3(0.0'f32, 1.0'f32, 0.0'f32),
  )
  doAssert cross == api.constructHLSLToCPPFloat3(0.0'f32, 0.0'f32, 1.0'f32)
  let normalized = api.hlslNormalize(
    api.JoltApi,
    api.constructHLSLToCPPFloat2(3.0'f32, 4.0'f32),
  )
  doAssert closeEnough(api.hlslLength(api.JoltApi, normalized), 1.0'f32)
  let rounded = api.hlslRound(
    api.JoltApi,
    api.constructHLSLToCPPFloat2(1.4'f32, 1.6'f32),
  )
  doAssert rounded == api.constructHLSLToCPPFloat2(1.0'f32, 2.0'f32)
  let f2a = api.constructHLSLToCPPFloat2(8.0'f32, 12.0'f32)
  let f2b = api.constructHLSLToCPPFloat2(2.0'f32, 3.0'f32)
  doAssert api.hlslSub(api.JoltApi, f2a, f2b) ==
    api.constructHLSLToCPPFloat2(6.0'f32, 9.0'f32)
  doAssert api.hlslMul(api.JoltApi, f2a, f2b) ==
    api.constructHLSLToCPPFloat2(16.0'f32, 36.0'f32)
  doAssert api.hlslDiv(api.JoltApi, f2a, f2b) ==
    api.constructHLSLToCPPFloat2(4.0'f32, 4.0'f32)
  doAssert api.hlslMin(api.JoltApi, f2a, f2b) == f2b
  doAssert api.hlslMax(api.JoltApi, f2a, f2b) == f2a
  doAssert api.clamp(
    api.JoltApi,
    f2a,
    api.constructHLSLToCPPFloat2(0.0'f32),
    api.constructHLSLToCPPFloat2(10.0'f32),
  ) == api.constructHLSLToCPPFloat2(8.0'f32, 10.0'f32)
  doAssert api.hlslSub(api.JoltApi, f2b) ==
    api.constructHLSLToCPPFloat2(-2.0'f32, -3.0'f32)

  let f4 = api.constructHLSLToCPPFloat4(1.0'f32, 2.0'f32, 3.0'f32, 4.0'f32)
  doAssert api.hlslDotFloat4(api.JoltApi, f4, f4) == 30.0'f32
  doAssert f4.swizzle_wxyz() ==
    api.constructHLSLToCPPFloat4(4.0'f32, 1.0'f32, 2.0'f32, 3.0'f32)

  let uints = api.constructHLSLToCPPUint4(1'u32, 2'u32, 3'u32, 4'u32)
  doAssert api.hlslDotUint4(api.JoltApi, uints, uints) == 30'u32
  let uint3s = api.constructHLSLToCPPUint3(2'u32, 3'u32, 4'u32)
  doAssert api.hlslDotUint3(api.JoltApi, uint3s, uint3s) == 29'u32
  doAssert api.hlslAdd(api.JoltApi, uint3s, uint3s).x == 4'u32
  doAssert api.hlslMulRight(api.JoltApi, uints, 2'u32).w == 8'u32
  doAssert api.hlslDiv(api.JoltApi, uints, uints).z == 1'u32
  let ints = api.constructHLSLToCPPInt4(-1.cint, 2.cint, -3.cint, 4.cint)
  doAssert api.hlslDotInt4(api.JoltApi, ints, ints) == 30
  let int3s = api.constructHLSLToCPPInt3(-2.cint, 3.cint, -4.cint)
  doAssert api.hlslDotInt3(api.JoltApi, int3s, int3s) == 29
  doAssert api.hlslSub(api.JoltApi, int3s).x == 2
  doAssert api.hlslMulRight(api.JoltApi, ints, 2.cint).w == 8
  var assignedInts = ints
  discard assignedInts.mulAssign(2.cint)
  discard assignedInts.divAssign(api.constructHLSLToCPPInt4(2.cint))
  doAssert assignedInts == ints
  let matrix = api.constructHLSLToCPPMat44(
    api.constructHLSLToCPPFloat4(1.0'f32),
    api.constructHLSLToCPPFloat4(2.0'f32),
    api.constructHLSLToCPPFloat4(3.0'f32),
    api.constructHLSLToCPPFloat4(4.0'f32),
  )
  doAssert matrix[api.uint(2)].x == 3.0'f32

  var atomicValue = 10.cint
  doAssert api.hlslAtomicAdd(api.JoltApi, atomicValue, 5.cint) == 15
  doAssert atomicValue == 15
  let bitPattern = api.asint(
    api.JoltApi,
    api.constructHLSLToCPPFloat4(1.0'f32, 0.0'f32, -1.0'f32, 2.0'f32),
  )
  doAssert bitPattern.x != 0

proc testNamespaceUtilities() =
  let half = api.fromFloatNearest(api.JoltApi, 1.5'f32)
  doAssert half == api.fromFloatFallbackNearest(api.JoltApi, 1.5'f32)
  discard api.fromFloatNegInf(api.JoltApi, 1.1'f32)
  discard api.fromFloatPosInf(api.JoltApi, 1.1'f32)
  discard api.fromFloatFallbackNegInf(api.JoltApi, 1.1'f32)
  discard api.fromFloatFallbackPosInf(api.JoltApi, 1.1'f32)
  let unpacked = api.ToFloat(api.JoltApi, api.sReplicate(api.UVec4, uint32(half)))
  doAssert closeEnough(unpacked.GetX(), 1.5'f32)
  doAssert closeEnough(cfloat(api.realLiteral(api.JoltApi, clongdouble(2.5))), 2.5'f32)

  let scale = api.constructVec3(2.0'f32, 2.0'f32, 2.0'f32)
  doAssert api.IsUniformScale(api.JoltApi, scale)
  doAssert api.MakeUniformScale(api.JoltApi, scale).GetX() == 2.0'f32

  let a = api.constructVec3(1.0'f32, 0.0'f32, 0.0'f32)
  let b = api.constructVec3(0.0'f32, 1.0'f32, 0.0'f32)
  let c = api.constructVec3(0.0'f32, 0.0'f32, 1.0'f32)
  var feature = 0'u32
  let closest = api.GetClosestPointOnTriangle(api.JoltApi, a, b, c, feature)
  doAssert closeEnough(closest.GetX(), 1.0'f32 / 3.0'f32)
  doAssert closeEnough(closest.GetY(), 1.0'f32 / 3.0'f32)
  doAssert closeEnough(closest.GetZ(), 1.0'f32 / 3.0'f32)
  var tetraFeature = 0'u32
  discard api.GetClosestPointOnTetrahedron(
    api.JoltApi,
    a,
    b,
    c,
    api.constructVec3(-1.0'f32, -1.0'f32, -1.0'f32),
    tetraFeature,
  )
  var u, v, w: cfloat
  doAssert api.GetBaryCentricCoordinates(api.JoltApi, a, b, c, u, v, w)
  discard api.OriginOutsideOfTetrahedronPlanes(
    api.JoltApi,
    a,
    b,
    c,
    api.constructVec3(-1.0'f32, -1.0'f32, -1.0'f32),
  )
  discard api.IsEdgeActive(api.JoltApi, a, b, c, 0.5'f32)
  discard api.FixNormal(api.JoltApi, a, b, c, a, 7'u8, b, c, a)

proc testHairShaderRegistration() =
  api.RegisterDefaultAllocator(api.JoltApi)
  api.joltFactoryInstance = api.newJoltFactory()
  api.RegisterTypes(api.JoltApi)

  let systemResult = api.CreateComputeSystemCPU(api.JoltApi)
  doAssert systemResult.IsValid()
  let systemBase = systemResult.Get()[].GetPtr()
  let system = api.asComputeSystemCPU(systemBase)
  doAssert system != nil
  api.HairRegisterShaders(api.JoltApi, system)

  block:
    var storage = api.constructStdStringStream()
    var writer = api.constructStreamOutWrapper(storage)
    let writerBase = api.asStreamOut(addr writer)
    api.SaveObjectReference(
      api.JoltApi,
      writerBase[],
      cast[ptr api.PhysicsMaterial](nil),
      cast[ptr api.ObjectToIDMap[api.PhysicsMaterial]](nil),
    )
    storage.rewindToStart()
    var reader = api.constructStreamInWrapper(storage)
    let readerBase = api.asStreamIn(addr reader)
    var restoredIDs = api.constructIDToObjectMap[api.PhysicsMaterial]()
    let restored = api.RestoreObjectReference(
      api.JoltApi,
      readerBase[],
      restoredIDs,
    )
    doAssert restored.IsValid()
    doAssert restored.Get()[].GetPtr() == nil

  block:
    var storage = api.constructStdStringStream()
    var writer = api.constructStreamOutWrapper(storage)
    let writerBase = api.asStreamOut(addr writer)
    let emptyMaterials = api.constructPhysicsMaterialList()
    var savedIDs = api.constructObjectToIDMap[api.PhysicsMaterial]()
    api.SaveObjectArray(api.JoltApi, writerBase[], emptyMaterials, addr savedIDs)
    storage.rewindToStart()
    var reader = api.constructStreamInWrapper(storage)
    let readerBase = api.asStreamIn(addr reader)
    var restoredIDs = api.constructIDToObjectMap[api.PhysicsMaterial]()
    let restored = api.RestoreObjectArray(
      api.JoltApi,
      readerBase[],
      restoredIDs,
      api.PhysicsMaterialList,
    )
    doAssert restored.IsValid()
    doAssert restored.Get()[].size() == 0

  block:
    var storage = api.constructStdStringStream()
    var reader = api.constructStreamInWrapper(storage)
    let readerBase = api.asStreamIn(addr reader)
    let member = api.getRestoreBinaryStateMember(api.HairSettings)
    let restored = api.RestoreObject(api.JoltApi, readerBase[], member)
    doAssert restored.HasError()

  api.UnregisterTypes(api.JoltApi)
  api.deleteJoltFactory(api.joltFactoryInstance)
  api.joltFactoryInstance = nil

when isMainModule:
  testHlslToCpp()
  testNamespaceUtilities()
  testHairShaderRegistration()
