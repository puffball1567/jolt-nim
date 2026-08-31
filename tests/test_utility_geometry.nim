import std/math
import jolt/raw as api

proc closeEnough(actual, expected: cfloat): bool =
  abs(actual - expected) < 1.0e-5

proc main() =
  api.RegisterDefaultAllocator(api.JoltApi)
  let configuration = api.GetConfigurationString(api.JoltApi)
  doAssert configuration != nil
  doAssert ($configuration).len > 0

  let color = api.constructColor(10'u8, 20'u8, 30'u8, 40'u8)
  doAssert api.apply(color, 0) == 10
  doAssert api.apply(color, 3) == 40
  var mutableColor = color
  api.apply(mutableColor, 1)[] = 25
  let readColor = mutableColor
  doAssert api.apply(readColor, 1) == 25

  let identity = api.sIdentity(api.Mat44)
  doAssert closeEnough(api.apply(identity, 0, 0), 1.0)
  var mutableIdentity = identity
  api.apply(mutableIdentity, 0, 1)[] = 2.0
  let readIdentity = mutableIdentity
  doAssert closeEnough(api.apply(readIdentity, 0, 1), 2.0)

  var emptyJob = api.constructJobSystem_JobHandle()
  var movedJob = api.constructJobSystem_JobHandleMove(emptyJob)
  doAssert not emptyJob.IsValid()
  doAssert not movedJob.IsValid()

  var matrix = api.constructDynMatrix(2, 3)
  doAssert matrix.GetRows() == 2
  doAssert matrix.GetCols() == 3
  api.apply(matrix, 1, 2)[] = 4.5
  doAssert closeEnough(api.apply(matrix, 1, 2)[], 4.5)

  doAssert api.sExpandBits(api.MortonCode, 0.0) == 0
  doAssert api.sExpandBits(api.MortonCode, 1.0) != 0
  var bounds = api.constructAABox(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 1.0, 1.0),
  )
  doAssert api.sGetMortonCode(
    api.MortonCode,
    api.constructVec3(0.0, 0.0, 0.0),
    bounds,
  ) == 0

  let origin = api.constructVec3(0.0, 0.0, -3.0)
  let direction = api.constructVec3(0.0, 0.0, 1.0)
  doAssert closeEnough(
    api.RaySphere(api.JoltApi, origin, direction,
                  api.constructVec3(0.0, 0.0, 0.0), 1.0),
    2.0,
  )
  doAssert api.RayTriangle(
    api.JoltApi,
    origin,
    direction,
    api.constructVec3(-1.0, -1.0, 0.0),
    api.constructVec3(1.0, -1.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
  ) > 0.0

  var triangles = api.constructTriangleList()
  triangles.push_back(api.constructTriangle(
    api.constructVec3(-1.0, -1.0, 0.0),
    api.constructVec3(1.0, -1.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
  ))
  var vertices = api.constructVertexList()
  var indexedTriangles = api.constructIndexedTriangleList()
  api.Indexify(api.JoltApi, triangles, vertices, indexedTriangles, 0.0)
  doAssert vertices.size() == 3
  doAssert indexedTriangles.size() == 1

  var restored = api.constructTriangleList()
  api.Deindexify(api.JoltApi, vertices, indexedTriangles, restored)
  doAssert restored.size() == 1

  var polygon = api.constructArrayFromInitializerList(
    api.constructVec3(-2.0, -2.0, 0.0),
    api.constructVec3(2.0, -2.0, 0.0),
    api.constructVec3(2.0, 2.0, 0.0),
    api.constructVec3(-2.0, 2.0, 0.0),
  )
  var clippedToPlane = api.constructArray[api.Vec3]()
  api.ClipPolyVsPlane(
    api.JoltApi,
    polygon,
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    clippedToPlane,
  )
  doAssert clippedToPlane.size() == 4

  var clippingPolygon = api.constructArrayFromInitializerList(
    api.constructVec3(-1.0, -1.0, 0.0),
    api.constructVec3(1.0, -1.0, 0.0),
    api.constructVec3(1.0, 1.0, 0.0),
    api.constructVec3(-1.0, 1.0, 0.0),
  )
  var clippedToPolygon = api.constructArray[api.Vec3]()
  api.ClipPolyVsPoly(
    api.JoltApi,
    polygon,
    clippingPolygon,
    api.constructVec3(0.0, 0.0, 1.0),
    clippedToPolygon,
  )
  doAssert clippedToPolygon.size() == 4

  var clippedToEdge = api.constructArray[api.Vec3]()
  api.ClipPolyVsEdge(
    api.JoltApi,
    polygon,
    api.constructVec3(-1.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
    clippedToEdge,
  )
  doAssert clippedToEdge.size() <= 2

  var clipBounds = api.constructAABox(
    api.constructVec3(-1.0, -1.0, -1.0),
    api.constructVec3(1.0, 1.0, 1.0),
  )
  var clippedToBounds = api.constructArray[api.Vec3]()
  api.ClipPolyVsAABox(api.JoltApi, polygon, clipBounds, clippedToBounds)
  doAssert clippedToBounds.size() == 4

  var eigenMatrix = api.constructMatrix[2, 2]()
  eigenMatrix.SetZero()
  var eigenVectors = api.constructMatrix[2, 2]()
  eigenVectors.SetIdentity()
  var eigenValues = api.constructVector[2]()
  eigenValues.SetZero()
  doAssert api.EigenValueSymmetric(
    api.JoltApi,
    eigenMatrix,
    eigenVectors,
    eigenValues,
  )
  doAssert eigenVectors.IsIdentity()
  doAssert eigenValues.IsZero()

  var root1, root2: cfloat
  doAssert api.FindRoot(api.JoltApi, 1.0.cfloat, -3.0.cfloat, 2.0.cfloat,
                        root1, root2) == 2
  doAssert (closeEnough(root1, 1.0) and closeEnough(root2, 2.0)) or
    (closeEnough(root1, 2.0) and closeEnough(root2, 1.0))

  var splitter = api.constructTriangleSplitterMean(vertices, indexedTriangles)
  var splitterStats: api.TriangleSplitter_Stats
  splitter.GetStats(splitterStats)
  doAssert splitterStats.mSplitterName != nil
  var initialRange = splitter.GetInitialRange()
  doAssert initialRange.Count() == 1

  var treeBuilder = api.constructAABBTreeBuilder(splitter, 1)
  var treeStats: api.AABBTreeBuilderStats
  let root = treeBuilder.Build(treeStats)
  doAssert root != nil
  let treeNodes = treeBuilder.GetNodes()
  doAssert treeNodes != nil
  doAssert treeNodes[].size() == 1
  doAssert treeBuilder.GetTriangles()[].size() == 1
  doAssert root[].GetTriangleCount() == 1
  doAssert not root[].HasChildren()
  doAssert root[].GetChild(0, treeNodes[]) == nil
  doAssert root[].GetMinDepth(treeNodes[]) == 1
  doAssert root[].GetMaxDepth(treeNodes[]) == 1
  doAssert root[].GetNodeCount(treeNodes[]) == 1
  doAssert root[].GetLeafNodeCount(treeNodes[]) == 1
  doAssert root[].GetTriangleCountInTree(treeNodes[]) == 1
  var averageTriangles: cfloat
  var minTriangles, maxTriangles: api.uint
  root[].GetTriangleCountPerNode(
    treeNodes[],
    averageTriangles,
    minTriangles,
    maxTriangles,
  )
  doAssert closeEnough(averageTriangles, 1.0)
  doAssert minTriangles == 1
  doAssert maxTriangles == 1
  doAssert api.CalculateSAHCost(root[], treeNodes[], 1.0, 1.0) > 0.0
  var children = api.constructArray[api.AABBTreeBuilder_ConstNodePtr]()
  api.GetNChildren(root[], treeNodes[], 4, children)
  doAssert children.size() == 0

  var codecError: api.ConvexHullBuilder_ErrorPtr
  var treeBuffer = api.constructAABBTreeToBufferDefault()
  doAssert treeBuffer.Convert(
    treeBuilder.GetTriangles()[],
    treeNodes[],
    vertices,
    root,
    true,
    codecError,
  )
  doAssert treeBuffer.GetBuffer()[].size() > 0
  let nodeHeader = treeBuffer.GetNodeHeader()
  let triangleHeader = treeBuffer.GetTriangleHeader()
  let encodedRoot = treeBuffer.GetRoot()
  doAssert nodeHeader != nil
  doAssert triangleHeader != nil
  doAssert encodedRoot != nil
  let encodedBuffer = treeBuffer.GetBuffer()
  let encodedBufferStart = encodedBuffer[].data()

  var validation = api.constructTriangleCodecValidationContext(
    indexedTriangles,
    vertices,
  )
  doAssert not validation.IsDegenerate(indexedTriangles[0][])
  let blockHeader = cast[
    ptr api.TriangleCodecIndexed8BitPackSOA4Flags_TriangleBlockHeader
  ](encodedRoot)
  doAssert blockHeader[].GetVertexData() != nil
  doAssert blockHeader[].GetTriangleBlock() != nil
  doAssert blockHeader[].GetUserData() != nil

  var triangleDecoder = api.constructTriangleCodecDecodingContext(
    triangleHeader,
  )
  var decodedVertices: array[3, api.Vec3]
  var decodedFlag: uint8
  triangleDecoder.Unpack(
    encodedRoot,
    1,
    addr decodedVertices[0],
    addr decodedFlag,
  )
  var decodedV1, decodedV2, decodedV3: api.Vec3
  triangleDecoder.GetTriangle(
    encodedRoot,
    0,
    decodedV1,
    decodedV2,
    decodedV3,
  )
  doAssert triangleDecoder.GetUserData(encodedRoot, 0) == 0
  doAssert decodedFlag == 0
  var flags: array[1, uint8]
  api.sGetFlags(
    api.TriangleCodecIndexed8BitPackSOA4Flags_DecodingContext,
    encodedRoot,
    1,
    addr flags[0],
  )
  doAssert api.sGetFlags(
    api.TriangleCodecIndexed8BitPackSOA4Flags_DecodingContext,
    encodedRoot,
    0,
  ) == flags[0]
  var closestTriangle: uint32
  doAssert triangleDecoder.TestRay(
    api.constructVec3(0.0, 0.0, -2.0),
    api.constructVec3(0.0, 0.0, 1.0),
    encodedRoot,
    1,
    high(cfloat),
    closestTriangle,
  ) < high(cfloat)
  doAssert closestTriangle == 0

  var nodeDecoder = api.constructNodeCodecDecodingContext(nodeHeader)
  doAssert not nodeDecoder.IsDoneWalking()
  discard api.sTriangleBlockIDBits(
    api.NodeCodecQuadTreeHalfFloat_DecodingContext,
    nodeHeader,
  )
  doAssert api.sGetTriangleBlockStart(
    api.NodeCodecQuadTreeHalfFloat_DecodingContext,
    encodedBufferStart,
    0,
  ) == cast[pointer](encodedBufferStart)

  var nodeEncoder = api.constructNodeCodecEncodingContext()
  var estimatedNodeBytes = 0'u64
  nodeEncoder.PrepareNodeAllocate(root, estimatedNodeBytes)
  doAssert estimatedNodeBytes == 0
  var nodeBuffer = api.constructByteBuffer()
  var childBoundsMin, childBoundsMax: array[4, api.Vec3]
  doAssert nodeEncoder.NodeAllocate(
    root,
    root[].mBounds.mMin,
    root[].mBounds.mMax,
    children,
    addr childBoundsMin[0],
    addr childBoundsMax[0],
    nodeBuffer,
    codecError,
  ) == 0
  doAssert nodeEncoder.NodeFinalize(
    root,
    0,
    0,
    nil,
    nil,
    nodeBuffer,
    codecError,
  )
  var standaloneHeader: api.NodeCodecQuadTreeHalfFloat_Header
  doAssert nodeEncoder.Finalize(
    addr standaloneHeader,
    root,
    high(csize_t),
    0,
    codecError,
  )

  var hull3DPositions = api.constructArrayFromInitializerList(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
    api.constructVec3(0.0, 0.0, 1.0),
  )
  var hull3D = api.constructConvexHullBuilder(hull3DPositions)
  var hullError: api.ConvexHullBuilder_ErrorPtr
  doAssert hull3D.Initialize(high(cint), 1.0e-5, hullError) ==
    api.ConvexHullBuilder_EResult.Success
  doAssert hull3D.GetNumVerticesUsed() == 4
  var candidateFace = api.constructArrayFromInitializerList(
    0.cint,
    1.cint,
    2.cint,
  )
  discard hull3D.ContainsFace(candidateFace)
  var centerOfMass: api.Vec3
  var hullVolume: cfloat
  hull3D.GetCenterOfMassAndVolume(centerOfMass, hullVolume)
  doAssert hullVolume > 0.0
  var maxErrorFace: ptr api.ConvexHullBuilder_Face
  var maxError, coplanarDistance: cfloat
  var maxErrorPosition: cint
  hull3D.DetermineMaxError(
    maxErrorFace,
    maxError,
    maxErrorPosition,
    coplanarDistance,
  )
  let hullFaces = hull3D.GetFaces()
  doAssert hullFaces != nil
  doAssert hullFaces[].size() == 4
  let firstFace = cast[ptr UncheckedArray[ptr api.ConvexHullBuilder_Face]](
    hullFaces[].data()
  )[0]
  doAssert firstFace != nil
  firstFace[].CalculateNormalAndCentroid(hull3DPositions.data())
  discard firstFace[].IsFacing(api.constructVec3(2.0, 2.0, 2.0))
  doAssert firstFace[].mFirstEdge != nil
  doAssert firstFace[].mFirstEdge[].GetPreviousEdge() != nil

  var epaPoints = api.constructEPAConvexHullBuilder_Points()
  epaPoints.push_back(api.constructVec3(1.0, 0.0, 0.0))
  epaPoints.push_back(api.constructVec3(0.0, 1.0, 0.0))
  epaPoints.push_back(api.constructVec3(0.0, 0.0, 1.0))
  epaPoints.push_back(api.constructVec3(-1.0, -1.0, -1.0))
  doAssert epaPoints.size() == 4
  doAssert epaPoints.GetSizeRef()[] == 4

  var directTriangle = api.constructEPAConvexHullBuilder_Triangle(
    0,
    1,
    2,
    epaPoints.data(),
  )

  var standaloneHullEdge = api.constructConvexHullBuilder_Edge(nil, 7)
  doAssert standaloneHullEdge.mFace == nil
  doAssert standaloneHullEdge.mStartIdx == 7
  discard directTriangle.IsFacing(api.constructVec3(2.0, 2.0, 2.0))
  discard directTriangle.IsFacingOrigin()
  doAssert directTriangle.GetNextEdge(0) != nil

  var triangleFactory = api.constructEPAConvexHullBuilder_TriangleFactory()
  let factoryTriangle = triangleFactory.CreateTriangle(
    0,
    1,
    2,
    epaPoints.data(),
  )
  doAssert factoryTriangle != nil
  triangleFactory.FreeTriangle(factoryTriangle)
  triangleFactory.Clear()

  var queueTriangle = api.constructEPAConvexHullBuilder_Triangle(
    0,
    1,
    2,
    epaPoints.data(),
  )
  var triangleQueue = api.constructEPAConvexHullBuilder_TriangleQueue()
  triangleQueue.push_back(addr queueTriangle)
  doAssert triangleQueue.PeekClosest() == addr queueTriangle
  doAssert triangleQueue.PopClosest() == addr queueTriangle
  discard api.sTriangleSorter(
    api.EPAConvexHullBuilder_TriangleQueue,
    addr directTriangle,
    addr queueTriangle,
  )

  var epaHull = api.constructEPAConvexHullBuilder(epaPoints)
  epaHull.Initialize(0, 1, 2)
  doAssert epaHull.HasNextTriangle()
  doAssert epaHull.PeekClosestTriangleInQueue() != nil
  var facingDistance: cfloat
  let facingTriangle = epaHull.FindFacingTriangle(
    api.constructVec3(-1.0, -1.0, -1.0),
    facingDistance,
  )
  if facingTriangle != nil:
    var newTriangles = api.constructEPAConvexHullBuilder_NewTriangles()
    doAssert epaHull.AddPoint(
      facingTriangle,
      3,
      high(cfloat),
      newTriangles,
    )
    doAssert newTriangles.size() > 0
  doAssert epaHull.PopClosestTriangleFromQueue() != nil

  var pointA: api.PointConvexSupport
  pointA.mPoint = api.constructVec3(0.0, 0.0, 0.0)
  var pointB: api.PointConvexSupport
  pointB.mPoint = api.constructVec3(2.0, 0.0, 0.0)
  doAssert pointA.GetSupport(api.constructVec3(1.0, 0.0, 0.0)) ==
    pointA.mPoint
  var triangleSupportA = api.constructTriangleConvexSupport(
    api.constructVec3(-1.0, -1.0, 0.0),
    api.constructVec3(1.0, -1.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
  )
  var triangleSupportB = api.constructTriangleConvexSupport(
    api.constructVec3(-0.5, -0.5, 0.0),
    api.constructVec3(0.5, -0.5, 0.0),
    api.constructVec3(0.0, 0.5, 0.0),
  )
  doAssert triangleSupportA.GetSupport(api.constructVec3(0.0, 1.0, 0.0)) ==
    api.constructVec3(0.0, 1.0, 0.0)
  var supportFace = api.constructArray[api.Vec3]()
  triangleSupportA.GetSupportingFace(
    api.constructVec3(0.0, 0.0, 1.0),
    supportFace,
  )
  doAssert supportFace.size() == 3

  var gjk = api.constructGJKClosestPoint()
  var separatingAxis = api.constructVec3(1.0, 0.0, 0.0)
  var closestA, closestB: api.Vec3
  let distanceSq = gjk.GetClosestPoints(
    pointA,
    pointB,
    1.0e-5,
    100.0,
    separatingAxis,
    closestA,
    closestB,
  )
  doAssert closeEnough(distanceSq, 4.0)
  var simplexY, simplexP, simplexQ: array[4, api.Vec3]
  var simplexCount: api.uint
  gjk.GetClosestPointsSimplex(
    addr simplexY[0],
    addr simplexP[0],
    addr simplexQ[0],
    simplexCount,
  )
  doAssert simplexCount > 0
  var pointSame = pointA
  separatingAxis = api.constructVec3(0.0, 0.0, 0.0)
  doAssert gjk.Intersects(pointA, pointSame, 1.0e-5, separatingAxis)
  var rayLambda = 1.0.cfloat
  doAssert gjk.CastRay(
    api.constructVec3(-2.0, 0.0, 0.0),
    api.constructVec3(4.0, 0.0, 0.0),
    1.0e-5,
    pointA,
    rayLambda,
  )
  doAssert closeEnough(rayLambda, 0.5)
  var castLambda = 1.0.cfloat
  doAssert gjk.CastShape(
    api.sTranslation(api.Mat44, api.constructVec3(-2.0, 0.0, 0.0)),
    api.constructVec3(4.0, 0.0, 0.0),
    1.0e-5,
    pointA,
    pointSame,
    castLambda,
  )
  castLambda = 1.0
  var castPointA, castPointB, castAxis: api.Vec3
  doAssert gjk.CastShape(
    api.sTranslation(api.Mat44, api.constructVec3(-2.0, 0.0, 0.0)),
    api.constructVec3(4.0, 0.0, 0.0),
    1.0e-5,
    pointA,
    pointSame,
    0.0,
    0.0,
    castLambda,
    castPointA,
    castPointB,
    castAxis,
  )

  var epa = api.constructEPAPenetrationDepth()
  var penetrationAxis = api.constructVec3(1.0, 0.0, 0.0)
  var penetrationPointA, penetrationPointB: api.Vec3
  doAssert epa.GetPenetrationDepthStepGJK(
    pointA,
    0.0,
    pointB,
    0.0,
    1.0e-5,
    penetrationAxis,
    penetrationPointA,
    penetrationPointB,
  ) == api.EPAPenetrationDepth_EStatus.NotColliding
  penetrationAxis = api.constructVec3(1.0, 0.0, 0.0)
  doAssert not epa.GetPenetrationDepth(
    pointA,
    pointA,
    0.0,
    pointB,
    pointB,
    0.0,
    1.0e-5,
    1.0e-4,
    penetrationAxis,
    penetrationPointA,
    penetrationPointB,
  )
  penetrationAxis = api.constructVec3(1.0, 0.0, 0.0)
  discard epa.GetPenetrationDepthStepGJK(
    triangleSupportA,
    0.0,
    triangleSupportB,
    0.0,
    1.0e-5,
    penetrationAxis,
    penetrationPointA,
    penetrationPointB,
  )
  discard epa.GetPenetrationDepthStepEPA(
    triangleSupportA,
    triangleSupportB,
    1.0e-4,
    penetrationAxis,
    penetrationPointA,
    penetrationPointB,
  )
  var epaCastLambda = 1.0.cfloat
  discard epa.CastShape(
    api.sTranslation(api.Mat44, api.constructVec3(-2.0, 0.0, 0.0)),
    api.constructVec3(4.0, 0.0, 0.0),
    1.0e-5,
    1.0e-4,
    pointA,
    pointSame,
    0.0,
    0.0,
    false,
    epaCastLambda,
    penetrationPointA,
    penetrationPointB,
    penetrationAxis,
  )

  var hullPositions: api.ConvexHullBuilder2D_Positions
  hullPositions.push_back(api.constructVec3(-1.0, -1.0, 0.0))
  hullPositions.push_back(api.constructVec3(1.0, -1.0, 0.0))
  hullPositions.push_back(api.constructVec3(0.0, 1.0, 0.0))
  var hull = api.constructConvexHullBuilder2D(hullPositions)
  var hullEdges: api.ConvexHullBuilder2D_Edges
  doAssert hull.Initialize(0, 1, 2, 16, 1.0e-5, hullEdges) ==
    api.ConvexHullBuilder2D_EResult.Success
  doAssert hullEdges.size() == 3

  var skeleton1 = api.constructSkeleton()
  var skeleton2 = api.constructSkeleton()
  let rootName = api.constructStdStringView("root")
  discard skeleton1.AddJoint(rootName, -1)
  discard skeleton2.AddJoint(rootName, -1)
  var neutralPose1 = api.sIdentity(api.Mat44)
  var neutralPose2 = api.sIdentity(api.Mat44)
  let mapper = api.newSkeletonMapper()
  doAssert mapper != nil
  mapper[].Initialize(
    addr skeleton1,
    addr neutralPose1,
    addr skeleton2,
    addr neutralPose2,
  )
  doAssert mapper[].GetMappedJointIdx(0) == 0
  doAssert mapper[].GetMappings()[].size() == 1
  doAssert mapper[].GetChains()[].size() == 0
  doAssert mapper[].GetUnmapped()[].size() == 0
  var mappedPose = api.sIdentity(api.Mat44)
  mapper[].Map(addr neutralPose1, addr neutralPose2, addr mappedPose)
  mapper[].MapReverse(addr mappedPose, addr neutralPose1)
  api.deleteSkeletonMapper(mapper)

when isMainModule:
  main()
