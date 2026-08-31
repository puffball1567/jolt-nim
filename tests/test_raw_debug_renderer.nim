import jolt/raw as api

type CallbackState = object
  lines: int
  triangles: int
  texts: int
  lastTextLength: int

proc onLine(userData: pointer; fromPosition, toPosition: ptr api.RVec3;
            color: ptr api.Color) {.cdecl.} =
  let state = cast[ptr CallbackState](userData)
  doAssert fromPosition != nil
  doAssert toPosition != nil
  doAssert color != nil
  inc state[].lines

proc onTriangle(userData: pointer; v1, v2, v3: ptr api.RVec3;
                color: ptr api.Color;
                castShadow: api.DebugRenderer_ECastShadow) {.cdecl.} =
  let state = cast[ptr CallbackState](userData)
  doAssert v1 != nil
  doAssert v2 != nil
  doAssert v3 != nil
  doAssert color != nil
  doAssert castShadow in {
    api.DebugRenderer_ECastShadow.On,
    api.DebugRenderer_ECastShadow.Off,
  }
  inc state[].triangles

proc onText(userData: pointer; position: ptr api.RVec3; text: cstring;
            textLength: csize_t; color: ptr api.Color;
            height: cfloat) {.cdecl.} =
  let state = cast[ptr CallbackState](userData)
  doAssert position != nil
  doAssert text != nil
  doAssert color != nil
  doAssert height > 0.0
  inc state[].texts
  state[].lastTextLength = textLength.int

proc recordFrame() =
  var buffer = api.constructStdStringStream()
  var stream = api.constructStreamOutWrapper(buffer)
  let streamBase = api.asStreamOut(addr stream)
  doAssert streamBase != nil
  var recorder = api.constructDebugRendererRecorder(streamBase[])
  let red = api.constructColor(255'u8, 0'u8, 0'u8)
  recorder.DrawLine(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    red,
  )
  recorder.DrawTriangle(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
    red,
    api.DebugRenderer_ECastShadow.Off,
  )
  recorder.DrawText3D(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructStdStringView("recorded"),
    red,
    0.5,
  )
  recorder.EndFrame()
  doAssert not stream.IsFailed()

proc main() =
  api.RegisterDefaultAllocator(api.JoltApi)

  recordFrame()

  var state: CallbackState
  let adapter = api.newDebugRendererSimpleAdapter(
    onLine,
    onTriangle,
    onText,
    addr state,
  )
  doAssert adapter != nil
  let renderer = api.asDebugRenderer(adapter)
  let simple = api.asDebugRendererSimple(adapter)
  doAssert renderer != nil
  doAssert simple != nil
  simple[].SetCameraPos(api.constructVec3(0.0, 0.0, -5.0))

  let white = api.constructColor(255'u8, 255'u8, 255'u8)
  renderer[].DrawLine(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    white,
  )
  renderer[].DrawTriangle(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
    white,
  )
  renderer[].DrawText3D(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructStdStringView("debug"),
    white,
  )
  doAssert state.lines == 1
  doAssert state.triangles == 1
  doAssert state.texts == 1
  doAssert state.lastTextLength == 5

  let bounds = api.constructAABox(
    api.constructVec3(-1.0, -1.0, -1.0),
    api.constructVec3(1.0, 1.0, 1.0),
  )
  renderer[].DrawWireBox(bounds, white)
  doAssert state.lines == 13
  renderer[].DrawBox(bounds, white)
  doAssert state.triangles > 1

  var triangle = api.constructDebugRenderer_Triangle(
    api.constructVec3(0.0, 0.0, 0.0),
    api.constructVec3(1.0, 0.0, 0.0),
    api.constructVec3(0.0, 1.0, 0.0),
    white,
  )
  let calculatedBounds = api.sCalculateBounds(
    api.DebugRenderer,
    addr triangle.mV[0],
    3,
  )
  doAssert calculatedBounds.mMin == api.constructVec3(0.0, 0.0, 0.0)
  doAssert calculatedBounds.mMax == api.constructVec3(1.0, 1.0, 0.0)
  let batch = renderer[].CreateTriangleBatch(addr triangle, 1)
  doAssert batch.GetPtr() != nil
  var geometry = api.constructDebugRenderer_Geometry(batch, calculatedBounds)
  doAssert geometry.GetLOD(
    api.constructVec3(0.0, 0.0, -1.0),
    calculatedBounds,
    1.0,
  ) != nil

  var emptyInput = api.constructStdStringStream()
  var streamIn = api.constructStreamInWrapper(emptyInput)
  let streamInBase = api.asStreamIn(addr streamIn)
  var playback = api.constructDebugRendererPlayback(renderer[])
  playback.Parse(streamInBase[])
  doAssert playback.GetNumFrames() == 0

  renderer[].NextFrame()
  api.delete(adapter)

  let filter = api.newBodyDrawFilterAdapter(nil)
  doAssert filter != nil
  doAssert api.asBodyDrawFilter(filter) != nil
  api.delete(filter)

when isMainModule:
  main()
