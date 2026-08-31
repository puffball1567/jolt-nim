import jolt/raw

proc compileMetalSurface() =
  var device: MTLDeviceRef
  var reflection: ptr MTLComputePipelineReflection
  var pipelineState: MTLComputePipelineStateRef
  var rtti: ptr RTTI
  var system: ptr ComputeSystemMTL

  discard system[].GetRTTI()
  discard system[].CastTo(rtti)
  ComputeSystemMTL.sCreateRTTI(rtti[])
  discard system[].Initialize(device)
  system[].Shutdown()
  discard system[].CreateComputeShader("smoke", 1, 1, 1)
  discard system[].CreateComputeBuffer(EType.Buffer, 16, 4, nil)
  discard system[].CreateComputeQueue()
  discard system[].GetDevice()

  var implementation: ptr ComputeSystemMTLImpl
  discard implementation[].GetRTTI()
  discard implementation[].CastTo(rtti)
  ComputeSystemMTLImpl.sCreateRTTI(rtti[])
  discard implementation[].Initialize()

  var buffer = constructComputeBufferMTL(system, EType.Buffer, 16, 4)
  discard buffer.Initialize(nil)
  discard buffer.CreateReadBackBuffer()
  discard buffer.GetBuffer()

  var shader = constructComputeShaderMTL(
    pipelineState, reflection, 1, 1, 1)
  discard shader.GetPipelineState()
  discard shader.NameToBindingIndex("input")

  var queue = constructComputeQueueMTL(device)
  queue.SetShader(nil)
  queue.SetConstantBuffer("constants", nil)
  queue.SetBuffer("input", nil)
  queue.SetRWBuffer("output", nil)
  queue.ScheduleReadback(nil, nil)
  queue.Dispatch(1, 1, 1)
  queue.Execute()
  queue.Wait()

compileMetalSurface()
