import jolt/raw

proc compileDX12Surface() =
  var device: ptr ID3D12Device
  var commandList: ptr ID3D12GraphicsCommandList
  var rtti: ptr RTTI
  var systemResult = constructComputeSystemResult()
  var bufferResult = constructComputeBufferResult()
  var queueResult = constructComputeQueueResult()

  var system: ptr ComputeSystemDX12
  discard system[].GetRTTI()
  discard system[].CastTo(rtti)
  ComputeSystemDX12.sCreateRTTI(rtti[])
  discard system[].Initialize(device, systemResult)
  system[].Shutdown()
  discard system[].CreateComputeShader("smoke", 1, 1, 1)
  discard system[].CreateComputeBuffer(EType.Buffer, 16, 4, nil)
  discard system[].CreateComputeQueue()
  discard system[].GetDevice()
  discard system[].CreateD3DResource(
    D3D12_HEAP_TYPE.D3D12_HEAP_TYPE_DEFAULT,
    D3D12_RESOURCE_STATES.D3D12_RESOURCE_STATE_COPY_DEST,
    D3D12_RESOURCE_FLAGS.D3D12_RESOURCE_FLAG_NONE,
    16)

  var implementation: ptr ComputeSystemDX12Impl
  discard implementation[].GetRTTI()
  discard implementation[].CastTo(rtti)
  ComputeSystemDX12Impl.sCreateRTTI(rtti[])
  discard implementation[].Initialize(systemResult)
  discard implementation[].GetDXGIFactory()

  var buffer = constructComputeBufferDX12(system, EType.Buffer, 16, 4)
  discard buffer.Initialize(nil)
  discard buffer.GetResourceCPU()
  discard buffer.GetResourceGPU()
  discard buffer.ReleaseResourceCPU()
  discard buffer.Barrier(commandList,
    D3D12_RESOURCE_STATES.D3D12_RESOURCE_STATE_COPY_DEST)
  buffer.RWBarrier(commandList)
  discard buffer.SyncCPUToGPU(commandList)
  discard buffer.CreateReadBackBuffer()

  var queue: ComputeQueueDX12
  discard queue.Initialize(device,
    D3D12_COMMAND_LIST_TYPE.D3D12_COMMAND_LIST_TYPE_COMPUTE, queueResult)
  discard queue.Start()
  queue.SetShader(nil)
  queue.SetConstantBuffer("constants", nil)
  queue.SetBuffer("input", nil)
  queue.SetRWBuffer("output", nil)
  queue.ScheduleReadback(nil, nil)
  queue.Dispatch(1, 1, 1)
  queue.Execute()
  queue.Wait()

  var rootSignature: D3D12RootSignatureRef
  var pipelineState: D3D12PipelineStateRef
  var bindingNames: ComputeShaderDX12_BindingNames
  var nameToIndex: ComputeShaderDX12_NameToIndex
  var shader = constructComputeShaderDX12(rootSignature, pipelineState,
    bindingNames, nameToIndex, 1, 1, 1)
  discard shader.NameToIndex("input")
  discard shader.GetPipelineState()
  discard shader.GetRootSignature()

  discard JoltApi.HRFailed(0)
  discard JoltApi.HRFailed(0, bufferResult)

compileDX12Surface()
