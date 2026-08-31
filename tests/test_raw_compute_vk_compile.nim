import jolt/raw

proc compileVulkanSurface() =
  var instance: VkInstance
  var physicalDevice: VkPhysicalDevice
  var device: VkDevice
  var getInstanceProcAddr: PFN_vkGetInstanceProcAddr
  var getDeviceProcAddr: PFN_vkGetDeviceProcAddr
  var commandBuffer: VkCommandBuffer
  var bufferHandle: VkBuffer
  var systemResult = constructComputeSystemResult()
  var bufferResult = constructComputeBufferResult()
  var shaderResult = constructComputeShaderResult()
  var queueResult = constructComputeQueueResult()
  var nativeBuffer: BufferVK
  var rtti: ptr RTTI

  var system: ptr ComputeSystemVK
  discard system[].GetRTTI()
  discard system[].CastTo(rtti)
  ComputeSystemVK.sCreateRTTI(rtti[])
  discard system[].Initialize(
    physicalDevice, getDeviceProcAddr, device, 0, systemResult)
  system[].Shutdown()
  discard system[].CreateComputeShader("smoke", 1, 1, 1)
  discard system[].CreateComputeBuffer(EType.Buffer, 16, 4, nil)
  discard system[].CreateComputeQueue()
  discard system[].GetDevice()
  discard system[].CreateBuffer(16, 0, 0, nativeBuffer)
  system[].FreeBuffer(nativeBuffer)
  discard system[].MapBuffer(nativeBuffer)
  system[].UnmapBuffer(nativeBuffer)

  var allocator: ptr ComputeSystemVKWithAllocator
  discard allocator[].GetRTTI()
  discard allocator[].CastTo(rtti)
  ComputeSystemVKWithAllocator.sCreateRTTI(rtti[])
  discard allocator[].Initialize(instance, physicalDevice, getInstanceProcAddr,
    getDeviceProcAddr, device, 0, systemResult)
  discard allocator[].CreateBuffer(16, 0, 0, nativeBuffer)
  allocator[].FreeBuffer(nativeBuffer)
  discard allocator[].MapBuffer(nativeBuffer)
  allocator[].UnmapBuffer(nativeBuffer)

  var implementation: ptr ComputeSystemVKImpl
  discard implementation[].GetRTTI()
  discard implementation[].CastTo(rtti)
  ComputeSystemVKImpl.sCreateRTTI(rtti[])
  discard implementation[].Initialize(systemResult)

  var buffer = constructComputeBufferVK(system, EType.Buffer, 16, 4)
  discard buffer.Initialize(nil)
  discard buffer.CreateReadBackBuffer()
  discard buffer.GetBufferCPU()
  discard buffer.GetBufferGPU()
  discard buffer.ReleaseBufferCPU()
  buffer.Barrier(commandBuffer, 0, VkAccessFlagBits.VK_ACCESS_NONE, false)
  discard buffer.SyncCPUToGPU(commandBuffer)

  var shader = constructComputeShaderVK(system, 1, 1, 1)
  var spirv: ComputeByteArray
  discard shader.Initialize(spirv, bufferHandle, shaderResult)
  discard shader.NameToBufferInfoIndex("input")
  discard shader.GetPipeline()
  discard shader.GetPipelineLayout()
  discard shader.GetDescriptorSetLayout()
  discard shader.GetLayoutBindings()
  discard shader.GetBufferInfos()

  var queue = constructComputeQueueVK(system)
  discard queue.Initialize(0, queueResult)
  queue.SetShader(nil)
  queue.SetConstantBuffer("constants", nil)
  queue.SetBuffer("input", nil)
  queue.SetRWBuffer("output", nil)
  queue.ScheduleReadback(nil, nil)
  queue.Dispatch(1, 1, 1)
  queue.Execute()
  queue.Wait()

  discard JoltApi.VKFailed(VkResult.VK_SUCCESS)
  discard JoltApi.VKFailed(VkResult.VK_SUCCESS, bufferResult)

compileVulkanSurface()
