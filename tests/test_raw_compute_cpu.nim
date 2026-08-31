import jolt/raw as api

proc main() =
  api.RegisterDefaultAllocator(api.JoltApi)
  api.joltFactoryInstance = api.newJoltFactory()
  api.RegisterTypes(api.JoltApi)

  let systemResult = api.CreateComputeSystemCPU(api.JoltApi)
  doAssert systemResult.IsValid()
  let systemRef = systemResult.Get()
  doAssert systemRef != nil
  let systemBase = systemRef[].GetPtr()
  doAssert systemBase != nil
  let system = api.asComputeSystemCPU(systemBase)
  doAssert system != nil
  doAssert system[].GetRTTI() != nil
  doAssert system[].CastTo(system[].GetRTTI()) == cast[pointer](system)

  var input = [10'u32, 20, 30, 40]
  let bufferResult = system[].CreateComputeBuffer(
    api.EType.Buffer,
    uint64(input.len),
    api.uint(sizeof(uint32)),
    addr input[0],
  )
  doAssert bufferResult.IsValid()
  let bufferBase = bufferResult.Get()[].GetPtr()
  doAssert bufferBase != nil
  let buffer = api.asComputeBufferCPU(bufferBase)
  doAssert buffer != nil
  doAssert buffer[].GetData() != nil
  let mapped = bufferBase[].Map(api.EMode.Read, uint32)
  doAssert mapped[] == 10
  doAssert cast[ptr UncheckedArray[uint32]](mapped)[3] == 40
  bufferBase[].Unmap()
  let untypedMapped = bufferBase[].Map(api.EMode.Read)
  doAssert untypedMapped != nil
  doAssert cast[ptr uint32](untypedMapped)[] == 10
  bufferBase[].Unmap()
  doAssert buffer[].CreateReadBackBuffer().IsValid()

  let queueResult = system[].CreateComputeQueue()
  doAssert queueResult.IsValid()
  let queueBase = queueResult.Get()[].GetPtr()
  doAssert queueBase != nil
  let queue = api.asComputeQueueCPU(queueBase)
  doAssert queue != nil
  queue[].SetConstantBuffer("unused", nil)
  queue[].SetBuffer("unused", nil)
  queue[].SetRWBuffer("unused", nil)
  queue[].ScheduleReadback(bufferBase, bufferBase)
  queue[].Execute()
  queue[].Wait()

  var noShader: api.ComputeShaderCPU_CreateShader = nil
  system[].RegisterShader("nim-null-shader", noShader)
  let shaderResult = system[].CreateComputeShader("nim-null-shader", 1, 1, 1)
  doAssert shaderResult.IsValid()
  let shaderBase = shaderResult.Get()[].GetPtr()
  doAssert shaderBase != nil
  doAssert api.asComputeShaderCPU(shaderBase) != nil

  api.UnregisterTypes(api.JoltApi)
  api.deleteJoltFactory(api.joltFactoryInstance)
  api.joltFactoryInstance = nil

when isMainModule:
  main()
