import jolt/raw as api

proc writeTokens[T](writer: var T) =
  var value8 = 7'u8
  var value16 = 1_023'u16
  var valueInt = -42.cint
  var value32 = 0x1234_5678'u32
  var value64 = 0x1234_5678_9abc_def0'u64
  var valueFloat = 1.25.cfloat
  var valueDouble = 2.5.cdouble
  var valueBool = true
  var valueString: api.String
  api.setJoltString(addr valueString, "raw-object-stream")
  var valueFloat3 = api.constructFloat3(1.0, 2.0, 3.0)
  var valueFloat4 = api.constructFloat4(4.0, 5.0, 6.0, 7.0)
  var valueDouble3 = api.constructDouble3(8.0, 9.0, 10.0)
  var valueVec3 = api.constructVec3(11.0, 12.0, 13.0)
  var valueDVec3 = api.constructDVec3(14.0, 15.0, 16.0)
  var valueVec4 = api.constructVec4(17.0, 18.0, 19.0, 20.0)
  var valueUVec4 = api.constructUVec4(21, 22, 23, 24)
  var valueQuat = api.constructQuat(0.0, 0.0, 0.0, 1.0)
  var valueMat44 = api.sIdentity(api.Mat44)
  var valueDMat44 = api.sIdentity(api.DMat44)

  template nextToken() = writer.HintNextItem()

  nextToken()
  writer.WriteDataType(api.EOSDataType.T_uint32)
  nextToken()
  writer.WriteName("tokens")
  nextToken()
  writer.WriteIdentifier(77)
  nextToken()
  writer.WriteCount(19)
  nextToken()
  writer.WritePrimitiveData(value8)
  nextToken()
  writer.WritePrimitiveData(value16)
  nextToken()
  writer.WritePrimitiveData(valueInt)
  nextToken()
  writer.WritePrimitiveData(value32)
  nextToken()
  writer.WritePrimitiveData(value64)
  nextToken()
  writer.WritePrimitiveData(valueFloat)
  nextToken()
  writer.WritePrimitiveData(valueDouble)
  nextToken()
  writer.WritePrimitiveData(valueBool)
  nextToken()
  writer.WritePrimitiveData(valueString)
  nextToken()
  writer.WritePrimitiveData(valueFloat3)
  nextToken()
  writer.WritePrimitiveData(valueFloat4)
  nextToken()
  writer.WritePrimitiveData(valueDouble3)
  nextToken()
  writer.WritePrimitiveData(valueVec3)
  nextToken()
  writer.WritePrimitiveData(valueDVec3)
  nextToken()
  writer.WritePrimitiveData(valueVec4)
  nextToken()
  writer.WritePrimitiveData(valueUVec4)
  nextToken()
  writer.WritePrimitiveData(valueQuat)
  nextToken()
  writer.WritePrimitiveData(valueMat44)
  nextToken()
  writer.WritePrimitiveData(valueDMat44)

proc readTokens[T](reader: var T) =
  var dataType: api.EOSDataType
  var name: api.String
  var identifier: api.ObjectStream_Identifier
  var count: uint32
  doAssert reader.ReadDataType(dataType)
  doAssert dataType == api.EOSDataType.T_uint32
  doAssert reader.ReadName(name)
  doAssert $name.joltCppStringCStr_String() == "tokens"
  doAssert reader.ReadIdentifier(identifier)
  doAssert identifier == 77
  doAssert reader.ReadCount(count)
  doAssert count == 19

  var value8: uint8
  var value16: uint16
  var valueInt: cint
  var value32: uint32
  var value64: uint64
  var valueFloat: cfloat
  var valueDouble: cdouble
  var valueBool: bool
  var valueString: api.String
  var valueFloat3: api.Float3
  var valueFloat4: api.Float4
  var valueDouble3: api.Double3
  var valueVec3: api.Vec3
  var valueDVec3: api.DVec3
  var valueVec4: api.Vec4
  var valueUVec4: api.UVec4
  var valueQuat: api.Quat
  var valueMat44: api.Mat44
  var valueDMat44: api.DMat44

  doAssert reader.ReadPrimitiveData(value8)
  doAssert reader.ReadPrimitiveData(value16)
  doAssert reader.ReadPrimitiveData(valueInt)
  doAssert reader.ReadPrimitiveData(value32)
  doAssert reader.ReadPrimitiveData(value64)
  doAssert reader.ReadPrimitiveData(valueFloat)
  doAssert reader.ReadPrimitiveData(valueDouble)
  doAssert reader.ReadPrimitiveData(valueBool)
  doAssert reader.ReadPrimitiveData(valueString)
  doAssert reader.ReadPrimitiveData(valueFloat3)
  doAssert reader.ReadPrimitiveData(valueFloat4)
  doAssert reader.ReadPrimitiveData(valueDouble3)
  doAssert reader.ReadPrimitiveData(valueVec3)
  doAssert reader.ReadPrimitiveData(valueDVec3)
  doAssert reader.ReadPrimitiveData(valueVec4)
  doAssert reader.ReadPrimitiveData(valueUVec4)
  doAssert reader.ReadPrimitiveData(valueQuat)
  doAssert reader.ReadPrimitiveData(valueMat44)
  doAssert reader.ReadPrimitiveData(valueDMat44)

  doAssert value8 == 7
  doAssert value16 == 1_023
  doAssert valueInt == -42
  doAssert value32 == 0x1234_5678'u32
  doAssert value64 == 0x1234_5678_9abc_def0'u64
  doAssert valueFloat == 1.25
  doAssert valueDouble == 2.5
  doAssert valueBool
  doAssert $valueString.joltCppStringCStr_String() == "raw-object-stream"
  doAssert valueFloat3 == api.constructFloat3(1.0, 2.0, 3.0)
  doAssert valueFloat4 == api.constructFloat4(4.0, 5.0, 6.0, 7.0)
  doAssert valueDouble3 == api.constructDouble3(8.0, 9.0, 10.0)
  doAssert valueVec3 == api.constructVec3(11.0, 12.0, 13.0)
  doAssert valueDVec3 == api.constructDVec3(14.0, 15.0, 16.0)
  doAssert valueVec4 == api.constructVec4(17.0, 18.0, 19.0, 20.0)
  doAssert valueUVec4 == api.constructUVec4(21, 22, 23, 24)
  doAssert valueQuat == api.constructQuat(0.0, 0.0, 0.0, 1.0)
  doAssert valueMat44 == api.sIdentity(api.Mat44)
  doAssert valueDMat44 == api.sIdentity(api.DMat44)

proc main() =
  api.RegisterDefaultAllocator(api.JoltApi)
  api.joltFactoryInstance = api.newJoltFactory()
  api.RegisterTypes(api.JoltApi)

  let boxRTTI = api.joltFactoryInstance[].Find("BoxShapeSettings")
  doAssert boxRTTI != nil
  var halfExtentAttribute: ptr api.SerializableAttribute
  for index in 0 ..< boxRTTI[].GetAttributeCount():
    let attribute = boxRTTI[].GetAttribute(index)
    if $attribute[].GetName() == "mHalfExtent":
      halfExtentAttribute = attribute
      break
  doAssert halfExtentAttribute != nil
  doAssert halfExtentAttribute[].GetMemberPrimitiveType() != nil
  doAssert halfExtentAttribute[].IsType(0, api.EOSDataType.T_Vec3, "")
  var boxSettings = api.constructBoxShapeSettings()
  let halfExtent = api.GetMemberPointer[api.Vec3](
    halfExtentAttribute[],
    addr boxSettings,
    api.Vec3,
  )
  doAssert halfExtent != nil
  halfExtent[] = api.constructVec3(1.0, 2.0, 3.0)
  doAssert halfExtent[] == api.constructVec3(1.0, 2.0, 3.0)
  var copiedAttribute = api.constructSerializableAttribute(
    halfExtentAttribute[],
    0,
  )
  copiedAttribute.SetName("copiedHalfExtent")
  doAssert $copiedAttribute.GetName() == "copiedHalfExtent"

  var primitiveValue = 0x89ab_cdef'u32
  doAssert api.GetPrimitiveTypeOfType(api.JoltApi, addr primitiveValue) != nil
  doAssert api.OSIsType(
    api.JoltApi,
    addr primitiveValue,
    0,
    api.EOSDataType.T_uint32,
    "",
  )
  let attributeCount = boxRTTI[].GetAttributeCount()
  api.AddSerializableAttributeTyped(
    api.JoltApi,
    boxRTTI[],
    0,
    "nimRawTyped",
    api.Vec3,
  )
  api.AddSerializableAttributeEnum(
    api.JoltApi,
    boxRTTI[],
    0,
    "nimRawEnum",
    uint32,
  )
  doAssert boxRTTI[].GetAttributeCount() == attributeCount + 2
  doAssert boxRTTI[].GetAttribute(attributeCount)[].IsType(
    0,
    api.EOSDataType.T_Vec3,
    "",
  )
  doAssert boxRTTI[].GetAttribute(attributeCount + 1)[].IsType(
    0,
    api.EOSDataType.T_uint32,
    "",
  )

  block:
    var stream = api.constructStdStringStream()
    var bytesOut = [11'u8, 22, 33, 44, 55]
    var writer = api.constructStreamOutWrapper(stream)
    writer.WriteBytes(addr bytesOut[0], bytesOut.len.csize_t)
    doAssert not writer.IsFailed()
    stream.rewindToStart()
    var bytesIn: array[5, uint8]
    var reader = api.constructStreamInWrapper(stream)
    reader.ReadBytes(addr bytesIn[0], bytesIn.len.csize_t)
    doAssert not reader.IsEOF()
    doAssert not reader.IsFailed()
    doAssert bytesIn == bytesOut

  block:
    var buffer = api.constructStdStringStream()
    var writer = api.constructObjectStreamTextOut(buffer)
    let writerBase = api.asIObjectStreamOut(addr writer)
    api.OSWriteDataType(api.JoltApi, writerBase[], addr primitiveValue)
    api.OSWriteData(api.JoltApi, writerBase[], primitiveValue)
    buffer.rewindForRead()
    var reader = api.constructObjectStreamTextIn(buffer)
    let readerBase = api.asIObjectStreamIn(addr reader)
    var dataType: api.EOSDataType
    var readValue: uint32
    doAssert readerBase[].ReadDataType(dataType)
    doAssert dataType == api.EOSDataType.T_uint32
    doAssert api.OSReadData(api.JoltApi, readerBase[], readValue)
    doAssert readValue == primitiveValue

  block:
    var buffer = api.constructStdStringStream()
    var writer = api.constructObjectStreamTextOut(buffer)
    writer.HintIndentUp()
    writeTokens(writer)
    writer.HintIndentDown()
    buffer.rewindForRead()
    var reader = api.constructObjectStreamTextIn(buffer)
    readTokens(reader)

  block:
    var buffer = api.constructStdStringStream()
    var writer = api.constructObjectStreamBinaryOut(buffer)
    writeTokens(writer)
    buffer.rewindForRead()
    var reader = api.constructObjectStreamBinaryIn(buffer)
    readTokens(reader)

  api.UnregisterTypes(api.JoltApi)
  api.deleteJoltFactory(api.joltFactoryInstance)
  api.joltFactoryInstance = nil

when isMainModule:
  main()
