import jolt/bridge

static:
  doAssert sizeof(EMotionType) == sizeof(uint8)
  doAssert sizeof(EActivation) == sizeof(cint)

doAssert verifyJoltVersionID()
doAssert acquireJolt()
releaseJolt()
