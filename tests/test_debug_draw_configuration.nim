import std/unittest
import jolt

suite "Jolt debug drawing configuration":
  test "default builds report that debug drawing is unavailable":
    when defined(joltDebugRenderer):
      check debugRendererEnabled()
    else:
      check not debugRendererEnabled()
      expect JoltError:
        discard captureDebugDraw(nil)
