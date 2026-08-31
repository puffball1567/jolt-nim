## Minimal raylib surface used only by the optional visual demos.

type
  RlVector2* {.importc: "Vector2", header: "raylib.h", bycopy.} = object
    x*, y*: cfloat

  RlVector3* {.importc: "Vector3", header: "raylib.h", bycopy.} = object
    x*, y*, z*: cfloat

  RlColor* {.importc: "Color", header: "raylib.h", bycopy.} = object
    r*, g*, b*, a*: uint8

  RlCamera3D* {.importc: "Camera3D", header: "raylib.h", bycopy.} = object
    position*: RlVector3
    target*: RlVector3
    up*: RlVector3
    fovy*: cfloat
    projection*: cint

const
  FlagWindowResizable* = 0x00000004'u32
  FlagWindowHidden* = 0x00000080'u32
  FlagMsaa4xHint* = 0x00000020'u32
  CameraPerspective* = 0.cint
  CameraOrbital* = 2.cint
  LogWarning* = 4.cint
  KeySpace* = 32.cint
  KeyA* = 65.cint
  KeyB* = 66.cint
  KeyC* = 67.cint
  KeyD* = 68.cint
  KeyF* = 70.cint
  KeyG* = 71.cint
  KeyH* = 72.cint
  KeyJ* = 74.cint
  KeyL* = 76.cint
  KeyM* = 77.cint
  KeyN* = 78.cint
  KeyP* = 80.cint
  KeyS* = 83.cint
  KeyT* = 84.cint
  KeyW* = 87.cint
  KeyY* = 89.cint
  KeyV* = 86.cint
  KeyOne* = 49.cint
  KeyTwo* = 50.cint
  KeyThree* = 51.cint
  KeyFour* = 52.cint
  KeyFive* = 53.cint
  KeySix* = 54.cint
  KeySeven* = 55.cint
  KeyEight* = 56.cint
  KeyNine* = 57.cint
  KeyZero* = 48.cint
  KeyR* = 82.cint

proc rlVector2*[X, Y: SomeNumber](x: X; y: Y): RlVector2 =
  RlVector2(x: cfloat(x), y: cfloat(y))

proc rlVector3*[X, Y, Z: SomeNumber](x: X; y: Y; z: Z): RlVector3 =
  RlVector3(x: cfloat(x), y: cfloat(y), z: cfloat(z))

proc color*(r, g, b: uint8; a = 255'u8): RlColor =
  RlColor(r: r, g: g, b: b, a: a)

proc setConfigFlags*(flags: uint32) {.importc: "SetConfigFlags", cdecl, header: "raylib.h".}
proc setTraceLogLevel*(level: cint) {.importc: "SetTraceLogLevel", cdecl, header: "raylib.h".}
proc initWindow*(width, height: cint; title: cstring) {.importc: "InitWindow", cdecl, header: "raylib.h".}
proc closeWindow*() {.importc: "CloseWindow", cdecl, header: "raylib.h".}
proc windowShouldClose*(): bool {.importc: "WindowShouldClose", cdecl, header: "raylib.h".}
proc setTargetFPS*(fps: cint) {.importc: "SetTargetFPS", cdecl, header: "raylib.h".}
proc isKeyPressed*(key: cint): bool {.importc: "IsKeyPressed", cdecl, header: "raylib.h".}
proc isKeyDown*(key: cint): bool {.importc: "IsKeyDown", cdecl, header: "raylib.h".}
proc updateCamera*(camera: ptr RlCamera3D; mode: cint) {.importc: "UpdateCamera", cdecl, header: "raylib.h".}

proc beginDrawing*() {.importc: "BeginDrawing", cdecl, header: "raylib.h".}
proc endDrawing*() {.importc: "EndDrawing", cdecl, header: "raylib.h".}
proc clearBackground*(value: RlColor) {.importc: "ClearBackground", cdecl, header: "raylib.h".}
proc beginMode3D*(camera: RlCamera3D) {.importc: "BeginMode3D", cdecl, header: "raylib.h".}
proc endMode3D*() {.importc: "EndMode3D", cdecl, header: "raylib.h".}
proc drawGrid*(slices: cint; spacing: cfloat) {.importc: "DrawGrid", cdecl, header: "raylib.h".}
proc drawLine3D*(startPosition, endPosition: RlVector3; tint: RlColor) {.importc: "DrawLine3D", cdecl, header: "raylib.h".}
proc drawTriangle3D*(v1, v2, v3: RlVector3; tint: RlColor) {.importc: "DrawTriangle3D", cdecl, header: "raylib.h".}
proc drawCubeV*(position, size: RlVector3; tint: RlColor) {.importc: "DrawCubeV", cdecl, header: "raylib.h".}
proc drawCubeWiresV*(position, size: RlVector3; tint: RlColor) {.importc: "DrawCubeWiresV", cdecl, header: "raylib.h".}
proc drawSphereEx*(position: RlVector3; radius: cfloat; rings, slices: cint;
                   tint: RlColor) {.importc: "DrawSphereEx", cdecl, header: "raylib.h".}
proc drawSphereWires*(position: RlVector3; radius: cfloat; rings, slices: cint;
                      tint: RlColor) {.importc: "DrawSphereWires", cdecl, header: "raylib.h".}
proc drawCylinder*(position: RlVector3; radiusTop, radiusBottom, height: cfloat;
                   slices: cint; tint: RlColor) {.importc: "DrawCylinder", cdecl, header: "raylib.h".}
proc drawCylinderWires*(position: RlVector3; radiusTop, radiusBottom, height: cfloat;
                        slices: cint; tint: RlColor) {.importc: "DrawCylinderWires", cdecl, header: "raylib.h".}
proc drawCapsule*(startPosition, endPosition: RlVector3; radius: cfloat;
                  slices, rings: cint; tint: RlColor) {.importc: "DrawCapsule", cdecl, header: "raylib.h".}
proc drawCapsuleWires*(startPosition, endPosition: RlVector3; radius: cfloat;
                       slices, rings: cint; tint: RlColor) {.importc: "DrawCapsuleWires", cdecl, header: "raylib.h".}
proc drawRectangle*(x, y, width, height: cint; tint: RlColor) {.importc: "DrawRectangle", cdecl, header: "raylib.h".}
proc drawText*(text: cstring; x, y, fontSize: cint; tint: RlColor) {.importc: "DrawText", cdecl, header: "raylib.h".}
proc drawFPS*(x, y: cint) {.importc: "DrawFPS", cdecl, header: "raylib.h".}
proc takeScreenshot*(path: cstring) {.importc: "TakeScreenshot", cdecl, header: "raylib.h".}

proc rlPushMatrix*() {.importc: "rlPushMatrix", cdecl, header: "rlgl.h".}
proc rlPopMatrix*() {.importc: "rlPopMatrix", cdecl, header: "rlgl.h".}
proc rlTranslatef*(x, y, z: cfloat) {.importc: "rlTranslatef", cdecl, header: "rlgl.h".}
proc rlRotatef*(angleDegrees, x, y, z: cfloat) {.importc: "rlRotatef", cdecl, header: "rlgl.h".}
proc rlScalef*(x, y, z: cfloat) {.importc: "rlScalef", cdecl, header: "rlgl.h".}
