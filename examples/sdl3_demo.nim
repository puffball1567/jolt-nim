## Optional SDL3 validation demo. SDL3's 2D renderer draws a perspective
## wireframe projection of live 3D Jolt transforms.

import std/[math, os, strformat, strutils]
import jolt
import sdl3_nim

const
  width = 1120
  height = 720
  dt = 1.0'f32 / 60.0'f32

type
  Color = tuple[r, g, b: uint8]
  ScreenPoint = object
    x, y, depth: float32
    visible: bool
  Drawable = object
    body: Body
    halfExtent: Vec3
    color: Color

func `+`(a, b: Vec3): Vec3 = vec3(a.x + b.x, a.y + b.y, a.z + b.z)
func `-`(a, b: Vec3): Vec3 = vec3(a.x - b.x, a.y - b.y, a.z - b.z)
func `*`(a: Vec3; scale: float32): Vec3 =
  vec3(a.x * scale, a.y * scale, a.z * scale)
func dot(a, b: Vec3): float32 = a.x * b.x + a.y * b.y + a.z * b.z
func cross(a, b: Vec3): Vec3 =
  vec3(a.y * b.z - a.z * b.y,
       a.z * b.x - a.x * b.z,
       a.x * b.y - a.y * b.x)
func normalized(value: Vec3): Vec3 =
  let length = sqrt(value.dot(value))
  value * (1.0'f32 / length)

func rotate(rotation: Quat; point: Vec3): Vec3 =
  let q = vec3(rotation.x, rotation.y, rotation.z)
  let twice = q.cross(point) * 2
  point + twice * rotation.w + q.cross(twice)

func project(point: Vec3): ScreenPoint =
  const
    camera = Vec3(x: 11, y: 8, z: 15)
    target = Vec3(x: 0, y: 2, z: 0)
  let forward = (target - camera).normalized
  let right = forward.cross(vec3(0, 1, 0)).normalized
  let up = right.cross(forward)
  let relative = point - camera
  result.depth = relative.dot(forward)
  result.visible = result.depth > 0.1
  if result.visible:
    let scale = 780.0'f32 / result.depth
    result.x = width.float32 * 0.5'f32 + relative.dot(right) * scale
    result.y = height.float32 * 0.52'f32 - relative.dot(up) * scale

proc setColor(renderer: ptr SDL_Renderer; color: Color) =
  discard SDL_SetRenderDrawColor(
    renderer, color.r, color.g, color.b, 255'u8)

proc line(renderer: ptr SDL_Renderer; a, b: Vec3; color: Color) =
  let pa = a.project
  let pb = b.project
  if pa.visible and pb.visible:
    renderer.setColor(color)
    discard SDL_RenderLine(renderer, pa.x, pa.y, pb.x, pb.y)

proc drawBox(renderer: ptr SDL_Renderer; drawable: Drawable) =
  let center = drawable.body.position
  let rotation = drawable.body.rotation
  let h = drawable.halfExtent
  var corners: array[8, Vec3]
  for index in 0 ..< 8:
    let local = vec3(
      (if (index and 1) == 0: -h.x else: h.x),
      (if (index and 2) == 0: -h.y else: h.y),
      (if (index and 4) == 0: -h.z else: h.z))
    corners[index] = center + rotation.rotate(local)
  const edges = [
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
  for edge in edges:
    renderer.line(corners[edge[0]], corners[edge[1]], drawable.color)

proc drawCircle3D(renderer: ptr SDL_Renderer; center: Vec3; radius: float32;
                  axis: int; color: Color) =
  const segments = 24
  for index in 0 ..< segments:
    let a = 2.0'f32 * PI.float32 * index.float32 / segments.float32
    let b = 2.0'f32 * PI.float32 * (index + 1).float32 / segments.float32
    var p1, p2: Vec3
    case axis
    of 0:
      p1 = center + vec3(0, cos(a) * radius, sin(a) * radius)
      p2 = center + vec3(0, cos(b) * radius, sin(b) * radius)
    of 1:
      p1 = center + vec3(cos(a) * radius, 0, sin(a) * radius)
      p2 = center + vec3(cos(b) * radius, 0, sin(b) * radius)
    else:
      p1 = center + vec3(cos(a) * radius, sin(a) * radius, 0)
      p2 = center + vec3(cos(b) * radius, sin(b) * radius, 0)
    renderer.line(p1, p2, color)

proc drawSphere(renderer: ptr SDL_Renderer; body: Body; radius: float32;
                color: Color) =
  for axis in 0 .. 2:
    renderer.drawCircle3D(body.position, radius, axis, color)

proc drawCharacter(renderer: ptr SDL_Renderer; character: Character) =
  let position = character.position
  let color: Color = (70'u8, 225'u8, 250'u8)
  renderer.drawCircle3D(position + vec3(0, 0.6, 0), 0.35, 1, color)
  renderer.drawCircle3D(position + vec3(0, 0.6, 0), 0.35, 2, color)
  renderer.line(position + vec3(-0.35, 0.6, 0),
                position + vec3(-0.35, -0.6, 0), color)
  renderer.line(position + vec3(0.35, 0.6, 0),
                position + vec3(0.35, -0.6, 0), color)
  renderer.drawCircle3D(position + vec3(0, -0.6, 0), 0.35, 1, color)
  renderer.drawCircle3D(position + vec3(0, -0.6, 0), 0.35, 2, color)

proc saveScreenshot(renderer: ptr SDL_Renderer; path: string) =
  let surface = SDL_RenderReadPixels(renderer, nil)
  if surface.isNil:
    raise newException(IOError, "SDL_RenderReadPixels failed: " & $SDL_GetError())
  defer: SDL_DestroySurface(surface)
  if not SDL_SaveBMP(surface, path.cstring):
    raise newException(IOError, "SDL_SaveBMP failed: " & $SDL_GetError())

proc main() =
  let maxFrames = if paramCount() >= 1: parseInt(paramStr(1)) else: 0
  let screenshot = if paramCount() >= 2: paramStr(2) else: ""
  if not SDL_Init(SDL_INIT_VIDEO):
    raise newException(IOError, "SDL_Init failed: " & $SDL_GetError())
  defer: SDL_Quit_proc()

  var window: ptr SDL_Window
  var renderer: ptr SDL_Renderer
  if not SDL_CreateWindowAndRenderer(
      "jolt-nim SDL3 3D projection", width, height,
      SDL_WINDOW_RESIZABLE, addr window, addr renderer):
    raise newException(IOError, "SDL window creation failed: " & $SDL_GetError())
  defer:
    SDL_DestroyRenderer(renderer)
    SDL_DestroyWindow(window)
  discard SDL_SetRenderVSync(renderer, 1)

  let world = newWorld()
  defer: world.close()
  var boxes: seq[Drawable]
  let floor = world.addStaticBody(
    boxShape(vec3(8, 0.3, 6)), vec3(0, -0.3, 0))
  boxes.add(Drawable(
    body: floor, halfExtent: vec3(8, 0.3, 6), color: (95'u8, 110'u8, 135'u8)))
  let ramp = world.addStaticBody(
    boxShape(vec3(2.4, 0.18, 2)), vec3(-4.2, 1.0, 0),
    quatFromAxisAngle(vec3(0, 0, 1), -0.32))
  boxes.add(Drawable(
    body: ramp, halfExtent: vec3(2.4, 0.18, 2), color: (245'u8, 165'u8, 65'u8)))

  for level in 0 ..< 5:
    for column in 0 ..< 5 - level:
      let body = world.addDynamicBody(
        boxShape(vec3(0.38, 0.38, 0.38)),
        vec3(column.float32 * 0.82 - (4 - level).float32 * 0.41,
             0.42 + level.float32 * 0.8, 0))
      boxes.add(Drawable(
        body: body, halfExtent: vec3(0.38, 0.38, 0.38),
        color: (85'u8, uint8(145 + level * 18), 245'u8)))

  var spheres: seq[Body]
  for index in 0 ..< 8:
    spheres.add(world.addDynamicBody(
      sphereShape(0.32),
      vec3(-5.5 + index.float32 * 0.45, 5.2 + index.float32 * 0.55, 0)))

  var characterConfig = defaultCharacterConfig()
  characterConfig.maxQueuedContactEvents = 256
  let character = world.newCharacter(
    capsuleShape(0.6, 0.35), vec3(-6, 0, 2.7), characterConfig)
  world.optimizeBroadPhase()

  var running = true
  var frame = 0
  var direction = 1.0'f32
  var contactCount = 0'u64
  while running:
    var event: SDL_Event
    while SDL_PollEvent(addr event):
      if event.type_field == uint32(SDL_EVENT_QUIT) or
          (event.type_field == uint32(SDL_EVENT_KEY_DOWN) and
           event.key.scancode == SDL_SCANCODE_ESCAPE):
        running = false

    if character.position.x > 6: direction = -1
    elif character.position.x < -6: direction = 1
    character.move(vec3(direction * 2.5, 0, 0), dt)
    let errors = world.step(dt)
    if errors != {}:
      raise newException(JoltError, "Jolt update capacity exceeded")
    contactCount += character.drainContactEvents().len.uint64

    discard SDL_SetRenderDrawColor(renderer, 17, 21, 31, 255)
    discard SDL_RenderClear(renderer)
    for grid in -8 .. 8:
      renderer.line(vec3(grid.float32, 0.01, -6),
                    vec3(grid.float32, 0.01, 6), (42'u8, 51'u8, 67'u8))
    for grid in -6 .. 6:
      renderer.line(vec3(-8, 0.01, grid.float32),
                    vec3(8, 0.01, grid.float32), (42'u8, 51'u8, 67'u8))
    for drawable in boxes:
      renderer.drawBox(drawable)
    for sphere in spheres:
      renderer.drawSphere(sphere, 0.32, (240'u8, 100'u8, 125'u8))
    renderer.drawCharacter(character)
    let reachedFrameLimit = maxFrames > 0 and frame + 1 >= maxFrames
    if reachedFrameLimit and screenshot.len > 0:
      renderer.saveScreenshot(screenshot)
    discard SDL_RenderPresent(renderer)

    inc frame
    if frame mod 30 == 0:
      let title =
        &"jolt-nim + SDL3 | frame {frame} | character events {contactCount}"
      discard SDL_SetWindowTitle(window, title.cstring)
    if reachedFrameLimit:
      running = false

main()
