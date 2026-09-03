import std/[math, options, unittest]
import jolt

proc debugWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  newWorld(config)

proc finite(value: Vec3): bool =
  classify(value.x) notin {fcNan, fcInf, fcNegInf} and
    classify(value.y) notin {fcNan, fcInf, fcNegInf} and
    classify(value.z) notin {fcNan, fcInf, fcNegInf}

suite "Jolt high-level debug drawing":
  test "default capture returns detached body triangles":
    check debugRendererEnabled()
    let world = debugWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(2, 0.25, 2)), vec3(0, -0.25, 0))
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))

    let frame = world.captureDebugDraw()
    check frame.triangles.len > 0
    check not frame.truncated
    for triangle in frame.triangles:
      check triangle.v1.finite
      check triangle.v2.finite
      check triangle.v3.finite
      check triangle.color.a > 0

    let capturedY = frame.triangles[0].v1.y
    body.setTransform(vec3(0, 5, 0), quatIdentity())
    check frame.triangles[0].v1.y == capturedY

  test "wireframe capture honors reusable body filters":
    let world = debugWorld()
    defer: world.close()
    let box = world.addStaticBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(-2, 0, 0))
    let sphere = world.addStaticBody(sphereShape(0.5), vec3(2, 0, 0))

    var options = defaultDebugDrawOptions()
    options.bodySettings.drawShapeWireframe = true
    let allBodies = world.captureDebugDraw(options)
    check allBodies.lines.len > 0
    check allBodies.triangles.len == 0

    options.bodyFilter = includeBodies([box.id])
    let boxOnly = world.captureDebugDraw(options)
    check boxOnly.lines.len > 0
    check boxOnly.lines.len < allBodies.lines.len

    options.bodyFilter = excludeBodies([box.id])
    let sphereOnly = world.captureDebugDraw(options)
    check sphereOnly.lines.len > 0
    check sphereOnly.lines.len < allBodies.lines.len

    options.bodyFilter = world.queryBodyFilter(
      bodyQueryCriteria(userData = some(high(uint64))))
    let noBodies = world.captureDebugDraw(options)
    check noBodies.lines.len == 0
    check noBodies.triangles.len == 0
    discard sphere

  test "constraint diagnostics and text are captured with bounded storage":
    let world = debugWorld()
    defer: world.close()
    let anchor = world.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)), vec3(0, 3, 0))
    let bob = world.addDynamicBody(sphereShape(0.5), vec3(0, 1, 0))
    discard addPointConstraint(anchor, bob, vec3(0, 3, 0))

    var options = defaultDebugDrawOptions()
    options.bodySettings.drawShape = false
    options.bodySettings.drawMassAndInertia = true
    options.drawConstraints = true
    options.drawConstraintLimits = true
    options.drawConstraintReferenceFrames = true
    let diagnostics = world.captureDebugDraw(options)
    check diagnostics.lines.len > 0
    check diagnostics.texts.len > 0
    check diagnostics.texts[0].text.len > 0
    check diagnostics.texts[0].height > 0

    options.bodySettings.drawShape = true
    options.limits.maxLines = 1
    options.limits.maxTriangles = 1
    options.limits.maxTexts = 0
    options.limits.maxTextBytes = 0
    let bounded = world.captureDebugDraw(options)
    check bounded.lines.len <= 1
    check bounded.triangles.len <= 1
    check bounded.texts.len == 0
    check bounded.droppedLines > 0
    check bounded.droppedTriangles > 0
    check bounded.droppedTexts > 0
    check bounded.truncated

  test "capture settings and world lifetime are validated":
    let world = debugWorld()
    var options = defaultDebugDrawOptions()
    options.cameraPosition = vec3(NaN, 0, 0)
    expect ValueError:
      discard world.captureDebugDraw(options)

    when sizeof(int) >= 8:
      options = defaultDebugDrawOptions()
      options.limits.maxLines = uint(high(uint32)) + 1'u
      expect ValueError:
        discard world.captureDebugDraw(options)

    world.close()
    expect JoltError:
      discard world.captureDebugDraw()
