import std/[math, unittest]
import jolt

proc shapeWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 256
  config.maxBodyPairs = 1_024
  config.maxContactConstraints = 1_024
  config.numThreads = 1
  newWorld(config)

proc addFloor(world: World): Body =
  world.addStaticBody(boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))

suite "Jolt extended shapes":
  test "an upright cylinder settles at its half height":
    let world = shapeWorld()
    defer: world.close()
    let floor = world.addFloor()
    let cylinder = world.addDynamicBody(cylinderShape(1, 0.5), vec3(0, 4, 0))

    for _ in 0 ..< 240:
      check world.step(1.0'f32 / 60.0'f32) == {}
    check abs(cylinder.position.y - 1.0) < 0.06
    check cylinder.shape.kind == ShapeKind.Cylinder
    check abs(cylinder.shape.halfHeight - 1.0) < 0.001
    check abs(cylinder.shape.radius - 0.5) < 0.001

    cylinder.close()
    floor.close()

  test "a horizontal cylinder converts travel into rolling motion":
    let world = shapeWorld()
    defer: world.close()
    let floor = world.addFloor()
    let cylinder = world.addDynamicBody(
      cylinderShape(0.8, 0.45),
      vec3(-3, 0.5, 0),
      quatFromAxisAngle(vec3(0, 0, 1), PI.float32 * 0.5)
    )
    cylinder.setFriction(0.9)
    cylinder.setLinearVelocity(vec3(0, 0, 4))

    let initialZ = cylinder.position.z
    var maximumSpin = 0.0'f32
    for _ in 0 ..< 90:
      check world.step(1.0'f32 / 60.0'f32) == {}
      maximumSpin = max(maximumSpin, abs(cylinder.angularVelocity.x))
    check abs(cylinder.position.z - initialZ) > 0.25
    check maximumSpin > 0.1

    cylinder.close()
    floor.close()

  test "cylinders participate in mixed-shape contacts":
    let world = shapeWorld()
    defer: world.close()
    let floor = world.addFloor()
    let cylinder = world.addDynamicBody(cylinderShape(0.7, 0.65), vec3(0, 1, 0))
    let sphere = world.addDynamicBody(sphereShape(0.5), vec3(0, 4, 0))
    let capsule = world.addDynamicBody(capsuleShape(0.6, 0.35), vec3(0.1, 6, 0))

    var contactAdded = 0
    discard world.drainEvents()
    for _ in 0 ..< 240:
      check world.step(1.0'f32 / 60.0'f32) == {}
      for event in world.drainEvents():
        if event.kind == PhysicsEventKind.ContactAdded:
          inc contactAdded
    check contactAdded >= 3
    check cylinder.position.y > 0
    check sphere.position.y > 0
    check capsule.position.y > 0

    capsule.close()
    sphere.close()
    cylinder.close()
    floor.close()

  test "cylinder convex radius is clamped to its dimensions":
    let shape = cylinderShape(0.3, 0.7, convexRadius = 2)
    check shape.kind == ShapeKind.Cylinder
    check abs(shape.convexRadius - 0.3) < 0.001

  test "invalid cylinder dimensions are rejected":
    expect ValueError:
      discard cylinderShape(0, 1)
    expect ValueError:
      discard cylinderShape(1, -1)
    expect ValueError:
      discard cylinderShape(1, 1, convexRadius = -0.1)
    expect ValueError:
      discard cylinderShape(Inf.float32, 1)
