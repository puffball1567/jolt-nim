import std/[math, options, unittest]
import jolt

proc queryWorld(): World =
  var config = defaultWorldConfig()
  config.maxBodies = 128
  config.maxBodyPairs = 512
  config.maxContactConstraints = 512
  config.numThreads = 1
  newWorld(config)

suite "Jolt constraints and queries":
  test "closest-hit ray cast identifies a body":
    let world = queryWorld()
    defer:
      world.close()

    let floor = world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)),
      vec3(0, -0.5, 0)
    )
    let sphere = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))

    let hit = world.castRay(vec3(0, 5, 0), vec3(0, -1, 0), 10)
    require hit.isSome
    check hit.get.hits(sphere)
    check hit.get.bodyId == sphere.id
    check abs(hit.get.distance - 2.5) < 0.05
    check abs(hit.get.position.y - 2.5) < 0.05

    let miss = world.castRay(vec3(0, 5, 0), vec3(1, 0, 0), 5)
    check miss.isNone

    sphere.close()
    floor.close()

  test "point constraint owns its body dependencies":
    let world = queryWorld()
    defer:
      world.close()

    let anchor = world.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)),
      vec3(0, 4, 0)
    )
    let bob = world.addDynamicBody(sphereShape(0.5), vec3(0, 2, 0))
    let joint = addPointConstraint(anchor, bob, vec3(0, 4, 0))
    check joint.isAlive
    check joint.kind == ConstraintKind.Point
    expect JoltError:
      bob.close()

    bob.addImpulse(vec3(3, 0, 0))
    for _ in 0 ..< 120:
      check world.step(1.0'f32 / 60.0'f32) == {}
    let offset = bob.position
    let pivotDistance = sqrt(
      offset.x * offset.x +
      (offset.y - 4) * (offset.y - 4) +
      offset.z * offset.z
    )
    check pivotDistance > 1.8
    check pivotDistance < 2.2
    let pointImpulse = joint.solverImpulse
    check classify(pointImpulse.position.x) notin {fcNan, fcInf, fcNegInf}
    check classify(pointImpulse.position.y) notin {fcNan, fcInf, fcNegInf}
    check classify(pointImpulse.position.z) notin {fcNan, fcInf, fcNegInf}

    joint.close()
    joint.close()
    check not joint.isAlive
    bob.close()
    anchor.close()

  test "distance constraint maintains its configured length":
    let world = queryWorld()
    defer:
      world.close()

    world.setGravity(vec3(0, 0, 0))
    let anchor = world.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)),
      vec3(0, 2, 0)
    )
    let bob = world.addDynamicBody(sphereShape(0.4), vec3(2, 2, 0))
    let tether = addDistanceConstraint(
      anchor,
      bob,
      anchor.position,
      bob.position,
      2,
      2
    )
    bob.addImpulse(vec3(5, 0, 3))
    for _ in 0 ..< 120:
      check world.step(1.0'f32 / 60.0'f32) == {}
    let position = bob.position
    let distance = sqrt(
      position.x * position.x +
      (position.y - 2) * (position.y - 2) +
      position.z * position.z
    )
    check distance > 1.9
    check distance < 2.1

    tether.close()
    bob.close()
    anchor.close()

  test "hinge constraint limits rotation around one axis":
    let world = queryWorld()
    defer:
      world.close()

    world.setGravity(vec3(0, 0, 0))
    let anchor = world.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)),
      vec3(0, 6, 0)
    )
    let door = world.addDynamicBody(
      boxShape(vec3(1.0, 0.8, 0.15)),
      vec3(1, 3, 0)
    )
    let hinge = addHingeConstraint(
      anchor,
      door,
      vec3(0, 3, 0),
      vec3(0, 1, 0),
      -0.45,
      0.45
    )
    check hinge.kind == ConstraintKind.Hinge
    hinge.setLimits(-0.4, 0.4)
    hinge.setFriction(0.05)
    hinge.setFriction(0)
    door.setAngularVelocity(vec3(0, 4, 0))

    var maximumAngle = 0.0'f32
    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}
      let angle = hinge.currentAngle
      maximumAngle = max(maximumAngle, abs(angle))
      check angle >= -0.42
      check angle <= 0.42
    check maximumAngle > 0.1

    hinge.close()
    door.close()
    anchor.close()

  test "slider constraint confines translation to a range":
    let world = queryWorld()
    defer:
      world.close()

    world.setGravity(vec3(0, 0, 0))
    let anchor = world.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)),
      vec3(0, 5, 0)
    )
    let carriage = world.addDynamicBody(
      boxShape(vec3(0.5, 0.3, 0.5)),
      vec3(0, 3, 0)
    )
    let rail = addSliderConstraint(
      anchor,
      carriage,
      vec3(0, 3, 0),
      vec3(1, 0, 0),
      -1.5,
      1.5
    )
    check rail.kind == ConstraintKind.Slider
    rail.setLimits(-1.25, 1.25)
    rail.setFriction(0.05)
    rail.setFriction(0)
    carriage.setLinearVelocity(vec3(6, 0, 0))

    var maximumTravel = 0.0'f32
    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}
      let sliderPosition = rail.currentPosition
      maximumTravel = max(maximumTravel, abs(sliderPosition))
      check sliderPosition >= -1.28
      check sliderPosition <= 1.28
    check maximumTravel > 0.25
    check abs(carriage.position.y - 3) < 0.05
    check abs(carriage.position.z) < 0.05

    rail.close()
    carriage.close()
    anchor.close()

  test "fixed constraint preserves the current relative transform":
    let world = queryWorld()
    defer:
      world.close()

    let anchor = world.addStaticBody(
      boxShape(vec3(0.25, 0.25, 0.25)),
      vec3(0, 6, 0)
    )
    let welded = world.addDynamicBody(
      boxShape(vec3(0.8, 0.25, 0.25)),
      vec3(1.1, 6, 0),
      quatFromAxisAngle(vec3(0, 0, 1), 0.3)
    )
    let fixed = addFixedConstraint(
      anchor, welded,
      anchor.position, anchor.position,
      vec3(1, 0, 0), vec3(0, 1, 0),
      vec3(1, 0, 0), vec3(0, 1, 0))
    check fixed.kind == ConstraintKind.Fixed
    welded.addImpulse(vec3(8, -4, 3))
    welded.addAngularImpulse(vec3(2, 3, 4))
    for _ in 0 ..< 180:
      check world.step(1.0'f32 / 60.0'f32) == {}
    check abs(welded.position.x - 1.1) < 0.03
    check abs(welded.position.y - 6) < 0.03
    check abs(welded.position.z) < 0.03
    let fixedImpulse = fixed.solverImpulse
    check classify(fixedImpulse.position.y) notin {fcNan, fcInf, fcNegInf}
    check classify(fixedImpulse.rotation.x) notin {fcNan, fcInf, fcNegInf}
    expect ValueError:
      discard addFixedConstraint(
        anchor, welded,
        anchor.position, anchor.position,
        vec3(1, 0, 0), vec3(1, 0, 0),
        vec3(1, 0, 0), vec3(0, 1, 0))
    fixed.close()
    welded.close()
    anchor.close()

  test "common solver controls round trip and constraints can be paused":
    let world = queryWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let anchor = world.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)), vec3(0, 2, 0))
    let bob = world.addDynamicBody(sphereShape(0.4), vec3(1, 2, 0))
    let tether = addDistanceConstraint(
      anchor, bob, anchor.position, bob.position, 1, 1)

    check tether.distanceLimits == (1'f32, 1'f32)
    tether.setDistanceLimits(0.8, 1.2)
    check tether.distanceLimits == (0.8'f32, 1.2'f32)
    let distanceSpring = springSettings(6, 0.8)
    tether.setLimitSpring(distanceSpring)
    check tether.limitSpring.mode == distanceSpring.mode
    check abs(tether.limitSpring.value - distanceSpring.value) < 0.001
    check abs(tether.limitSpring.damping - distanceSpring.damping) < 0.001
    tether.setDistanceLimits(1, 1)
    expect ValueError:
      tether.setDistanceLimits(2, 1)

    check tether.isEnabled
    check tether.priority == 0
    check tether.solverStepOverrides == (0'u32, 0'u32)
    check tether.userData == 0
    tether.setPriority(37)
    tether.setSolverStepOverrides(7, 4)
    tether.setUserData(0x1234_5678_9abc_def0'u64)
    tether.resetWarmStart()
    check tether.priority == 37
    check tether.solverStepOverrides == (7'u32, 4'u32)
    check tether.userData == 0x1234_5678_9abc_def0'u64
    expect ValueError:
      tether.setSolverStepOverrides(256, 0)

    bob.setLinearVelocity(vec3(3, 0, 0))
    var maximumSolverImpulse = 0.0'f32
    for _ in 0 ..< 30:
      check world.step(1.0'f32 / 60.0'f32) == {}
      maximumSolverImpulse = max(
        maximumSolverImpulse, abs(tether.solverImpulse.position.x))
    check bob.position.x < 1.1
    check maximumSolverImpulse > 0.01

    tether.setEnabled(false)
    check not tether.isEnabled
    bob.setLinearVelocity(vec3(3, 0, 0))
    for _ in 0 ..< 30:
      check world.step(1.0'f32 / 60.0'f32) == {}
    check bob.position.x > 2

    tether.setEnabled(true)
    check tether.isEnabled
    tether.close()
    expect JoltError:
      discard tether.isEnabled
    bob.close()
    anchor.close()

  test "closing a world invalidates remaining constraints":
    let world = queryWorld()
    let first = world.addStaticBody(boxShape(vec3(0.2, 0.2, 0.2)), vec3(0, 2, 0))
    let second = world.addDynamicBody(sphereShape(0.4), vec3(1, 2, 0))
    let tether = addDistanceConstraint(
      first, second, first.position, second.position, 1, 1)

    world.close()
    check not tether.isAlive
    check not first.isAlive
    check not second.isAlive
    tether.close()

  test "invalid constraint and ray inputs are rejected":
    let firstWorld = queryWorld()
    let secondWorld = queryWorld()
    defer:
      firstWorld.close()
      secondWorld.close()

    let first = firstWorld.addDynamicBody(sphereShape(0.5), vec3(0, 1, 0))
    let second = secondWorld.addDynamicBody(sphereShape(0.5), vec3(0, 1, 0))
    expect JoltError:
      discard addPointConstraint(first, second, vec3(0, 1, 0))
    expect ValueError:
      discard firstWorld.castRay(vec3(0, 0, 0), vec3(0, 0, 0), 10)
    expect ValueError:
      discard firstWorld.castRay(vec3(0, 0, 0), vec3(1, 0, 0), 0)

    let third = firstWorld.addStaticBody(
      boxShape(vec3(0.2, 0.2, 0.2)), vec3(0, 3, 0))
    expect ValueError:
      discard addHingeConstraint(
        third, first, vec3(0, 2, 0), vec3(0, 0, 0), -0.5, 0.5)
    expect ValueError:
      discard addHingeConstraint(
        third, first, vec3(0, 2, 0), vec3(0, 1, 0), 0.1, 0.5)
    expect ValueError:
      discard addSliderConstraint(
        third, first, vec3(0, 2, 0), vec3(1, 0, 0), 0.1, 1.0)

    let point = addPointConstraint(third, first, vec3(0, 2, 0))
    expect ValueError:
      discard point.currentAngle
    expect ValueError:
      point.setFriction(1)
    point.close()

    first.close()
    second.close()
    third.close()
