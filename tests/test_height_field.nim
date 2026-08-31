import std/[math, options, unittest]
import jolt

const
  dt = 1.0'f32 / 60.0'f32
  slopeSampleCount = 8

proc slopedSamples(): seq[float32] =
  result = newSeq[float32](slopeSampleCount * slopeSampleCount)
  for z in 0 ..< slopeSampleCount:
    for x in 0 ..< slopeSampleCount:
      result[z * slopeSampleCount + x] = float32(z) * 0.25'f32

proc flatSamples(sampleCount: int): seq[float32] =
  newSeq[float32](sampleCount * sampleCount)

suite "Jolt height field":
  test "a cooked height field supports bodies and spatial queries":
    let world = newWorld()
    defer: world.close()
    let terrainShape = heightFieldShape(
      slopedSamples(), slopeSampleCount,
      offset = vec3(-3.5, 0, -3.5))
    let terrain = world.addStaticBody(terrainShape, vec3(0, 0, 0))
    let box = world.addDynamicBody(
      boxShape(vec3(0.5, 0.5, 0.5)), vec3(0, 4, 0))
    terrain.setFriction(1)
    box.setFriction(1)

    for _ in 0 ..< 240:
      discard world.step(dt)

    check box.position.y > 1.25
    check box.position.y < 1.55
    check terrain.shape.kind == ShapeKind.HeightField
    check terrain.shape.sampleCount == uint32(slopeSampleCount)
    check terrain.shape.heightSamples.len == slopeSampleCount * slopeSampleCount
    check terrain.shape.heightOffset == vec3(-3.5, 0, -3.5)
    let ray = world.castRay(vec3(2, 5, 0), vec3(0, -1, 0), 10)
    check ray.isSome
    check ray.get.hits(terrain)
    let sphereHit = world.castSphere(
      0.25, vec3(-2, 5, 0), vec3(0, -1, 0), 10)
    check sphereHit.isSome
    check sphereHit.get.hits(terrain)
    check world.overlapSphere(vec3(2, 1, 0), 0.3).len > 0

  test "a virtual character climbs a height field slope":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      heightFieldShape(
        slopedSamples(), slopeSampleCount,
        offset = vec3(-3.5, 0, -3.5)),
      vec3(0, 0, 0))
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 0.2, -3))

    for _ in 0 ..< 30:
      character.move(vec3(0, 0, 0), dt)
      discard world.step(dt)
    for _ in 0 ..< 180:
      character.move(vec3(0, 0, 2), dt)
      discard world.step(dt)

    check character.position.z > 2
    check character.position.y > 1.2
    check character.groundState == CharacterGroundState.OnGround

  test "vehicle wheel casts contact a height field":
    let world = newWorld()
    defer: world.close()
    let terrain = world.addStaticBody(
      heightFieldShape(
        flatSamples(16), 16,
        offset = vec3(-15, 0, -15),
        scale = vec3(2, 1, 2)),
      vec3(0, 0, 0))
    let chassis = world.addDynamicBody(
      boxShape(vec3(0.9, 0.3, 2)), vec3(0, 1.2, -8))
    let vehicle = chassis.newVehicle()

    for _ in 0 ..< 150:
      vehicle.setInput(0, 0)
      discard world.step(dt)
    var contacts = 0
    for wheel in 0 ..< vehicle.wheelCount:
      let state = vehicle.wheelState(wheel)
      if state.hasContact:
        inc contacts
        check state.contactBodyId.isSome
        check state.contactBodyId.get == terrain.id
    check contacts >= 3

    let startZ = chassis.position.z
    for _ in 0 ..< 240:
      vehicle.setInput(1, 0)
      discard world.step(dt)
    check abs(chassis.position.z - startZ) > 3

  test "no-collision samples create a queryable terrain hole":
    var samples = flatSamples(slopeSampleCount)
    for z in 3 .. 4:
      for x in 3 .. 4:
        samples[z * slopeSampleCount + x] = heightFieldNoCollision
    let world = newWorld()
    defer: world.close()
    let terrain = world.addStaticBody(
      heightFieldShape(
        samples, slopeSampleCount,
        offset = vec3(-3.5, 0, -3.5)),
      vec3(0, 0, 0))

    let throughHole = world.castRay(
      vec3(0, 3, 0), vec3(0, -1, 0), 6)
    let besideHole = world.castRay(
      vec3(-2, 3, 0), vec3(0, -1, 0), 6)
    check throughHole.isNone
    check besideHole.isSome
    check besideHole.get.hits(terrain)

  test "height field descriptions and static-only use are validated":
    expect(ValueError):
      discard heightFieldShape(flatSamples(3), 3)
    expect(ValueError):
      discard heightFieldShape(newSeq[float32](15), 4)
    expect(ValueError):
      discard heightFieldShape(flatSamples(4), 4, scale = vec3(1, 0, 1))
    expect(ValueError):
      var samples = flatSamples(4)
      samples[3] = NaN.float32
      discard heightFieldShape(samples, 4)
    expect(ValueError):
      discard heightFieldShape(flatSamples(4), 4, blockSize = 1)
    expect(ValueError):
      discard heightFieldShape(flatSamples(4), 4, bitsPerSample = 17)

    let world = newWorld()
    defer: world.close()
    expect(ValueError):
      discard world.addDynamicBody(
        heightFieldShape(flatSamples(4), 4), vec3(0, 0, 0))
