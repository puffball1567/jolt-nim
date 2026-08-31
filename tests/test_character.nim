import std/[math, options, sequtils, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc settle(character: Character; world: World; frames = 180) =
  for _ in 0 ..< frames:
    character.move(vec3(0, 0, 0), dt)
    discard world.step(dt)

suite "Jolt virtual character":
  test "a capsule character falls, settles and reports its ground":
    let world = newWorld()
    defer: world.close()
    let floor = world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 4, 0))

    character.settle(world)

    check abs(character.position.y) < 0.08
    check character.groundState == CharacterGroundState.OnGround
    check character.isSupported
    check character.activeContactCount > 0
    check character.groundNormal.y > 0.95
    check character.groundBodyId.isSome
    check character.groundBodyId.get == floor.id

  test "extended update walks up a step and stops at a wall":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 4)), vec3(0, -0.5, 0))
    discard world.addStaticBody(
      boxShape(vec3(0.8, 0.15, 1.5)), vec3(-0.2, 0.15, 0))
    discard world.addStaticBody(
      boxShape(vec3(0.25, 2, 2)), vec3(3.2, 2, 0))
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(-3, 0, 0))
    character.settle(world, 30)

    var maximumHeight = character.position.y
    for _ in 0 ..< 240:
      character.move(vec3(2.5, 0, 0), dt)
      discard world.step(dt)
      maximumHeight = max(maximumHeight, character.position.y)

    check maximumHeight > 0.2
    check character.position.x > 2.0
    check character.position.x < 2.7

  test "jumping leaves the ground and lands again":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 0, 0))
    character.settle(world, 20)

    character.move(vec3(0, 0, 0), dt, jump = true, jumpSpeed = 6)
    discard world.step(dt)
    check character.linearVelocity.y > 5

    var peak = character.position.y
    for _ in 0 ..< 240:
      character.move(vec3(0, 0, 0), dt)
      discard world.step(dt)
      peak = max(peak, character.position.y)

    check peak > 1.5
    check abs(character.position.y) < 0.08
    check character.groundState == CharacterGroundState.OnGround

  test "a character inherits moving platform velocity":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(10, 0.5, 10)), vec3(0, -0.5, 0))
    let platform = world.addKinematicBody(
      boxShape(vec3(2, 0.25, 2)), vec3(0, 1, 0))
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 1.25, 0))
    character.settle(world, 20)

    for frame in 0 ..< 120:
      let targetX = (frame + 1).float32 * 0.01
      platform.moveKinematic(vec3(targetX, 1, 0), quatIdentity(), dt)
      character.move(vec3(0, 0, 0), dt)
      discard world.step(dt)

    check character.position.x > 0.7
    check character.groundVelocity.x > 0.4

  test "contact collection settings are configurable at creation and runtime":
    let world = newWorld()
    defer: world.close()
    var config = defaultCharacterConfig()
    config.predictiveContactDistance = 0.15
    config.maxNumHits = 48
    config.hitReductionCosMaxAngle = 0.95
    config.penetrationRecoverySpeed = 0.6
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0), config)

    check abs(character.configuration.predictiveContactDistance - 0.15) < 0.0001
    check character.maxNumHits == 48
    check abs(character.hitReductionCosMaxAngle - 0.95) < 0.0001
    check abs(character.penetrationRecoverySpeed - 0.6) < 0.0001
    check not character.maxHitsExceeded

    character.setMaxNumHits(96)
    character.setHitReductionCosMaxAngle(-1)
    character.setPenetrationRecoverySpeed(0.25)
    check character.maxNumHits == 96
    check character.configuration.maxNumHits == 96
    check character.hitReductionCosMaxAngle == -1
    check character.configuration.hitReductionCosMaxAngle == -1
    check abs(character.penetrationRecoverySpeed - 0.25) < 0.0001

  test "penetration recovery speed changes overlap correction":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    var slowConfig = defaultCharacterConfig()
    slowConfig.penetrationRecoverySpeed = 0
    var fastConfig = slowConfig
    fastConfig.penetrationRecoverySpeed = 1
    let slow = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(-2, -0.25, 0), slowConfig)
    let fast = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(2, -0.25, 0), fastConfig)

    for _ in 0 ..< 12:
      slow.update(dt, vec3(0, 0, 0))
      fast.update(dt, vec3(0, 0, 0))
      discard world.step(dt)

    check fast.position.y > slow.position.y + 0.15

  test "virtual characters collide with each other and expose contact identity":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let left = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(-1.5, 0, 0))
    let right = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(1.5, 0, 0))
    left.settle(world, 20)
    right.settle(world, 20)

    for _ in 0 ..< 120:
      left.move(vec3(2, 0, 0), dt)
      right.move(vec3(-2, 0, 0), dt)
      discard world.step(dt)

    check right.position.x - left.position.x > 0.65
    check left.hasCollidedWith(right) or right.hasCollidedWith(left)
    var foundCharacterContact = false
    for contact in left.contacts & right.contacts:
      if contact.characterId.isSome and contact.hadCollision:
        foundCharacterContact = true
        check contact.bodyId.isNone
        check contact.surfaceNormal.y < 0.2
    check foundCharacterContact

    right.close()
    for _ in 0 ..< 10:
      left.move(vec3(2, 0, 0), dt)
      discard world.step(dt)
    check left.isAlive

  test "an inner body is queryable follows teleports and changes shape":
    let world = newWorld()
    defer: world.close()
    var config = defaultCharacterConfig()
    config.userData = 0xCAFE
    config.innerBodyShape = some(sphereShape(0.28))
    config.innerBodyLayer = movingLayer
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 1, 0), config)
    check character.innerBodyId.isSome
    let innerId = character.innerBodyId.get

    let firstHit = world.castRay(
      vec3(0, 4, 0), vec3(0, -1, 0), 6,
      bodyFilter = includeBodies([innerId]))
    check firstHit.isSome
    check firstHit.get.bodyId == innerId

    let saved = world.saveState()
    defer: saved.close()

    character.setPosition(vec3(4, 1, 0))
    let movedHit = world.castRay(
      vec3(4, 4, 0), vec3(0, -1, 0), 6,
      bodyFilter = includeBodies([innerId]))
    check movedHit.isSome
    world.restoreState(saved)
    check abs(character.position.x) < 0.001
    let restoredHit = world.castRay(
      vec3(0, 4, 0), vec3(0, -1, 0), 6,
      bodyFilter = includeBodies([innerId]))
    check restoredHit.isSome
    character.setInnerBodyShape(boxShape(vec3(0.3, 0.4, 0.3)))
    check character.configuration.innerBodyShape.get.kind == ShapeKind.Box

  test "contacts and runtime character properties expose native state":
    let world = newWorld()
    defer: world.close()
    var floorConfig = defaultBodyConfig()
    floorConfig.userData = 77
    let floor = world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0),
      config = floorConfig)
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 3, 0))
    character.settle(world)

    check character.hasCollidedWith(floor)
    var foundFloor = false
    for contact in character.contacts:
      if contact.bodyId == some(floor.id) and contact.hadCollision:
        foundFloor = true
        check contact.characterId.isNone
        check contact.motionType == MotionType.Static
        check contact.userData == 77
        check contact.contactNormal.y > 0.9
        check contact.surfaceNormal.y > 0.9
        check not contact.isSensor
        check not contact.wasDiscarded
    check foundFloor

    character.setMass(95)
    character.setMaxStrength(250)
    character.setEnhancedInternalEdgeRemoval(false)
    character.setUserData(1234)
    check abs(character.mass - 95) < 0.001
    check abs(character.maxStrength - 250) < 0.001
    check not character.enhancedInternalEdgeRemoval
    check character.userData == 1234
    check character.setShape(capsuleShape(0.35, 0.3), 0.1)
    check abs(character.shape.halfHeight - 0.35) < 0.001
    check abs(character.position.y) < 0.08

  test "advanced settings low-level movement and inner-body capacity are safe":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    var config = defaultCharacterConfig()
    config.backFaceMode = CharacterBackFaceMode.IgnoreBackFaces
    config.maxCollisionIterations = 8
    config.maxConstraintIterations = 24
    config.minTimeRemaining = 0.0002
    config.collisionTolerance = 0.002
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 0, 0), config)
    character.settle(world, 20)
    let clamped = character.cancelVelocityTowardsSteepSlopes(vec3(1, 0, 0))
    check abs(clamped.x - 1) < 0.001
    character.setPosition(vec3(0, 0.2, 0), refreshContacts = false)
    check character.stickToFloor(vec3(0, -0.5, 0))
    check abs(character.position.y) < 0.08
    expect(ValueError):
      discard character.walkStairs(
        0, vec3(0, 0.4, 0), vec3(0.1, 0, 0),
        vec3(0.15, 0, 0), vec3(0, 0, 0))

    var limitedConfig = defaultWorldConfig()
    limitedConfig.maxBodies = 1
    let limitedWorld = newWorld(limitedConfig)
    let blocker = limitedWorld.addStaticBody(
      boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))
    var innerConfig = defaultCharacterConfig()
    innerConfig.innerBodyShape = some(sphereShape(0.25))
    expect(JoltError):
      discard limitedWorld.newCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 2, 0), innerConfig)
    blocker.close()
    let recycled = limitedWorld.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0), innerConfig)
    check recycled.innerBodyId.isSome
    limitedWorld.close()
    check not recycled.isAlive

  test "the bounded listener queue reports body lifecycle and solver events":
    let world = newWorld()
    defer: world.close()
    var floorConfig = defaultBodyConfig()
    floorConfig.userData = 0xBEEF
    let floor = world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0),
      config = floorConfig)
    var config = defaultCharacterConfig()
    config.maxQueuedContactEvents = 4096
    config.canPushCharacter = false
    config.canReceiveImpulses = false
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 3, 0), config)
    character.settle(world)

    check character.pendingContactEventCount > 0
    let events = character.drainContactEvents()
    check character.pendingContactEventCount == 0
    var foundAdded, foundPersisted, foundSolved = false
    for event in events:
      if event.bodyId == some(floor.id):
        case event.kind
        of CharacterContactEventKind.BodyContactAdded:
          foundAdded = true
          check event.userData == 0xBEEF
          check not event.canPushCharacter
          check not event.canReceiveImpulses
        of CharacterContactEventKind.BodyContactPersisted:
          foundPersisted = true
        of CharacterContactEventKind.BodyContactSolved:
          foundSolved = true
          check event.normal.y > 0.9
          check abs(event.position.y) < 0.1
        else: discard
    check foundAdded
    check foundPersisted
    check foundSolved

    character.setPosition(vec3(0, 5, 0))
    let removed = character.drainContactEvents()
    check removed.anyIt(
      it.kind == CharacterContactEventKind.BodyContactRemoved and
      it.bodyId == some(floor.id))

    character.setContactResponse(true, true, true)
    check character.configuration.canPushCharacter
    check character.configuration.canReceiveImpulses
    check character.configuration.preventSliding
    character.setMaxQueuedContactEvents(4)
    character.setPosition(vec3(0, 1, 0))
    character.settle(world, 30)
    check character.pendingContactEventCount <= 4
    check character.droppedContactEventCount() > 0
    check character.droppedContactEventCount(reset = true) > 0
    check character.droppedContactEventCount() == 0

  test "listener events identify other virtual characters and restore clears queues":
    let world = newWorld()
    defer: world.close()
    discard world.addStaticBody(
      boxShape(vec3(8, 0.5, 8)), vec3(0, -0.5, 0))
    let left = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(-1.3, 0, 0))
    let right = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(1.3, 0, 0))
    left.settle(world, 20)
    right.settle(world, 20)
    discard left.drainContactEvents()
    discard right.drainContactEvents()
    let saved = world.saveState()
    defer: saved.close()

    for _ in 0 ..< 80:
      left.move(vec3(2, 0, 0), dt)
      right.move(vec3(-2, 0, 0), dt)
      discard world.step(dt)
    let events = left.drainContactEvents()
    check events.anyIt(
      it.kind == CharacterContactEventKind.VirtualContactAdded and
      it.characterId == some(right.characterId))
    check events.anyIt(
      it.kind == CharacterContactEventKind.VirtualContactSolved and
      it.characterId == some(right.characterId))

    left.move(vec3(0, 0, 0), dt)
    discard world.step(dt)
    check left.pendingContactEventCount > 0
    world.restoreState(saved)
    check left.pendingContactEventCount == 0
    check right.pendingContactEventCount == 0

  test "shared spatial broad phase prunes a large virtual-character crowd":
    var worldConfig = defaultWorldConfig()
    worldConfig.characterBroadPhaseCellSize = 2
    let world = newWorld(worldConfig)
    defer: world.close()
    var characters: seq[Character]
    for z in 0 ..< 8:
      for x in 0 ..< 16:
        characters.add(world.newCharacter(
          capsuleShape(0.6, 0.35),
          vec3(float32(x) * 10, 0, float32(z) * 10)))

    var stats = world.characterBroadPhaseStats
    check stats.registeredCharacters == 128
    check stats.occupiedCells == 128
    world.resetCharacterBroadPhaseStats()
    characters[0].refreshContacts()
    stats = world.characterBroadPhaseStats
    check stats.queryCount > 0
    check stats.candidateCount < 8
    check stats.narrowPhaseTestCount == 0

    characters[1].setPosition(vec3(0.5, 0, 0), refreshContacts = false)
    world.resetCharacterBroadPhaseStats()
    characters[0].refreshContacts()
    stats = world.characterBroadPhaseStats
    check stats.candidateCount > 0
    check stats.candidateCount < 8
    check stats.narrowPhaseTestCount > 0
    check characters[0].contacts.anyIt(
      it.characterId == some(characters[1].characterId))

    let saved = world.saveState()
    defer: saved.close()
    characters[1].setPosition(vec3(40, 0, 40), refreshContacts = false)
    world.restoreState(saved)
    world.resetCharacterBroadPhaseStats()
    characters[0].refreshContacts()
    stats = world.characterBroadPhaseStats
    check stats.candidateCount > 0
    check characters[0].contacts.anyIt(
      it.characterId == some(characters[1].characterId))

    characters[1].close()
    check world.characterBroadPhaseStats.registeredCharacters == 127

  test "character validation and lifetime reject invalid use":
    let world = newWorld()
    let character = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0))
    expect(ValueError):
      discard world.newCharacter(boxShape(vec3(1, 1, 1)), vec3(0, 0, 0))
    var bad = defaultCharacterConfig()
    bad.padding = 0.5
    expect(ValueError):
      discard world.newCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 0, 0), bad)
    bad = defaultCharacterConfig()
    bad.maxNumHits = 0
    expect(ValueError):
      discard world.newCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 0, 0), bad)
    bad = defaultCharacterConfig()
    bad.penetrationRecoverySpeed = 1.1
    expect(ValueError):
      discard world.newCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 0, 0), bad)
    bad = defaultCharacterConfig()
    bad.maxCollisionIterations = 0
    expect(ValueError):
      discard world.newCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 0, 0), bad)
    bad = defaultCharacterConfig()
    bad.maxQueuedContactEvents = 0
    expect(ValueError):
      discard world.newCharacter(
        capsuleShape(0.6, 0.35), vec3(0, 0, 0), bad)
    var badWorld = defaultWorldConfig()
    badWorld.characterBroadPhaseCellSize = 0
    expect(ValueError): discard newWorld(badWorld)
    expect(ValueError): character.setMaxNumHits(0)
    expect(ValueError): character.setHitReductionCosMaxAngle(1.1)
    expect(ValueError): character.setPenetrationRecoverySpeed(-0.1)
    expect(ValueError): character.setMaxQueuedContactEvents(0)
    expect(ValueError):
      character.move(vec3(0, 0, 0), 0)

    character.close()
    check not character.isAlive
    expect(JoltError): discard character.position

    let remaining = world.newCharacter(
      capsuleShape(0.6, 0.35), vec3(0, 2, 0))
    world.close()
    check not remaining.isAlive
    expect(JoltError): remaining.refreshContacts()
