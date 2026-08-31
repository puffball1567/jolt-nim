import std/[options, unittest]
import jolt

const dt = 1.0'f32 / 60.0'f32

proc makePair(world: World): tuple[left, right: Body] =
  result.left = world.addDynamicBody(sphereShape(0.5), vec3(-2, 0, 0))
  result.right = world.addDynamicBody(sphereShape(0.5), vec3(2, 0, 0))
  result.left.setGravityFactor(0)
  result.right.setGravityFactor(0)
  result.left.setLinearVelocity(vec3(3, 0, 0))
  result.right.setLinearVelocity(vec3(-3, 0, 0))

proc simulate(world: World; frames = 90) =
  for _ in 0 ..< frames:
    discard world.step(dt)

suite "Jolt collision groups":
  test "disabled subgroups pass through each other":
    let world = newWorld()
    defer: world.close()
    let filter = newCollisionGroupFilter(3)
    filter.setCollisionEnabled(0, 1, false)
    let pair = world.makePair()
    pair.left.setCollisionGroup(filter.bodyCollisionGroup(10, 0))
    pair.right.setCollisionGroup(filter.bodyCollisionGroup(10, 1))

    world.simulate()

    check pair.left.position.x > pair.right.position.x
    check pair.left.collisionGroup.isSome
    check pair.left.collisionGroup.get.groupId == 10
    check pair.left.collisionGroup.get.subgroupId == 0

  test "enabled subgroups collide and different group IDs ignore the table":
    block enabledPair:
      let world = newWorld()
      defer: world.close()
      let filter = newCollisionGroupFilter(2)
      let pair = world.makePair()
      pair.left.setCollisionGroup(filter.bodyCollisionGroup(7, 0))
      pair.right.setCollisionGroup(filter.bodyCollisionGroup(7, 1))
      world.simulate()
      check pair.left.position.x < pair.right.position.x

    block differentMainGroups:
      let world = newWorld()
      defer: world.close()
      let filter = newCollisionGroupFilter(2)
      filter.setCollisionEnabled(0, 1, false)
      let pair = world.makePair()
      pair.left.setCollisionGroup(filter.bodyCollisionGroup(1, 0))
      pair.right.setCollisionGroup(filter.bodyCollisionGroup(2, 1))
      world.simulate()
      check pair.left.position.x < pair.right.position.x

  test "same subgroup never collides and groups can be cleared":
    let world = newWorld()
    defer: world.close()
    let filter = newCollisionGroupFilter(2)
    let pair = world.makePair()
    pair.left.setCollisionGroup(filter.bodyCollisionGroup(5, 0))
    pair.right.setCollisionGroup(filter.bodyCollisionGroup(5, 0))
    world.simulate()
    check pair.left.position.x > pair.right.position.x

    pair.left.clearCollisionGroup()
    check pair.left.collisionGroup.isNone

  test "native bodies retain a closed filter":
    let world = newWorld()
    defer: world.close()
    let filter = newCollisionGroupFilter(2)
    filter.setCollisionEnabled(0, 1, false)
    let pair = world.makePair()
    pair.left.setCollisionGroup(filter.bodyCollisionGroup(3, 0))
    pair.right.setCollisionGroup(filter.bodyCollisionGroup(3, 1))
    filter.close()

    world.simulate()
    check pair.left.position.x > pair.right.position.x

  test "filter tables and subgroup indices are validated":
    expect(ValueError): discard newCollisionGroupFilter(1)
    let filter = newCollisionGroupFilter(3)
    defer: filter.close()
    check filter.subgroupCount == 3
    check filter.collisionEnabled(0, 1)
    check not filter.collisionEnabled(1, 1)
    expect(ValueError): filter.setCollisionEnabled(1, 1, true)
    expect(IndexDefect): discard filter.collisionEnabled(0, 3)

    let world = newWorld()
    defer: world.close()
    let body = world.addDynamicBody(sphereShape(0.5), vec3(0, 1, 0))
    expect(ValueError):
      body.setCollisionGroup(BodyCollisionGroup(groupId: 1, subgroupId: 0))
