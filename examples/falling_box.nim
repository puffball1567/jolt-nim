import jolt

proc main() =
  let world = newWorld()
  defer:
    world.close()

  let floor = world.addStaticBody(
    boxShape(vec3(100, 1, 100)),
    vec3(0, -1, 0)
  )
  let box = world.addDynamicBody(
    boxShape(vec3(0.5, 0.5, 0.5)),
    vec3(0, 2, 0)
  )

  world.optimizeBroadPhase()
  for frame in 0 ..< 120:
    let errors = world.step(1.0'f32 / 60.0'f32)
    if errors != {}:
      raise newException(JoltError, "physics update capacity exceeded")
    if frame mod 15 == 0:
      let p = box.position
      echo "frame ", frame, ": y = ", p.y

  floor.close()
  box.close()

main()
