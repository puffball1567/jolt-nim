import std/[math, options, unittest]
import jolt

proc checkNear(actual, expected: float32; epsilon = 1.0e-4'f32) =
  check abs(actual - expected) <= epsilon

proc sourceSkeleton(): SkeletonDefinition =
  skeletonDefinition(@[
    skeletonJoint(
      "root", -1, skeletonTransform(vec3(0, 0, 0))),
    skeletonJoint(
      "hand", 0, skeletonTransform(vec3(0, 2, 0)))
  ])

proc targetSkeleton(): SkeletonDefinition =
  skeletonDefinition(@[
    skeletonJoint(
      "root", -1, skeletonTransform(vec3(0, 0, 0))),
    skeletonJoint(
      "elbow", 0, skeletonTransform(vec3(0, 1, 0))),
    skeletonJoint(
      "hand", 1, skeletonTransform(vec3(0, 2, 0))),
    skeletonJoint(
      "finger", 2, skeletonTransform(vec3(0, 2.4, 0)))
  ])

proc targetLocalPose(): seq[SkeletonTransform] =
  @[
    skeletonTransform(vec3(0, 0, 0)),
    skeletonTransform(vec3(0, 1, 0)),
    skeletonTransform(vec3(0, 1, 0)),
    skeletonTransform(vec3(0, 0.4, 0))
  ]

suite "Jolt skeleton mapper":
  test "maps low-detail model pose through target chains and leaves":
    let mapper = newSkeletonMapper(sourceSkeleton(), targetSkeleton())
    defer: mapper.close()
    check mapper.sourceJointCount == 2
    check mapper.targetJointCount == 4
    check mapper.mappingCount == 2
    check mapper.chainCount == 1
    check mapper.unmappedJointCount == 1
    check mapper.mappedJoint(0) == some(0)
    check mapper.mappedJoint(1) == some(2)

    let sourcePose = @[
      skeletonTransform(vec3(0, 0, 0)),
      skeletonTransform(vec3(2, 0, 0),
        quatFromAxisAngle(vec3(0, 0, 1), -PI.float32 * 0.5))
    ]
    let mapped = mapper.mappedPose(sourcePose, targetLocalPose())
    checkNear(mapped[0].position.x, 0)
    checkNear(mapped[2].position.x, 2)
    checkNear(mapped[2].position.y, 0)
    check mapped[1].position.x > 0.9
    check abs(mapped[1].position.y) < 0.1
    checkNear(mapped[3].position.x, 2.4, 2.0e-3)

    let reversed = mapper.reverseMappedPose(mapped)
    checkNear(reversed[0].position.x, sourcePose[0].position.x)
    checkNear(reversed[1].position.x, sourcePose[1].position.x)
    checkNear(reversed[1].position.y, sourcePose[1].position.y)

  test "translation locks preserve neutral target bone lengths":
    let mapper = newSkeletonMapper(sourceSkeleton(), targetSkeleton())
    defer: mapper.close()
    mapper.lockAllTranslations()
    check not mapper.isTranslationLocked(0)
    check mapper.isTranslationLocked(1)
    check mapper.isTranslationLocked(2)
    check mapper.isTranslationLocked(3)

    let sourcePose = @[
      skeletonTransform(vec3(3, 1, 0)),
      skeletonTransform(vec3(3, 3.4, 0))
    ]
    let mapped = mapper.mappedPose(sourcePose, targetLocalPose())
    let elbowDelta = vec3(
      mapped[1].position.x - mapped[0].position.x,
      mapped[1].position.y - mapped[0].position.y,
      mapped[1].position.z - mapped[0].position.z)
    checkNear(sqrt(
      elbowDelta.x * elbowDelta.x + elbowDelta.y * elbowDelta.y +
      elbowDelta.z * elbowDelta.z), 1, 2.0e-3)

  test "selected translation locks and lifetime are safe":
    let mapper = newSkeletonMapper(sourceSkeleton(), targetSkeleton())
    mapper.lockTranslations([false, true, false, true])
    check mapper.isTranslationLocked(1)
    check not mapper.isTranslationLocked(2)
    check mapper.isTranslationLocked(3)
    expect ValueError:
      mapper.lockTranslations([true])
    expect IndexDefect:
      discard mapper.mappedJoint(10)
    mapper.close()
    mapper.close()
    check not mapper.isAlive
    expect JoltError:
      discard mapper.mappingCount

  test "mapper and physics world lifetimes remain independent":
    let world = newWorld()
    let mapper = newSkeletonMapper(sourceSkeleton(), targetSkeleton())
    mapper.close()
    discard world.addDynamicBody(
      sphereShape(0.2), vec3(0, 1, 0))
    check world.step(1.0'f32 / 60.0'f32) == {}

    let secondMapper = newSkeletonMapper(sourceSkeleton(), targetSkeleton())
    world.close()
    let mapped = secondMapper.mappedPose(@[
      skeletonTransform(vec3(0, 0, 0)),
      skeletonTransform(vec3(0, 2, 0))], targetLocalPose())
    check mapped.len == 4
    secondMapper.close()

  test "invalid skeletons and pose sizes are rejected":
    var source = sourceSkeleton()
    var target = targetSkeleton()
    source.joints[1].parent = 2
    expect ValueError:
      discard newSkeletonMapper(source, target)
    source = sourceSkeleton()
    source.joints[1].name = "missing"
    expect ValueError:
      discard newSkeletonMapper(source, target)
    target.joints[1].name = "root"
    expect ValueError:
      discard newSkeletonMapper(sourceSkeleton(), target)

    let mapper = newSkeletonMapper(sourceSkeleton(), targetSkeleton())
    defer: mapper.close()
    expect ValueError:
      discard mapper.mappedPose(
        @[skeletonTransform(vec3(0, 0, 0))], targetLocalPose())
    expect ValueError:
      discard mapper.reverseMappedPose(
        @[skeletonTransform(vec3(0, 0, 0))])
