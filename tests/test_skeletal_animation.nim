import std/[math, unittest]
import jolt

proc checkNear(actual, expected: float32; epsilon = 1.0e-4'f32) =
  check abs(actual - expected) <= epsilon

proc animationSkeleton(): SkeletonDefinition =
  skeletonDefinition(@[
    skeletonJoint("root", -1, skeletonTransform(vec3(0, 0, 0))),
    skeletonJoint("arm", 0, skeletonTransform(vec3(0, 1, 0))),
    skeletonJoint("hand", 1, skeletonTransform(vec3(0, 1, 0)))
  ])

proc testAnimation(looping = true): SkeletalAnimation =
  newSkeletalAnimation(animationSkeleton(), [
    skeletalAnimationTrack("arm", @[
      skeletalAnimationKeyframe(0, vec3(0, 1, 0)),
      skeletalAnimationKeyframe(
        1, vec3(0, 1, 0), quatFromAxisAngle(vec3(0, 0, 1), PI.float32))]),
    skeletalAnimationTrack("root", @[
      skeletalAnimationKeyframe(0, vec3(0, 0, 0)),
      skeletalAnimationKeyframe(2, vec3(2, 0, 0))])
  ], looping)

suite "Jolt skeletal animation":
  test "interpolates translations and rotations through Jolt":
    let animation = testAnimation()
    defer: animation.close()
    check animation.jointCount == 3
    check animation.trackCount == 2
    check animation.keyframeCount == 4
    checkNear(animation.duration, 2)

    let pose = animation.sampleLocalPose(0.5)
    checkNear(pose[0].position.x, 0.5)
    checkNear(abs(pose[1].rotation.z), sin(PI.float32 * 0.25), 2.0e-4)
    checkNear(abs(pose[1].rotation.w), cos(PI.float32 * 0.25), 2.0e-4)
    checkNear(pose[2].position.y, 1)

  test "looping and non-looping sampling follow Jolt semantics":
    let animation = testAnimation()
    defer: animation.close()
    check animation.isLooping
    checkNear(animation.sampleLocalPose(2.5)[0].position.x, 0.5)
    animation.setLooping(false)
    check not animation.isLooping
    checkNear(animation.sampleLocalPose(2.5)[0].position.x, 2)
    checkNear(abs(animation.sampleLocalPose(2.5)[1].rotation.w), 0, 2.0e-4)

  test "model pose composes hierarchy and preserves neutral untracked joints":
    let animation = testAnimation(looping = false)
    defer: animation.close()
    let pose = animation.sampleModelPose(1)
    checkNear(pose[0].position.x, 1)
    checkNear(pose[1].position.x, 1)
    checkNear(pose[1].position.y, 1)
    checkNear(pose[2].position.x, 1, 2.0e-4)
    checkNear(pose[2].position.y, 0, 2.0e-4)

  test "joint scaling changes animated translations":
    let animation = testAnimation(looping = false)
    defer: animation.close()
    animation.scaleJoints(2)
    let pose = animation.sampleLocalPose(1)
    checkNear(pose[0].position.x, 2)
    checkNear(pose[1].position.y, 2)
    checkNear(pose[2].position.y, 1)
    expect ValueError:
      animation.scaleJoints(0)

  test "sampled model poses feed skeleton mapping":
    let source = animationSkeleton()
    let target = skeletonDefinition(@[
      skeletonJoint("root", -1, skeletonTransform(vec3(0, 0, 0))),
      skeletonJoint("arm", 0, skeletonTransform(vec3(0, 1, 0))),
      skeletonJoint("hand", 1, skeletonTransform(vec3(0, 1, 0))),
      skeletonJoint("finger", 2, skeletonTransform(vec3(0, 0.25, 0)))])
    let animation = testAnimation(looping = false)
    let mapper = newSkeletonMapper(source, target)
    defer:
      mapper.close()
      animation.close()
    let mapped = mapper.mappedPose(animation.sampleModelPose(1), @[
      skeletonTransform(vec3(0, 0, 0)),
      skeletonTransform(vec3(0, 1, 0)),
      skeletonTransform(vec3(0, 1, 0)),
      skeletonTransform(vec3(0, 0.25, 0))])
    check mapped.len == 4
    checkNear(mapped[2].position.x, 1, 2.0e-3)
    checkNear(mapped[2].position.y, 0, 2.0e-3)

  test "animation samples drive a matching ragdoll":
    let world = newWorld()
    defer: world.close()
    world.setGravity(vec3(0, 0, 0))
    let ragdoll = world.addRagdoll(ragdollConfig(@[
      ragdollPart(
        "root", sphereShape(0.2), vec3(0, 2, 0), ragdollRootJoint()),
      ragdollPart(
        "arm", capsuleShape(0.3, 0.12), vec3(0, 2.6, 0),
        ragdollJoint(
          0, vec3(0, 2.3, 0), twistAxis = vec3(0, 1, 0),
          planeAxis = vec3(0, 0, 1), maxMotorTorque = 200))]))
    let skeleton = skeletonDefinition(@[
      skeletonJoint("root", -1, skeletonTransform(vec3(0, 2, 0))),
      skeletonJoint("arm", 0, skeletonTransform(vec3(0, 0.6, 0)))])
    let animation = newSkeletalAnimation(skeleton, [
      skeletalAnimationTrack("root", @[
        skeletalAnimationKeyframe(0, vec3(0, 2, 0)),
        skeletalAnimationKeyframe(1, vec3(0, 2, 0))]),
      skeletalAnimationTrack("arm", @[
        skeletalAnimationKeyframe(0, vec3(0, 0.6, 0)),
        skeletalAnimationKeyframe(
          1, vec3(0, 0.6, 0),
          quatFromAxisAngle(vec3(0, 0, 1), 0.4))])])
    defer: animation.close()
    ragdoll.driveMotors(animation, 0, 0.5, 0.5)
    check ragdoll.isActive
    check world.step(1.0'f32 / 60.0'f32) == {}

    let mismatch = newSkeletalAnimation(skeletonDefinition(@[
      skeletonJoint("root", -1, skeletonTransform(vec3(0, 2, 0))),
      skeletonJoint("other", 0, skeletonTransform(vec3(0, 0.6, 0)))]), [
      skeletalAnimationTrack("root", @[
        skeletalAnimationKeyframe(0, vec3(0, 2, 0))])])
    defer: mismatch.close()
    expect ValueError:
      discard mismatch.sampleRagdollLocalPose(ragdoll, 0)

  test "native binary round trip preserves animation behavior":
    let original = testAnimation(looping = false)
    original.scaleJoints(1.5)
    let data = original.serialize()
    check data.len > 28
    let restored = restoreSkeletalAnimation(animationSkeleton(), data)
    defer:
      restored.close()
      original.close()
    check restored.jointCount == 3
    check restored.trackCount == 2
    check restored.keyframeCount == 4
    check not restored.isLooping
    checkNear(restored.duration, original.duration)
    let expected = original.sampleLocalPose(0.5)
    let actual = restored.sampleLocalPose(0.5)
    for index in 0 ..< expected.len:
      checkNear(actual[index].position.x, expected[index].position.x)
      checkNear(actual[index].position.y, expected[index].position.y)
      checkNear(actual[index].rotation.z, expected[index].rotation.z)
      checkNear(actual[index].rotation.w, expected[index].rotation.w)

    let secondData = restored.serialize()
    let second = restoreSkeletalAnimation(animationSkeleton(), secondData)
    defer: second.close()
    checkNear(second.sampleModelPose(1)[2].position.x,
      restored.sampleModelPose(1)[2].position.x)

  test "binary restore rejects damaged data and skeleton mismatches":
    let animation = testAnimation()
    let data = animation.serialize()
    animation.close()
    expect JoltError:
      discard animation.serialize()
    expect ValueError:
      discard restoreSkeletalAnimation(animationSkeleton(), data[0 .. 10])

    var corrupt = data[0 .. ^1]
    corrupt[^1] = corrupt[^1] xor 0x80'u8
    expect ValueError:
      discard restoreSkeletalAnimation(animationSkeleton(), corrupt)

    var wrongVersion = data[0 .. ^1]
    wrongVersion[8] = 2
    expect ValueError:
      discard restoreSkeletalAnimation(animationSkeleton(), wrongVersion)

    let mismatchedSkeleton = skeletonDefinition(@[
      skeletonJoint("root", -1, skeletonTransform(vec3(0, 0, 0))),
      skeletonJoint("other", 0, skeletonTransform(vec3(0, 1, 0))),
      skeletonJoint("hand", 1, skeletonTransform(vec3(0, 1, 0)))])
    expect ValueError:
      discard restoreSkeletalAnimation(mismatchedSkeleton, data)

  test "invalid resources, samples and mismatched ragdolls are rejected":
    expect ValueError:
      discard newSkeletalAnimation(animationSkeleton(), [])
    expect ValueError:
      discard newSkeletalAnimation(animationSkeleton(), [
        skeletalAnimationTrack("missing", @[
          skeletalAnimationKeyframe(0, vec3(0, 0, 0))])])
    expect ValueError:
      discard newSkeletalAnimation(animationSkeleton(), [
        skeletalAnimationTrack("root", @[
          skeletalAnimationKeyframe(1, vec3(0, 0, 0)),
          skeletalAnimationKeyframe(1, vec3(1, 0, 0))])])

    let animation = testAnimation()
    expect ValueError:
      discard animation.sampleLocalPose(-0.1)
    animation.close()
    animation.close()
    check not animation.isAlive
    expect JoltError:
      discard animation.duration
