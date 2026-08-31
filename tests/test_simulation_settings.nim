import std/[math, unittest]
import jolt

proc smallWorldConfig(): WorldConfig =
  result = defaultWorldConfig()
  result.maxBodies = 64
  result.maxBodyPairs = 256
  result.maxContactConstraints = 256
  result.numThreads = 1

suite "Jolt simulation settings":
  test "Jolt defaults match the high-level value":
    let world = newWorld(smallWorldConfig())
    defer: world.close()
    check world.simulationSettings == defaultSimulationSettings()

  test "all settings round trip through the native world":
    let world = newWorld(smallWorldConfig())
    defer: world.close()
    var settings = defaultSimulationSettings()
    settings.maxInFlightBodyPairs = 4_096
    settings.stepListenersBatchSize = 4
    settings.stepListenerBatchesPerJob = 3
    settings.baumgarte = 0.3
    settings.speculativeContactDistance = 0.03
    settings.penetrationSlop = 0.01
    settings.linearCastThreshold = 0.6
    settings.linearCastMaxPenetration = 0.15
    settings.manifoldTolerance = 0.002
    settings.maxPenetrationDistance = 0.1
    settings.bodyPairCacheMaxDeltaPositionSquared = 2.0e-6
    settings.bodyPairCacheCosMaxDeltaRotationDiv2 = 0.99
    settings.contactNormalCosMaxDeltaRotation = 0.98
    settings.contactPointPreserveLambdaMaxDistanceSquared = 2.0e-4
    settings.internalEdgeRemovalVertexToleranceSquared = 2.0e-8
    settings.numVelocitySteps = 12
    settings.numPositionSteps = 4
    settings.minVelocityForRestitution = 0.5
    settings.timeBeforeSleep = 0.75
    settings.pointVelocitySleepThreshold = 0.02
    settings.deterministicSimulation = false
    settings.constraintWarmStart = false
    settings.useBodyPairContactCache = false
    settings.useManifoldReduction = false
    settings.useLargeIslandSplitter = false
    settings.allowSleeping = false
    settings.checkActiveEdges = false

    world.setSimulationSettings(settings)
    check world.simulationSettings == settings

    var snapshot = world.simulationSettings
    snapshot.baumgarte = 0.4
    check world.simulationSettings.baumgarte == settings.baumgarte

  test "world creation applies settings before use":
    var settings = defaultSimulationSettings()
    settings.numVelocitySteps = 14
    settings.numPositionSteps = 5
    settings.allowSleeping = false
    let world = newWorld(smallWorldConfig(), settings)
    defer: world.close()
    check world.simulationSettings == settings
    discard world.addDynamicBody(
      sphereShape(0.5), vec3(0, 2, 0), config = defaultBodyConfig())
    check world.step(1.0'f32 / 60) == {}

    let convenienceWorld = newWorld(settings)
    defer: convenienceWorld.close()
    check convenienceWorld.simulationSettings == settings

  test "invalid settings are rejected without changing the world":
    let world = newWorld(smallWorldConfig())
    defer: world.close()
    let original = world.simulationSettings

    var settings = original
    settings.numVelocitySteps = 1
    expect ValueError:
      world.setSimulationSettings(settings)
    check world.simulationSettings == original

    settings = original
    settings.bodyPairCacheCosMaxDeltaRotationDiv2 = 2
    expect ValueError:
      world.setSimulationSettings(settings)

    settings = original
    settings.manifoldTolerance = -0.1
    expect ValueError:
      discard newWorld(smallWorldConfig(), settings)

    settings = original
    settings.baumgarte = NaN.float32
    expect ValueError:
      world.setSimulationSettings(settings)

  test "closed worlds reject settings access":
    let world = newWorld(smallWorldConfig())
    world.close()
    expect JoltError:
      discard world.simulationSettings
    expect JoltError:
      world.setSimulationSettings(defaultSimulationSettings())
