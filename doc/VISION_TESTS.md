# VisionFilter Architecture

This document explains how vision observations are filtered and scored before being fed to the pose estimator.

## Overview

The scoring and filtering logic lives in `VisionFilter.java`, separated from `Vision.java`. This separation keeps `Vision.java` focused on IO and coordination, and makes the scoring logic unit-testable in isolation (no hardware dependencies).

```
Vision.java          → IO, logging, pose estimator integration
VisionFilter.java    → Scoring tests, correlation boost, filtering logic
VisionConstants.java → Tunable thresholds
```

---

## Weighted test scoring

Each `VisionFilter.Test` has a weight determining its influence on the final score:

```java
withinBoundaries(1.0),    // Critical
moreThanZeroTags(1.0),    // Critical
velocityConsistency(0.9), // Important
unambiguous(0.8),         // Important
pitchError(0.7),          // Moderate
rollError(0.7),           // Moderate
heightError(0.7),         // Moderate
distanceToTags(0.5),      // Quality hint
```

Scores are combined using **weighted geometric mean**:

```
totalScore = (score1^w1 × score2^w2 × ...)^(1/sumOfWeights)
```

This gives the average score per test, weighted by importance. An observation must score at or above `minScore` (currently `0.6`) to be accepted.

---

## TestContext

Tests receive a `TestContext` with the observation plus camera state:

```java
testContext
    .observation(observation)
    .cameraIndex(cameraIndex)
    .lastAcceptedPose(lastAcceptedPose[cameraIndex])
    .lastAcceptedTimestamp(lastAcceptedTimestamp[cameraIndex]);

for (var test : enabledTests) {
    testResults.put(test, test.test(testContext));
}
```

---

## Velocity consistency test

Compares each observation to the last accepted pose from the same camera. If the implied velocity exceeds `maxReasonableVelocityMps` (1.5× max drivetrain speed), the observation is penalized using a sigmoid for smooth falloff rather than a hard cutoff.

When velocity can't be verified (no previous accepted pose, or the last accepted pose is older than `velocityCheckTimeoutSeconds`), the test returns `velocityUncertainScore` (currently `0.7`) rather than a perfect `1.0`. This is critical: returning `1.0` for unverified poses was the root cause of cascading bad-pose acceptance — a bad pose would be accepted, become the velocity reference, and make the next bad pose from the same camera appear consistent.

**Why not use actual odometry velocity?** Circular dependency (velocity estimate uses vision), and the robot might be carried (wheels not moving but robot is).

---

## Cross-camera correlation boost

When multiple cameras report similar poses at similar times, their scores are boosted.

**Algorithm:**
1. Each observation starts in its own cluster
2. Find observations from different cameras that agree (within `correlationTimeWindowSeconds` = 50ms and `correlationPoseThresholdMeters` = 15cm)
3. Merge clusters when observations agree
4. If a cluster has a **majority** of reporting cameras, boost those scores by `correlationBoostFactor` (1.3×)

**Why majority?** If front cameras agree on pose A and back cameras agree on pose B, but A ≠ B, there's a conflict — neither should be boosted.

| Scenario | Result |
|----------|--------|
| 4 agree | All boosted |
| 3 agree, 1 differs | The 3 boosted |
| 2v2 split | Nobody boosted |

---

## Score threshold rationale

With `minScore = 0.6` and `velocityUncertainScore = 0.7`, the effective score ranges are:

| Observation type | Typical score | Result |
|-----------------|---------------|--------|
| Bad first-in-a-while pose | ~0.58 | Rejected |
| Good single-tag, no history | ~0.71 | Accepted |
| Good single-tag, with history | ~0.75 | Accepted |
| Multi-tag | ~0.86–0.93 | Accepted |
| With correlation boost (1.3×) | ~0.92 | Accepted |

The gap between bad-pose scores (~0.58) and good single-tag scores (~0.71) is what the threshold exploits. See [VISION_FILTER_TUNING.md](VISION_FILTER_TUNING.md) for the analysis behind these values.

---

## Configuration (`VisionConstants.java`)

```java
// Acceptance threshold
minScore = 0.6

// Velocity consistency
velocityCheckTimeoutSeconds = 0.5
velocityUncertainScore = 0.7
maxReasonableVelocityMps = drivetrainSpeedLimit * 1.5

// Cross-camera correlation
correlationTimeWindowSeconds = 0.050
correlationPoseThresholdMeters = 0.15
correlationBoostFactor = 1.3
```

**Tuning guidance:**
- Too aggressive (rejecting good observations): decrease `minScore` or increase `velocityUncertainScore`
- Too permissive (accepting bad poses): increase `minScore`, decrease `velocityUncertainScore`, or decrease `velocityCheckTimeoutSeconds`

After changing these values, check `Vision/Summary/ObservationScore` in the log to verify the score distribution matches expectations.

---

## Unit test suite

```bash
./gradlew test --tests "frc.robot.subsystems.vision.VisionFilterTest"
```

| Category | Coverage |
|----------|----------|
| `Test.unambiguous` | Single/multi-tag ambiguity handling |
| `Test.pitchError` | Pitch tolerance, symmetry |
| `Test.rollError` | Roll tolerance |
| `Test.heightError` | Height tolerance, negative values |
| `Test.withinBoundaries` | Field boundary enforcement |
| `Test.moreThanZeroTags` | Zero-tag rejection |
| `Test.distanceToTags` | Distance penalty curve |
| `Test.velocityConsistency` | Impossible velocity detection |
| `scoreObservation` | Weighted geometric mean, test selection |
| `applyCorrelationBoost` | Majority rule, 2v2 splits, boost limits |
| `normalizedSigmoid` | Edge cases, steepness behavior |
| Integration | End-to-end scoring scenarios |
