# Vision Filter Tuning: Score Threshold and Velocity Uncertainty

This document explains why `minScore = 0.6` and `velocityUncertainScore = 0.7` were chosen, and what the scoring distribution looks like in practice.

## The problem with a low threshold

With `minScore = 0.02` (the previous value), the vision filter was not actually filtering. The only observations rejected were ones off the field or with zero tags. The scoring system computed quality scores but the acceptance bar was so low that scores only affected standard deviations — they never caused a rejection. Bad poses scored 0.60–0.66 and sailed through.

The score distribution from a typical session with the old threshold:

```
0.90-1.00 : #################### 20.9%  multi-tag
0.75-0.90 : ############################################# 40.7%  good single-tag
0.70-0.75 : ######################## 23.5%  decent single-tag
0.60-0.70 : ############ 12.2%  BAD AND GOOD OVERLAP HERE
< 0.60    : ## 2.8%
```

85% of observations scored 0.70+. The 12% zone from 0.60–0.70 is where bad poses and marginal-but-legitimate observations overlap.

## The velocity uncertainty loophole

PhotonVision's PnP solver occasionally returns the wrong solution for single-tag observations — a reflected pose, often meters from reality. The `velocityConsistency` test should catch these, but it had a loophole: when a camera hadn't reported in over 0.5s, it returned `1.0` (perfect pass) instead of expressing uncertainty.

The cascade that resulted:
1. Camera goes quiet for >0.5s
2. Bad pose arrives → velocity check returns 1.0 (no history) → observation accepted
3. Bad pose becomes the velocity reference for that camera
4. Next bad pose from the same camera appears consistent with the bad reference → accepted again
5. Eventually a good pose breaks the cycle (300–500ms later)

The fix: return `velocityUncertainScore = 0.7` instead of `1.0` when velocity can't be verified.

```java
// Before: no-history and timeout both gave a free pass
if (ctx.lastAcceptedPose() == null) return 1.0;
if (dt > velocityCheckTimeoutSeconds) return 1.0;

// After: express uncertainty instead of confidence
if (ctx.lastAcceptedPose() == null) return velocityUncertainScore;  // 0.7
if (dt > velocityCheckTimeoutSeconds) return velocityUncertainScore; // 0.7
```

## How the threshold and uncertainty penalty work together

With `velocityUncertainScore = 0.7`, a bad first-in-a-while pose drops from ~0.63 to ~0.58. With `minScore = 0.6`, it is now rejected. Good single-tag observations with no history drop from ~0.75 to ~0.71 — still above the threshold.

| Observation type | Score before | Score after | Result |
|-----------------|-------------|-------------|--------|
| Bad first-in-a-while pose | ~0.63 | ~0.58 | **Rejected** |
| Good single-tag, no history | ~0.75 | ~0.71 | Accepted |
| Good single-tag, with history | ~0.75 | ~0.75 | Accepted |
| Multi-tag | ~0.90 | ~0.86 | Accepted |

The gap between ~0.58 (bad) and ~0.71 (good) provides enough margin for the 0.6 threshold to be effective without being overly aggressive.

## Why this is safe to tune aggressively

With the current camera configuration, good observations arrive multiple times per second. Rejecting a marginally-scored observation that would have been replaced moments later does less damage than accepting a bad pose that corrupts the estimator for 300–500ms. The filter can afford to be selective.

## Checking the current score distribution

After any change to `VisionConstants`, verify the score distribution in AdvantageScope:

1. Record a session moving the robot around the field.
2. Plot `Vision/Summary/ObservationScore` — the distribution should be centered around 0.70–0.75 for single-tag observations and 0.85+ for multi-tag.
3. Check `Vision/Summary/RobotPosesAccepted` for teleportation events (they appear as jumps in the accepted pose trajectory).

If you see a high rejection rate with no teleportations, the threshold may be too tight. If you see teleportations, the threshold or `velocityUncertainScore` needs to be more aggressive.
