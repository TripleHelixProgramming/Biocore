# I/O Architecture: Background Threading and Performance

This document explains the performance architecture of the robot code: why sensor reads happen on background threads, how Phoenix 6 signals are handled, and patterns used to keep the main robot loop fast.

---

## The problem: blocking I/O on the main loop

The WPILib robot loop runs every 20ms. If any I/O operation blocks waiting for a response, it eats directly into that budget. REV Spark motor controllers use blocking CAN calls — `encoder.getPosition()` waits for a CAN frame to return, which can take several milliseconds per device. With multiple Sparks, this compounds quickly. Similarly, PhotonVision uses NetworkTables, which has variable latency.

The solution is to move all blocking reads to background threads and have the main loop read cached values (a memory operation taking nanoseconds).

---

## Background threads

### SparkOdometryThread

A singleton background thread (via WPILib's `Notifier`) running at 100Hz. Each registered `SparkInputs` wraps a `SparkBase` device and caches position, velocity, applied voltage, output current, connection status, and timestamp.

`update()` reads all values atomically — it only updates the cache if all reads succeeded, so you never see partially-updated data. All cached fields are `volatile` for cross-thread visibility.

**How to use it:** In any Spark-based IO constructor, register with:
```java
SparkOdometryThread.getInstance().registerSpark(spark);
```

Then in `updateInputs()`, read from `sparkInputs` instead of calling the device directly:
```java
inputs.position = new Rotation2d(sparkInputs.getPosition());
inputs.velocityRadPerSec = sparkInputs.getVelocity();
inputs.appliedVolts = sparkInputs.getAppliedVolts();
inputs.currentAmps = sparkInputs.getOutputCurrent();
inputs.connected = connectedDebounce.calculate(sparkInputs.isConnected());
```

### CanandgyroThread

Same pattern as `SparkOdometryThread` but for the Redux Canandgyro. Runs at 100Hz and caches `isConnected()`, `isCalibrating()`, `getYaw()`, and `getAngularVelocityYaw()`.

Note: high-frequency odometry samples from the gyro still flow through `PhoenixOdometryThread` (see below) — `CanandgyroThread` only handles the periodic `updateInputs()` values.

### VisionThread

Runs at 50Hz. Each registered `VisionIO` is polled on the background thread. Uses an **immutable snapshot** pattern: the background thread creates a new `VisionIOInputsSnapshot` object each update with cloned arrays. The main thread reads a volatile reference to this snapshot, ensuring it never reads partially-updated data.

### PhoenixOdometryThread (pre-existing)

The swerve drive odometry uses Phoenix 6's `PhoenixOdometryThread` at 250Hz for TalonFX motors. This thread is independent of the above and handles high-frequency position sampling for accurate odometry integration.

---

## Phoenix 6: relying on auto-refresh

Phoenix 6 automatically updates status signals at their configured frequency. The main loop does not call `BaseStatusSignal.refreshAll()` — it reads cached values that Phoenix 6 updates in the background. This is the correct usage pattern for Phoenix 6 and avoids the blocking CAN latency that synchronous `refreshAll()` calls introduce.

Each signal's update frequency is configured once in the IO constructor (e.g., 50Hz for telemetry signals, 250Hz for odometry signals via `PhoenixOdometryThread`). The main loop just reads `.getValueAsDouble()`, which returns the most recently received value.

For connection status, check `.getStatus().isOK()` after a refresh only when you explicitly need to validate freshness:
```java
var status = BaseStatusSignal.refreshAll(signal1, signal2, signal3);
if (!status.isOK()) return;
inputs.value = signal1.getValueAsDouble();
```

---

## Deferred logging in Launcher

`Logger.recordOutput()` calls can take 10–30ms due to serialization overhead. The `aim()` method is called frequently during targeting and must be fast. To avoid logging overhead in the hot path, `aim()` populates a set of cached fields instead of calling Logger directly. `periodic()` then flushes these cached values via `logCachedAimData()` once per robot loop.

The same principle applies generally: IO `updateInputs()` methods should not call `Logger.recordOutput()` directly. Instead, populate the inputs struct and let the subsystem's `periodic()` handle logging via `Logger.processInputs()`.

---

## Pre-allocated constants

`Units.X.in(Y)` involves virtual method calls and some overhead. Constants used every loop iteration are pre-computed once at class load time:

```java
public static final Distance wheelRadius = Inches.of(2);
public static final double wheelRadiusMeters = wheelRadius.in(Meters);  // pre-computed
```

`Module.java` uses `wheelRadiusMeters` directly. The same pattern is used for vision tolerance constants in `VisionConstants`.

---

## Profiling infrastructure

Timing instrumentation exists throughout the codebase. Each subsystem logs a timing breakdown when its `periodic()` exceeds a threshold:

| Subsystem | Threshold | Metrics |
|-----------|-----------|---------|
| `Robot.java` | >20ms | scheduler, gameState |
| `Drive.java` | >5ms | lock, gyroUpdate, gyroLog, modules, disabled, odometry |
| `Module.java` | >2ms | updateInputs, log, rest |
| `Vision.java` | >5ms | snapshot, processInputs, cameraLoop, consumer, summaryLog |
| `Launcher.java` | >3ms | update, log, aimLog, ballistics |
| `Launcher.aim()` | >500μs | v0nom, baseSpeeds, v0replan, setPos, rest |
| `Intake.java` | >2ms | update, log |
| `Feeder.java` | >2ms | spindexer, kicker, spindexerLog, kickerLog |
| `Hopper.java` | >2ms | update, log |

Overruns print to the console:
```
[Drive] lock=0ms gyroUpdate=2ms modules=5ms odometry=8ms total=16ms
```

This makes it straightforward to identify which subsystem is causing a loop overrun and which phase within it.

---

## Vision logging throttle

`Logger.processInputs()` serialization is expensive for the vision subsystem, which serializes `PoseObservation` arrays from multiple cameras each cycle. The constant `kLoggingDivisor` in `VisionConstants` controls how often vision inputs are logged:

- `kLoggingDivisor = 1` (default): log every cycle — full data for AdvantageKit replay
- `kLoggingDivisor = 2`: log every other cycle — ~50% reduction in vision logging overhead

Increasing this reduces replay granularity but is useful when the robot loop is under pressure.

---

## Caveats

### Data latency

| Thread | Update rate | Max staleness |
|--------|-------------|---------------|
| SparkOdometryThread | 100Hz | 10ms |
| CanandgyroThread | 100Hz | 10ms |
| VisionThread | 50Hz | 20ms |

For hood/turret control and vision pose estimation, 10–20ms latency is negligible. If you add a new Spark subsystem that needs accurate readings immediately at boot (e.g., for a homing sequence), add an explicit "first reading received" flag — the initial cached values are zero while `isConnected()` is optimistically true.

### Non-atomic multi-field reads

Each Spark cache field is `volatile`, but reading multiple fields is not atomic. Position and velocity could come from different update cycles (at most 10ms apart). For hood and turret this is negligible. If you need atomicity, use a snapshot pattern like `VisionThread` does.

### Stale data on CAN errors

If CAN communication fails persistently, `isConnected()` returns false but `getPosition()` returns the last good value. This is intentional — stale data is better than garbage. Always check `isConnected()` if you need to know data freshness.

### Simulation

Simulation uses `HoodIOSim`, `TurretIOSim`, etc., which read directly from physics simulation without background threading. This is correct — there is no real CAN latency in simulation.
