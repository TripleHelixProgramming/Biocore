# Drive Subsystem: Pose and Heading Architecture

This document explains how pose estimation and driver-relative heading are separated in the drive subsystem, and why.

---

## The core principle: one source of truth

`Drive` owns the robot's physical state. `DriveCommands` owns the human interface.

These two things used to be conflated: `Drive` held a `headingOffset` field that represented the driver's notion of "forward." This created two parallel rotations in the system — the pose estimator's rotation and the gyro-plus-offset rotation — and different parts of the code picked whichever one they wanted.

Now there is one rotation: `getPose().getRotation()`. `getHeading()` returns exactly that.

---

## What `Drive` provides

### `getPose()` and `getHeading()`

`getPose()` returns the full pose from the Kalman filter pose estimator. `getHeading()` returns `getPose().getRotation()`. They are the same rotation. There is no other rotation in `Drive`.

### `setPose(Pose2d pose)`

Resets the full pose estimator: position, heading, and all internal state. It calls `visionPose.resetPosition(rawGyroRotation, getModulePositions(), pose)`. This is used by auto routines at the start of a trajectory — both Choreo's `resetOdometry()` and PathPlanner's `AutoBuilder` call `setPose()`, so every auto starts with the robot's pose initialized to the trajectory's expected starting position.

### `addVisionMeasurement()`

On the first vision estimate received while the robot is disabled, the full pose is reset from vision (position and heading together). Subsequent vision measurements are fused normally through the Kalman filter.

### `wheelRadiusCharacterization()`

Uses `getRawGyroRotation()` directly, not `getHeading()`. This is intentional: wheel radius characterization is measuring a physical quantity (how far the wheels rotate per radian of robot rotation) and should not be affected by vision corrections mid-measurement. The raw gyro signal is stable and unaffected by the pose estimator.

---

## What `DriveCommands` provides

The driver's "forward" direction lives in `DriveCommands` as a static field:

```java
private static Rotation2d driverForwardDirection = Rotation2d.kZero;

public static void resetDriverForward(Drive drive) {
    driverForwardDirection = drive.getPose().getRotation();
}
```

A private helper incorporates the forward offset and the alliance flip into the heading used for joystick-to-field conversion:

```java
private static Rotation2d getDriverRelativeHeading(Drive drive, boolean fieldRotated) {
    Rotation2d heading = drive.getPose().getRotation().minus(driverForwardDirection);
    return fieldRotated ? heading.plus(Rotation2d.kPi) : heading;
}
```

The driver reset button calls `DriveCommands.resetDriverForward()`. `Drive` has no knowledge of this preference.

---

## Why trajectory following needs a consistent frame

`followTrajectory()` computes position error in the field frame, then converts the correction velocity from field-relative to robot-relative using `fromFieldRelativeSpeeds()`. That conversion needs to know which direction the robot is facing.

Both the error computation and the velocity conversion use `getPose().getRotation()`. If they used different rotations — which was the case before this refactor, where one path used the pose estimator and the other used `getHeading()` (gyro + offset) — corrections would be rotated into the wrong frame and the robot would drive in the wrong direction.

---

## Downstream impact

Code that interacts with drive should use:

- `getPose()` — for position and rotation in all field-relative computations
- `getHeading()` — equivalent to `getPose().getRotation()`, use either
- `getRawGyroRotation()` — only when you specifically need the raw unfiltered gyro (e.g., characterization)
- `DriveCommands.resetDriverForward()` — to reset the driver-relative heading offset

Do not access any rotation other than through these methods.
