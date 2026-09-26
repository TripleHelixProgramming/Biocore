// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.autoselect;

import static org.wpilib.units.Units.Centimeters;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.util.MathUtil;

/**
 * How far the robot must move to reach a target pose, in the robot's own frame.
 *
 * @param forwardCm distance to move forward (negative = backward)
 * @param leftCm distance to move left (negative = right)
 * @param headingDeg rotation to make, CCW positive, wrapped to ±180 degrees
 */
public record PoseSeekError(double forwardCm, double leftCm, double headingDeg) {

  /**
   * Computes the error from the robot's current pose to a target pose.
   *
   * @param currentPose the robot's current pose
   * @param targetPose the target pose to reach
   * @return the robot-relative error
   */
  public static PoseSeekError between(Pose2d currentPose, Pose2d targetPose) {
    // Pose2d.minus expresses the target in the current pose's frame, so this is robot-relative
    var delta = targetPose.minus(currentPose);

    var heading = MathUtil.inputModulus(delta.getRotation().getDegrees(), -180, 180);

    return new PoseSeekError(
        delta.getTranslation().getMeasureX().in(Centimeters),
        delta.getTranslation().getMeasureY().in(Centimeters),
        heading);
  }

  /** Returns the straight-line distance to the target, in centimeters. */
  public double distanceCm() {
    return Math.hypot(forwardCm, leftCm);
  }

  /** Returns true when the forward error is strictly within the tolerance. */
  public boolean forwardOk(double toleranceCm) {
    return Math.abs(forwardCm) < toleranceCm;
  }

  /** Returns true when the left error is strictly within the tolerance. */
  public boolean leftOk(double toleranceCm) {
    return Math.abs(leftCm) < toleranceCm;
  }

  /** Returns true when the heading error is strictly within the tolerance. */
  public boolean headingOk(double toleranceDeg) {
    return Math.abs(headingDeg) < toleranceDeg;
  }
}
