// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.autoselect;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class PoseSeekErrorTest {
  private static final double EPS = 1e-9;

  private static Pose2d pose(double xMeters, double yMeters, double degrees) {
    return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(degrees));
  }

  @Test
  void targetAheadMeansMoveForward() {
    var error = PoseSeekError.between(pose(1, 1, 0), pose(1.5, 1, 0));
    assertEquals(50, error.forwardCm(), EPS);
    assertEquals(0, error.leftCm(), EPS);
    assertEquals(0, error.headingDeg(), EPS);
  }

  @Test
  void targetToTheLeftMeansMoveLeft() {
    var error = PoseSeekError.between(pose(1, 1, 0), pose(1, 1.2, 0));
    assertEquals(0, error.forwardCm(), EPS);
    assertEquals(20, error.leftCm(), EPS);
  }

  @Test
  void ccwTargetHeadingIsPositive() {
    var error = PoseSeekError.between(pose(1, 1, 0), pose(1, 1, 10));
    assertEquals(10, error.headingDeg(), 1e-6);
  }

  @Test
  void headingWrapsAcross180() {
    var error = PoseSeekError.between(pose(1, 1, 179), pose(1, 1, -179));
    assertEquals(2, error.headingDeg(), 1e-6);
  }

  @Test
  void offsetIsExpressedInTheRobotFrame() {
    // Robot faces +Y, so a target further along field +X is to the robot's right
    var error = PoseSeekError.between(pose(1, 1, 90), pose(1.3, 1, 90));
    assertEquals(0, error.forwardCm(), 1e-6);
    assertEquals(-30, error.leftCm(), 1e-6);
  }

  @Test
  void offsetIsExpressedInTheRobotFrameAtAnAngle() {
    // Robot faces 45 degrees; a target 1 m along its heading is straight ahead
    var d = Math.sqrt(0.5);
    var error = PoseSeekError.between(pose(1, 1, 45), pose(1 + d, 1 + d, 45));
    assertEquals(100, error.forwardCm(), 1e-6);
    assertEquals(0, error.leftCm(), 1e-6);
  }

  @Test
  void distanceIsStraightLine() {
    assertEquals(50, new PoseSeekError(30, -40, 0).distanceCm(), EPS);
  }

  @Test
  void toleranceChecksAreStrict() {
    var inside = new PoseSeekError(4.9, -5.9, -2.9);
    assertTrue(inside.forwardOk(5));
    assertTrue(inside.leftOk(6));
    assertTrue(inside.headingOk(3));

    var atTolerance = new PoseSeekError(-5, 6, 3);
    assertFalse(atTolerance.forwardOk(5));
    assertFalse(atTolerance.leftOk(6));
    assertFalse(atTolerance.headingOk(3));
  }
}
