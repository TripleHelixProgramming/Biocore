// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.generated.ChoreoTraj;
import frc.robot.generated.ChoreoVars;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;

/** Checks that every generated ChoreoTraj constant loads from the deploy directory. */
class ChoreoTrajTest {
  // ChoreoTraj rounds its poses to 5 decimal places
  private static final double GENERATED_TOLERANCE = 1e-5;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize();
  }

  @Test
  void everyTrajectoryLoadsAndMatchesItsConstant() {
    ChoreoTraj.ALL_TRAJECTORIES.forEach(
        (key, constant) -> {
          Trajectory<SwerveSample> trajectory = load(constant);
          assertPoseEquals(
              constant.initialPoseBlue(), trajectory.getInitialPose(false).orElseThrow(), key);
          assertPoseEquals(
              constant.endPoseBlue(), trajectory.getFinalPose(false).orElseThrow(), key);
          assertEquals(constant.totalTimeSecs(), trajectory.getTotalTime(), 1e-9, key);
        });
  }

  /** Red and blue paths are drawn separately, so each must start on its own alliance's side. */
  @Test
  void pathsStartOnTheirAlliancesSide() {
    double centerX = ChoreoVars.FieldCenterX.baseUnitMagnitude();
    ChoreoTraj.ALL_TRAJECTORIES.forEach(
        (key, constant) -> {
          double startX = constant.initialPoseBlue().getX();
          if (key.startsWith("Red")) {
            assertTrue(startX > centerX, key + " starts on the blue side");
          } else if (key.startsWith("Blue")) {
            assertTrue(startX < centerX, key + " starts on the red side");
          } else {
            fail(key + " is not named for an alliance");
          }
        });
  }

  private static Trajectory<SwerveSample> load(ChoreoTraj constant) {
    Trajectory<SwerveSample> whole =
        Choreo.<SwerveSample>loadTrajectory(constant.name())
            .orElseThrow(() -> new AssertionError(constant.name() + ".traj did not load"));
    if (constant.segment().isEmpty()) return whole;
    int segment = constant.segment().getAsInt();
    return whole
        .getSplit(segment)
        .orElseThrow(() -> new AssertionError(constant.name() + " has no segment " + segment));
  }

  private static void assertPoseEquals(Pose2d expected, Pose2d actual, String message) {
    assertEquals(expected.getX(), actual.getX(), GENERATED_TOLERANCE, message);
    assertEquals(expected.getY(), actual.getY(), GENERATED_TOLERANCE, message);
    assertEquals(
        0.0,
        actual.getRotation().minus(expected.getRotation()).getRadians(),
        GENERATED_TOLERANCE,
        message);
  }
}
