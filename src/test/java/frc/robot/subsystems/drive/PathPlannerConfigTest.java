// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Checks that PathPlanner can plan a drivable trajectory with the robot's config. */
class PathPlannerConfigTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize();
  }

  /**
   * PathPlanner models drive losses as the torque needed to hold ModuleConfig's top speed. A top
   * speed the motor cannot reach within the current limit leaves no torque to accelerate, and every
   * trajectory comes out with zero duration.
   */
  @Test
  void straightPathMovesTheRobot() {
    var start = new Pose2d(1, 1, Rotation2d.ZERO);
    var end = new Pose2d(4, 1, Rotation2d.ZERO);
    var path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(start, end),
            PATH_FOLLOWING_CONSTRAINTS,
            null,
            new GoalEndState(0.0, Rotation2d.ZERO));
    PathPlannerTrajectory trajectory =
        path.generateTrajectory(new ChassisVelocities(), Rotation2d.ZERO, PP_CONFIG);

    double peakVelocity = 0.0;
    for (var state : trajectory.getStates()) {
      peakVelocity = Math.max(peakVelocity, state.linearVelocity);
    }
    double totalTime = trajectory.getTotalTimeSeconds();

    assertTrue(Double.isFinite(totalTime) && totalTime > 0, "total time " + totalTime);
    assertTrue(peakVelocity > 0, "peak velocity " + peakVelocity);
    assertTrue(
        peakVelocity <= PLANNED_MODULE_SPEED_LIMIT.in(MetersPerSecond) + 1e-6,
        "peak velocity " + peakVelocity + " exceeds the planned speed limit");
  }
}
