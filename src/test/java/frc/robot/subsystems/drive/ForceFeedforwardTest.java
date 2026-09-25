// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.WHEEL_RADIUS_METERS;
import static org.junit.jupiter.api.Assertions.*;

import choreo.trajectory.SwerveSample;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/** Checks how Choreo module forces become drive feedforward torques. */
class ForceFeedforwardTest {
  private static final double TOLERANCE = 1e-9;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize();
  }

  @Test
  void choreoModuleOrderIsRemapped() {
    // Choreo orders module forces FL, BL, BR, FR
    var sample = sample(0.0, new double[] {1, 2, 3, 4}, new double[] {0, 0, 0, 0});
    Translation2d[] forces = Drive.robotRelativeModuleForces(sample, Rotation2d.ZERO);
    // Our modules are FL, FR, BL, BR
    assertEquals(1, forces[0].getX(), TOLERANCE, "FL");
    assertEquals(4, forces[1].getX(), TOLERANCE, "FR");
    assertEquals(2, forces[2].getX(), TOLERANCE, "BL");
    assertEquals(3, forces[3].getX(), TOLERANCE, "BR");
  }

  @Test
  void fieldForcesRotateIntoTheRobotFrame() {
    // A robot facing field +y has field +x on its right, which is robot -y
    var sample = sample(Math.PI / 2, new double[] {10, 10, 10, 10}, new double[] {0, 0, 0, 0});
    for (Translation2d force : Drive.robotRelativeModuleForces(sample, Rotation2d.CCW_PI_2)) {
      assertEquals(0, force.getX(), TOLERANCE);
      assertEquals(-10, force.getY(), TOLERANCE);
    }
  }

  @Test
  void onlyTheForceAlongTheWheelNeedsTorque() {
    var forward = new Translation2d(10, 0);
    assertEquals(10 * WHEEL_RADIUS_METERS, Module.wheelTorque(forward, Rotation2d.ZERO), TOLERANCE);
    assertEquals(0, Module.wheelTorque(forward, Rotation2d.CCW_PI_2), TOLERANCE);
    assertEquals(
        10 * Math.cos(Math.PI / 4) * WHEEL_RADIUS_METERS,
        Module.wheelTorque(forward, Rotation2d.fromDegrees(45)),
        TOLERANCE);
    assertEquals(-10 * WHEEL_RADIUS_METERS, Module.wheelTorque(forward, Rotation2d.PI), TOLERANCE);
  }

  /**
   * A module facing backward is optimized to drive in reverse. The torque must reverse with the
   * velocity, so both still push the robot forward.
   */
  @Test
  void aReversedModuleReversesItsTorque() {
    var io = new RecordingModuleIO(Rotation2d.PI);
    var module = new Module(io, "ReversedFeedforwardTestModule");
    module.periodic();

    module.runSetpoint(new SwerveModuleVelocity(1.0, Rotation2d.ZERO), new Translation2d(10, 0));

    assertEquals(-1.0 / WHEEL_RADIUS_METERS, io.velocityRadPerSec, TOLERANCE);
    assertEquals(-10 * WHEEL_RADIUS_METERS, io.wheelTorqueNm, TOLERANCE);
  }

  @Test
  void runningWithoutForcesAddsNoFeedforward() {
    var io = new RecordingModuleIO(Rotation2d.ZERO);
    var module = new Module(io, "NoForceFeedforwardTestModule");
    module.periodic();

    module.runSetpoint(new SwerveModuleVelocity(1.0, Rotation2d.ZERO));

    assertEquals(0.0, io.wheelTorqueNm, TOLERANCE);
  }

  private static SwerveSample sample(double heading, double[] fx, double[] fy) {
    return new SwerveSample(0, 0, 0, heading, 0, 0, 0, 0, 0, 0, fx, fy);
  }

  /** Reports a fixed turn angle and records the drive velocity and feedforward it receives. */
  private static class RecordingModuleIO implements ModuleIO {
    final Rotation2d turnPosition;
    double velocityRadPerSec = Double.NaN;
    double wheelTorqueNm = Double.NaN;

    RecordingModuleIO(Rotation2d turnPosition) {
      this.turnPosition = turnPosition;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.turnPosition = turnPosition;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double feedforwardWheelTorqueNm) {
      this.velocityRadPerSec = velocityRadPerSec;
      this.wheelTorqueNm = feedforwardWheelTorqueNm;
    }
  }
}
