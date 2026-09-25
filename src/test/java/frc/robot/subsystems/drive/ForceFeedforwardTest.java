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
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;

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
    var sample = sample(0.0, new double[] {1, 2, 3, 4}, new double[] {10, 20, 30, 40});
    Translation2d[] forces = Drive.robotRelativeModuleForces(sample, Rotation2d.ZERO);
    // Our modules are FL, FR, BL, BR
    assertEquals(new Translation2d(1, 10), forces[0], "FL");
    assertEquals(new Translation2d(4, 40), forces[1], "FR");
    assertEquals(new Translation2d(2, 20), forces[2], "BL");
    assertEquals(new Translation2d(3, 30), forces[3], "BR");
  }

  @Test
  void fieldForcesRotateIntoTheRobotFrame() {
    // A robot facing field +y has field +x on its right (robot -y) and field +y ahead (robot +x)
    var sample = sample(Math.PI / 2, new double[] {10, 10, 10, 10}, new double[] {5, 5, 5, 5});
    for (Translation2d force : Drive.robotRelativeModuleForces(sample, Rotation2d.CCW_PI_2)) {
      assertEquals(5, force.getX(), TOLERANCE);
      assertEquals(-10, force.getY(), TOLERANCE);
    }
  }

  /** The TalonFX feedforward is in amps for TorqueCurrentFOC and in volts for Voltage. */
  @Test
  void talonFeedforwardConvertsWheelTorqueToMotorUnits() {
    double gearRatio = 6.0;
    double wheelTorqueNm = 1.2;
    double motorTorqueNm = wheelTorqueNm / gearRatio;
    DCMotor motor = DriveConstants.DRIVE_GEARBOX;
    assertEquals(
        motorTorqueNm / motor.Kt,
        ModuleIOTalonFXBase.driveFeedforward(
            ClosedLoopOutputType.TorqueCurrentFOC, wheelTorqueNm, gearRatio),
        TOLERANCE);
    assertEquals(
        motorTorqueNm / motor.Kt * motor.R,
        ModuleIOTalonFXBase.driveFeedforward(
            ClosedLoopOutputType.Voltage, wheelTorqueNm, gearRatio),
        TOLERANCE);
  }

  /** With no velocity to hold, the sim applies exactly the voltage that makes the torque. */
  @Test
  void simFeedforwardAppliesTheVoltageForTheTorque() {
    var io = new ModuleIOSimWPI(DriveConstants.FRONT_LEFT);
    var inputs = new ModuleIO.ModuleIOInputs();
    double wheelTorqueNm = 1.2;
    double gearRatio = DriveConstants.FRONT_LEFT.DriveMotorGearRatio;
    DCMotor motor = DriveConstants.DRIVE_GEARBOX;

    io.setDriveVelocity(0.0, wheelTorqueNm);
    io.updateInputs(inputs);

    assertEquals(wheelTorqueNm / gearRatio / motor.Kt * motor.R, inputs.driveAppliedVolts, 1e-6);
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
