// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified work Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.Timer;

/** Physics sim implementation of module IO. */
public class ModuleIOSimWPI implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private final double driveFrictionVolts;
  private final double turnFrictionVolts;
  private final double driveKs;
  private final double driveKv;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSimWPI(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim models
    driveSim = DriveConstants.createDriveSim();
    turnSim = DriveConstants.createTurnSim();
    driveFrictionVolts = constants.DriveFrictionVoltage;
    turnFrictionVolts = constants.SteerFrictionVoltage;

    // The drive model settles at w = V * Kv / G, so this feedforward inverts it exactly;
    // kS cancels the modeled static friction.
    driveKv = constants.DriveMotorGearRatio / DriveConstants.DRIVE_GEARBOX.Kv;
    driveKs = driveFrictionVolts;

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // Update simulation state
    double busVoltage = RoboRioSim.getVInVoltage();
    driveSim.setInputVoltage(
        applyFriction(Math.clamp(driveAppliedVolts, -busVoltage, busVoltage), driveFrictionVolts));
    turnSim.setInputVoltage(
        applyFriction(Math.clamp(turnAppliedVolts, -busVoltage, busVoltage), turnFrictionVolts));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDraw());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  /**
   * Models static friction the same way as Phoenix's SimSwerveDrivetrain: voltages below the
   * friction voltage produce no motion, and larger voltages lose the friction voltage.
   */
  static double applyFriction(double volts, double frictionVolts) {
    if (Math.abs(volts) < frictionVolts) {
      return 0.0;
    }
    return volts - Math.copySign(frictionVolts, volts);
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
