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
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;

/**
 * Module IO implementation using Phoenix 6 TalonFX sim layer for hardware-in-the-loop style
 * simulation. Uses the same hardware configuration and control requests as ModuleIOTalonFX, with
 * DCMotorSim physics driving the Phoenix sim state.
 */
public class ModuleIOSimTalonFX extends ModuleIOTalonFXBase {
  private final TalonFXSimState driveSimState;
  private final TalonFXSimState turnSimState;
  private final CANcoderSimState cancoderSimState;

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  public ModuleIOSimTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    super(constants);

    driveSimState = driveTalon.getSimState();
    turnSimState = turnTalon.getSimState();
    cancoderSimState = cancoder.getSimState();

    driveSim = DriveConstants.createDriveSim();
    turnSim = DriveConstants.createTurnSim();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Feed battery voltage into Phoenix sim devices
    double busVoltage = RoboRioSim.getVInVoltage();
    driveSimState.setSupplyVoltage(busVoltage);
    turnSimState.setSupplyVoltage(busVoltage);
    cancoderSimState.setSupplyVoltage(busVoltage);

    // Drive physics: Phoenix output voltage → DCMotorSim → rotor state back to Phoenix
    driveSim.setInputVoltage(
        MathUtil.clamp(driveSimState.getMotorVoltage(), -busVoltage, busVoltage));
    turnSim.setInputVoltage(
        MathUtil.clamp(turnSimState.getMotorVoltage(), -busVoltage, busVoltage));
    driveSim.update(Robot.defaultPeriodSecs);
    turnSim.update(Robot.defaultPeriodSecs);

    // Feed physics results back to Phoenix sim state (rotor level = mechanism × gear ratio)
    driveSimState.setRawRotorPosition(
        Units.radiansToRotations(driveSim.getAngularPosition()) * constants.DriveMotorGearRatio);
    driveSimState.setRotorVelocity(
        Units.radiansToRotations(driveSim.getAngularVelocity()) * constants.DriveMotorGearRatio);
    turnSimState.setRawRotorPosition(
        Units.radiansToRotations(turnSim.getAngularPosition()) * constants.SteerMotorGearRatio);
    turnSimState.setRotorVelocity(
        Units.radiansToRotations(turnSim.getAngularVelocity()) * constants.SteerMotorGearRatio);
    // CANcoder reports mechanism position (1 rotation = full wheel turn)
    cancoderSimState.setRawPosition(Units.radiansToRotations(turnSim.getAngularPosition()));
    cancoderSimState.setVelocity(Units.radiansToRotations(turnSim.getAngularVelocity()));

    readSignalInputs(inputs);

    inputs.driveConnected = true;
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;

    // 50Hz odometry (high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }
}
