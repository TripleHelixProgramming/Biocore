// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.CANBusPorts.SC1;
import frc.robot.Robot;

/**
 * Module IO implementation using Phoenix 6 TalonFX sim layer for hardware-in-the-loop style
 * simulation. Uses the same hardware configuration and control requests as ModuleIOTalonFX, with
 * DCMotorSim physics driving the Phoenix sim state.
 */
public class ModuleIOSimTalonFX implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  // Sim states
  private final TalonFXSimState driveSimState;
  private final TalonFXSimState turnSimState;
  private final CANcoderSimState cancoderSimState;

  // Physics simulation (same gearboxes as ModuleIOSimWPI)
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Status signals
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  public ModuleIOSimTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.DriveMotorId, SC1.BUS);
    turnTalon = new TalonFX(constants.SteerMotorId, SC1.BUS);
    cancoder = new CANcoder(constants.EncoderId, SC1.BUS);

    // Configure drive motor
    tryUntilOk(
        5,
        () -> driveTalon.getConfigurator().apply(DriveConstants.buildDriveConfig(constants), 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    tryUntilOk(
        5,
        () -> turnTalon.getConfigurator().apply(DriveConstants.buildTurnConfig(constants), 0.25));

    // Configure CANcoder
    cancoder.getConfigurator().refresh(cancoderConfig);
    DriveConstants.configureCANcoder(cancoderConfig, constants);
    cancoder.getConfigurator().apply(cancoderConfig);

    // Get Phoenix sim states
    driveSimState = driveTalon.getSimState();
    turnSimState = turnTalon.getSimState();
    cancoderSimState = cancoder.getSimState();

    // Create physics models
    driveSim = DriveConstants.createDriveSim();
    turnSim = DriveConstants.createTurnSim();

    // Create status signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();
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

    // Refresh and read status signals
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.driveConnected = true;
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnZero = Rotation2d.fromRotations(cancoderConfig.MagnetSensor.MagnetOffset);
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // 50Hz odometry (high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC ->
              positionTorqueCurrentRequest.withPosition(rotation.getRotations());
        });
  }

  @Override
  public void setTurnZero(Rotation2d rotation) {
    cancoderConfig.MagnetSensor.MagnetOffset = rotation.getRotations();
    cancoder.getConfigurator().apply(cancoderConfig);
  }
}
