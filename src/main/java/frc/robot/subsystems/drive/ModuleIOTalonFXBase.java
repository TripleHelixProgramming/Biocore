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

import static frc.robot.util.odometry.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import frc.lib.hardware.PhoenixFirmware;
import frc.robot.Constants.CANBusPorts.SC1;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/** Shared base for TalonFX-based module IO implementations (hardware and sim). */
public abstract class ModuleIOTalonFXBase implements ModuleIO {
  protected final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  protected final TalonFX driveTalon;
  protected final TalonFX turnTalon;
  protected final CANcoder cancoder;
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Status signals
  protected final StatusSignal<Angle> drivePosition;
  protected final StatusSignal<AngularVelocity> driveVelocity;
  protected final StatusSignal<Voltage> driveAppliedVolts;
  protected final StatusSignal<Current> driveCurrent;
  protected final StatusSignal<Angle> turnAbsolutePosition;
  protected final StatusSignal<Angle> turnPosition;
  protected final StatusSignal<AngularVelocity> turnVelocity;
  protected final StatusSignal<Voltage> turnAppliedVolts;
  protected final StatusSignal<Current> turnCurrent;
  private final StatusSignal<Integer> driveVersion;
  private final StatusSignal<Integer> turnVersion;
  private final StatusSignal<Integer> turnEncoderVersion;

  // Setup results. The encoder refresh result stays fixed after construction: a failed refresh
  // means cancoderConfig holds defaults, so its magnet offset is not the device's real zero.
  private final StatusCode driveInitStatus;
  private final StatusCode turnConfigStatus;
  private final StatusCode turnEncoderRefreshStatus;
  private StatusCode turnEncoderApplyStatus = StatusCode.OK;

  // Results of the most recent setControl calls, which report when Phoenix blocks output
  private StatusCode driveControlStatus = StatusCode.OK;
  private StatusCode turnControlStatus = StatusCode.OK;

  protected ModuleIOTalonFXBase(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.DriveMotorId, SC1.BUS);
    turnTalon = new TalonFX(constants.SteerMotorId, SC1.BUS);
    cancoder = new CANcoder(constants.EncoderId, SC1.BUS);

    driveInitStatus =
        firstError(
            tryUntilOk(
                5,
                () -> driveTalon.getConfigurator().apply(constants.DriveMotorInitialConfigs, 0.25)),
            tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25)));
    turnConfigStatus =
        tryUntilOk(
            5, () -> turnTalon.getConfigurator().apply(constants.SteerMotorInitialConfigs, 0.25));

    // Read the CANcoder's config first so the apply keeps settings made in Phoenix Tuner. If the
    // read fails, skip the apply rather than write defaults over the device.
    turnEncoderRefreshStatus =
        tryUntilOk(5, () -> cancoder.getConfigurator().refresh(cancoderConfig));
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    if (turnEncoderRefreshStatus.isOK()) {
      turnEncoderApplyStatus =
          tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));
    }

    // getVersion() refreshes on each call, so keep the signals and refresh them with the others
    driveVersion = driveTalon.getVersion();
    turnVersion = turnTalon.getVersion();
    turnEncoderVersion = cancoder.getVersion();

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

  /** Refreshes all status signals and populates the non-connection, non-odometry inputs. */
  protected void readSignalInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        driveVersion,
        turnVersion,
        turnEncoderVersion);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnZero = Rotation2d.fromRotations(cancoderConfig.MagnetSensor.MagnetOffset);
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    inputs.driveInitStatus = driveInitStatus.getName();
    inputs.turnConfigStatus = turnConfigStatus.getName();
    inputs.turnEncoderRefreshStatus = turnEncoderRefreshStatus.getName();
    inputs.turnEncoderApplyStatus = turnEncoderApplyStatus.getName();
    inputs.driveControlStatus = driveControlStatus.getName();
    inputs.turnControlStatus = turnControlStatus.getName();

    inputs.driveFirmware = firmwareVersion(driveVersion);
    inputs.turnFirmware = firmwareVersion(turnVersion);
    inputs.turnEncoderFirmware = firmwareVersion(turnEncoderVersion);
  }

  /** Returns the signal's firmware version as "major.minor.bugfix.build", or "" when unknown. */
  private static String firmwareVersion(StatusSignal<Integer> version) {
    return version.getStatus().isOK() ? PhoenixFirmware.format(version.getValue()) : "";
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveControlStatus =
        driveTalon.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
              case Voltage -> voltageRequest.withOutput(output);
              case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnControlStatus =
        turnTalon.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
              case Voltage -> voltageRequest.withOutput(output);
              case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforwardWheelTorqueNm) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    double feedforward =
        driveFeedforward(
            constants.DriveMotorClosedLoopOutput,
            feedforwardWheelTorqueNm,
            constants.DriveMotorGearRatio);
    driveControlStatus =
        driveTalon.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
              case Voltage ->
                  velocityVoltageRequest
                      .withVelocity(velocityRotPerSec)
                      .withFeedForward(feedforward);
              case TorqueCurrentFOC ->
                  velocityTorqueCurrentRequest
                      .withVelocity(velocityRotPerSec)
                      .withFeedForward(feedforward);
            });
  }

  /**
   * Returns the drive feedforward that makes the wheel apply the given torque, in the units of the
   * closed-loop output: volts for Voltage, amps for TorqueCurrentFOC.
   *
   * @param outputType The drive motor's closed-loop output type
   * @param wheelTorqueNm Torque at the wheel in newton meters
   * @param gearRatio Drive motor rotations per wheel rotation
   */
  static double driveFeedforward(
      ClosedLoopOutputType outputType, double wheelTorqueNm, double gearRatio) {
    double motorTorqueNm = wheelTorqueNm / gearRatio;
    return switch (outputType) {
      case Voltage -> DriveConstants.DRIVE_GEARBOX.getVoltage(motorTorqueNm, 0.0);
      case TorqueCurrentFOC -> DriveConstants.DRIVE_GEARBOX.getCurrent(motorTorqueNm);
    };
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnControlStatus =
        turnTalon.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
              case Voltage -> motionMagicVoltageRequest.withPosition(rotation.getRotations());
              case TorqueCurrentFOC ->
                  motionMagicTorqueCurrentRequest.withPosition(rotation.getRotations());
            });
  }

  @Override
  public void setTurnZero(Rotation2d rotation) {
    cancoderConfig.MagnetSensor.MagnetOffset = rotation.getRotations();
    // One attempt only: this runs on the main loop, where retries would stall the robot
    turnEncoderApplyStatus = cancoder.getConfigurator().apply(cancoderConfig);
  }
}
