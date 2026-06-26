// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.Queue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX extends ModuleIOTalonFXBase {
  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // High-frequency odometry queues
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    super(constants);

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());

    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    readSignalInputs(inputs);

    inputs.driveConnected =
        driveConnectedDebounce.calculate(
            drivePosition.getStatus().isOK()
                && driveVelocity.getStatus().isOK()
                && driveAppliedVolts.getStatus().isOK()
                && driveCurrent.getStatus().isOK());
    inputs.turnConnected =
        turnConnectedDebounce.calculate(
            turnPosition.getStatus().isOK()
                && turnVelocity.getStatus().isOK()
                && turnAppliedVolts.getStatus().isOK()
                && turnCurrent.getStatus().isOK());
    inputs.turnEncoderConnected =
        turnEncoderConnectedDebounce.calculate(turnAbsolutePosition.getStatus().isOK());

    inputs.odometryTimestamps = new double[timestampQueue.size()];
    for (int i = 0; i < inputs.odometryTimestamps.length; i++) {
      inputs.odometryTimestamps[i] = timestampQueue.poll();
    }
    inputs.odometryDrivePositionsRad = new double[drivePositionQueue.size()];
    for (int i = 0; i < inputs.odometryDrivePositionsRad.length; i++) {
      inputs.odometryDrivePositionsRad[i] = Units.rotationsToRadians(drivePositionQueue.poll());
    }
    inputs.odometryTurnPositions = new Rotation2d[turnPositionQueue.size()];
    for (int i = 0; i < inputs.odometryTurnPositions.length; i++) {
      inputs.odometryTurnPositions[i] = Rotation2d.fromRotations(turnPositionQueue.poll());
    }
  }
}
