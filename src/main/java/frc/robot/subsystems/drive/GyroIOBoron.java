// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANBusPorts.SC0;
import frc.robot.util.CanandgyroThread;
import frc.robot.util.CanandgyroThread.GyroInputs;
import java.util.Queue;

/** IO implementation for Redux Canandgyro. */
public class GyroIOBoron implements GyroIO {
  private final Canandgyro canandgyro;
  private final GyroInputs gyroInputs;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  public GyroIOBoron() {
    canandgyro = new Canandgyro(SC0.GYRO);
    gyroInputs = CanandgyroThread.getInstance().registerCanandgyro(canandgyro);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Read from cached values (non-blocking) - updated by CanandgyroThread
    inputs.connected = gyroInputs.isConnected();
    inputs.calibrated = !gyroInputs.isCalibrating();
    inputs.yawPosition = Rotation2d.fromRotations(gyroInputs.getYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyroInputs.getAngularVelocityYaw());

    inputs.odometryYawTimestamps = new double[yawTimestampQueue.size()];
    for (int i = 0; i < inputs.odometryYawTimestamps.length; i++) {
      inputs.odometryYawTimestamps[i] = yawTimestampQueue.poll();
    }
    inputs.odometryYawPositions = new Rotation2d[yawPositionQueue.size()];
    for (int i = 0; i < inputs.odometryYawPositions.length; i++) {
      inputs.odometryYawPositions[i] = Rotation2d.fromRotations(yawPositionQueue.poll());
    }
  }
}
