// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified work Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import frc.lib.hardware.PhoenixFirmware;
import frc.robot.Constants.FeatureFlags;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.preferences.Preferences;
import org.wpilib.util.Alert;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String name;
  // Built once because runSetpoint logs it every loop
  private final String feedforwardKey;
  private boolean encoderInitialized = false;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private final Alert driveInitFailedAlert;
  private final Alert turnConfigFailedAlert;
  private final Alert turnEncoderConfigFailedAlert;
  private final Alert driveFirmwareBlockedAlert;
  private final Alert turnFirmwareBlockedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
    feedforwardKey = "Drive/Module" + name + "/FeedforwardWheelTorqueNm";
    driveDisconnectedAlert =
        new Alert(
            "Module/" + name + "/driveDisconnected",
            "Disconnected drive motor on module " + name + ".",
            Alert.Level.HIGH);
    turnDisconnectedAlert =
        new Alert(
            "Module/" + name + "/turnDisconnected",
            "Disconnected turn motor on module " + name + ".",
            Alert.Level.HIGH);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Module/" + name + "/turnEncoderDisconnected",
            "Disconnected turn encoder on module " + name + ".",
            Alert.Level.HIGH);
    driveInitFailedAlert = new Alert("Module/" + name + "/driveInitFailed", "", Alert.Level.HIGH);
    turnConfigFailedAlert = new Alert("Module/" + name + "/turnConfigFailed", "", Alert.Level.HIGH);
    turnEncoderConfigFailedAlert =
        new Alert("Module/" + name + "/turnEncoderConfigFailed", "", Alert.Level.HIGH);
    driveFirmwareBlockedAlert =
        new Alert("Module/" + name + "/driveFirmwareBlocked", "", Alert.Level.HIGH);
    turnFirmwareBlockedAlert =
        new Alert("Module/" + name + "/turnFirmwareBlocked", "", Alert.Level.HIGH);
  }

  public void periodic() {
    long t0 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    io.updateInputs(inputs);
    long t1 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    Logger.processInputs("Drive/Module" + name, inputs);
    long t2 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    if (!encoderInitialized) {
      // Set turn zero from preferences. The CANcoder's zero seeds a missing preference only when
      // its config was read. After a failed read the zero is a default, and saving it would
      // replace the module's calibration.
      Rotation2d turnZeroFromCancoder = inputs.turnZero;
      if ("OK".equals(inputs.turnEncoderRefreshStatus)) {
        Preferences.initDouble(ZERO_ROTATION_KEY + "/" + name, turnZeroFromCancoder.getRadians());
      }
      Rotation2d turnZeroFromPreferences =
          new Rotation2d(
              Preferences.getDouble(
                  ZERO_ROTATION_KEY + "/" + name, turnZeroFromCancoder.getRadians()));
      io.setTurnZero(turnZeroFromPreferences);
      Logger.recordOutput(
          "Drive/Module" + name + "/TurnZeroRad", turnZeroFromPreferences.getRadians());
      encoderInitialized = true;
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS_METERS;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    setStatusAlert(
        driveInitFailedAlert,
        inputs.driveInitStatus,
        "Drive motor setup failed on module " + name + " (" + inputs.driveInitStatus + ").");
    setStatusAlert(
        turnConfigFailedAlert,
        inputs.turnConfigStatus,
        "Turn motor config failed on module " + name + " (" + inputs.turnConfigStatus + ").");
    boolean turnEncoderReadOk = "OK".equals(inputs.turnEncoderRefreshStatus);
    String turnEncoderStatus =
        turnEncoderReadOk ? inputs.turnEncoderApplyStatus : inputs.turnEncoderRefreshStatus;
    setStatusAlert(
        turnEncoderConfigFailedAlert,
        turnEncoderStatus,
        turnEncoderReadOk
            ? "Turn encoder config write failed on module " + name + " (" + turnEncoderStatus + ")."
            : "Turn encoder config read failed on module "
                + name
                + " ("
                + turnEncoderStatus
                + "); turn zero not saved.");
    setBlockedAlert(driveFirmwareBlockedAlert, inputs.driveControlStatus, "drive motor");
    setBlockedAlert(turnFirmwareBlockedAlert, inputs.turnControlStatus, "turn motor");
    Logger.recordOutput("Faults/Module" + name + "/DriveDisconnected", !inputs.driveConnected);
    Logger.recordOutput("Faults/Module" + name + "/TurnDisconnected", !inputs.turnConnected);
    long t3 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Profiling output
    if (FeatureFlags.PROFILING_ENABLED) {
      long totalMs = (t3 - t0) / 1_000_000;
      if (totalMs > 2) {
        System.out.println(
            "[Module"
                + name
                + "] updateInputs="
                + (t1 - t0) / 1_000_000
                + "ms log="
                + (t2 - t1) / 1_000_000
                + "ms rest="
                + (t3 - t2) / 1_000_000
                + "ms");
      }
    }
  }

  /** Activates the alert when the Phoenix status isn't "OK", with text naming the status. */
  private static void setStatusAlert(Alert alert, String status, String text) {
    boolean failed = !"OK".equals(status);
    if (failed && !text.equals(alert.getText())) alert.setText(text);
    alert.set(failed);
  }

  /** Activates the alert when the setControl status shows Phoenix blocking the motor's output. */
  private void setBlockedAlert(Alert alert, String controlStatus, String device) {
    boolean blocked = PhoenixFirmware.isBlocked(controlStatus);
    if (blocked) {
      String text =
          "Phoenix is blocking "
              + device
              + " output on module "
              + name
              + ": "
              + controlStatus
              + ". Update the motor firmware or the Phoenix library.";
      if (!text.equals(alert.getText())) alert.setText(text);
    }
    alert.set(blocked);
  }

  /** Runs the module with the specified setpoint state. */
  public void runSetpoint(SwerveModuleVelocity state) {
    runSetpoint(state, Translation2d.ZERO);
  }

  /**
   * Runs the module with the specified setpoint state, adding a feedforward for the force the
   * module must apply to the robot.
   *
   * @param state The setpoint state
   * @param forceNewtons The force at the module, robot relative, in newtons
   */
  public void runSetpoint(SwerveModuleVelocity state, Translation2d forceNewtons) {
    state = state.optimize(getAngle());
    state = state.cosineScale(inputs.turnPosition);

    double wheelTorqueNm = wheelTorque(forceNewtons, inputs.turnPosition);
    Logger.recordOutput(feedforwardKey, wheelTorqueNm);
    io.setDriveVelocity(state.velocity / WHEEL_RADIUS_METERS, wheelTorqueNm);
    io.setTurnPosition(state.angle);
  }

  /**
   * Returns the torque the wheel must apply to push with the given force. Only the component of the
   * force along the wheel's measured direction counts. That direction flips when the module is
   * optimized to drive backward, which flips the torque with it.
   */
  static double wheelTorque(Translation2d forceNewtons, Rotation2d wheelDirection) {
    double alongWheel =
        forceNewtons.getX() * wheelDirection.getCos()
            + forceNewtons.getY() * wheelDirection.getSin();
    return alongWheel * WHEEL_RADIUS_METERS;
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.ZERO);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the total motor current draw for battery simulation. */
  public double getSimCurrentDrawAmps() {
    return inputs.driveCurrentAmps + inputs.turnCurrentAmps;
  }

  /** Sets the zero position of the turn axis to the current rotation */
  public void setTurnZero() {
    Rotation2d newTurnZero = inputs.turnZero.minus(inputs.turnPosition);
    io.setTurnZero(newTurnZero);
    Preferences.setDouble(ZERO_ROTATION_KEY + "/" + name, newTurnZero.getRadians());
    Logger.recordOutput("Drive/Module" + name + "/TurnZeroRad", newTurnZero.getRadians());
  }
}
