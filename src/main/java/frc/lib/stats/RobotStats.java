// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.stats;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;
import org.wpilib.util.Preferences;

/**
 * Persists cumulative lifecycle statistics for the robot across power cycles using WPILib
 * Preferences. Call {@link #update()} from robotPeriodic() and the mode callbacks from each init
 * method. Toggle the "Triggers/Clear Robot Stats" NT entry to reset all stats from the dashboard.
 */
public final class RobotStats {
  private enum Mode {
    DISABLED,
    AUTON,
    TELEOP
  }

  private static final String KEY_BOOT_COUNT = "Stats/BootCount";
  private static final String KEY_POWER_ON_SECS = "Stats/PowerOnSecs";
  private static final String KEY_AUTON_SECS = "Stats/AutonSecs";
  private static final String KEY_TELEOP_SECS = "Stats/TeleopSecs";
  private static final String KEY_DISTANCE_M = "Stats/DistanceMeters";
  private static final double SAVE_INTERVAL_SECS = 30.0;

  private int bootCount;
  private double powerOnSecs;
  private double autonSecs;
  private double teleopSecs;
  private double distanceMeters;
  private double lastDriveDistanceMeters;

  private Mode mode = Mode.DISABLED;
  private final Timer saveTimer = new Timer();
  private final Timer elapsedTimer = new Timer();

  private final DoubleSupplier driveDistanceSupplier;
  private final BooleanPublisher clearStatsPub;

  public RobotStats(DoubleSupplier driveDistanceSupplier) {
    this.driveDistanceSupplier = driveDistanceSupplier;

    Preferences.initInt(KEY_BOOT_COUNT, 0);
    bootCount = Preferences.getInt(KEY_BOOT_COUNT, 0) + 1;
    Preferences.setInt(KEY_BOOT_COUNT, bootCount);

    Preferences.initDouble(KEY_POWER_ON_SECS, 0.0);
    Preferences.initDouble(KEY_AUTON_SECS, 0.0);
    Preferences.initDouble(KEY_TELEOP_SECS, 0.0);
    Preferences.initDouble(KEY_DISTANCE_M, 0.0);

    powerOnSecs = Preferences.getDouble(KEY_POWER_ON_SECS, 0.0);
    autonSecs = Preferences.getDouble(KEY_AUTON_SECS, 0.0);
    teleopSecs = Preferences.getDouble(KEY_TELEOP_SECS, 0.0);
    distanceMeters = Preferences.getDouble(KEY_DISTANCE_M, 0.0);

    lastDriveDistanceMeters = driveDistanceSupplier.getAsDouble();
    saveTimer.start();
    elapsedTimer.start();

    var clearStatsTopic =
        NetworkTableInstance.getDefault().getTable("Triggers").getBooleanTopic("Clear Robot Stats");
    var clearStatsSub = clearStatsTopic.subscribe(false);
    clearStatsPub = clearStatsTopic.publish();
    clearStatsPub.set(false);
    new Trigger(clearStatsSub::get).onTrue(Commands.runOnce(this::resetAll).ignoringDisable(true));
  }

  public void update() {
    double elapsedSecs = elapsedTimer.get();
    elapsedTimer.reset();

    powerOnSecs += elapsedSecs;
    if (mode == Mode.AUTON) autonSecs += elapsedSecs;
    if (mode == Mode.TELEOP) teleopSecs += elapsedSecs;

    double current = driveDistanceSupplier.getAsDouble();
    distanceMeters += current - lastDriveDistanceMeters;
    lastDriveDistanceMeters = current;

    if (saveTimer.advanceIfElapsed(SAVE_INTERVAL_SECS)) {
      save();
    }

    logValues();
  }

  public void onAutonEnabled() {
    mode = Mode.AUTON;
    save();
  }

  public void onTeleopEnabled() {
    mode = Mode.TELEOP;
    save();
  }

  public void onDisabled() {
    mode = Mode.DISABLED;
    save();
  }

  public void resetAll() {
    bootCount = 0;
    powerOnSecs = 0.0;
    autonSecs = 0.0;
    teleopSecs = 0.0;
    distanceMeters = 0.0;
    lastDriveDistanceMeters = driveDistanceSupplier.getAsDouble();
    clearStatsPub.set(false);
    save();
  }

  private void save() {
    Preferences.setInt(KEY_BOOT_COUNT, bootCount);
    Preferences.setDouble(KEY_POWER_ON_SECS, powerOnSecs);
    Preferences.setDouble(KEY_AUTON_SECS, autonSecs);
    Preferences.setDouble(KEY_TELEOP_SECS, teleopSecs);
    Preferences.setDouble(KEY_DISTANCE_M, distanceMeters);
  }

  private void logValues() {
    Logger.recordOutput("RobotStats/BootCount", bootCount);
    Logger.recordOutput("RobotStats/PowerOnHours", powerOnSecs / 3600.0);
    Logger.recordOutput("RobotStats/AutonEnabledMins", autonSecs / 60.0);
    Logger.recordOutput("RobotStats/TeleopEnabledMins", teleopSecs / 60.0);
    Logger.recordOutput("RobotStats/DistanceTraveledMeters", distanceMeters);
    Logger.recordOutput("RobotStats/DistanceTraveledMiles", distanceMeters / 1609.344);
  }
}
