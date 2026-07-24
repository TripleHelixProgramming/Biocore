// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.util.Preferences;

/**
 * Persists cumulative lifecycle statistics for the robot across power cycles using WPILib
 * Preferences. Call {@link #update()} from robotPeriodic() and the mode callbacks from each init
 * method. Toggle the "Triggers/Clear Robot Stats" NT entry to reset all stats from the dashboard.
 */
public final class RobotStats {
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

  private boolean inAuton = false;
  private boolean inTeleop = false;
  private double saveTimer = 0.0;

  private final double periodSecs;
  private final DoubleSupplier driveDistanceSupplier;
  private final BooleanSubscriber clearStatsSub;
  private final BooleanPublisher clearStatsPub;
  private boolean lastClearState = false;

  public RobotStats(double periodSecs, DoubleSupplier driveDistanceSupplier) {
    this.periodSecs = periodSecs;
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

    var clearStatsTopic =
        NetworkTableInstance.getDefault().getTable("Triggers").getBooleanTopic("Clear Robot Stats");
    clearStatsSub = clearStatsTopic.subscribe(false);
    clearStatsPub = clearStatsTopic.publish();
    clearStatsPub.set(false);
  }

  public void update() {
    boolean clearState = clearStatsSub.get();
    if (clearState && !lastClearState) {
      resetAll();
    }
    lastClearState = clearState;

    powerOnSecs += periodSecs;
    if (inAuton) autonSecs += periodSecs;
    if (inTeleop) teleopSecs += periodSecs;

    double current = driveDistanceSupplier.getAsDouble();
    distanceMeters += current - lastDriveDistanceMeters;
    lastDriveDistanceMeters = current;

    saveTimer += periodSecs;
    if (saveTimer >= SAVE_INTERVAL_SECS) {
      save();
      saveTimer = 0.0;
    }

    logValues();
  }

  public void onAutonEnabled() {
    inAuton = true;
    inTeleop = false;
    save();
  }

  public void onTeleopEnabled() {
    inTeleop = true;
    inAuton = false;
    save();
  }

  public void onDisabled() {
    inAuton = false;
    inTeleop = false;
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
