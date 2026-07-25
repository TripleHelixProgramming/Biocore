// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Reads back a third-party library's Alert group (e.g. PathPlanner, Choreo, PhotonVision) from the
 * SmartDashboard NT table it publishes to, so it shows up in AdvantageKit logs.
 */
public class LoggedAlertGroup {
  @AutoLog
  public static class AlertGroupInputs {
    public String[] errors = new String[0];
    public String[] warnings = new String[0];
    public String[] infos = new String[0];
  }

  private final NetworkTable table;
  private final String key;
  private final AlertGroupInputsAutoLogged inputs = new AlertGroupInputsAutoLogged();

  /**
   * Creates a logged alert group reader.
   *
   * @param group the SmartDashboard alert group name (also used as the log key)
   */
  public LoggedAlertGroup(String group) {
    this.table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(group);
    this.key = "Alerts/" + group;
  }

  public void log() {
    inputs.errors = table.getEntry("errors").getStringArray(new String[0]);
    inputs.warnings = table.getEntry("warnings").getStringArray(new String[0]);
    inputs.infos = table.getEntry("infos").getStringArray(new String[0]);
    Logger.processInputs(key, inputs);
  }
}
