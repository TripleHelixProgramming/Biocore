// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Centimeters;
import static org.wpilib.units.Units.Inches;

import frc.lib.autoselect.AllianceSelector;
import frc.lib.autoselect.AutoOption;
import frc.lib.autoselect.AutoSelector;
import frc.lib.autoselect.PoseSeekError;
import frc.robot.Constants.PoseSeekConstants;
import frc.robot.Constants.USBStorageConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationDisplay;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;

/**
 * Shows the pre-match checks on the Driver Station display: the alliance and auto chosen by the
 * physical switches, how far the robot is from the auto's start pose, and a low USB storage
 * warning.
 *
 * <p>This is display only. Selection comes from the switches, and nothing here feeds back into it.
 *
 * <p>Call {@link #show} every disabled loop. The display sends at most once every 230 ms and drops
 * lines added in between, so every line is added on every call. The display keeps its last lines
 * while the robot is enabled.
 *
 * <p>The FIRST Driver Station shows about 26 characters per line, scrolling sideways past that, and
 * about 8 lines (measured by eye on the DS, 2026-09-26). Lines hold one value each to fit.
 */
public final class PreMatchDisplay {

  /**
   * ANSI black-on-yellow, applied to lines that need the drive team's attention. Yellow keeps
   * warnings distinct from the red and blue alliance colors.
   */
  static final String WARNING_STYLE = "\033[1;30;43m";

  /** Prefix on warning lines, so they stand out even where color doesn't render. */
  static final String WARNING_MARK = "!! ";

  /** ANSI white-on-red and white-on-blue, for naming an alliance source in its alliance color. */
  static final String RED_CHIP_STYLE = "\033[1;37;41m";

  static final String BLUE_CHIP_STYLE = "\033[1;37;44m";

  /** ANSI reset, ending a colored span within a line. */
  static final String RESET_STYLE = "\033[0m";

  private PreMatchDisplay() {}

  /** One display line, keyed so that a repeated key replaces the earlier line. */
  record Line(String key, String text) {}

  /**
   * The values shown on the display.
   *
   * @param switchAlliance alliance from the alliance switch
   * @param dsAlliance alliance from the FMS, or from the DS setting when no FMS is attached
   * @param fmsAttached whether an FMS is attached
   * @param switchPosition position read from the auto switches
   * @param option the selected auto option, empty when none is defined at the position
   * @param poseError error from the robot to the auto's start pose, empty when there is none
   * @param usbFreeBytes free bytes on the USB log drive
   */
  record Snapshot(
      Alliance switchAlliance,
      Optional<Alliance> dsAlliance,
      boolean fmsAttached,
      int switchPosition,
      Optional<AutoOption> option,
      Optional<PoseSeekError> poseError,
      long usbFreeBytes) {}

  /**
   * Sends the current pre-match checks to the Driver Station display. Call after both selectors'
   * {@code disabledPeriodic()}.
   *
   * @param allianceSelector the alliance switch
   * @param autoSelector the auto switches
   * @param robotPose the robot's current pose
   */
  public static void show(
      AllianceSelector allianceSelector, AutoSelector autoSelector, Pose2d robotPose) {
    var option = autoSelector.get();
    var snapshot =
        new Snapshot(
            allianceSelector.getAllianceColor(),
            MatchState.getAlliance(),
            RobotState.isFMSAttached(),
            autoSelector.getSwitchPosition(),
            option,
            option
                .flatMap(AutoOption::getInitialPose)
                .map(start -> PoseSeekError.between(robotPose, start)),
            Robot.getUSBStorageFreeSpace());

    for (var line : format(snapshot)) {
      DriverStationDisplay.addKeyedLine(line.key(), line.text());
    }
    DriverStationDisplay.updateLines();
  }

  /** Builds the display lines for a snapshot. */
  static List<Line> format(Snapshot s) {
    var lines = new ArrayList<Line>(7);
    lines.add(new Line("Alliance", allianceLine(s)));
    lines.add(new Line("Auto", autoLine(s)));
    s.poseError()
        .ifPresent(
            error -> {
              lines.add(
                  new Line(
                      "Distance", "Distance " + Math.round(inches(error.distanceCm())) + "in"));
              lines.add(
                  new Line(
                      "Forward",
                      value(
                          "Forward",
                          inches(error.forwardCm()),
                          "in",
                          error.forwardOk(PoseSeekConstants.X_TOL_CM))));
              lines.add(
                  new Line(
                      "Left",
                      value(
                          "Left",
                          inches(error.leftCm()),
                          "in",
                          error.leftOk(PoseSeekConstants.Y_TOL_CM))));
              lines.add(
                  new Line(
                      "Heading",
                      value(
                          "Heading",
                          error.headingDeg(),
                          "deg",
                          error.headingOk(PoseSeekConstants.HEADING_TOL_DEGREES))));
            });
    if (s.usbFreeBytes() < USBStorageConstants.LOW_FREE_BYTES) {
      lines.add(new Line("USB", warning("USB " + s.usbFreeBytes() / 1024 / 1024 + " MB free")));
    }
    return lines;
  }

  /**
   * Names each alliance source on a background of the alliance it reports. On a mismatch, the
   * warning style covers the label, and each source keeps its alliance color.
   */
  private static String allianceLine(Snapshot s) {
    var switchChip = allianceChip("Switch", s.switchAlliance());
    if (s.dsAlliance().isEmpty()) {
      return "Alliance: " + switchChip + " no DS";
    }
    var source = s.fmsAttached() ? "FMS" : "DS";
    var other = s.dsAlliance().get();
    var label = other != s.switchAlliance() ? warning("Alliance: ") : "Alliance: ";
    return label + switchChip + allianceChip(source, other);
  }

  private static String allianceChip(String label, Alliance alliance) {
    var background = alliance == Alliance.RED ? RED_CHIP_STYLE : BLUE_CHIP_STYLE;
    return background + " " + label + " " + RESET_STYLE;
  }

  private static String autoLine(Snapshot s) {
    if (s.option().isEmpty()) {
      return warning("Auto " + s.switchPosition() + ": none for " + s.switchAlliance());
    }
    // The option's own number, since the selected option can lag the switch input by one loop.
    // The name goes last because it is the only open-ended text; a long name scrolls.
    var option = s.option().get();
    if (!option.hasAutoMode()) {
      return "Auto " + option.getOptionNumber() + ": none (reserved)";
    }
    return "Auto " + option.getOptionNumber() + " " + option.getName();
  }

  private static double inches(double centimeters) {
    return Centimeters.of(centimeters).in(Inches);
  }

  /** Formats a signed, rounded value, or OK when it is within tolerance. */
  private static String value(String label, double value, String unit, boolean ok) {
    return ok ? label + " OK" : String.format("%s %+d%s", label, Math.round(value), unit);
  }

  private static String warning(String text) {
    return WARNING_STYLE + WARNING_MARK + text;
  }
}
