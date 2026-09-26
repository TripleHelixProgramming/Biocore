// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.autoselect.AutoOption;
import frc.lib.autoselect.NamedAuto;
import frc.lib.autoselect.PoseSeekError;
import frc.robot.Constants.USBStorageConstants;
import frc.robot.PreMatchDisplay.Line;
import frc.robot.PreMatchDisplay.Snapshot;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.Alliance;

class PreMatchDisplayTest {
  private static final long PLENTY_OF_SPACE = Long.MAX_VALUE;

  // Measured by eye on the FIRST Driver Station, 2026-09-26
  private static final int MAX_VISIBLE_CHARS = 26;
  private static final int MAX_VISIBLE_LINES = 8;

  private static final AutoOption NAMED =
      new AutoOption(Alliance.BLUE, 3, new FakeAuto("R_BranchExampleAuto"));
  private static final AutoOption RESERVED = new AutoOption(Alliance.BLUE, 2);

  /** A baseline snapshot: switch and FMS agree, a named auto, no start pose, plenty of USB. */
  private static Snapshot snapshot() {
    return new Snapshot(
        Alliance.BLUE,
        Optional.of(Alliance.BLUE),
        true,
        3,
        Optional.of(NAMED),
        Optional.empty(),
        PLENTY_OF_SPACE);
  }

  private static Snapshot withAlliance(Optional<Alliance> ds, boolean fms) {
    var s = snapshot();
    return new Snapshot(
        s.switchAlliance(),
        ds,
        fms,
        s.switchPosition(),
        s.option(),
        s.poseError(),
        s.usbFreeBytes());
  }

  private static Snapshot withOption(int switchPosition, Optional<AutoOption> option) {
    var s = snapshot();
    return new Snapshot(
        s.switchAlliance(),
        s.dsAlliance(),
        s.fmsAttached(),
        switchPosition,
        option,
        s.poseError(),
        s.usbFreeBytes());
  }

  private static Snapshot withPoseError(PoseSeekError error) {
    var s = snapshot();
    return new Snapshot(
        s.switchAlliance(),
        s.dsAlliance(),
        s.fmsAttached(),
        s.switchPosition(),
        s.option(),
        Optional.of(error),
        s.usbFreeBytes());
  }

  private static Snapshot withUsbFree(long bytes) {
    var s = snapshot();
    return new Snapshot(
        s.switchAlliance(),
        s.dsAlliance(),
        s.fmsAttached(),
        s.switchPosition(),
        s.option(),
        s.poseError(),
        bytes);
  }

  private static Optional<String> line(List<Line> lines, String key) {
    return lines.stream().filter(l -> l.key().equals(key)).map(Line::text).findFirst();
  }

  private static String text(Snapshot s, String key) {
    return line(PreMatchDisplay.format(s), key).orElseThrow();
  }

  private static boolean isWarning(String text) {
    return text.startsWith(PreMatchDisplay.WARNING_STYLE + PreMatchDisplay.WARNING_MARK);
  }

  private static String visible(String text) {
    return text.replaceAll("\033\\[[0-9;]*m", "");
  }

  /** The expected text for an alliance source named on its alliance's background. */
  private static String chip(String label, Alliance alliance) {
    var style =
        alliance == Alliance.RED ? PreMatchDisplay.RED_CHIP_STYLE : PreMatchDisplay.BLUE_CHIP_STYLE;
    return style + " " + label + " " + PreMatchDisplay.RESET_STYLE;
  }

  @Test
  void allianceSourcesAreShownInTheirAllianceColor() {
    assertEquals(
        "Alliance: " + chip("Switch", Alliance.BLUE) + chip("FMS", Alliance.BLUE),
        text(withAlliance(Optional.of(Alliance.BLUE), true), "Alliance"));
    assertEquals(
        "Alliance: " + chip("Switch", Alliance.BLUE) + chip("DS", Alliance.BLUE),
        text(withAlliance(Optional.of(Alliance.BLUE), false), "Alliance"));
    assertEquals(
        "Alliance: " + chip("Switch", Alliance.BLUE) + " no DS",
        text(withAlliance(Optional.empty(), false), "Alliance"));
  }

  @Test
  void allianceMismatchIsAWarningThatKeepsTheSourceColors() {
    var mismatch = text(withAlliance(Optional.of(Alliance.RED), true), "Alliance");
    assertTrue(isWarning(mismatch));
    assertEquals(
        PreMatchDisplay.WARNING_STYLE
            + PreMatchDisplay.WARNING_MARK
            + "Alliance: "
            + chip("Switch", Alliance.BLUE)
            + chip("FMS", Alliance.RED),
        mismatch);
    assertEquals("!! Alliance:  Switch  FMS ", visible(mismatch));

    assertFalse(
        text(withAlliance(Optional.of(Alliance.BLUE), true), "Alliance")
            .contains(PreMatchDisplay.WARNING_MARK));
  }

  @Test
  void autoLineShowsNamedReservedAndUndefinedPositions() {
    assertEquals("Auto 3 R_BranchExampleAuto", text(snapshot(), "Auto"));
    assertEquals("Auto 2: none (reserved)", text(withOption(2, Optional.of(RESERVED)), "Auto"));

    var undefined = text(withOption(5, Optional.empty()), "Auto");
    assertTrue(isWarning(undefined));
    assertEquals("!! Auto 5: none for BLUE", visible(undefined));
  }

  @Test
  void autoNumberComesFromTheSelectedOption() {
    // The switches already read 4, but the selection still holds the option for position 3
    assertEquals("Auto 3 R_BranchExampleAuto", text(withOption(4, Optional.of(NAMED)), "Auto"));
  }

  @Test
  void startPoseLinesAreOmittedWithoutAStartPose() {
    var lines = PreMatchDisplay.format(snapshot());
    for (var key : List.of("Distance", "Forward", "Left", "Heading")) {
      assertTrue(line(lines, key).isEmpty(), key);
    }
  }

  @Test
  void startPoseLinesShowErrorsAndMarkValuesWithinTolerance() {
    // 30.48 cm = 12 in, -40.64 cm = -16 in, and the 3-4-5 triangle gives 20 in
    var off = withPoseError(new PoseSeekError(30.48, -40.64, 7.4));
    assertEquals("Distance 20in", text(off, "Distance"));
    assertEquals("Forward +12in", text(off, "Forward"));
    assertEquals("Left -16in", text(off, "Left"));
    assertEquals("Heading +7deg", text(off, "Heading"));

    var close = withPoseError(new PoseSeekError(4.9, -5.9, -2.9));
    assertEquals("Distance 3in", text(close, "Distance"));
    assertEquals("Forward OK", text(close, "Forward"));
    assertEquals("Left OK", text(close, "Left"));
    assertEquals("Heading OK", text(close, "Heading"));
  }

  @Test
  void usbLineAppearsOnlyWhenLow() {
    assertTrue(line(PreMatchDisplay.format(snapshot()), "USB").isEmpty());
    assertTrue(
        line(PreMatchDisplay.format(withUsbFree(USBStorageConstants.LOW_FREE_BYTES)), "USB")
            .isEmpty());

    var low = text(withUsbFree(1840L * 1024 * 1024), "USB");
    assertTrue(isWarning(low));
    assertEquals("!! USB 1840 MB free", visible(low));
  }

  @Test
  void worstCaseLinesAreAsciiAndFitTheDisplay() {
    // Far across the field, every warning showing
    var worst =
        new Snapshot(
            Alliance.BLUE,
            Optional.of(Alliance.RED),
            true,
            7,
            Optional.empty(),
            Optional.of(new PoseSeekError(-1654, -808, -179.6)),
            0);
    // The longest auto name, with every line present
    var longest =
        new Snapshot(
            Alliance.BLUE,
            Optional.empty(),
            false,
            3,
            Optional.of(NAMED),
            Optional.of(new PoseSeekError(-1654, -808, -179.6)),
            0);

    for (var s : List.of(worst, longest, withOption(2, Optional.of(RESERVED)))) {
      var lines = PreMatchDisplay.format(s);
      assertTrue(lines.size() <= MAX_VISIBLE_LINES, "too many lines: " + lines.size());
      for (var l : lines) {
        var shown = visible(l.text());
        assertTrue(shown.chars().allMatch(c -> c >= 0x20 && c < 0x7f), "non-ASCII: " + shown);
        assertTrue(shown.length() <= MAX_VISIBLE_CHARS, "too long: " + shown);
      }
    }
  }

  private record FakeAuto(String name) implements NamedAuto {
    @Override
    public String getName() {
      return name;
    }

    @Override
    public Command getAutoCommand() {
      throw new UnsupportedOperationException("not used by the display");
    }
  }
}
