// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import frc.lib.autoselect.AllianceSelector;
import frc.lib.autoselect.AutoOption;
import frc.lib.autoselect.AutoSelector;
import frc.robot.Constants.DIOPorts;
import frc.robot.generated.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSimWPI;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DIOSim;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Runs the placeholder autos on the simulated drivetrain, stepping the robot loop in fixed 20 ms
 * increments, and checks that each one follows its path and stops at the end.
 */
class DriveOutAutoSimTest {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final List<String> initializedCommands = new ArrayList<>();

  // Drive and the autos allocate uniquely named Alerts, so each can only be built once per JVM,
  // just as on the robot
  private static Drive drive;
  private static AutoMode blueAuto;
  private static AutoMode redAuto;

  @BeforeAll
  static void setUpRobot() {
    HAL.initialize();
    SimHooks.pauseTiming();
    CommandScheduler.getInstance()
        .onCommandInitialize(command -> initializedCommands.add(command.getName()));
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSimWPI(DriveConstants.FRONT_LEFT),
            new ModuleIOSimWPI(DriveConstants.FRONT_RIGHT),
            new ModuleIOSimWPI(DriveConstants.BACK_LEFT),
            new ModuleIOSimWPI(DriveConstants.BACK_RIGHT));
    blueAuto = new B_DriveOutAuto(drive);
    redAuto = new R_DriveOutAuto(drive);
  }

  @AfterEach
  void disable() {
    setMode(false);
    CommandScheduler.getInstance().cancelAll();
    step(1.0); // let the modules coast to a stop before the next test
  }

  @AfterAll
  static void tearDownRobot() {
    SimHooks.resumeTiming();
  }

  @Test
  void blueAutoFollowsItsPath() {
    runAndCheck(blueAuto, ChoreoTraj.BlueDriveOut);
  }

  @Test
  void redAutoFollowsItsPath() {
    runAndCheck(redAuto, ChoreoTraj.RedDriveOut);
  }

  /**
   * The alliance switch reads high for red. Each auto switch adds its bit to the position when it
   * reads low, so position 1 is the first switch low and the others high.
   */
  @Test
  void switchesSelectTheAllianceAuto() {
    var allianceSelector = new AllianceSelector(DIOPorts.ALLIANCE_COLOR_SELECTOR);
    var autoSelector =
        new AutoSelector(DIOPorts.AUTONOMOUS_MODE_SELECTOR, allianceSelector::getAllianceColor);
    autoSelector.addAuto(new AutoOption(Alliance.BLUE, 1, blueAuto));
    autoSelector.addAuto(new AutoOption(Alliance.RED, 1, redAuto));

    setSwitches(true, 1);
    assertEquals(
        "R_DriveOutAuto", selected(allianceSelector, autoSelector).map(AutoOption::getName).get());
    setSwitches(false, 1);
    assertEquals(
        "B_DriveOutAuto", selected(allianceSelector, autoSelector).map(AutoOption::getName).get());
    setSwitches(false, 0);
    assertTrue(selected(allianceSelector, autoSelector).isEmpty());
  }

  private static void setSwitches(boolean red, int position) {
    new DIOSim(DIOPorts.ALLIANCE_COLOR_SELECTOR).setValue(red);
    int[] ports = DIOPorts.AUTONOMOUS_MODE_SELECTOR;
    for (int i = 0; i < ports.length; i++) {
      new DIOSim(ports[i]).setValue((position & (1 << i)) == 0);
    }
  }

  private static Optional<AutoOption> selected(
      AllianceSelector allianceSelector, AutoSelector autoSelector) {
    // Each selector polls its change events before reading, so it takes two cycles to settle
    for (int i = 0; i < 2; i++) {
      allianceSelector.disabledPeriodic();
      autoSelector.disabledPeriodic();
    }
    return autoSelector.get();
  }

  /** Bindings are made once, in the constructor, so a second run schedules the same commands. */
  @Test
  void secondRunSchedulesTheSameCommands() {
    List<String> first = runOnce(blueAuto, ChoreoTraj.BlueDriveOut);
    setMode(false);
    step(0.5);
    List<String> second = runOnce(blueAuto, ChoreoTraj.BlueDriveOut);
    assertEquals(first, second);
  }

  private void runAndCheck(AutoMode auto, ChoreoTraj path) {
    // The trajectory plotted before the match covers every segment, not just the first
    Pose2d[] plotted = auto.getLoggableTrajectory();
    assertTrue(
        plotted[0].getTranslation().getDistance(path.initialPoseBlue().getTranslation()) < 1e-4,
        "The plotted trajectory does not start where the auto starts");
    assertTrue(
        plotted[plotted.length - 1]
                .getTranslation()
                .getDistance(path.endPoseBlue().getTranslation())
            < 1e-4,
        "The plotted trajectory stops short of the end of the auto");

    List<String> commands = runOnce(auto, path);

    // The routine command, then reset odometry followed by segment 0, then segment 1, then stop
    assertEquals(
        List.of(
            "ConditionalCommand",
            "SequentialCommandGroup",
            "Trajectory_" + path.name() + "[1]",
            "Stop Drive"),
        commands);

    // Measured in sim when written: 7.5 mm and 0.71 deg at the end, 4.8 cm from the path at most
    Pose2d end = drive.getPose();
    Pose2d expected = path.endPoseBlue();
    double endErrorMeters = end.getTranslation().getDistance(expected.getTranslation());
    double endErrorDegrees = end.getRotation().minus(expected.getRotation()).getDegrees();
    assertTrue(endErrorMeters < 0.025, "Ended " + endErrorMeters + " m from the path end");
    assertTrue(
        Math.abs(endErrorDegrees) < 2.0, "Ended " + endErrorDegrees + " deg from the path end");
    assertTrue(maxDeviation < 0.10, "Strayed " + maxDeviation + " m from the path");
  }

  private double maxDeviation;

  /**
   * Enables autonomous, runs the auto for the path's duration plus one second, returns commands.
   */
  private List<String> runOnce(AutoMode auto, ChoreoTraj path) {
    Pose2d[] pathPoses = Choreo.<SwerveSample>loadTrajectory(path.name()).orElseThrow().getPoses();
    initializedCommands.clear();
    setMode(true);
    CommandScheduler.getInstance().schedule(auto.getAutoCommand());
    maxDeviation = 0.0;
    int steps = (int) Math.ceil((path.totalTimeSecs() + 1.0) / LOOP_PERIOD_SECS);
    for (int i = 0; i < steps; i++) {
      // The pose is only on the path once the sequence has reset odometry
      boolean odometryReset = initializedCommands.contains("SequentialCommandGroup");
      step(LOOP_PERIOD_SECS);
      if (odometryReset) {
        maxDeviation = Math.max(maxDeviation, distanceToPath(drive.getPose(), pathPoses));
      }
    }
    return List.copyOf(initializedCommands);
  }

  private static double distanceToPath(Pose2d pose, Pose2d[] pathPoses) {
    double best = Double.POSITIVE_INFINITY;
    for (Pose2d p : pathPoses) {
      best = Math.min(best, pose.getTranslation().getDistance(p.getTranslation()));
    }
    return best;
  }

  private static void setMode(boolean autonomousEnabled) {
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(autonomousEnabled);
    DriverStationSim.notifyNewData();
    DriverStationBackend.refreshData();
  }

  private static void step(double seconds) {
    for (double t = 0; t < seconds - 1e-9; t += LOOP_PERIOD_SECS) {
      SimHooks.stepTiming(LOOP_PERIOD_SECS);
      DriverStationSim.notifyNewData();
      DriverStationBackend.refreshData();
      CommandScheduler.getInstance().run();
    }
  }
}
