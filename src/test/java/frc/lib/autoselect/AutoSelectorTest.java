// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.autoselect;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.Alliance;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DIOSim;

class AutoSelectorTest {
  // Ports not used by the robot's selectors, which other tests allocate
  private static final int ALLIANCE_PORT = 7;
  private static final int[] AUTO_PORTS = {4, 5, 6};

  private static final Pose2d[] BLUE_PATH = {
    new Pose2d(1, 1, Rotation2d.ZERO), new Pose2d(2, 1, Rotation2d.ZERO)
  };

  private static AllianceSelector allianceSelector;
  private static AutoSelector autoSelector;

  @BeforeAll
  static void setUp() {
    HAL.initialize();
    allianceSelector = new AllianceSelector(ALLIANCE_PORT);
    autoSelector = new AutoSelector(AUTO_PORTS, allianceSelector::getAllianceColor);
    autoSelector.addAuto(new AutoOption(Alliance.BLUE, 1, new FakeAuto(BLUE_PATH)));
    autoSelector.addAuto(new AutoOption(Alliance.BLUE, 2)); // reserved for no auto
  }

  @Test
  void selectingNoAutoErasesThePlottedTrajectory() {
    select(1);
    assertArrayEquals(BLUE_PATH, autoSelector.getLoggedTrajectory());

    select(0); // no option at this position
    assertEquals(0, autoSelector.getLoggedTrajectory().length);

    select(1);
    assertArrayEquals(BLUE_PATH, autoSelector.getLoggedTrajectory());

    select(2); // an option reserved for no auto
    assertEquals(0, autoSelector.getLoggedTrajectory().length);
  }

  /** Sets the switches to blue and the given position, then lets the selectors settle. */
  private static void select(int position) {
    new DIOSim(ALLIANCE_PORT).setValue(false);
    for (int i = 0; i < AUTO_PORTS.length; i++) {
      new DIOSim(AUTO_PORTS[i]).setValue((position & (1 << i)) == 0);
    }
    // Each selector polls its change events before reading, so it takes two cycles to settle
    for (int i = 0; i < 2; i++) {
      allianceSelector.disabledPeriodic();
      autoSelector.disabledPeriodic();
    }
  }

  private record FakeAuto(Pose2d[] path) implements NamedAuto {
    @Override
    public String getName() {
      return "FakeAuto";
    }

    @Override
    public Command getAutoCommand() {
      return Commands.none();
    }

    @Override
    public Pose2d[] getLoggableTrajectory() {
      return path;
    }
  }
}
