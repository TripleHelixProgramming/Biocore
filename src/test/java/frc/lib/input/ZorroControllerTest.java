// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.input;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

class ZorroControllerTest {

  private static final int PORT = 2;
  private static final int BUTTON_COUNT = 14;

  private final ZorroController zorro = new ZorroController(PORT);

  @BeforeAll
  static void initializeHal() {
    HAL.initialize();
    DriverStationSim.setDsAttached(true);
  }

  /** Presses exactly one Driver Station button bit on the test port. */
  private void pressOnly(int bit) {
    DriverStationSim.setJoystickButtonsMaximumIndex(PORT, BUTTON_COUNT);
    for (int i = 0; i < BUTTON_COUNT; i++) {
      DriverStationSim.setJoystickButton(PORT, i, i == bit);
    }
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void releaseAll() {
    pressOnly(-1);
  }

  @Test
  void firstDriverStationButtonIsSwitchBDown() {
    // The Zorro's first button (button 1 in 2026, bit 0 in the Driver Station data) is B down.
    pressOnly(0);
    assertTrue(zorro.getBDown());
    assertFalse(zorro.getBMid());
  }

  @Test
  void lastDriverStationButtonIsSwitchHIn() {
    pressOnly(BUTTON_COUNT - 1);
    assertTrue(zorro.getHIn());
    assertFalse(zorro.getDIn());
  }

  @Test
  void switchEUpIsFifthButton() {
    // E up selects field-relative driving.
    pressOnly(4);
    assertTrue(zorro.getEUp());
    assertFalse(zorro.getEDown());
    assertFalse(zorro.getAIn());
  }
}
