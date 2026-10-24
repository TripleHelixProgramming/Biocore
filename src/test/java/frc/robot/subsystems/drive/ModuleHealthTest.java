// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Tests Module's pure health helpers. None of them touch the HAL, Preferences or Phoenix. */
class ModuleHealthTest {
  @Test
  void turnZeroIsUsableOnlyAfterASuccessfulRead() {
    assertTrue(Module.canUseTurnZero("OK"));
    assertFalse(Module.canUseTurnZero("TxFailed"));
    assertFalse(Module.canUseTurnZero(""));
    assertFalse(Module.canUseTurnZero(null));
  }

  @Test
  void successfulEncoderReadHasNoFailure() {
    assertNull(Module.setupFailure("FrontLeft", "OK"));
  }

  @Test
  void failedEncoderReadSaysTheZeroWasNotSaved() {
    assertEquals(
        "Setup failed on module BackRight: turn encoder config read (TxFailed); turn zero not"
            + " saved.",
        Module.setupFailure("BackRight", "TxFailed"));
  }
}
