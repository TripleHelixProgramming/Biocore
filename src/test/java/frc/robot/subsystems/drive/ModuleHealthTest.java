// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

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
  void healthySetupHasNoFailure() {
    assertNull(Module.setupFailure("FrontLeft", "OK", "OK", "OK", "OK"));
  }

  @Test
  void singleFailureNamesTheDeviceAndStatus() {
    assertEquals(
        "Setup failed on module FrontLeft: drive motor setup (ConfigFailed).",
        Module.setupFailure("FrontLeft", "ConfigFailed", "OK", "OK", "OK"));
  }

  @Test
  void failedEncoderReadSaysTheZeroWasNotSaved() {
    assertEquals(
        "Setup failed on module BackRight: turn encoder config read (TxFailed); turn zero not"
            + " saved.",
        Module.setupFailure("BackRight", "OK", "OK", "TxFailed", "OK"));
  }

  @Test
  void failedEncoderWriteAfterAGoodRead() {
    assertEquals(
        "Setup failed on module BackRight: turn encoder config write (TxFailed).",
        Module.setupFailure("BackRight", "OK", "OK", "OK", "TxFailed"));
  }

  @Test
  void everyFailureIsListed() {
    assertEquals(
        "Setup failed on module FrontRight: drive motor setup (ConfigFailed); turn motor config"
            + " (TxFailed); turn encoder config read (TxFailed); turn zero not saved.",
        Module.setupFailure("FrontRight", "ConfigFailed", "TxFailed", "TxFailed", "OK"));
  }

  @ParameterizedTest
  @ValueSource(strings = {"FirmwareTooOld", "ApiTooOld"})
  void compliancyMismatchIsBlocked(String status) {
    assertEquals(
        "Phoenix is blocking output on module FrontLeft: drive motor ("
            + status
            + "). Update the motor firmware or the Phoenix library.",
        Module.firmwareBlocked("FrontLeft", status, "OK"));
  }

  @Test
  void bothBlockedMotorsAreListed() {
    assertEquals(
        "Phoenix is blocking output on module FrontLeft: drive motor (FirmwareTooOld), turn motor"
            + " (ApiTooOld). Update the motor firmware or the Phoenix library.",
        Module.firmwareBlocked("FrontLeft", "FirmwareTooOld", "ApiTooOld"));
  }

  @ParameterizedTest
  @ValueSource(strings = {"OK", "TxFailed", ""})
  void otherControlStatusesAreNotBlocked(String status) {
    assertNull(Module.firmwareBlocked("FrontLeft", status, status));
  }
}
