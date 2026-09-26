// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class PhoenixFirmwareTest {
  @Test
  void firmwareTooOldIsBlocked() {
    assertTrue(PhoenixFirmware.isBlocked("FirmwareTooOld"));
  }

  @Test
  void apiTooOldIsBlocked() {
    assertTrue(PhoenixFirmware.isBlocked("ApiTooOld"));
  }

  @Test
  void otherStatusesAreNotBlocked() {
    assertFalse(PhoenixFirmware.isBlocked("OK"));
    assertFalse(PhoenixFirmware.isBlocked("TxFailed"));
    assertFalse(PhoenixFirmware.isBlocked(""));
    assertFalse(PhoenixFirmware.isBlocked(null));
  }

  @Test
  void formatsMostSignificantByteFirst() {
    assertEquals("26.2.3.0", PhoenixFirmware.format(0x1A020300));
  }

  @Test
  void formatsBytesAsUnsigned() {
    assertEquals("255.0.0.0", PhoenixFirmware.format(0xFF000000));
    assertEquals("128.255.128.255", PhoenixFirmware.format(0x80FF80FF));
  }

  @Test
  void zeroMeansUnknown() {
    assertEquals("", PhoenixFirmware.format(0));
  }
}
