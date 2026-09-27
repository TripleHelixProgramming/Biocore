// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class LoggedPowerDistributionTest {
  @Test
  void connectedModuleIsReadEveryCycle() {
    assertTrue(LoggedPowerDistribution.shouldRead(true, 10.02, 10.0));
  }

  @Test
  void missingModuleIsReadOncePerRetryPeriod() {
    assertFalse(LoggedPowerDistribution.shouldRead(false, 10.02, 10.0));
    assertFalse(LoggedPowerDistribution.shouldRead(false, 10.98, 10.0));
    assertTrue(LoggedPowerDistribution.shouldRead(false, 11.0, 10.0));
  }

  @Test
  void firstCycleAlwaysReads() {
    assertTrue(LoggedPowerDistribution.shouldRead(false, 0.0, Double.NEGATIVE_INFINITY));
  }
}
