// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.hardware.CANBusHealth.Severity;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class CANBusHealthTest {
  @ParameterizedTest(name = "status={0} state={1} busOffRose={2} restartRose={3} stale={4}")
  @CsvSource({
    "OK, ErrorActive, false, false, false, NONE",
    "OK, ErrorWarning, false, false, false, MEDIUM",
    "OK, ErrorPassive, false, false, false, HIGH",
    "OK, BusOff, false, false, false, HIGH",
    "OK, Stopped, false, false, false, HIGH",
    "InvalidNetwork, ErrorActive, false, false, false, HIGH", // failed status read
    "OK, ErrorActive, true, false, false, HIGH",
    "OK, ErrorActive, false, true, false, HIGH",
    "OK, ErrorActive, false, false, true, HIGH",
  })
  void severityOfOneSample(
      String status,
      String state,
      boolean busOffRose,
      boolean restartRose,
      boolean stale,
      Severity expected) {
    assertEquals(expected, CANBusHealth.severity(status, state, busOffRose, restartRose, stale));
  }

  @Test
  void noSamplesNeverGoStale() {
    var health = new CANBusHealth();
    for (double t = 0; t < 10; t += 0.02) {
      health.update(t, 0, "OK", "ErrorActive", 0, 0);
      assertFalse(health.isHighActive(t));
    }
  }

  @Test
  void firstSampleWithOldCountsIsNotARise() {
    var health = new CANBusHealth();
    health.update(0.0, 1, "OK", "ErrorActive", 7, 3);
    assertFalse(health.isHighActive(0.0));
  }

  @Test
  void busOffIncreaseAfterFirstSampleIsHigh() {
    var health = new CANBusHealth();
    health.update(0.0, 1, "OK", "ErrorActive", 7, 3);
    health.update(0.4, 2, "OK", "ErrorActive", 8, 3);
    assertTrue(health.isHighActive(0.4));
  }

  @Test
  void restartIncreaseAfterFirstSampleIsHigh() {
    var health = new CANBusHealth();
    health.update(0.0, 1, "OK", "ErrorActive", 0, 3);
    health.update(0.4, 2, "OK", "ErrorActive", 0, 4);
    assertTrue(health.isHighActive(0.4));
  }

  @Test
  void stalledReaderGoesStale() {
    var health = new CANBusHealth();
    health.update(0.0, 1, "OK", "ErrorActive", 0, 0);
    health.update(1.0, 1, "OK", "ErrorActive", 0, 0);
    assertFalse(health.isHighActive(1.0));
    health.update(1.6, 1, "OK", "ErrorActive", 0, 0);
    assertTrue(health.isHighActive(1.6));
  }

  @Test
  void advancingReaderStaysFresh() {
    var health = new CANBusHealth();
    long count = 1;
    for (double t = 0; t < 5; t += 0.02) {
      if (t >= count * 0.4) count++;
      health.update(t, count, "OK", "ErrorActive", 0, 0);
      assertFalse(health.isHighActive(t));
    }
  }

  @Test
  void highHoldsForHalfASecondAfterLastError() {
    var health = new CANBusHealth();
    health.update(0.0, 1, "OK", "BusOff", 0, 0);
    health.update(0.02, 1, "OK", "ErrorActive", 0, 0);
    assertTrue(health.isHighActive(0.49));
    assertFalse(health.isHighActive(0.5));
  }

  @Test
  void mediumHoldsForHalfASecondAfterLastWarning() {
    var health = new CANBusHealth();
    health.update(0.0, 1, "OK", "ErrorWarning", 0, 0);
    assertTrue(health.isMediumActive(0.0));
    assertFalse(health.isHighActive(0.0));
    health.update(0.02, 1, "OK", "ErrorActive", 0, 0);
    assertTrue(health.isMediumActive(0.49));
    assertFalse(health.isMediumActive(0.5));
  }

  @Test
  void startsInactive() {
    var health = new CANBusHealth();
    assertFalse(health.isHighActive(0.0));
    assertFalse(health.isMediumActive(0.0));
  }
}
