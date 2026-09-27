// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class CANChainTrackerTest {
  private static final double STEP = 0.02;

  @Test
  void noBreakReportsNothing() {
    var tracker = new CANChainTracker(2.5);
    for (double t = 0; t < 5; t += STEP) assertEquals(-1, tracker.update(t, -1));
  }

  @Test
  void breakIsReportedOnlyAfterTheHoldTime() {
    var tracker = new CANChainTracker(2.5);
    assertEquals(-1, tracker.update(10.0, 5));
    assertEquals(-1, tracker.update(12.49, 5));
    assertEquals(5, tracker.update(12.5, 5));
  }

  @Test
  void changingIndexRestartsTheHoldTime() {
    var tracker = new CANChainTracker(2.5);
    tracker.update(0.0, 5);
    assertEquals(-1, tracker.update(2.0, 4));
    assertEquals(-1, tracker.update(4.49, 4));
    assertEquals(4, tracker.update(4.5, 4));
  }

  @Test
  void recoveryClearsImmediately() {
    var tracker = new CANChainTracker(2.5);
    tracker.update(0.0, 5);
    assertEquals(5, tracker.update(3.0, 5));
    assertEquals(-1, tracker.update(3.02, -1));
  }

  /**
   * A break before a slow device: the fast devices after it drop first, so the pattern briefly
   * shows the break one link too far along, then settles. Only the settled index is ever reported.
   */
  @Test
  void skewedDetectionOnlyReportsTheSettledBreak() {
    var tracker = new CANChainTracker(2.5);
    double t = 0.0;
    for (; t < 1.5; t += STEP) assertEquals(-1, tracker.update(t, 4));
    for (; t < 6.0; t += STEP) {
      int reported = tracker.update(t, 3);
      assertNotEquals(4, reported);
    }
    assertEquals(3, tracker.update(t, 3));
  }
}
