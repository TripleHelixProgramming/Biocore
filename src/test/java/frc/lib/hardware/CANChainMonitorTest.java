// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import static org.junit.jupiter.api.Assertions.*;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

class CANChainMonitorTest {
  private static final double STEP = 0.02;
  private static final int DEVICES = 6;
  private static int busCount = 0;

  /** Each device's connection state, which a test changes to simulate a break. */
  private final boolean[] connected = {true, true, true, true, true, true};

  /** Builds a monitor over six devices with IDs 1..6. Each gets its own bus name. */
  private CANChainMonitor monitor(String traced) {
    busCount++;
    var chain = new CANChain("TEST" + busCount, CANPort.CAN_S1);
    Map<CANChain.Device, BooleanSupplier> sources = new HashMap<>();
    for (int i = 0; i < DEVICES; i++) {
      int index = i;
      sources.put(chain.add(i + 1, "Device " + (i + 1)), () -> connected[index]);
    }
    return new CANChainMonitor(chain, traced, sources, 2.5);
  }

  /** Disconnects every device from index k on. */
  private void breakAt(int k) {
    for (int i = 0; i < DEVICES; i++) connected[i] = i < k;
  }

  @Test
  void healthyChainShowsNothing() {
    var monitor = monitor("test");
    for (double t = 0; t < 5; t += STEP) assertEquals(-1, monitor.update(t));
  }

  @Test
  void breakShowsOnlyAfterTheHoldTime() {
    var monitor = monitor("test");
    breakAt(3);
    assertEquals(-1, monitor.update(10.0));
    assertEquals(-1, monitor.update(12.49));
    assertEquals(3, monitor.update(12.5));
  }

  @Test
  void recoveryClearsImmediately() {
    var monitor = monitor("test");
    breakAt(3);
    monitor.update(0.0);
    assertEquals(3, monitor.update(3.0));
    breakAt(DEVICES);
    assertEquals(-1, monitor.update(3.02));
  }

  /**
   * A break before a slow device: the fast devices after it drop first, so the pattern briefly
   * shows the break one link too far along, then settles. Only the settled break is ever shown.
   */
  @Test
  void skewedDetectionOnlyShowsTheSettledBreak() {
    var monitor = monitor("test");
    double t = 0.0;
    breakAt(4);
    for (; t < 1.5; t += STEP) assertEquals(-1, monitor.update(t));
    breakAt(3);
    for (; t < 6.0; t += STEP) assertNotEquals(4, monitor.update(t));
    assertEquals(3, monitor.update(t));
  }

  @Test
  void untracedChainStaysOff() {
    var monitor = monitor(null);
    breakAt(3);
    for (double t = 0; t < 5; t += STEP) assertEquals(-1, monitor.update(t));
  }
}
