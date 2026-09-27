// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Pure logic for locating a break along a CAN daisy chain.
 *
 * <p>A chain lists a bus's devices in wiring order from the SystemCore. A single break in the cable
 * leaves every device before it connected and every device after it disconnected. This class finds
 * that split and describes where to look. It uses no WPILib or Phoenix types.
 */
public final class CANChain {
  private CANChain() {}

  /**
   * Finds a single break in the chain.
   *
   * <p>Returns {@code k} when devices {@code 0..k-1} are connected, devices {@code k..n-1} are
   * disconnected, and at least two devices are disconnected. With one device down, a failed device
   * and a failed cable look the same, and that device's own alert already names it. Any other
   * pattern, such as a single device down in the middle, is not a chain break.
   *
   * @param connected each device's connection state, in chain order
   * @return the index of the first disconnected device, or -1 if the pattern is not a chain break
   */
  public static int findBreak(boolean[] connected) {
    int n = connected.length;
    int k = 0;
    while (k < n && connected[k]) k++;
    for (int i = k; i < n; i++) {
      if (connected[i]) return -1;
    }
    return n - k >= 2 ? k : -1;
  }

  /**
   * Describes where to look for a break found by {@link #findBreak}.
   *
   * @param bus the bus name, e.g. "SC1"
   * @param chain the bus's devices in chain order
   * @param k the index of the first disconnected device
   * @param traced when and by whom the chain order was traced from the wiring
   */
  public static String hint(
      String bus, List<? extends CANChainDevice> chain, int k, String traced) {
    int last = chain.size() - 1;
    String prefix = "CAN chain break on " + bus + " (order traced " + traced + "): ";
    if (k == 0) {
      return prefix
          + "no device responds. Check the SystemCore "
          + bus
          + " port and plug, and the cable to "
          + describe(chain, 0)
          + ".";
    }
    return prefix
        + "#0–#"
        + (k - 1)
        + " respond, #"
        + k
        + "–#"
        + last
        + " don't. Check "
        + describe(chain, k - 1)
        + "'s outgoing connector, the cable, and "
        + describe(chain, k)
        + "'s incoming connector.";
  }

  private static String describe(List<? extends CANChainDevice> chain, int index) {
    CANChainDevice device = chain.get(index);
    return "#" + index + " " + device.label() + " (ID " + device.id() + ")";
  }

  /**
   * Checks that a chain and its connection sources match one to one.
   *
   * @param chain the bus's devices in chain order
   * @param sourceIds the CAN IDs that have a connection source
   * @return a description of the first problem found, or null if the chain is usable
   */
  public static String validate(List<? extends CANChainDevice> chain, Set<Integer> sourceIds) {
    Set<Integer> chainIds = new HashSet<>();
    for (CANChainDevice device : chain) {
      if (!chainIds.add(device.id())) {
        return "CAN ID " + device.id() + " appears twice in the chain";
      }
      if (!sourceIds.contains(device.id())) {
        return device.label() + " (ID " + device.id() + ") has no connection source";
      }
    }
    for (int id : sourceIds) {
      if (!chainIds.contains(id)) {
        return "CAN ID " + id + " has a connection source but is not in the chain";
      }
    }
    return null;
  }
}
