// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.wpilib.hardware.bus.CANPort;

/**
 * A CAN bus's devices in daisy-chain order, and the logic for locating a break along it.
 *
 * <p>Declare the chain first, then one device per line, starting with the device wired closest to
 * the SystemCore:
 *
 * <pre>{@code
 * public static final CANChain CHAIN = new CANChain("SC1", CANPort.CAN_S1);
 * public static final CANChain.Device FRONT_LEFT_DRIVE = CHAIN.add(28, "FrontLeft drive");
 * public static final CANChain.Device FRONT_LEFT_TURN = CHAIN.add(29, "FrontLeft turn");
 * }</pre>
 *
 * <p>Java runs static field initializers in the order they are written, and finishes them all
 * before any other class can read the chain (JLS §12.4.2). So the order of the {@code add} lines is
 * the chain order, and a device's position is its CAN index. Each device carries its bus, so code
 * that uses a device never has to name the bus again.
 *
 * <p>A single break in the cable leaves every device before it connected and every device after it
 * disconnected. The static methods find that split and describe where to look. Apart from the
 * {@link CANPort} enum, they use no WPILib or Phoenix types.
 */
public class CANChain {
  /**
   * A device on the chain. A CAN ID is only unique within one bus, so the device carries both.
   *
   * @param port the SystemCore CAN port the device is wired to
   * @param id the device's CAN ID
   * @param label a name that tells the pit crew where the device is, e.g. "FrontLeft drive"
   */
  public record Device(CANPort port, int id, String label) {}

  private final String name;
  private final CANPort port;
  private final List<Device> devices = new ArrayList<>();

  /**
   * @param name the bus name used in logs and alerts, e.g. "SC1"
   * @param port the SystemCore CAN port this chain starts from
   */
  public CANChain(String name, CANPort port) {
    this.name = name;
    this.port = port;
  }

  /** Returns the bus name used in logs and alerts. */
  public String name() {
    return name;
  }

  /** Returns the SystemCore CAN port this chain starts from. */
  public CANPort port() {
    return port;
  }

  /**
   * Adds the next device along the chain.
   *
   * @param id the device's CAN ID
   * @param label a name that tells the pit crew where the device is
   * @return the device, on this chain's port
   */
  public Device add(int id, String label) {
    Device device = new Device(port, id, label);
    devices.add(device);
    return device;
  }

  /** Returns the devices in chain order, as a read-only view. */
  public List<Device> devices() {
    return Collections.unmodifiableList(devices);
  }

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
   * @param k the index of the first disconnected device
   * @param traced when and by whom the chain order was traced from the wiring
   */
  public String hint(int k, String traced) {
    int last = devices.size() - 1;
    String prefix = "CAN chain break on " + name + " (order traced " + traced + "): ";
    if (k == 0) {
      return prefix
          + "no device responds. Check the SystemCore "
          + name
          + " port and plug, and the cable to "
          + describe(0)
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
        + describe(k - 1)
        + "'s outgoing connector, the cable, and "
        + describe(k)
        + "'s incoming connector.";
  }

  private String describe(int index) {
    Device device = devices.get(index);
    return "#" + index + " " + device.label() + " (ID " + device.id() + ")";
  }

  /**
   * Checks that every device on a chain has a connection source.
   *
   * @param chain the bus's devices in chain order
   * @param sources the devices that have a connection source, on any bus
   * @return a description of the first problem found, or null if the chain is usable
   */
  public static String validate(List<Device> chain, Set<Device> sources) {
    Set<Integer> chainIds = new HashSet<>();
    for (Device device : chain) {
      if (!chainIds.add(device.id())) {
        return "CAN ID " + device.id() + " appears twice in the chain";
      }
      if (!sources.contains(device)) {
        return device.label() + " (ID " + device.id() + ") has no connection source";
      }
    }
    return null;
  }
}
