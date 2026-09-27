// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.wpilib.hardware.bus.CANPort;

/**
 * A CAN bus's devices in daisy-chain order, and the logic for locating a break along it.
 *
 * <p>Declare the chain first, then one device per line, starting with the device wired closest to
 * the SystemCore:
 *
 * <pre>{@code
 * public static final CANChain CHAIN = new CANChain(BUS_ID);
 * public static final int FRONT_LEFT_DRIVE = CHAIN.add(28, "FrontLeft drive");
 * public static final int FRONT_LEFT_TURN = CHAIN.add(29, "FrontLeft turn");
 * }</pre>
 *
 * <p>Java runs static field initializers in the order they are written, so the order of the {@code
 * add} lines is the chain order, and a device's position is its CAN index. The chain freezes the
 * first time {@link #devices()} is read; a later {@code add} throws.
 *
 * <p>A single break in the cable leaves every device before it connected and every device after it
 * disconnected. The static methods find that split and describe where to look. Apart from the
 * {@link CANPort} enum, they use no WPILib or Phoenix types.
 */
public class CANChain {
  /**
   * A device on the chain.
   *
   * @param id the device's CAN ID
   * @param label a name that tells the pit crew where the device is, e.g. "FrontLeft drive"
   */
  public record Device(int id, String label) {}

  /**
   * Where a device sits: its bus and CAN ID. A CAN ID is only unique within one bus.
   *
   * @param port the SystemCore CAN port the device is wired to
   * @param id the device's CAN ID
   */
  public record Address(CANPort port, int id) {}

  private final CANPort port;
  private final List<Device> devices = new ArrayList<>();
  private boolean frozen = false;

  /**
   * @param port the SystemCore CAN port this chain starts from
   */
  public CANChain(CANPort port) {
    this.port = port;
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
   * @return the CAN ID, so the constant holds the ID
   */
  public int add(int id, String label) {
    if (frozen) {
      throw new IllegalStateException(
          label + " (ID " + id + ") was added after the chain was first read.");
    }
    devices.add(new Device(id, label));
    return id;
  }

  /** Returns the devices in chain order. No devices can be added afterward. */
  public List<Device> devices() {
    frozen = true;
    return List.copyOf(devices);
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
   * @param bus the bus name, e.g. "SC1"
   * @param chain the bus's devices in chain order
   * @param k the index of the first disconnected device
   * @param traced when and by whom the chain order was traced from the wiring
   */
  public static String hint(String bus, List<Device> chain, int k, String traced) {
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

  private static String describe(List<Device> chain, int index) {
    Device device = chain.get(index);
    return "#" + index + " " + device.label() + " (ID " + device.id() + ")";
  }

  /**
   * Checks that a chain and its connection sources match one to one.
   *
   * @param chain the bus's devices in chain order
   * @param sourceIds the CAN IDs that have a connection source
   * @return a description of the first problem found, or null if the chain is usable
   */
  public static String validate(List<Device> chain, Set<Integer> sourceIds) {
    Set<Integer> chainIds = new HashSet<>();
    for (Device device : chain) {
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

  /**
   * Collects the connection states of the devices on one bus.
   *
   * <p>Each subsystem reports every device it owns, on any bus. This keeps the ones on {@code
   * port}, keyed by CAN ID.
   *
   * @param port the bus to collect
   * @param sources each subsystem's connection states, keyed by address
   * @return the connection states on {@code port}, keyed by CAN ID
   * @throws IllegalArgumentException if two sources report the same address
   */
  @SafeVarargs
  public static Map<Integer, BooleanSupplier> connectionsOn(
      CANPort port, Map<Address, BooleanSupplier>... sources) {
    Map<Integer, BooleanSupplier> onPort = new HashMap<>();
    for (Map<Address, BooleanSupplier> source : sources) {
      for (var entry : source.entrySet()) {
        Address address = entry.getKey();
        if (address.port() != port) continue;
        if (onPort.put(address.id(), entry.getValue()) != null) {
          throw new IllegalArgumentException(
              "CAN ID " + address.id() + " on " + port + " is reported twice");
        }
      }
    }
    return onPort;
  }
}
