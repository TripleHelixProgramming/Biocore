// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import java.util.ArrayList;
import java.util.List;

/**
 * Collects a bus's devices in the order they are declared, which is their daisy-chain order.
 *
 * <p>Use it in a constants class, one device per line, then build the chain after the last device:
 *
 * <pre>{@code
 * private static final CANChainBuilder CHAIN_BUILDER = new CANChainBuilder();
 * public static final int FRONT_LEFT_DRIVE = CHAIN_BUILDER.add(28, "FrontLeft drive");
 * public static final int FRONT_LEFT_TURN = CHAIN_BUILDER.add(29, "FrontLeft turn");
 * public static final List<CANChainDevice> CHAIN = CHAIN_BUILDER.build();
 * }</pre>
 *
 * <p>Java runs static field initializers in the order they are written, so the order of the {@code
 * add} lines is the chain order. A device added after {@link #build()} throws while the class
 * loads, so a misplaced line fails at startup instead of silently leaving the chain.
 */
public class CANChainBuilder {
  private final List<CANChainDevice> devices = new ArrayList<>();
  private boolean built = false;

  /**
   * Adds the next device along the chain.
   *
   * @param id the device's CAN ID
   * @param label a name that tells the pit crew where the device is
   * @return the CAN ID, so the constant holds the ID
   */
  public int add(int id, String label) {
    if (built) {
      throw new IllegalStateException(
          label + " (ID " + id + ") is declared after the chain was built. Move it above CHAIN.");
    }
    devices.add(new CANChainDevice(id, label));
    return id;
  }

  /** Returns the devices in chain order. No devices can be added afterward. */
  public List<CANChainDevice> build() {
    built = true;
    return List.copyOf(devices);
  }
}
