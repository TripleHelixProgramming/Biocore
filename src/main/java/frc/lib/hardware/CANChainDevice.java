// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

/**
 * A device on a CAN bus daisy chain. Enum constants implement {@link #name()} automatically, so
 * each device's name in alerts is its constant name, e.g. FRONT_LEFT_DRIVE.
 */
public interface CANChainDevice {
  /** Returns the device's CAN ID. */
  int id();

  /** Returns the device's name, as written in the code. */
  String name();
}
