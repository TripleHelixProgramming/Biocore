// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

/**
 * A device on a CAN bus daisy chain.
 *
 * @param id the device's CAN ID
 * @param label a name that tells the pit crew where the device is, e.g. "FrontLeft drive"
 */
public record CANChainDevice(int id, String label) {}
