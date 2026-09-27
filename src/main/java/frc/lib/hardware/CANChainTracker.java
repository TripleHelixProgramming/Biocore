// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

/**
 * Reports a chain break only after the break index has held steady.
 *
 * <p>Devices notice a lost connection at different speeds. For example, Phoenix connection flags
 * here drop 0.5 s after the last message, while a Redux device's {@code isConnected()} waits 2 s.
 * During a break, the fast devices drop first, so the pattern can briefly show a split further
 * along the chain than the real one. The hold time must be longer than the gap between the slowest
 * and fastest devices on the bus, so the reported index is the settled one.
 */
public class CANChainTracker {
  private final double stableSeconds;
  private int currentBreak = -1;
  private double since = 0.0;

  /**
   * @param stableSeconds how long the same break index must hold before it is reported
   */
  public CANChainTracker(double stableSeconds) {
    this.stableSeconds = stableSeconds;
  }

  /**
   * Takes this cycle's break index and returns the settled one.
   *
   * @param now the current timestamp in seconds
   * @param breakIndex this cycle's result from {@link CANChain#findBreak}
   * @return the break index once it has held for the hold time, otherwise -1
   */
  public int update(double now, int breakIndex) {
    if (breakIndex != currentBreak) {
      currentBreak = breakIndex;
      since = now;
    }
    return currentBreak >= 0 && now - since >= stableSeconds ? currentBreak : -1;
  }
}
