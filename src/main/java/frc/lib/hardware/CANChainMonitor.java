// Copyright (c) 2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.util.Alert;

/**
 * Raises an alert naming where a CAN bus's daisy chain is broken.
 *
 * <p>Each cycle it reads every device's connection state in chain order, logs the raw break index,
 * and raises a HIGH alert once the same break has held for the hold time (see {@link
 * CANChainTracker}). The connection sources should read logged inputs, so replay reproduces the
 * alert.
 *
 * <p>The monitor stays off when the chain order hasn't been traced, or when the chain and the
 * connection sources don't match one to one. A mismatch is reported once to the Driver Station.
 */
public class CANChainMonitor {
  /** Default hold time. It exceeds the 2 s Redux timeout minus the 0.5 s Phoenix debounce. */
  public static final double DEFAULT_STABLE_SECONDS = 2.5;

  private final String bus;
  private final List<? extends CANChainDevice> chain;
  private final String traced;
  private final BooleanSupplier[] sources;
  private final boolean enabled;
  private final CANChainTracker tracker;
  private final Alert alert;
  private final String indexKey;
  private int shownBreak = -1;

  /**
   * @param bus the bus name, e.g. "SC1"
   * @param chain the bus's devices in chain order
   * @param traced when and by whom the chain order was traced, or null if not yet
   * @param connectedById each device's connection state, keyed by CAN ID
   * @param stableSeconds how long a break must hold before the alert shows
   */
  public CANChainMonitor(
      String bus,
      List<? extends CANChainDevice> chain,
      String traced,
      Map<Integer, BooleanSupplier> connectedById,
      double stableSeconds) {
    this.bus = bus;
    this.chain = chain;
    this.traced = traced;
    this.tracker = new CANChainTracker(stableSeconds);
    this.alert = new Alert("CANBus/" + bus + "/chainBreak", "", Alert.Level.HIGH);
    this.indexKey = "CANBus/" + bus + "/ChainBreakIndex";

    String problem = traced == null ? null : CANChain.validate(chain, connectedById.keySet());
    if (problem != null) {
      DriverStationErrors.reportWarning(
          "CAN chain hint for " + bus + " is off: " + problem + ".", false);
    }
    enabled = traced != null && problem == null;
    sources = new BooleanSupplier[chain.size()];
    if (enabled) {
      for (int i = 0; i < sources.length; i++) {
        sources[i] = connectedById.get(chain.get(i).id());
      }
    }
  }

  /**
   * Checks the chain for a break and updates the alert.
   *
   * @param now the current timestamp in seconds
   */
  public void update(double now) {
    if (!enabled) return;
    boolean[] connected = new boolean[sources.length];
    for (int i = 0; i < sources.length; i++) connected[i] = sources[i].getAsBoolean();
    int rawBreak = CANChain.findBreak(connected);
    Logger.recordOutput(indexKey, rawBreak);

    int settledBreak = tracker.update(now, rawBreak);
    if (settledBreak >= 0 && settledBreak != shownBreak) {
      alert.setText(CANChain.hint(bus, chain, settledBreak, traced));
    }
    shownBreak = settledBreak;
    alert.set(settledBreak >= 0);
  }
}
