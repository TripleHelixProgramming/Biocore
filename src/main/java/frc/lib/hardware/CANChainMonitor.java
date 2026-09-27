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
 * and raises a HIGH alert once the same break has held for the hold time. The connection sources
 * should read logged inputs, so replay reproduces the alert.
 *
 * <p>The hold time matters because devices notice a lost connection at different speeds. Phoenix
 * connection flags here drop 0.5 s after the last message, while a Redux device's {@code
 * isConnected()} waits 2 s. During a break, the fast devices drop first, so the pattern can briefly
 * show a split further along the chain than the real one. The hold time must be longer than the gap
 * between the slowest and fastest devices on the bus, so the reported break is the settled one.
 *
 * <p>While the bus itself is faulted (the bus-health HIGH alert), a break further along the chain
 * is not trusted: a bus fault can drop devices anywhere. Only a break at the SystemCore end is
 * still shown, because "check the SystemCore port and plug" is also the right advice for a bus
 * fault.
 *
 * <p>The monitor stays off when the chain order hasn't been traced. It also stays off when a device
 * on the chain has no connection source, which is reported once to the Driver Station.
 */
public class CANChainMonitor {
  /** Default hold time. It exceeds the 2 s Redux timeout minus the 0.5 s Phoenix debounce. */
  public static final double DEFAULT_STABLE_SECONDS = 2.5;

  private final CANChain chain;
  private final String traced;
  private final BooleanSupplier[] sources;
  private final boolean enabled;
  private final double stableSeconds;
  private final Alert alert;
  private final String indexKey;
  private int rawBreak = -1;
  private double rawBreakSince = 0.0;
  private int shownBreak = -1;

  /**
   * @param chain the bus's devices in chain order
   * @param traced when and by whom the chain order was traced, or null if not yet
   * @param connections each device's connection state, on any bus
   * @param stableSeconds how long a break must hold before the alert shows
   */
  public CANChainMonitor(
      CANChain chain,
      String traced,
      Map<CANChain.Device, BooleanSupplier> connections,
      double stableSeconds) {
    this.chain = chain;
    String bus = chain.name();
    this.traced = traced;
    this.stableSeconds = stableSeconds;
    this.alert = new Alert("CANBus/" + bus + "/chainBreak", "", Alert.Level.HIGH);
    this.indexKey = "CANBus/" + bus + "/ChainBreakIndex";

    List<CANChain.Device> devices = chain.devices();
    String problem = traced == null ? null : CANChain.validate(devices, connections.keySet());
    if (problem != null) {
      DriverStationErrors.reportWarning(
          "CAN chain hint for " + bus + " is off: " + problem + ".", false);
    }
    enabled = traced != null && problem == null;
    sources = new BooleanSupplier[devices.size()];
    if (enabled) {
      for (int i = 0; i < sources.length; i++) sources[i] = connections.get(devices.get(i));
    }
  }

  /**
   * Checks the chain for a break and updates the alert.
   *
   * @param now the current timestamp in seconds
   * @param busFaulted whether the bus-health HIGH alert is active
   * @return the index of the break the alert shows, or -1 when it shows none
   */
  public int update(double now, boolean busFaulted) {
    if (!enabled) return -1;
    boolean[] connected = new boolean[sources.length];
    for (int i = 0; i < sources.length; i++) connected[i] = sources[i].getAsBoolean();
    int breakIndex = CANChain.findBreak(connected);
    Logger.recordOutput(indexKey, breakIndex);

    int candidate = busFaulted && breakIndex > 0 ? -1 : breakIndex;
    if (candidate != rawBreak) {
      rawBreak = candidate;
      rawBreakSince = now;
    }
    int settledBreak = rawBreak >= 0 && now - rawBreakSince >= stableSeconds ? rawBreak : -1;
    if (settledBreak >= 0 && settledBreak != shownBreak) {
      alert.setText(chain.hint(settledBreak, traced));
    }
    shownBreak = settledBreak;
    alert.set(settledBreak >= 0);
    return settledBreak;
  }
}
