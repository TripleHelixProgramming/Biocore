// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import com.ctre.phoenix6.CANBus;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.system.Timer;

/** Logs a CAN bus's status and watches its daisy chain for a break. */
public class LoggedCANBus {
  @AutoLog
  public static class CANBusStatusInputs {
    public double busUtilization = 0.0;
    public long busOffCount = 0;
    public long txFullCount = 0;
    public long receiveErrorCount = 0;
    public long transmitErrorCount = 0;
  }

  private final String name;
  private final CANBus bus;
  private final String key;
  private final CANChain chain;
  private final String chainOrderTraced;
  private final CANBusStatusInputsAutoLogged inputs = new CANBusStatusInputsAutoLogged();
  private CANChainMonitor chainMonitor = null;

  /**
   * Creates a logged CAN bus status reporter.
   *
   * @param name the bus name (used as the log key)
   * @param bus the CAN bus to report status for
   * @param chain the bus's devices in daisy-chain order
   * @param chainOrderTraced when and by whom the chain order was traced, or null if not yet
   */
  public LoggedCANBus(String name, CANBus bus, CANChain chain, String chainOrderTraced) {
    this.name = name;
    this.bus = bus;
    this.key = "CANBus/" + name;
    this.chain = chain;
    this.chainOrderTraced = chainOrderTraced;
  }

  /**
   * Starts watching the daisy chain for a break. Call once, after the devices that report the
   * connection states exist.
   *
   * @param connectedById each device's connection state, keyed by CAN ID, read from logged inputs
   */
  public void monitorChain(Map<Integer, BooleanSupplier> connectedById) {
    chainMonitor =
        new CANChainMonitor(
            name, chain, chainOrderTraced, connectedById, CANChainMonitor.DEFAULT_STABLE_SECONDS);
  }

  public void log() {
    var status = bus.getStatus();
    inputs.busUtilization = status.BusUtilization;
    inputs.busOffCount = status.BusOffCount;
    inputs.txFullCount = status.TxFullCount;
    inputs.receiveErrorCount = status.REC;
    inputs.transmitErrorCount = status.TEC;
    Logger.processInputs(key, inputs);

    if (chainMonitor != null) chainMonitor.update(Timer.getTimestamp());
  }
}
