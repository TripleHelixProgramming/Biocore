// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import frc.lib.Util;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;

/**
 * Logs a CAN bus's controller status, raises alerts when the bus is in trouble, and watches its
 * daisy chain for a break.
 *
 * <p>{@link CANBus#getStatus()} can block for up to 1 ms (its Javadoc), so a background thread
 * reads it and the main loop only copies the latest sample. Without {@link #start()}, as in
 * simulation, the inputs keep their defaults. Alerts come only from the logged inputs, so replay
 * reproduces them.
 *
 * <p>Each {@link #log()} runs one bus-health step: log the status, update the bus alerts, then
 * update the chain-break hint, which is held back while the bus alert shows a bus-wide fault.
 */
public class LoggedCANBus {
  @AutoLog
  public static class CANBusStatusInputs {
    public double busUtilization = 0.0;
    public long busOffCount = 0;
    public long txFullCount = 0;
    public long receiveErrorCount = 0;
    public long transmitErrorCount = 0;
    public long busErrorCount = 0;
    public long arbitrationLostCount = 0;
    public long restartCount = 0;
    public String state = "ErrorActive";
    public String status = "OK";
    public long sampleCount = 0;
  }

  /** How often the background thread reads the bus status, in milliseconds. */
  private static final long SAMPLE_PERIOD_MS = 400;

  /** One status read, paired with its sequence number so the two are published together. */
  private record Sample(CANBusStatus status, long sequence) {}

  private final String name;
  private final String key;
  private final CANBus bus;
  private final CANChain chain;
  private final String chainOrderTraced;
  private final CANBusStatusInputsAutoLogged inputs = new CANBusStatusInputsAutoLogged();
  private final CANBusHealth health = new CANBusHealth();
  private final Alert errorAlert;
  private final Alert warningAlert;
  private CANChainMonitor chainMonitor = null;
  private volatile Sample latest = null;
  private Thread reader = null;

  /**
   * Creates a logged CAN bus status reporter.
   *
   * @param chain the bus's devices in daisy-chain order, which also names the bus
   * @param bus the Phoenix bus on the chain's port
   * @param chainOrderTraced when and by whom the chain order was traced, or null if not yet
   */
  public LoggedCANBus(CANChain chain, CANBus bus, String chainOrderTraced) {
    this.name = chain.name();
    this.key = "CANBus/" + name;
    this.bus = bus;
    this.chain = chain;
    this.chainOrderTraced = chainOrderTraced;
    errorAlert = new Alert(key + "/errors", "", Alert.Level.HIGH);
    warningAlert = new Alert(key + "/warning", "", Alert.Level.MEDIUM);
  }

  /**
   * Starts watching the daisy chain for a break. Call once, after the devices that report the
   * connection states exist. Does nothing while the chain order is untraced.
   *
   * @param connections each device's connection state, on any bus. The states should come from
   *     logged inputs, so replay reproduces the alert.
   */
  public void monitorChain(Map<CANChain.Device, BooleanSupplier> connections) {
    if (chainOrderTraced == null) return;
    chainMonitor =
        new CANChainMonitor(
            chain, chainOrderTraced, connections, CANChainMonitor.DEFAULT_STABLE_SECONDS);
  }

  /** Reads the status once, then starts the background thread that keeps reading it. */
  public synchronized void start() {
    if (reader != null) return;
    latest = new Sample(bus.getStatus(), 1);
    reader = new Thread(this::readLoop, "CANBusReader-" + name);
    reader.setDaemon(true);
    reader.start();
  }

  private void readLoop() {
    long sequence = latest.sequence();
    boolean warned = false;
    while (!Thread.currentThread().isInterrupted()) {
      try {
        Thread.sleep(SAMPLE_PERIOD_MS);
      } catch (InterruptedException e) {
        return;
      }
      try {
        sequence++;
        latest = new Sample(bus.getStatus(), sequence);
      } catch (RuntimeException e) {
        // Keep reading. A stalled sample count raises the stale alert.
        if (!warned) {
          DriverStationErrors.reportWarning(
              "CAN status read failed on " + name + ": " + e.getMessage(), false);
          warned = true;
        }
      }
    }
  }

  public void log() {
    Sample sample = latest;
    if (sample != null) {
      CANBusStatus status = sample.status();
      inputs.busUtilization = status.BusUtilization;
      inputs.busOffCount = status.BusOffCount;
      inputs.txFullCount = status.TxFullCount;
      inputs.receiveErrorCount = status.REC;
      inputs.transmitErrorCount = status.TEC;
      inputs.busErrorCount = status.BusErrorCount;
      inputs.arbitrationLostCount = status.ArbitrationLostCount;
      inputs.restartCount = status.RestartCount;
      inputs.state = String.valueOf(status.State);
      inputs.status = status.Status.getName();
      inputs.sampleCount = sample.sequence();
    }
    Logger.processInputs(key, inputs);

    double now = Timer.getTimestamp();
    health.update(
        now,
        inputs.sampleCount,
        inputs.status,
        inputs.state,
        inputs.busOffCount,
        inputs.restartCount);
    boolean busFaulted = health.isHighActive(now);
    Util.setAlert(
        errorAlert,
        busFaulted,
        "CAN errors on "
            + name
            + " ("
            + inputs.state
            + ", "
            + inputs.status
            + "); robot may not be controllable.");
    Util.setAlert(
        warningAlert,
        health.isMediumActive(now),
        "CAN error warning on " + name + " (" + inputs.state + ").");

    if (chainMonitor != null) chainMonitor.update(now, busFaulted);
  }
}
