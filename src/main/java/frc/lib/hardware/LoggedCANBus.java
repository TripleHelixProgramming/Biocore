// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;

/**
 * Logs a CAN bus's controller status from Phoenix and raises alerts when the bus is in trouble.
 *
 * <p>{@link CANBus#getStatus()} can block for up to 1 ms (its Javadoc), so a background thread
 * reads it and the main loop only copies the latest sample. Without {@link #start()}, as in
 * simulation, the inputs keep their defaults. Alerts come only from the logged inputs, so replay
 * reproduces them.
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

  private final CANBus bus;
  private final String name;
  private final String key;
  private final CANBusStatusInputsAutoLogged inputs = new CANBusStatusInputsAutoLogged();
  private final CANBusHealth health = new CANBusHealth();
  private final Alert errorAlert;
  private final Alert warningAlert;
  private volatile Sample latest = null;
  private Thread reader = null;

  /**
   * Creates a logged CAN bus status reporter.
   *
   * @param name the bus name (used as the log key)
   * @param bus the CAN bus to report status for
   */
  public LoggedCANBus(String name, CANBus bus) {
    this.bus = bus;
    this.name = name;
    this.key = "CANBus/" + name;
    errorAlert = new Alert("CANBus/" + name + "/errors", "", Alert.Level.HIGH);
    warningAlert = new Alert("CANBus/" + name + "/warning", "", Alert.Level.MEDIUM);
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
    setAlert(
        errorAlert,
        health.isHighActive(now),
        "CAN errors on "
            + name
            + " ("
            + inputs.state
            + ", "
            + inputs.status
            + "); robot may not be controllable.");
    setAlert(
        warningAlert,
        health.isMediumActive(now),
        "CAN error warning on " + name + " (" + inputs.state + ").");
  }

  private static void setAlert(Alert alert, boolean active, String text) {
    if (active && !text.equals(alert.getText())) alert.setText(text);
    alert.set(active);
  }
}
