// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;

/**
 * Logs a power distribution module and reports whether it is responding on CAN.
 *
 * <p>WPILib has no connected check for power distribution, so a voltage of exactly 0.0 serves as
 * one. On SystemCore, a REV PDH reading with no status frame in the last 40 ms fails with a CAN
 * timeout, and the HAL then returns a voltage of 0.0 ({@code REVPDH.cpp}, allwpilib
 * v2027.0.0-alpha-7). A responding PDH can't read 0 V, because it powers the SystemCore running
 * this code.
 *
 * <p>Each failed read sends an error to the Driver Station ({@code PowerDistributionJNI.cpp},
 * allwpilib v2027.0.0-alpha-7). While the module is missing, the other readings are skipped and the
 * voltage is read only once per {@link #MISSING_RETRY_SECONDS}, timed with {@link
 * Timer#advanceIfElapsed}. That limits the errors to about one per second. A module that comes back
 * is noticed at its next read.
 */
public class LoggedPowerDistribution extends PowerDistribution {
  @AutoLog
  public static class PowerDistributionInputs {
    public boolean connected = true;
  }

  /** How often a missing module is read again, in seconds. */
  public static final double MISSING_RETRY_SECONDS = 1.0;

  private final String key;
  private final CANChain.Device device;
  private final PowerDistributionInputsAutoLogged inputs = new PowerDistributionInputsAutoLogged();
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Alert disconnectedAlert;
  // Restarted on every connected read, so the first retry comes one period after the last one
  private final Timer retryTimer = Timer.createStarted();

  /**
   * Creates a logged power distribution module.
   *
   * @param device the power distribution module's CAN device
   * @param moduleType the type of power distribution module
   * @param logKey the AdvantageKit log key prefix
   */
  public LoggedPowerDistribution(CANChain.Device device, ModuleType moduleType, String logKey) {
    super(device.port(), device.id(), moduleType);
    this.key = logKey;
    this.device = device;
    disconnectedAlert =
        new Alert(
            logKey + "/disconnected",
            "Power distribution (ID " + device.id() + ") not responding on " + device.port() + ".",
            Alert.Level.HIGH);
  }

  /** Returns true while the power distribution module is responding on CAN. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Returns this module's connection state, keyed by its device, from the logged input. */
  public Map<CANChain.Device, BooleanSupplier> canConnections() {
    return Map.of(device, this::isConnected);
  }

  public void log() {
    double voltage = 0.0;
    if (inputs.connected || retryTimer.advanceIfElapsed(MISSING_RETRY_SECONDS)) {
      voltage = getVoltage();
      inputs.connected = connectedDebounce.calculate(voltage > 0.0);
      if (inputs.connected) retryTimer.restart();
    }
    Logger.processInputs(key, inputs);
    disconnectedAlert.set(!inputs.connected);
    if (voltage == 0.0) return;

    Logger.recordOutput(key + "/Voltage", voltage);
    Logger.recordOutput(key + "/TotalCurrentAmps", getTotalCurrent());
    Logger.recordOutput(key + "/TotalPowerWatts", getTotalPower());
    Logger.recordOutput(key + "/TotalEnergyJoules", getTotalEnergy());
    Logger.recordOutput(key + "/TemperatureCelsius", getTemperature());
    Logger.recordOutput(key + "/SwitchableChannelActive", getSwitchableChannel());
    int n = getNumChannels();
    double[] currents = new double[n];
    for (int i = 0; i < n; i++) currents[i] = getCurrent(i);
    Logger.recordOutput(key + "/ChannelCurrentsAmps", currents);
  }
}
