// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.util.Alert;

/**
 * Logs a power distribution module and reports whether it is responding on CAN.
 *
 * <p>WPILib has no connected check for power distribution, so a voltage of exactly 0.0 serves as
 * one. On SystemCore, a REV PDH reading with no status frame in the last 40 ms fails with a CAN
 * timeout, and the HAL then returns a voltage of 0.0 ({@code REVPDH.cpp}, allwpilib
 * v2027.0.0-alpha-7). A responding PDH can't read 0 V, because it powers the SystemCore running
 * this code. While it reads 0.0, the other readings are skipped: each failed read sends an error to
 * the Driver Station.
 */
public class LoggedPowerDistribution extends PowerDistribution {
  @AutoLog
  public static class PowerDistributionInputs {
    public boolean connected = true;
  }

  private final String key;
  private final PowerDistributionInputsAutoLogged inputs = new PowerDistributionInputsAutoLogged();
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Alert disconnectedAlert;

  /**
   * Creates a logged power distribution module.
   *
   * @param busId the SystemCore CAN port
   * @param module the CAN device ID of the power distribution module
   * @param moduleType the type of power distribution module
   * @param logKey the AdvantageKit log key prefix
   */
  public LoggedPowerDistribution(CANPort busId, int module, ModuleType moduleType, String logKey) {
    super(busId, module, moduleType);
    this.key = logKey;
    disconnectedAlert =
        new Alert(
            logKey + "/disconnected",
            "Power distribution (ID " + module + ") not responding on " + busId + ".",
            Alert.Level.HIGH);
  }

  /** Returns true while the power distribution module is responding on CAN. */
  public boolean isConnected() {
    return inputs.connected;
  }

  public void log() {
    double voltage = getVoltage();
    inputs.connected = connectedDebounce.calculate(voltage > 0.0);
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
