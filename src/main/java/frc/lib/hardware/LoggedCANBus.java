// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import com.ctre.phoenix6.CANBus;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class LoggedCANBus {
  @AutoLog
  public static class CANBusStatusInputs {
    public double busUtilization = 0.0;
    public long busOffCount = 0;
    public long txFullCount = 0;
    public long receiveErrorCount = 0;
    public long transmitErrorCount = 0;
  }

  private final CANBus bus;
  private final String key;
  private final CANBusStatusInputsAutoLogged inputs = new CANBusStatusInputsAutoLogged();

  /**
   * Creates a logged CAN bus status reporter.
   *
   * @param name the bus name (used as the log key)
   * @param bus the CAN bus to report status for
   */
  public LoggedCANBus(String name, CANBus bus) {
    this.bus = bus;
    this.key = "CANBus/" + name;
  }

  public void log() {
    var status = bus.getStatus();
    inputs.busUtilization = status.BusUtilization;
    inputs.busOffCount = status.BusOffCount;
    inputs.txFullCount = status.TxFullCount;
    inputs.receiveErrorCount = status.REC;
    inputs.transmitErrorCount = status.TEC;
    Logger.processInputs(key, inputs);
  }
}
