// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.pneumatic.Compressor;
import org.wpilib.hardware.pneumatic.PneumaticsModuleType;

public class LoggedCompressor extends Compressor {
  private final String key;

  /**
   * Creates a logged compressor.
   *
   * @param busId the CAN bus ID (SystemCore transceiver number)
   * @param moduleType the type of pneumatics module
   * @param logKey the AdvantageKit log key prefix
   */
  public LoggedCompressor(int busId, PneumaticsModuleType moduleType, String logKey) {
    super(busId, moduleType);
    this.key = logKey;
  }

  public void log() {
    Logger.recordOutput(key + "/Enabled", isEnabled());
    Logger.recordOutput(key + "/PressureSwitch", getPressureSwitchValue());
    Logger.recordOutput(key + "/CurrentAmps", getCurrent());
    Logger.recordOutput(key + "/PressurePSI", getPressure());
  }
}
