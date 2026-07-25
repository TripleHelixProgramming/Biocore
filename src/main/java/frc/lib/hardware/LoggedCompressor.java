// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.pneumatic.Compressor;
import org.wpilib.hardware.pneumatic.PneumaticsModuleType;

public class LoggedCompressor extends Compressor {
  @AutoLog
  public static class CompressorInputs {
    public boolean enabled = false;
    public boolean pressureSwitch = false;
    public double currentAmps = 0.0;
    public double pressurePSI = 0.0;
  }

  private final String key;
  private final CompressorInputsAutoLogged inputs = new CompressorInputsAutoLogged();

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
    inputs.enabled = isEnabled();
    inputs.pressureSwitch = getPressureSwitchValue();
    inputs.currentAmps = getCurrent();
    inputs.pressurePSI = getPressure();
    Logger.processInputs(key, inputs);
  }
}
