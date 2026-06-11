package frc.lib;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.littletonrobotics.junction.Logger;

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
