package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface LifterIO {

  @AutoLog
  public static class LifterIOInputs {
    public boolean connected = false;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(LifterIOInputs inputs) {}

  public default void setOpenLoop(double volts) {}

  public default void setPosition(double positionMeters) {}
}
