package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  private final LifterIO lifterIO;
  private final LifterIOInputsAutoLogged lifterInputs = new LifterIOInputsAutoLogged();

  public Elevator(LifterIO lifterIO) {
    this.lifterIO = lifterIO;
  }

  @Override
  public void periodic() {
    lifterIO.updateInputs(lifterInputs);
  }

  public Command getStopCommand() {
    // return startEnd(this::stop, () -> {}).withName("Stop");
    return startEnd(() -> lifterIO.setOpenLoop(0.0), () -> {}).withName("Stop");
  }

  public Command getJoystickMoveCommand(DoubleSupplier joystick) {
    return run(() -> {
          // m                  // m/s                   // s
          double deltaPositionMeters = joystick.getAsDouble() * 0.02;
          lifterIO.setPosition(lifterInputs.positionMeters + deltaPositionMeters);
        })
        .withName("Joystick move");
  }

  public Command getGoToPositionCommand(DoubleSupplier positionMeters) {
    return runOnce(() -> lifterIO.setPosition(positionMeters.getAsDouble()))
        .withName("Go to position");
  }
}
