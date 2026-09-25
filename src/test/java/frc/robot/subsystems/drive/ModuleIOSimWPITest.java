package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

class ModuleIOSimWPITest {

  @BeforeAll
  static void initializeHal() {
    HAL.initialize();
  }

  @Test
  void frictionMatchesPhoenixModel() {
    assertEquals(0.0, ModuleIOSimWPI.applyFriction(0.1, 0.2));
    assertEquals(0.0, ModuleIOSimWPI.applyFriction(-0.1, 0.2));
    assertEquals(0.0, ModuleIOSimWPI.applyFriction(0.2, 0.2), 1e-12);
    assertEquals(0.8, ModuleIOSimWPI.applyFriction(1.0, 0.2), 1e-12);
    assertEquals(-0.8, ModuleIOSimWPI.applyFriction(-1.0, 0.2), 1e-12);
    assertEquals(1.0, ModuleIOSimWPI.applyFriction(1.0, 0.0));
  }

  @Test
  void moduleConstantsCarryFrictionVoltage() {
    for (var module :
        List.of(
            DriveConstants.FRONT_LEFT,
            DriveConstants.FRONT_RIGHT,
            DriveConstants.BACK_LEFT,
            DriveConstants.BACK_RIGHT)) {
      assertEquals(0.2, module.DriveFrictionVoltage, 1e-12);
      assertEquals(0.2, module.SteerFrictionVoltage, 1e-12);
    }
  }

  @Test
  void driveBelowFrictionVoltageDoesNotMove() {
    assertEquals(0.0, spinDrive(0.15), 1e-9);
  }

  @Test
  void driveAboveFrictionVoltageMoves() {
    assertTrue(spinDrive(1.0) > 0.0);
  }

  /** Runs the drive motor open-loop for one second and returns its final velocity (rad/s). */
  private static double spinDrive(double volts) {
    var io = new ModuleIOSimWPI(DriveConstants.FRONT_LEFT);
    var inputs = new ModuleIO.ModuleIOInputs();
    io.setDriveOpenLoop(volts);
    for (int i = 0; i < 50; i++) {
      io.updateInputs(inputs);
    }
    return inputs.driveVelocityRadPerSec;
  }
}
