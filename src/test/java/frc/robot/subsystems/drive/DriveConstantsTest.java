package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.List;
import org.junit.jupiter.api.Test;

class DriveConstantsTest {

  private static final List<
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      MODULES =
          List.of(
              DriveConstants.FRONT_LEFT,
              DriveConstants.FRONT_RIGHT,
              DriveConstants.BACK_LEFT,
              DriveConstants.BACK_RIGHT);

  @Test
  void eachSteerMotorFusesWithItsOwnCancoder() {
    for (var module : MODULES) {
      assertEquals(
          module.EncoderId,
          module.SteerMotorInitialConfigs.Feedback.FeedbackRemoteSensorID,
          "Steer motor " + module.SteerMotorId + " uses the wrong CANcoder");
    }
  }

  @Test
  void modulesDoNotShareConfigObjects() {
    for (int i = 0; i < MODULES.size(); i++) {
      for (int j = i + 1; j < MODULES.size(); j++) {
        assertNotSame(
            MODULES.get(i).SteerMotorInitialConfigs, MODULES.get(j).SteerMotorInitialConfigs);
        assertNotSame(
            MODULES.get(i).DriveMotorInitialConfigs, MODULES.get(j).DriveMotorInitialConfigs);
      }
    }
  }
}
