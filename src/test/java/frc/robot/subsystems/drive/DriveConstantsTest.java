// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

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

  @Test
  void moduleConstantsUseTheirDevicesIds() {
    assertEquals(MODULES.size(), DriveConstants.MODULE_DEVICES.size());
    for (int i = 0; i < MODULES.size(); i++) {
      var module = MODULES.get(i);
      var devices = DriveConstants.MODULE_DEVICES.get(i);
      assertEquals(devices.drive().id(), module.DriveMotorId);
      assertEquals(devices.turn().id(), module.SteerMotorId);
      assertEquals(devices.turnEncoder().id(), module.EncoderId);
    }
  }

  @Test
  void moduleIdsMatchTheWiredDevices() {
    // drive, turn, turn encoder
    int[][] expected = {{28, 29, 43}, {20, 21, 33}, {10, 11, 45}, {18, 19, 31}};
    for (int i = 0; i < MODULES.size(); i++) {
      var module = MODULES.get(i);
      assertArrayEquals(
          expected[i], new int[] {module.DriveMotorId, module.SteerMotorId, module.EncoderId});
    }
  }

  @Test
  void allModuleDevicesAreOnTheDrivetrainBus() {
    for (var devices : DriveConstants.MODULE_DEVICES) {
      for (var device : List.of(devices.drive(), devices.turn(), devices.turnEncoder())) {
        assertEquals(CANPort.CAN_S1, device.port(), device.label());
      }
    }
  }

  @Test
  void moduleDevicesCannotBeReplaced() {
    var devices = DriveConstants.MODULE_DEVICES;
    assertThrows(UnsupportedOperationException.class, () -> devices.set(0, devices.get(1)));
  }
}
