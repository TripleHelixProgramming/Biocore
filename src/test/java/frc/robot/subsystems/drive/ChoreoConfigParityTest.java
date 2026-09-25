// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.auto.ChoreoProjectFiles.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Test;

/**
 * Checks that the robot config in the Choreo project matches DriveConstants. Choreo plans every
 * path around this config, and it cannot read Java, so the two must be kept in step by hand.
 */
class ChoreoConfigParityTest {
  private static final double TOLERANCE = 1e-6;
  // Choreo converts pounds with the exact 0.45359237 kg/lb; WPILib's Pounds unit uses 0.453592,
  // so 130 lb differs by 48 mg between the two
  private static final double MASS_TOLERANCE_KG = 1e-4;
  private final JsonObject config = project().getAsJsonObject("config");

  private double configVal(String key) {
    return val(config.get(key));
  }

  @Test
  void moduleLocationsMatch() {
    var frontLeft = config.getAsJsonObject("frontLeft");
    var backLeft = config.getAsJsonObject("backLeft");
    assertEquals(DriveConstants.FRONT_LEFT.LocationX, val(frontLeft.get("x")), TOLERANCE);
    assertEquals(DriveConstants.FRONT_LEFT.LocationY, val(frontLeft.get("y")), TOLERANCE);
    assertEquals(DriveConstants.BACK_LEFT.LocationX, val(backLeft.get("x")), TOLERANCE);
    assertEquals(DriveConstants.BACK_LEFT.LocationY, val(backLeft.get("y")), TOLERANCE);
    // Choreo mirrors the left modules to get the right ones
    assertEquals(-DriveConstants.FRONT_RIGHT.LocationY, val(frontLeft.get("y")), TOLERANCE);
    assertEquals(-DriveConstants.BACK_RIGHT.LocationY, val(backLeft.get("y")), TOLERANCE);
  }

  @Test
  void drivetrainMatches() {
    assertEquals(DriveConstants.FRONT_LEFT.DriveMotorGearRatio, configVal("gearing"), TOLERANCE);
    assertEquals(DriveConstants.FRONT_LEFT.WheelRadius, configVal("radius"), TOLERANCE);
    assertEquals(DriveConstants.ROBOT_MASS.in(Kilograms), configVal("mass"), MASS_TOLERANCE_KG);
    assertEquals(
        DriveConstants.ROBOT_MOI.in(KilogramSquareMeters), configVal("inertia"), TOLERANCE);
  }

  @Test
  void deratesMatchAndStayBelowPhysicalLimits() {
    double vmax = DriveConstants.CHOREO_MOTOR_MAX_VELOCITY.in(RadiansPerSecond);
    double tmax = DriveConstants.CHOREO_MOTOR_MAX_TORQUE.in(NewtonMeters);
    assertEquals(vmax, configVal("vmax"), TOLERANCE);
    assertEquals(tmax, configVal("tmax"), TOLERANCE);
    assertEquals(DriveConstants.CHOREO_WHEEL_COF, configVal("cof"), TOLERANCE);

    assertTrue(vmax <= DriveConstants.DRIVE_GEARBOX.freeSpeed, "Choreo vmax exceeds free speed");
    assertTrue(tmax <= DriveConstants.DRIVE_GEARBOX.stallTorque, "Choreo tmax exceeds stall");
    assertTrue(DriveConstants.CHOREO_WHEEL_COF <= DriveConstants.WHEEL_COF, "Choreo cof too high");
  }
}
