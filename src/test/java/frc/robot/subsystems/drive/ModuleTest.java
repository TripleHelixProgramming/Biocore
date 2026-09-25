// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.preferences.Preferences;

class ModuleTest {

  /** Reports a CANcoder turn zero only once inputs are read, like the TalonFX IO. */
  private static class FakeModuleIO implements ModuleIO {
    final Rotation2d cancoderZero;
    final List<Rotation2d> appliedTurnZeros = new ArrayList<>();

    FakeModuleIO(Rotation2d cancoderZero) {
      this.cancoderZero = cancoderZero;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.turnZero = cancoderZero;
    }

    @Override
    public void setTurnZero(Rotation2d rotation) {
      appliedTurnZeros.add(rotation);
    }
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize();
  }

  @Test
  void turnZeroIsSeededFromCancoderOnFirstPeriodic() {
    String name = "TurnZeroTestModule";
    Preferences.remove(DriveConstants.ZERO_ROTATION_KEY + "/" + name);
    var io = new FakeModuleIO(Rotation2d.fromRadians(1.25));

    var module = new Module(io, name);
    assertTrue(io.appliedTurnZeros.isEmpty(), "Turn zero applied before inputs were read");

    module.periodic();
    assertEquals(1, io.appliedTurnZeros.size());
    assertEquals(1.25, io.appliedTurnZeros.get(0).getRadians(), 1e-9);
    assertEquals(
        1.25, Preferences.getDouble(DriveConstants.ZERO_ROTATION_KEY + "/" + name, 0.0), 1e-9);

    module.periodic();
    assertEquals(1, io.appliedTurnZeros.size(), "Turn zero applied more than once");
  }
}
