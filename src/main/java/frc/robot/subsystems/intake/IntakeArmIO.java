// Copyright (c) 2025 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
  @AutoLog
  public static class IntakeArmIOInputs {
    public DoubleSolenoid.Value isDeployed = DoubleSolenoid.Value.kReverse;
  }

  public default void updateInputs(IntakeArmIOInputs inputs) {}

  public default void deploy() {}

  public default void retract() {}
}
