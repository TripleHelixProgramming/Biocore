// Copyright (c) 2025 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public DoubleSolenoid.Value isDeployed = DoubleSolenoid.Value.kReverse;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void deploy() {}

  public default void retract() {}
}
