// Copyright (c) 2025 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.hopper;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.PneumaticChannels.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class HopperIOReal implements HopperIO {
  public final DoubleSolenoid hopperPneumatic;

  public HopperIOReal() {
    hopperPneumatic = new DoubleSolenoid(PneumaticsModuleType.REVPH, HOPPER_FWD, HOPPER_REV);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.isDeployed = hopperPneumatic.get();
  }

  @Override
  public void deploy() {
    hopperPneumatic.set(kForward);
  }

  @Override
  public void retract() {
    hopperPneumatic.set(kReverse);
  }
}
