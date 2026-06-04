// Copyright (c) 2025 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.PneumaticChannels.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;

public class IntakeArmIOSim implements IntakeArmIO {
  public final DoubleSolenoidSim intakeArmPneumatic;

  public IntakeArmIOSim() {
    intakeArmPneumatic =
        new DoubleSolenoidSim(PneumaticsModuleType.REVPH, INTAKE_ARM_FWD, INTAKE_ARM_REV);
    intakeArmPneumatic.set(kReverse);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    inputs.isDeployed = intakeArmPneumatic.get();
  }

  @Override
  public void deploy() {
    intakeArmPneumatic.set(kForward);
  }

  @Override
  public void retract() {
    intakeArmPneumatic.set(kReverse);
  }
}
