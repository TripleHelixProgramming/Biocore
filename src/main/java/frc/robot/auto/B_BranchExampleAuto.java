// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import choreo.auto.AutoTrajectory;
import frc.robot.generated.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import org.wpilib.command2.Commands;

/** Example of a branching auto: drives to a node, then takes branch A or branch B. */
public class B_BranchExampleAuto extends AutoMode {
  private final AutoTrajectory trunk;

  /**
   * @param drivetrain The drivetrain
   * @param takeBranchA Chooses branch A over branch B; read when the robot reaches the node
   */
  public B_BranchExampleAuto(Drive drivetrain, BooleanSupplier takeBranchA) {
    super(drivetrain);
    trunk = trajectory(ChoreoTraj.BlueBranchTrunk);
    AutoTrajectory branchA = trajectory(ChoreoTraj.BlueBranchA);
    AutoTrajectory branchB = trajectory(ChoreoTraj.BlueBranchB);

    routine.active().onTrue(Commands.sequence(trunk.resetOdometry(), trunk.cmd()));
    trunk.done().and(takeBranchA).onTrue(branchA.cmd());
    trunk.done().and(() -> !takeBranchA.getAsBoolean()).onTrue(branchB.cmd());
    routine.anyDone(branchA, branchB).onTrue(stopDrive());
  }

  @Override
  protected AutoTrajectory getInitialTrajectory() {
    return trunk;
  }
}
