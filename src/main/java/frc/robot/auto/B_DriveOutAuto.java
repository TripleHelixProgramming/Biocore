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
import org.wpilib.command2.Commands;

/** Placeholder: drives out, stops, then strafes while turning 90 degrees. */
public class B_DriveOutAuto extends AutoMode {
  private final AutoTrajectory driveOut;
  private final AutoTrajectory strafe;

  public B_DriveOutAuto(Drive drivetrain) {
    super(drivetrain);
    driveOut = trajectory(ChoreoTraj.BlueDriveOut$0);
    strafe = trajectory(ChoreoTraj.BlueDriveOut$1);

    routine.active().onTrue(Commands.sequence(driveOut.resetOdometry(), driveOut.cmd()));
    driveOut.chain(strafe);
    strafe.done().onTrue(stopDrive());
  }

  @Override
  protected AutoTrajectory getInitialTrajectory() {
    return driveOut;
  }
}
