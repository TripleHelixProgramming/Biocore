// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;

public abstract class AutoMode {
  protected final Drive drive;

  public AutoMode(Drive drivetrain) {
    this.drive = drivetrain;
  }

  public abstract Command getAutoCommand();

  public abstract String getName();

  public Optional<Pose2d> getInitialPose() {
    return Optional.empty();
  }

  public SwerveSample[] getLoggableTrajectory() {
    return new SwerveSample[0];
  }

  protected Command stopDrive() {
    return DriveCommands.getStopCommand(drive);
  }
}
