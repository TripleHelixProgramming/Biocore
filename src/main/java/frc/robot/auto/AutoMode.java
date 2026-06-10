// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.commandsv2.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public abstract class AutoMode {
  private final AutoFactory autoFactory;
  protected final Drive drive;

  public AutoMode(Drive drivetrain) {
    this.drive = drivetrain;
    autoFactory =
        new AutoFactory(
            drivetrain::getPose,
            drivetrain::setPose,
            drivetrain::followTrajectory,
            false,
            drivetrain);
  }

  public AutoFactory getAutoFactory() {
    return this.autoFactory;
  }

  public abstract AutoRoutine getAutoRoutine();

  protected abstract AutoTrajectory getInitialTrajectory();

  public abstract String getName();

  public Optional<Pose2d> getInitialPose() {
    return getInitialTrajectory().getInitialPose();
  }

  public SwerveSample[] getLoggableTrajectory() {
    SwerveSample[] trajArray = new SwerveSample[0];
    return getInitialTrajectory().getRawTrajectory().samples().toArray(trajArray);
  }

  protected Command stopDrive() {
    return DriveCommands.getStopCommand(drive);
  }
}
