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
import choreo.trajectory.Trajectory;
import frc.lib.autoselect.NamedAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;

/**
 * An autonomous mode built from Choreo trajectories. Each subclass loads its trajectories with
 * {@link #trajectory(ChoreoTraj)} and binds its commands to the routine's triggers in its
 * constructor, so the bindings exist exactly once.
 */
public abstract class AutoMode implements NamedAuto {
  private final AutoFactory autoFactory;
  protected final Drive drive;
  protected final AutoRoutine routine;
  private final List<AutoTrajectory> trajectories = new ArrayList<>();

  public AutoMode(Drive drivetrain) {
    this.drive = drivetrain;
    // Alliance flipping is off: red and blue paths are drawn separately in Choreo
    autoFactory =
        new AutoFactory(
            drivetrain::getPose,
            drivetrain::setPose,
            drivetrain::followTrajectory,
            false,
            drivetrain,
            AutoMode::logTrajectory);
    routine = autoFactory.newRoutine(getName());
  }

  public AutoFactory getAutoFactory() {
    return this.autoFactory;
  }

  public AutoRoutine getAutoRoutine() {
    return routine;
  }

  protected abstract AutoTrajectory getInitialTrajectory();

  public String getName() {
    return getClass().getSimpleName();
  }

  public Command getAutoCommand() {
    return getAutoRoutine().cmd();
  }

  public Optional<Pose2d> getInitialPose() {
    return getInitialTrajectory().getInitialPose();
  }

  /** Returns the poses of every trajectory in the routine, in the order they were loaded. */
  public Pose2d[] getLoggableTrajectory() {
    return trajectories.stream()
        .flatMap(t -> Arrays.stream(t.getRawTrajectory().getPoses()))
        .toArray(Pose2d[]::new);
  }

  /**
   * Loads a trajectory, or one segment of a split trajectory, into this routine. Loading every
   * trajectory through here lets the whole auto be plotted before the match.
   */
  protected AutoTrajectory trajectory(ChoreoTraj trajectory) {
    AutoTrajectory autoTrajectory = trajectory.asAutoTraj(routine);
    trajectories.add(autoTrajectory);
    return autoTrajectory;
  }

  protected Command stopDrive() {
    return DriveCommands.getStopCommand(drive);
  }

  private static void logTrajectory(Trajectory<SwerveSample> trajectory, boolean isStarting) {
    Logger.recordOutput(
        "Auto/ActiveTrajectory", isStarting ? trajectory.getPoses() : new Pose2d[0]);
  }
}
