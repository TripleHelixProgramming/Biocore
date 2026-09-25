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
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
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

  /**
   * Returns every trajectory in the routine as one continuous path for plotting, starting from the
   * initial trajectory. See {@link #exploreBranches}.
   */
  public Pose2d[] getLoggableTrajectory() {
    List<Pose2d[]> paths = new ArrayList<>();
    for (AutoTrajectory t : trajectories) paths.add(t.getRawTrajectory().getPoses());
    return exploreBranches(paths, trajectories.indexOf(getInitialTrajectory()));
  }

  // A trajectory that starts within this distance of another's end continues from it
  private static final double BRANCH_NODE_TOLERANCE_METERS = 0.01;

  /**
   * Joins paths into one continuous path, starting from the root. A path that starts where another
   * ends is a branch of it. Where several branches leave the same node, every branch but the last
   * is drawn out and back to the node before the next one, so the joined path never jumps from the
   * end of one branch to the start of another. Each path is drawn forward once. A path that
   * connects to nothing reachable from the root is appended after a jump.
   *
   * @param paths The poses of each trajectory
   * @param root The index of the path the auto starts on
   * @return The joined path
   */
  static Pose2d[] exploreBranches(List<Pose2d[]> paths, int root) {
    List<Pose2d> joined = new ArrayList<>();
    Set<Integer> visited = new HashSet<>();
    explore(root, paths, visited, joined, false);
    for (int i = 0; i < paths.size(); i++) {
      if (!visited.contains(i)) explore(i, paths, visited, joined, false);
    }
    return joined.toArray(Pose2d[]::new);
  }

  private static void explore(
      int node, List<Pose2d[]> paths, Set<Integer> visited, List<Pose2d> joined, boolean retrace) {
    visited.add(node);
    Pose2d[] path = paths.get(node);
    joined.addAll(Arrays.asList(path));

    List<Integer> branches = new ArrayList<>();
    for (int i = 0; i < paths.size(); i++) {
      if (!visited.contains(i) && continues(path, paths.get(i))) branches.add(i);
    }
    for (int b = 0; b < branches.size(); b++) {
      int branch = branches.get(b);
      // A branch can already be drawn if it rejoined through an earlier branch
      if (visited.contains(branch)) continue;
      boolean lastBranch = b == branches.size() - 1;
      explore(branch, paths, visited, joined, retrace || !lastBranch);
    }

    if (retrace) {
      for (int i = path.length - 1; i >= 0; i--) joined.add(path[i]);
    }
  }

  private static boolean continues(Pose2d[] from, Pose2d[] to) {
    return from.length > 0
        && to.length > 0
        && from[from.length - 1].getTranslation().getDistance(to[0].getTranslation())
            < BRANCH_NODE_TOLERANCE_METERS;
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
