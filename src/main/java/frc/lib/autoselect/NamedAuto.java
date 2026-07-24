// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.autoselect;

import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;

public interface NamedAuto {
  String getName();

  Command getAutoCommand();

  default Optional<Pose2d> getInitialPose() {
    return Optional.empty();
  }

  default Pose2d[] getLoggableTrajectory() {
    return new Pose2d[0];
  }
}
