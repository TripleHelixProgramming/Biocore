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
