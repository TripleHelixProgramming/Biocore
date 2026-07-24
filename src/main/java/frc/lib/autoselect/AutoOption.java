package frc.lib.autoselect;

import frc.lib.Util;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.util.Color;

public class AutoOption {
  private final Alliance allianceColor;
  private final int switchNumber;
  private final NamedAuto autoMode;

  /**
   * Constructs a selectable autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   * @param autoMode The autonomous mode to run
   */
  public AutoOption(Alliance color, int option, NamedAuto autoMode) {
    this.allianceColor = color;
    this.switchNumber = option;
    this.autoMode = autoMode;
  }

  /**
   * Constructs a null autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   */
  public AutoOption(Alliance color, int option) {
    this(color, option, null);
  }

  /**
   * @return Alliance for which the option is valid
   */
  public Alliance getAlliance() {
    return this.allianceColor;
  }

  /**
   * @return Color of the associated alliance
   */
  public Color getAllianceColor() {
    return Util.allianceToColor(getAlliance());
  }

  /**
   * @return Selector switch index for which the option is valid
   */
  public int getOptionNumber() {
    return this.switchNumber;
  }

  /**
   * @return The command which runs the selected autonomous mode
   */
  public synchronized Optional<Command> getAutoCommand() {
    return (autoMode == null) ? Optional.empty() : Optional.of(autoMode.getAutoCommand());
  }

  public Optional<Pose2d> getInitialPose() {
    return (autoMode == null) ? Optional.empty() : autoMode.getInitialPose();
  }

  public Optional<Pose2d[]> getInitialTrajectory() {
    return (autoMode == null) ? Optional.empty() : Optional.of(autoMode.getLoggableTrajectory());
  }

  public synchronized String getName() {
    return (autoMode == null) ? "None; this slot reserved for no auto" : autoMode.getName();
  }
}
