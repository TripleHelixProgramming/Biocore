// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified work Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.game.Field;
import frc.game.GameState;
import frc.lib.AllianceSelector;
import frc.lib.AutoSelector;
import frc.lib.CommandZorroController;
import frc.lib.ControllerSelector;
import frc.lib.ControllerSelector.ControllerType;
import frc.lib.ControllerSelector.DriverConfig;
import frc.lib.ControllerSelector.DriverController;
import frc.lib.ControllerSelector.OperatorConfig;
import frc.lib.LoggedPowerDistribution;
import frc.robot.Constants.CANBusPorts.SC0;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.FeatureFlags;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSimWPI;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.leds.LEDController;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CanandgyroThread;
import frc.robot.util.KernelLogMonitor;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.VisionThread;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // SESSION_DIR and SignalLogger.setPath() must be initialized before any CTRE device is
  // constructed. A static initializer guarantees this runs before the constructor or any
  // instance field initializer that could trigger CANHD class loading.
  private static final String SESSION_DIR;

  static {
    if (Constants.currentMode == Constants.Mode.REAL) {
      SESSION_DIR = createSessionDir();
      SignalLogger.setPath(SESSION_DIR);
    } else {
      SESSION_DIR = null;
    }
  }

  public static final AllianceSelector allianceSelector =
      new AllianceSelector(DIOPorts.ALLIANCE_COLOR_SELECTOR);
  public static final AutoSelector autoSelector =
      new AutoSelector(DIOPorts.AUTONOMOUS_MODE_SELECTOR, allianceSelector::getAllianceColor);
  public final LoggedPowerDistribution powerDistribution =
      new LoggedPowerDistribution(SC0.BUS_ID, SC0.PD, ModuleType.kRev, "PD");

  private final java.util.Set<String> activeCommands = new java.util.LinkedHashSet<>();

  // Subsystems
  private Drive drive;
  private Vision vision;
  private final LEDController leds = FeatureFlags.LEDS_ENABLED ? LEDController.getInstance() : null;
  // Battery simulation constants
  private static final double ELECTRONICS_OVERHEAD_AMPS = 4.5; // RoboRIO + radio + PDH + misc
  private final LinearFilter vBusFilter = LinearFilter.singlePoleIIR(0.04, Robot.defaultPeriodSecs);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL: // Running on a real robot
        // SESSION_DIR and SignalLogger.setPath() were already set in the static initializer.
        // SignalLogger will create a nested timestamp subdir inside SESSION_DIR for hoot files.
        Logger.addDataReceiver(new WPILOGWriter(SESSION_DIR));
        Logger.addDataReceiver(new NT4Publisher());

        // Instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOBoron(),
                new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
        if (FeatureFlags.VISION_ENABLED) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getFieldRelativeHeading,
                  new VisionIOPhotonVision(FRONT_RIGHT_CAMERA),
                  new VisionIOPhotonVision(FRONT_LEFT_CAMERA),
                  new VisionIOPhotonVision(BACK_RIGHT_CAMERA),
                  new VisionIOPhotonVision(BACK_LEFT_CAMERA));
        }
        // Start kernel log monitoring (singleton, starts automatically on first call)
        KernelLogMonitor.getInstance();
        break;

      case SIM: // Running a physics simulator
        // Log to NT
        Logger.addDataReceiver(new WPILOGWriter("logs/"));
        Logger.addDataReceiver(new NT4Publisher());

        // Instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSimWPI(DriveConstants.FRONT_LEFT),
                new ModuleIOSimWPI(DriveConstants.FRONT_RIGHT),
                new ModuleIOSimWPI(DriveConstants.BACK_LEFT),
                new ModuleIOSimWPI(DriveConstants.BACK_RIGHT));
        if (FeatureFlags.VISION_ENABLED) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getFieldRelativeHeading,
                  new VisionIOPhotonVisionSim(FRONT_RIGHT_CAMERA, drive::getPose),
                  new VisionIOPhotonVisionSim(FRONT_LEFT_CAMERA, drive::getPose),
                  new VisionIOPhotonVisionSim(BACK_RIGHT_CAMERA, drive::getPose),
                  new VisionIOPhotonVisionSim(BACK_LEFT_CAMERA, drive::getPose));
        }
        break;

      case REPLAY: // Replaying a log
      default:
        // Set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

        // Disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        if (FeatureFlags.VISION_ENABLED) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::getFieldRelativeHeading,
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {});
        }
        break;
    }

    // Start background threads (for non-blocking CAN/network reads)
    SparkOdometryThread.getInstance().start();
    if (FeatureFlags.VISION_ENABLED) VisionThread.getInstance().start();
    CanandgyroThread.getInstance().start();

    // Start AdvantageKit logger
    Logger.start();

    configureControlPanelBindings();
    configureAutoOptions();

    CommandScheduler.getInstance().onCommandInitialize(cmd -> activeCommands.add(cmd.getName()));
    CommandScheduler.getInstance().onCommandFinish(cmd -> activeCommands.remove(cmd.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(cmd -> activeCommands.remove(cmd.getName()));

    new Trigger(
            NetworkTableInstance.getDefault()
                    .getTable("Triggers")
                    .getBooleanTopic("Align Encoders")
                    .subscribe(false)
                ::get)
        .onTrue(new InstantCommand(drive::zeroAbsoluteEncoders).ignoringDisable(true));
    Field.plotRegions();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    long loopStart = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    long t1 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    logCANBus(Constants.CANBusPorts.SC0.NAME, Constants.CANBusPorts.SC0.BUS);
    logCANBus(Constants.CANBusPorts.SC1.NAME, Constants.CANBusPorts.SC1.BUS);
    powerDistribution.log();
    logHIDs();
    logScheduler();
    GameState.logValues();

    Logger.recordOutput("USB/FreeSpaceMB", getUSBStorageFreeSpace() / 1024 / 1024);
    long t2 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Publish kernel log events to NetworkTables (only runs on real robot)
    if (RobotBase.isReal()) {
      KernelLogMonitor.getInstance().publishToLogger();
    }
    long t3 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Profiling output
    if (FeatureFlags.PROFILING_ENABLED) {
      long schedulerMs = (t1 - loopStart) / 1_000_000;
      long gameStateMs = (t2 - t1) / 1_000_000;
      long kernelMonitorMs = (t3 - t2) / 1_000_000;
      long totalMs = (t3 - loopStart) / 1_000_000;
      if (totalMs > 20) {
        System.out.println(
            "[Robot] scheduler="
                + schedulerMs
                + "ms gameState="
                + gameStateMs
                + "ms kernelMonitor="
                + kernelMonitorMs
                + "ms total="
                + totalMs
                + "ms");
      }
    }

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if (leds != null) leds.clear();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    allianceSelector.disabledPeriodic();
    autoSelector.disabledPeriodic();
    ControllerSelector.getInstance().scan(false);
    if (leds != null) leds.displayAutoSelection();
    var autoOption = autoSelector.get();
    autoOption.ifPresent(
        a ->
            a.getInitialPose()
                .ifPresent(
                    targetPose -> {
                      if (leds != null) leds.displayPoseSeek(drive.getPose(), targetPose);
                    }));
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    drive.setDefaultCommand(Commands.runOnce(drive::stop, drive).withName("Stop"));
    autoSelector.scheduleAuto();
    if (leds != null) leds.clear();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop mode is enabled. */
  @Override
  public void teleopInit() {
    autoSelector.cancelAuto();
    ControllerSelector.getInstance().scan(true);
    if (leds != null) leds.clear();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    if (leds != null) leds.clear();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    if (leds != null) leds.clear();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
        vBusFilter.calculate(
            Math.max(
                0.0,
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                    drive.getSimCurrentDrawAmps(), ELECTRONICS_OVERHEAD_AMPS))));
  }

  private void configureControlPanelBindings() {
    ControllerSelector.configure(
        // ZORRO is always preferred as driver in REAL and SIM mode
        new DriverConfig(
            ControllerType.ZORRO, this::bindZorroDriver, Constants.Mode.REAL, Constants.Mode.SIM),
        // XBOX is always preferred as operator in REAL and SIM mode
        new OperatorConfig(
            ControllerType.XBOX, this::bindXboxOperator, Constants.Mode.REAL, Constants.Mode.SIM),
        // XBOX is permitted as driver in REAL and SIM mode
        new DriverConfig(
            ControllerType.XBOX, this::bindXboxDriver, Constants.Mode.REAL, Constants.Mode.SIM),
        // KEYBOARD is permitted as driver in SIM mode only
        new DriverConfig(ControllerType.KEYBOARD, this::bindKeyboardDriver, Constants.Mode.SIM));
  }

  public DriverController bindZorroDriver(int port) {
    var zorroDriver = new CommandZorroController(port);

    var controller =
        new DriverController() {
          public double getXTranslationInput() {
            return -zorroDriver.getRightYAxis();
          }

          public double getYTranslationInput() {
            return -zorroDriver.getRightXAxis();
          }

          public double getRotationInput() {
            return -zorroDriver.getLeftXAxis();
          }

          public boolean getFieldRelativeInput() {
            return zorroDriver.getHID().getEUp();
          }
        };

    // Drive in field-relative mode while switch E is up
    // Drive in robot-relative mode while switch E is down
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getXTranslationInput(),
            () -> controller.getYTranslationInput(),
            () -> controller.getRotationInput(),
            () -> controller.getFieldRelativeInput(),
            allianceSelector::fieldRotated));

    // Reset gyro to 0° when button G is pressed
    zorroDriver
        .GIn()
        .onTrue(
            Commands.runOnce(() -> DriveCommands.resetDriverForward(drive)).ignoringDisable(true));

    return controller;
  }

  public DriverController bindXboxDriver(int port) {
    var xboxDriver = new CommandXboxController(port);

    var controller =
        new DriverController() {
          public double getXTranslationInput() {
            return -xboxDriver.getLeftY();
          }

          public double getYTranslationInput() {
            return -xboxDriver.getLeftX();
          }

          public double getRotationInput() {
            return -xboxDriver.getRightX();
          }

          public boolean getFieldRelativeInput() {
            return !xboxDriver.getHID().getLeftBumperButton();
          }
        };

    // Drive in field-relative mode while left bumper is released
    // Drive in robot-relative mode while left bumper is pressed
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getXTranslationInput(),
            () -> controller.getYTranslationInput(),
            () -> controller.getRotationInput(),
            () -> controller.getFieldRelativeInput(),
            allianceSelector::fieldRotated));

    // Reset gyro to 0° when B button is pressed
    xboxDriver
        .b()
        .onTrue(
            Commands.runOnce(() -> DriveCommands.resetDriverForward(drive)).ignoringDisable(true));

    return controller;
  }

  public DriverController bindKeyboardDriver(int port) {
    var keyboard = new CommandGenericHID(port);

    // WPILib sim keyboard axis layout:
    //   Axis 0: A (negative) / D (positive)        — strafe
    //   Axis 1: W (negative) / S (positive)        — forward/back
    //   Axis 2: Left arrow (decrease) / Right arrow (increase) — rotation
    //           (configure in DS sim keyboard editor; see README for settings)
    //   Button 1 (Z): reset heading
    var controller =
        new DriverController() {
          public double getXTranslationInput() {
            return -keyboard.getRawAxis(1);
          }

          public double getYTranslationInput() {
            return -keyboard.getRawAxis(0);
          }

          public double getRotationInput() {
            return -keyboard.getRawAxis(2);
          }

          public boolean getFieldRelativeInput() {
            return true;
          }
        };

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getXTranslationInput(),
            () -> controller.getYTranslationInput(),
            () -> controller.getRotationInput(),
            () -> controller.getFieldRelativeInput(),
            allianceSelector::fieldRotated));

    // Reset heading to 0° when Z (button 1) is pressed
    keyboard
        .button(1)
        .onTrue(
            Commands.runOnce(() -> DriveCommands.resetDriverForward(drive)).ignoringDisable(true));

    return controller;
  }

  public void bindXboxOperator(int port, DriverController driver) {}

  public void configureAutoOptions() {}

  public static Alliance getAlliance() {
    return allianceSelector.getAllianceColor();
  }

  /** Returns the number of free bytes on the USB log drive at /U, or Long.MAX_VALUE in sim. */
  public static long getUSBStorageFreeSpace() {
    if (Constants.currentMode != Constants.Mode.REAL) return Long.MAX_VALUE;
    return new java.io.File("/U").getFreeSpace();
  }

  private static void logHIDs() {
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (!DriverStation.isJoystickConnected(port)) continue;
      String prefix = "HID/Port" + port;
      Logger.recordOutput(prefix + "/Name", DriverStation.getJoystickName(port));
      int axisCount = DriverStation.getStickAxisCount(port);
      double[] axes = new double[axisCount];
      for (int i = 0; i < axisCount; i++) axes[i] = DriverStation.getStickAxis(port, i);
      Logger.recordOutput(prefix + "/Axes", axes);
      int buttonCount = DriverStation.getStickButtonCount(port);
      boolean[] buttons = new boolean[buttonCount];
      for (int i = 0; i < buttonCount; i++) buttons[i] = DriverStation.getStickButton(port, i + 1);
      Logger.recordOutput(prefix + "/Buttons", buttons);
      int povCount = DriverStation.getStickPOVCount(port);
      String[] povs = new String[povCount];
      for (int i = 0; i < povCount; i++) povs[i] = DriverStation.getStickPOV(port, i).name();
      Logger.recordOutput(prefix + "/POVs", povs);
    }
  }

  private void logScheduler() {
    Logger.recordOutput("Commands/ActiveCommands", activeCommands.toArray(new String[0]));
    logSubsystem("Drive", drive);
    if (vision != null) logSubsystem("Vision", vision);
    logAlerts();
  }

  // Third-party library alerts (PathPlanner, Choreo, PhotonVision) still publish to SmartDashboard
  // via their own Alert objects, so we read them back from NT.
  private static void logAlerts() {
    logAlertGroup("PathPlanner");
    logAlertGroup("Choreo");
    logAlertGroup("PhotonAlerts");
  }

  private static void logAlertGroup(String group) {
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(group);
    Logger.recordOutput(
        "Alerts/" + group + "/Errors", table.getEntry("errors").getStringArray(new String[0]));
    Logger.recordOutput(
        "Alerts/" + group + "/Warnings", table.getEntry("warnings").getStringArray(new String[0]));
    Logger.recordOutput(
        "Alerts/" + group + "/Infos", table.getEntry("infos").getStringArray(new String[0]));
  }

  /**
   * Creates and returns a timestamped session directory under /U/logs/. Must be called before any
   * CTRE devices are constructed so that SignalLogger.setPath() takes effect before auto-logging
   * begins.
   */
  private static String createSessionDir() {
    java.io.File logsDir = new java.io.File("/U/logs");
    long maxCount = 0;
    java.io.File[] entries = logsDir.listFiles();
    if (entries != null) {
      for (java.io.File entry : entries) {
        String name = entry.getName();
        if (name.startsWith("session_")) {
          try {
            long n = Long.parseLong(name.substring("session_".length()));
            if (n > maxCount) maxCount = n;
          } catch (NumberFormatException e) {
            // not a session dir, skip
          }
        }
      }
    }
    String dir = "/U/logs/session_" + (maxCount + 1) + "/";
    new java.io.File(dir).mkdirs();
    return dir;
  }

  private static void logCANBus(String name, com.ctre.phoenix6.CANBus bus) {
    var status = bus.getStatus();
    Logger.recordOutput("CANBus/" + name + "/Utilization", status.BusUtilization);
    Logger.recordOutput("CANBus/" + name + "/BusOffCount", (long) status.BusOffCount);
    Logger.recordOutput("CANBus/" + name + "/TxFullCount", (long) status.TxFullCount);
    Logger.recordOutput("CANBus/" + name + "/REC", (long) status.REC);
    Logger.recordOutput("CANBus/" + name + "/TEC", (long) status.TEC);
  }

  private static void logSubsystem(String name, SubsystemBase subsystem) {
    Command current = subsystem.getCurrentCommand();
    Command defaultCmd = subsystem.getDefaultCommand();
    Logger.recordOutput("Subsystems/" + name + "/HasCommand", current != null);
    Logger.recordOutput(
        "Subsystems/" + name + "/Command", current != null ? current.getName() : "None");
    Logger.recordOutput("Subsystems/" + name + "/HasDefault", defaultCmd != null);
    Logger.recordOutput(
        "Subsystems/" + name + "/Default", defaultCmd != null ? defaultCmd.getName() : "None");
  }
}
