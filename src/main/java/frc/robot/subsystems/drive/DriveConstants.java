// Copyright (c) 2025-2026 Triple Helix Robotics, FRC Team 2363
// https://github.com/TripleHelixProgramming
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.Constants.CANBusPorts.SC1;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;

public class DriveConstants {

  public static final String ZERO_ROTATION_KEY = "ZeroRotation";

  // Robot physical dimensions
  public static final Distance WHEEL_BASE = Inches.of(23.75);
  public static final Distance TRACK_WIDTH = Inches.of(20.75);
  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(WHEEL_BASE.div(2.0), TRACK_WIDTH.div(2.0)),
        new Translation2d(WHEEL_BASE.div(2.0), TRACK_WIDTH.div(-2.0)),
        new Translation2d(WHEEL_BASE.div(-2.0), TRACK_WIDTH.div(2.0)),
        new Translation2d(WHEEL_BASE.div(-2.0), TRACK_WIDTH.div(-2.0))
      };
  public static final Distance DRIVE_BASE_RADIUS =
      Meters.of(Translation2d.kZero.getDistance(MODULE_TRANSLATIONS[0]));

  // Drive motor configuration
  public static final Distance WHEEL_RADIUS = Inches.of(2);
  public static final double WHEEL_RADIUS_METERS = WHEEL_RADIUS.in(Meters);

  private static enum DriveGearRatio {
    SDS_MK5i_R1(1, 54.0 / 12.0, 25.0 / 32.0, 30.0 / 15.0),
    SDS_MK5i_R2(1, 54.0 / 14.0, 25.0 / 32.0, 30.0 / 15.0),
    SDS_MK5i_R3(1, 54.0 / 16.0, 25.0 / 32.0, 30.0 / 15.0);

    private final int chassisStages;
    private final double[] reductions;

    /**
     * @param chassisStages the number of leading reduction stages fixed to the chassis reference
     *     frame; their product defines the couple ratio
     * @param reductions all gear reduction stages in order, chassis-frame stages first followed by
     *     azimuth-frame stages
     */
    private DriveGearRatio(int chassisStages, double... reductions) {
      this.chassisStages = chassisStages;
      this.reductions = reductions;
    }

    /**
     * Returns the total gear reduction from the drive motor to the wheel, combining all stages
     * regardless of reference frame. Used to convert drive motor rotations to wheel rotations.
     */
    public double getDriveMotorReduction() {
      double product = 1.0;
      for (double r : reductions) product *= r;
      return product;
    }

    /**
     * Returns the number of drive motor rotations per one full rotation of the steering azimuth,
     * due to mechanical coupling. This is the product of all gear reduction stages that are fixed
     * to the chassis reference frame (i.e., before the azimuth pivot). The swerve odometry uses
     * this to compensate for the apparent wheel displacement caused by azimuth rotation.
     */
    public double getCoupleRatio() {
      double product = 1.0;
      for (int i = 0; i < chassisStages; i++) product *= reductions[i];
      return product;
    }
  }

  private static final DriveGearRatio SELECTED_RATIO = DriveGearRatio.SDS_MK5i_R2;
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final LinearVelocity DRIVETRAIN_SPEED_LIMIT =
      MetersPerSecond.of(
          0.9
              * (WHEEL_RADIUS_METERS * 2.0 * Math.PI)
              * DRIVE_GEARBOX.freeSpeed
              / (2.0 * Math.PI)
              / SELECTED_RATIO.getDriveMotorReduction());

  // Chassis movement limits
  private static final LinearVelocity DRIVER_SPEED_LIMIT = MetersPerSecond.of(5);
  public static final LinearVelocity MAX_CHASSIS_VELOCITY =
      MetersPerSecond.of(
          Math.min(
              DRIVETRAIN_SPEED_LIMIT.in(MetersPerSecond), DRIVER_SPEED_LIMIT.in(MetersPerSecond)));
  public static final LinearAcceleration MAX_CHASSIS_ACCELERATION =
      MetersPerSecondPerSecond.of(3.0);

  public static final AngularVelocity MAX_CHASSIS_ANGULAR_VELOCITY =
      RadiansPerSecond.of(MAX_CHASSIS_VELOCITY.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
  public static final AngularAcceleration MAX_CHASSIS_ANGULAR_ACCELERATION =
      RadiansPerSecondPerSecond.of(30);

  public static final PathConstraints PATH_FOLLOWING_CONSTRAINTS =
      new PathConstraints(
          MAX_CHASSIS_VELOCITY.in(MetersPerSecond),
          MAX_CHASSIS_ACCELERATION.in(MetersPerSecondPerSecond),
          MAX_CHASSIS_ANGULAR_VELOCITY.in(RadiansPerSecond),
          MAX_CHASSIS_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  // Turn motor configuration
  public static final boolean TURN_INVERTED = false;
  public static final double TURN_MOTOR_REDUCTION = 26.0; // SDS MK5i
  public static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  // Absolute turn encoder configuration
  public static final boolean TURN_ENCODER_INVERTED = false;

  // PathPlanner configuration
  public static final Mass ROBOT_MASS = Pounds.of(35.4);
  public static final MomentOfInertia ROBOT_MOI =
      KilogramSquareMeters.of(2.5); // Mass*((Track_width/2)^2 + (Wheel_base/2)^2)
  public static final double WHEEL_COF = 1.2;
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS.in(Kilograms),
          ROBOT_MOI.in(KilogramSquareMeters),
          new ModuleConfig(
              WHEEL_RADIUS_METERS,
              DRIVETRAIN_SPEED_LIMIT.in(MetersPerSecond),
              WHEEL_COF,
              DRIVE_GEARBOX.withReduction(SELECTED_RATIO.getDriveMotorReduction()),
              KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT,
              1),
          MODULE_TRANSLATIONS);

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(300)
          .withKI(0)
          .withKD(12)
          .withKS(0.1)
          .withKV(1.91)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0.124);

  // TorqueCurrent peak at which the wheels start to slip; used for slip detection in
  // TorqueCurrentFOC control mode. This needs to be tuned to your individual robot.
  private static final int SLIP_CURRENT = 120;

  // Stator current limit for azimuth (steer) motors; lower than drive to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  private static final int STEER_STATOR_CURRENT_LIMIT = 60;

  private static final boolean INVERT_LEFT_SIDE = false;
  private static final boolean INVERT_RIGHT_SIDE = false;

  // These are only used for simulation
  private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.025);

  private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(DRIVE_GAINS)
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio(SELECTED_RATIO.getDriveMotorReduction()))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(SLIP_CURRENT)
                  .withPeakReverseTorqueCurrent(-SLIP_CURRENT))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true));

  // Azimuth does not require much torque; keep stator limit low to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(STEER_GAINS)
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withRotorToSensorRatio(TURN_MOTOR_REDUCTION))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(100.0 / TURN_MOTOR_REDUCTION)
                  .withMotionMagicAcceleration(100.0 / TURN_MOTOR_REDUCTION / 0.100)
                  .withMotionMagicExpo_kV(0.12 * TURN_MOTOR_REDUCTION)
                  .withMotionMagicExpo_kA(0.1))
          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true));

  static DCMotorSim createDriveSim() {
    return new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            DRIVE_GEARBOX,
            DRIVE_INERTIA.in(KilogramSquareMeters),
            SELECTED_RATIO.getDriveMotorReduction()),
        DRIVE_GEARBOX);
  }

  static DCMotorSim createTurnSim() {
    return new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            TURN_GEARBOX, STEER_INERTIA.in(KilogramSquareMeters), TURN_MOTOR_REDUCTION),
        TURN_GEARBOX);
  }

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      configureModule(
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              constants) {
    constants.DriveMotorInitialConfigs.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    constants.SteerMotorInitialConfigs.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    return constants;
  }

  public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
      new SwerveDrivetrainConstants().withCANBusName(SC1.BUS.getName());

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      CONSTANT_CREATOR =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(SELECTED_RATIO.getDriveMotorReduction())
              .withSteerMotorGearRatio(TURN_MOTOR_REDUCTION)
              .withCouplingGearRatio(SELECTED_RATIO.getCoupleRatio())
              .withWheelRadius(WHEEL_RADIUS)
              .withSteerMotorGains(STEER_GAINS)
              .withDriveMotorGains(DRIVE_GAINS)
              .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
              .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
              .withSlipCurrent(Amps.of(SLIP_CURRENT))
              .withSpeedAt12Volts(DRIVETRAIN_SPEED_LIMIT)
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
              .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT =
          configureModule(
              CONSTANT_CREATOR.createModuleConstants(
                  SC1.FRONT_LEFT_TURN,
                  SC1.FRONT_LEFT_DRIVE,
                  SC1.FRONT_LEFT_TURN_ABS_ENC,
                  Rotations.of(0),
                  WHEEL_BASE.div(2.0),
                  TRACK_WIDTH.div(2.0),
                  INVERT_LEFT_SIDE,
                  TURN_INVERTED,
                  TURN_ENCODER_INVERTED));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT =
          configureModule(
              CONSTANT_CREATOR.createModuleConstants(
                  SC1.FRONT_RIGHT_TURN,
                  SC1.FRONT_RIGHT_DRIVE,
                  SC1.FRONT_RIGHT_TURN_ABS_ENC,
                  Rotations.of(0),
                  WHEEL_BASE.div(2.0),
                  TRACK_WIDTH.div(-2.0),
                  INVERT_RIGHT_SIDE,
                  TURN_INVERTED,
                  TURN_ENCODER_INVERTED));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT =
          configureModule(
              CONSTANT_CREATOR.createModuleConstants(
                  SC1.BACK_LEFT_TURN,
                  SC1.BACK_LEFT_DRIVE,
                  SC1.BACK_LEFT_TURN_ABS_ENC,
                  Rotations.of(0),
                  WHEEL_BASE.div(-2.0),
                  TRACK_WIDTH.div(2.0),
                  INVERT_LEFT_SIDE,
                  TURN_INVERTED,
                  TURN_ENCODER_INVERTED));
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT =
          configureModule(
              CONSTANT_CREATOR.createModuleConstants(
                  SC1.BACK_RIGHT_TURN,
                  SC1.BACK_RIGHT_DRIVE,
                  SC1.BACK_RIGHT_TURN_ABS_ENC,
                  Rotations.of(0),
                  WHEEL_BASE.div(-2.0),
                  TRACK_WIDTH.div(-2.0),
                  INVERT_RIGHT_SIDE,
                  TURN_INVERTED,
                  TURN_ENCODER_INVERTED));

  /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *     y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
