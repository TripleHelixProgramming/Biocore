package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.CANBusPorts.CANHD;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.Robot;

public class LifterIOSimTalonFX implements LifterIO {
  // ASSUMED placeholder — tune once elevator hardware exists
  private static final double MOTOR_REDUCTION = 12.0;

  // ASSUMED placeholder — tune once elevator hardware exists
  private static final double DRUM_RADIUS_METERS = 0.02;

  // ASSUMED placeholder — tune once elevator hardware exists
  private static final double CARRIAGE_MASS_KG = 5.0;
  private static final double MIN_HEIGHT_METERS = 0.0;
  private static final double MAX_HEIGHT_METERS = 1.5;

  // ASSUMED placeholder gains — tune once elevator hardware exists
  private static final Slot0Configs GAINS =
      new Slot0Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKG(0.3)
          .withKS(0.1)
          .withKV(0.0)
          .withKA(0.0)
          .withKP(60.0)
          .withKI(0.0)
          .withKD(0.0);

  // ASSUMED placeholder — tune once elevator hardware exists (target ~0.75 m/s cruise, 1.5 m/s^2)
  private static final double CRUISE_VELOCITY_ROTATIONS_PER_SEC =
      Units.radiansToRotations(0.75 / DRUM_RADIUS_METERS);
  private static final double ACCELERATION_ROTATIONS_PER_SEC2 =
      Units.radiansToRotations(1.5 / DRUM_RADIUS_METERS);

  private final DCMotor GEARBOX = DCMotor.getKrakenX60(1);

  private final ElevatorSim elevatorSim;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final NeutralOut brake = new NeutralOut();

  private final TalonFX motor;
  private final TalonFXConfiguration config;

  // Inputs from lifter motor
  private final StatusSignal<Angle> angularPositon;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent, torqueCurrent;
  private final StatusSignal<Double> dutyCycle;

  public LifterIOSimTalonFX() {
    motor = new TalonFX(CANHD.LIFTER, CANHD.BUS);
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.Feedback.SensorToMechanismRatio = MOTOR_REDUCTION;
    config.Slot0 = GAINS;
    config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_ROTATIONS_PER_SEC;
    config.MotionMagic.MotionMagicAcceleration = ACCELERATION_ROTATIONS_PER_SEC2;
    config.TorqueCurrent.PeakForwardTorqueCurrent = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimit = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    elevatorSim =
        new ElevatorSim(
            GEARBOX,
            MOTOR_REDUCTION,
            CARRIAGE_MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            true,
            MIN_HEIGHT_METERS);

    velocity = motor.getVelocity();
    angularPositon = motor.getPosition();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    dutyCycle = motor.getDutyCycle();
    torqueCurrent = motor.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, angularPositon, appliedVolts, supplyCurrent, dutyCycle, torqueCurrent);
  }

  public void updateInputs(LifterIOInputs inputs) {
    // Check whether the motor controller is connected, and get updated status signals if so
    inputs.connected =
        connectedDebounce.calculate(
            BaseStatusSignal.refreshAll(
                    velocity, angularPositon, appliedVolts, supplyCurrent, dutyCycle, torqueCurrent)
                .isOK());

    // Update simulation state
    var simState = motor.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    elevatorSim.setInput(simState.getMotorVoltage());
    elevatorSim.update(Robot.defaultPeriodSecs);
    simState.setRawRotorPosition(
        Units.radiansToRotations(elevatorSim.getPositionMeters() / DRUM_RADIUS_METERS)
            * MOTOR_REDUCTION);
    simState.setRotorVelocity(
        Units.radiansToRotations(elevatorSim.getVelocityMetersPerSecond() / DRUM_RADIUS_METERS)
            * MOTOR_REDUCTION);

    // Update remaining inputs (Feedback.SensorToMechanismRatio means these signals already read
    // out in drum rotations, not raw rotor rotations)
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = supplyCurrent.getValueAsDouble();
    inputs.positionMeters =
        Units.rotationsToRadians(angularPositon.getValueAsDouble()) * DRUM_RADIUS_METERS;
    inputs.velocityMetersPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) * DRUM_RADIUS_METERS;
  }

  public void setOpenLoop(double volts) {
    if (Math.abs(volts) < 1e-6) {
      motor.setControl(brake);
    } else {
      motor.setControl(voltageRequest.withOutput(volts));
    }
  }

  public void setPosition(double positionMeters) {
    motor.setControl(
        motionMagicTorqueCurrentRequest.withPosition(
            Units.radiansToRotations(positionMeters / DRUM_RADIUS_METERS)));
  }
}
