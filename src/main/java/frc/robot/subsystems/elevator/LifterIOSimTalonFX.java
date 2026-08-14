package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.CANBusPorts.CANHD;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.Robot;

public class LifterIOSimTalonFX implements LifterIO {
    private final DCMotor GEARBOX = DCMotor.getKrakenX60(1);

    private final ElevatorSim elevatorSim;
    private final DCMotorSim motorSim;
    private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
        new VelocityTorqueCurrentFOC(0.0).withSlot(1);
    private final NeutralOut brake = new NeutralOut();

    private final TalonFX motor;
    private final TalonFXConfiguration config;

    // Inputs from intake motor
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
        config.Slot0 = VELOCITY_VOLTAGE_GAINS;
        config.TorqueCurrent.PeakForwardTorqueCurrent = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent =
            -KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

        motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(GEARBOX, 0.0005, 1.0), GEARBOX);

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
        var motorSim = motor.getSimState();
        motorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
        elevatorSim.setInput(motorSim.getMotorVoltage());
        elevatorSim.update(Robot.defaultPeriodSecs);
        motorSim.setRawRotorPosition(elevatorSim.getAngularPositionRotations() * MOTOR_REDUCTION);
        motorSim.setRotorVelocity(elevatorSim.getAngularVelocity().times(MOTOR_REDUCTION));

        // Update remaining inputs
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = supplyCurrent.getValueAsDouble();
        inputs.positionMeters = ;
        inputs.velocityMetersPerSec =
            velocity.getValue().in(RadiansPerSecond) * RADIUS.in(Meters) / MOTOR_REDUCTION;
    }

    public void setOpenLoop(double volts) {
        if (volts < 1e-6) {
            motor.setControl(brake);
        } else {
            motor.setControl(voltageRequest.withOutput(volts));
        }
    }

    public void setPosition(double positionMeters) {
        motionMagicTorqueCurrentRequest.withPosition(positionMeters);
    }
}
