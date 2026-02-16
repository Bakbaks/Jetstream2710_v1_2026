package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.robot.subsystems.Intake;

import java.util.List;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {

	private final TalonFX WheelMotor;
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final VelocityVoltage WheelvelocityRequest = new VelocityVoltage(0).withSlot(0);

	private double dashboardWheelPercent = IntakeConstants.kWheelPercent;

	private double dashboardWheelTargetRPM = IntakeConstants.kWheelRPM;

	public Intake() {
		WheelMotor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

		configureMotor(WheelMotor, InvertedValue.CounterClockwise_Positive);
	}

	private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
		final TalonFXConfiguration config = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs().withInverted(invertDirection).withNeutralMode(NeutralModeValue.Coast))
				.withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
				.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(IntakeConstants.kIntakeStatorCurrentLimit)).withStatorCurrentLimitEnable(true))
				.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(IntakeConstants.kIntakeSupplyCurrentLimit)).withSupplyCurrentLimitEnable(true))
				.withSlot0(
				new Slot0Configs()
					.withKP(IntakeConstants.KIntakeP)
					.withKI(IntakeConstants.KIntakeI)
					.withKD(IntakeConstants.KIntakeD)
					.withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond))); // 12 volts when requesting max RPS);
					
		motor.getConfigurator().apply(config);
	}

	/** Set percent outputs for both motors (range -1.0 .. 1.0). */
	public void setPercentOutputs(double wheelPercent) {
		WheelMotor.setControl(voltageRequest.withOutput(Volts.of(wheelPercent * 12.0)));
	}

	/** Stop both motors. */
	public void stop() {
		setPercentOutputs(0.0);
	}

	public void setWheelRPM() {
		WheelMotor.setControl(
			WheelvelocityRequest.withVelocity(RPM.of(dashboardWheelTargetRPM))
		);
	}

	private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
	
	@Override
	public void initSendable(SendableBuilder builder) {

        initSendable(builder, WheelMotor, "wheelMotor");

		builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
		builder.addDoubleProperty("Wheel Dashboard RPM", () -> dashboardWheelTargetRPM, v -> dashboardWheelTargetRPM = v);

		builder.addDoubleProperty("Wheel Target RPM", () -> WheelvelocityRequest.getVelocityMeasure().in(RPM), null);
	}
}