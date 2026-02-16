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
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.robot.subsystems.Flywheel;

import java.util.List;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Hopper extends SubsystemBase {

	private final TalonFX FloorMotor;
	private final TalonFX FeederMotor;
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final VelocityVoltage FloorvelocityRequest = new VelocityVoltage(0).withSlot(0);
	private final VelocityVoltage FeedervelocityRequest = new VelocityVoltage(0).withSlot(0);

	// Dashboard-configurable percent outputs
	private double dashboardFloorPercent = HopperConstants.kFloorPercent;
	private double dashboardFeederPercent = HopperConstants.kFeederPercent;

	// I'm going to replace percent with rpm
	private double dashboardFloorTargetRPM = HopperConstants.kFloorRPM;
	private double dashboardFeederTargetRPM = HopperConstants.kFeederRPM;

	public Hopper() {
		FloorMotor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);
		FeederMotor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

		configureMotor(FloorMotor, InvertedValue.Clockwise_Positive);
		configureMotor(FeederMotor, InvertedValue.CounterClockwise_Positive);
	}

	private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
		final TalonFXConfiguration config = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs().withInverted(invertDirection).withNeutralMode(NeutralModeValue.Coast))
				.withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
				.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(HopperConstants.kHopperStatorCurrentLimit)).withStatorCurrentLimitEnable(true))
				.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(HopperConstants.kHopperSupplyCurrentLimit)).withSupplyCurrentLimitEnable(true))
				.withSlot0(
				new Slot0Configs()
					.withKP(HopperConstants.KHopperP)
					.withKI(HopperConstants.KHopperI)
					.withKD(HopperConstants.KHopperD)
					.withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond))); // 12 volts when requesting max RPS);
					
		motor.getConfigurator().apply(config);
	}

	/** Set percent outputs for both motors (range -1.0 .. 1.0). */
	public void setPercentOutputs(double floorPercent, double feederPercent) {
		FloorMotor.setControl(voltageRequest.withOutput(Volts.of(floorPercent * 12.0)));
		FeederMotor.setControl(voltageRequest.withOutput(Volts.of(feederPercent * 12.0)));
	}

	/** Stop both motors. */
	public void stop() {
		setPercentOutputs(0.0, 0.0);
	}


	public void setFeederRPM() {
        FeederMotor.setControl(
            FeedervelocityRequest.withVelocity(RPM.of(dashboardFeederTargetRPM))
        );
    }

	public void setFloorRPM() {
		FloorMotor.setControl(
			FloorvelocityRequest.withVelocity(RPM.of(dashboardFloorTargetRPM))
		);
	}


	private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
	
	@Override
	public void initSendable(SendableBuilder builder) {

		initSendable(builder, FeederMotor, "feederMotor");
        initSendable(builder, FloorMotor, "floorMotor");

		builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
		builder.addDoubleProperty("Floor Dashboard RPM", () -> dashboardFloorTargetRPM, v -> dashboardFloorTargetRPM = v);
		builder.addDoubleProperty("Feeder Dashboard RPM", () -> dashboardFeederTargetRPM, v -> dashboardFeederTargetRPM = v);

		builder.addDoubleProperty("Floor Target RPM", () -> FloorvelocityRequest.getVelocityMeasure().in(RPM), null);
		builder.addDoubleProperty("Feeder Target RPM", () -> FeedervelocityRequest.getVelocityMeasure().in(RPM), null);


		// builder.addDoubleProperty("Floor Percent", () -> dashboardFloorPercent, v -> dashboardFloorPercent = v);
		// builder.addDoubleProperty("Feeder Percent", () -> dashboardFeederPercent, v -> dashboardFeederPercent = v);
		// builder.addDoubleProperty("Floor Motor Current", () -> floorMotor.getStatorCurrent().getValue().in(Amps), null);
		// builder.addDoubleProperty("Feeder Motor Current", () -> feederMotor.getStatorCurrent().getValue().in(Amps), null);
	}
}