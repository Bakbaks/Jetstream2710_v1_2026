package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.shooter.Rollers;

// two stage ahh
public class Conveyer extends SubsystemBase {

	private final TalonFX floorMotor;
	private final TalonFX feederMotor;
	private final VoltageOut voltageRequest = new VoltageOut(0);

	// Dashboard-configurable percent outputs
	private double dashboardFloorPercent = 0.5;
	private double dashboardFeederPercent = 1;

	public Conveyer() {
		floorMotor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);
		feederMotor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);

		configureMotor(floorMotor, InvertedValue.Clockwise_Positive);
		configureMotor(feederMotor, InvertedValue.CounterClockwise_Positive);
	}

	private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
		final TalonFXConfiguration config = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs().withInverted(invertDirection).withNeutralMode(NeutralModeValue.Coast))
				.withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
				.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(80)).withStatorCurrentLimitEnable(true));

		motor.getConfigurator().apply(config);
	}

	/** Set percent outputs for both motors (range -1.0 .. 1.0). */
	public void setPercentOutputs(double floorPercent, double feederPercent) {
		floorMotor.setControl(voltageRequest.withOutput(Volts.of(floorPercent * 12.0)));
		feederMotor.setControl(voltageRequest.withOutput(Volts.of(feederPercent * 12.0)));
	}

	/** Stop both motors. */
	public void stop() {
		setPercentOutputs(0.0, 0.0);
	}

	/**
	 * Returns a command that will run the conveyer at the given percents only while the shooter
	 * reports being up to speed. The command requires this subsystem.
	 */
	public Command runWhenShooterReady(Rollers shooter, double floorPercent, double feederPercent) {
		return Commands.run(() -> {
			if (shooter.isVelocityWithinTolerance()) {
				setPercentOutputs(floorPercent, feederPercent);
			} else {
				stop();
			}
		}, this)
				.finallyDo(interrupted -> stop())
				.withName("Conveyer::runWhenShooterReady");
	}

	/** Dashboard helper: run using dashboard-configured percents while shooter up to speed. */
	public Command dashboardRunWhenReady(Rollers shooter) {
		return runWhenShooterReady(shooter, dashboardFloorPercent, dashboardFeederPercent);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Floor Percent", () -> dashboardFloorPercent, v -> dashboardFloorPercent = v);
		builder.addDoubleProperty("Feeder Percent", () -> dashboardFeederPercent, v -> dashboardFeederPercent = v);
		builder.addDoubleProperty("Floor Motor Current", () -> floorMotor.getStatorCurrent().getValue().in(Amps), null);
		builder.addDoubleProperty("Feeder Motor Current", () -> feederMotor.getStatorCurrent().getValue().in(Amps), null);
	}
}
