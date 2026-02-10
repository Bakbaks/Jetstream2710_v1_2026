package frc.robot.subsystems.shooter;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Rollers extends SubsystemBase {

  private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX FrontLeftMotor, BackLeftMotor, FrontRightMotor, BackRightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 0.0;

  /** Creates a new subsystem. */
  public Rollers() {
    FrontLeftMotor = new TalonFX(Ports.kFrontLeftShooter, Ports.kRoboRioCANBus);
    BackLeftMotor = new TalonFX(Ports.kBackLeftShooter, Ports.kRoboRioCANBus);
    FrontRightMotor = new TalonFX(Ports.kFrontRightShooter, Ports.kRoboRioCANBus);
    BackRightMotor = new TalonFX(Ports.kBackRightShooter, Ports.kRoboRioCANBus);
    motors = List.of(FrontLeftMotor, BackLeftMotor, FrontRightMotor, BackRightMotor);

    configureMotor(FrontLeftMotor, InvertedValue.Clockwise_Positive);
    configureMotor(BackLeftMotor, InvertedValue.Clockwise_Positive);

    configureMotor(FrontRightMotor, InvertedValue.Clockwise_Positive);
    configureMotor(BackRightMotor, InvertedValue.Clockwise_Positive);
  
  }

  private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
    final TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(invertDirection)
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withVoltage(
            new VoltageConfigs()
                .withPeakReverseVoltage(Volts.of(0))
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(0.5)
                .withKI(2)
                .withKD(0)
                .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
        );
    
    motor.getConfigurator().apply(config);
  }

  public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, FrontLeftMotor, "FrontLeft");
        initSendable(builder, BackLeftMotor, "BackLeft");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
    }
}