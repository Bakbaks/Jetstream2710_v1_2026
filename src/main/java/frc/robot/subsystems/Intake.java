package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public enum Speed {
        STOP(0),
        INTAKE(IntakeConstants.kRollerPercent);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        DEFAULT(0),
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kExtendoReduction = IntakeConstants.kExtendoReduction;
    private static final AngularVelocity kMaxExtendoSpeed = KrakenX60.kFreeSpeed.div(kExtendoReduction);
    private static final Angle kPositionTolerance = IntakeConstants.kPositionTolerance;

    private final TalonFX ExtendoMotor, rollerMotor;
    private final VoltageOut ExtendoVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage ExtendoMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    //private boolean isHomed = false;

    public Intake() {
        ExtendoMotor = new TalonFX(Ports.kIntakeExtendo);
        rollerMotor = new TalonFX(Ports.kIntakeRollers);
        configureExtendoMotor();
        configureRollerMotor();
        //SmartDashboard.putData(this);
    }

    private void configureExtendoMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive) // need to set
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(IntakeConstants.kExtendoStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(IntakeConstants.kExtendoSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kExtendoReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxExtendoSpeed)
                    .withMotionMagicAcceleration(kMaxExtendoSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(IntakeConstants.kExtendoIntakeP)
                    .withKI(IntakeConstants.kExtendoIntakeI)
                    .withKD(IntakeConstants.kExtendoIntakeD)
                    .withKV(12.0 / kMaxExtendoSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        ExtendoMotor.getConfigurator().apply(config);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(IntakeConstants.kRollerStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(IntakeConstants.kRollerSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }

    // private boolean isPositionWithinTolerance() {
    //     final Angle currentPosition = ExtendoMotor.getPosition().getValue();
    //     final Angle targetPosition = ExtendoMotionMagicRequest.getPositionMeasure();
    //     return currentPosition.isNear(targetPosition, kPositionTolerance);
    // }

    public void setExtendoPercentOutput(double percentOutput) {
        ExtendoMotor.setControl(
            ExtendoVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

     public void setExtendoPosition(Position position) {
         ExtendoMotor.setControl(
             ExtendoMotionMagicRequest
                 .withPosition(position.angle())
         );
    }

    public void setIntakeSpeed(Speed speed) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }


    public void setRollerPercentOutput(double percentOutput) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    
    // public Command intakeCommand() {
    //     return startEnd(
    //         () -> {
    //             set(Position.INTAKE);
    //             set(Speed.INTAKE);
    //         },
    //         () -> set(Speed.STOP)
    //     );
    // }

    // public Command agitateCommand() {
    //     return runOnce(() -> set(Speed.INTAKE))
    //         .andThen(
    //             Commands.sequence(
    //                 runOnce(() -> set(Position.AGITATE)),
    //                 Commands.waitUntil(this::isPositionWithinTolerance),
    //                 runOnce(() -> set(Position.INTAKE)),
    //                 Commands.waitUntil(this::isPositionWithinTolerance)
    //             )
    //             .repeatedly()
    //         )
    //         .handleInterrupt(() -> {
    //             set(Position.INTAKE);
    //             set(Speed.STOP);
    //         });
    // }

    // public Command homingCommand() {
    //     return Commands.sequence(
    //         runOnce(() -> setExtendoPercentOutput(0.1)),
    //         Commands.waitUntil(() -> ExtendoMotor.getSupplyCurrent().getValue().in(Amps) > 6),
    //         runOnce(() -> {
    //             ExtendoMotor.setPosition(Position.HOMED.angle());
    //             isHomed = true;
    //             set(Position.STOWED);
    //         })
    //     )
    //     .unless(() -> isHomed)
    //     .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    // }

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
    //     builder.addDoubleProperty("Angle (degrees)", () -> ExtendoMotor.getPosition().getValue().in(Degrees), null);
    //     builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
    //     builder.addDoubleProperty("Extendo Supply Current", () -> ExtendoMotor.getSupplyCurrent().getValue().in(Amps), null);
    //     builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
    // }
}