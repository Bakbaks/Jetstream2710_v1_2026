package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
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
        DEFAULT(0.0),
        INTERMEDIATE(3.0),
        EXTENDED(6.0);

        private final double inches;
        Position(double inches) { this.inches = inches; }
        public double inches() { return inches; }
    }

    private static final double kMotorToPinionReduction = IntakeConstants.kMotorToPinionReduction;

    // This is *pinion* free speed (mechanism RPS) because we set mechanism = pinion rotations
    private static final AngularVelocity kMaxExtendoSpeed = KrakenX60.kFreeSpeed.div(kMotorToPinionReduction);

    private static final Distance kPositionTolerance = IntakeConstants.kPositionTolerance;

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
                    .withSensorToMechanismRatio(kMotorToPinionReduction)
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

    private static double inchesToPinionRot(double inches) {
        return inches / IntakeConstants.kInchesPerPinionRotation; // inches * 3/pi
    }

    private static double pinionRotToInches(double pinionRot) {
        return pinionRot * IntakeConstants.kInchesPerPinionRotation; // rot * pi/3
    }

    private boolean isPositionWithinTolerance() {
        double errIn = Math.abs(getExtendoInches() - getExtendoTargetInches());
        return errIn <= kPositionTolerance.in(Inches);
    }
    
    public void setExtendoPercentOutput(double percentOutput) {
        ExtendoMotor.setControl(
            ExtendoVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public void setExtendoInches(double inches) {
        double pinionRot = inchesToPinionRot(inches);
        ExtendoMotor.setControl(
            ExtendoMotionMagicRequest.withPosition(Rotations.of(pinionRot))
        );
    }

    public void setExtendoPosition(Position position) {
        setExtendoInches(position.inches());
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

    
    public Command intakeCommand() {
        return startEnd(
            () -> {
                setExtendoPosition(Position.EXTENDED);
                setIntakeSpeed(Speed.INTAKE);
            },
            () -> setIntakeSpeed(Speed.STOP)
        );
    }

    public double getExtendoPinionRotations() {
        return ExtendoMotor.getPosition().getValue().in(Rotations); // mechanism rotations = pinion rotations
    }

    public double getExtendoInches() {
        return pinionRotToInches(getExtendoPinionRotations());
    }

    public double getExtendoTargetInches() {
        double targetPinionRot = ExtendoMotionMagicRequest.getPositionMeasure().in(Rotations);
        return pinionRotToInches(targetPinionRot);
    }
}