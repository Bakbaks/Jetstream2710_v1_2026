package frc.robot.subsystems.Intake;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.commands.OutTake;
import frc.robot.Ports;

import frc.robot.Constants.IntakeConstants;

public class IntakeExtendo extends SubsystemBase {
    public enum Position {
        DEFAULT(0.0),
        INTERMEDIATE(3.0),
        EXTENDED(6.0);


        private final double inches;
        Position(double inches) { 
            this.inches = inches; 
        }

        public double inches() { return inches; }
    }

    private static final double kMotorToPinionReduction = IntakeConstants.kMotorToPinionReduction;

    // This is *pinion* free speed (mechanism RPS) because we set mechanism = pinion rotations
    private static final AngularVelocity kMaxExtendoSpeed = KrakenX60.kFreeSpeed.div(kMotorToPinionReduction);

    private static final Distance kPositionTolerance = IntakeConstants.kPositionTolerance;

    private final TalonFX ExtendoMotor;
    private final VoltageOut ExtendoVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage ExtendoMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    //private boolean isHomed = false;

    public IntakeExtendo() {
        ExtendoMotor = new TalonFX(Ports.kIntakeExtendo);
        configureExtendoMotor();
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

    /**
     * Move the extendo toward a goal Position (defaults to DEFAULT) while adding a tunable
     * "jiggle" (small forward/back oscillation) that fades out as you approach the goal.
     *
     * All tuning is pulled from Constants.IntakeConstants:
     *  - kExtendoJiggleFrequencyHz
     *  - kExtendoJiggleStep
     *  - kExtendoJiggleMinStep (optional)
     *  - kExtendoMinInches / kExtendoMaxInches (optional clamp)
     */
    public Command jiggleToDefault() {
        final double goalInches = Position.DEFAULT.inches();

        final double freqHz      = Math.max(0.0, IntakeConstants.kExtendoJiggleFrequencyHz);
        final double approachAmp = Math.max(0.0, IntakeConstants.kExtendoJiggleStep.in(Inches));
        final double minAmpIn    = Math.max(0.0, IntakeConstants.kExtendoJiggleMinStep.in(Inches));
        final double holdAmpIn   = Math.max(0.0, IntakeConstants.kExtendoJiggleHoldStep.in(Inches));

        // Clamp range from enum endpoints (DEFAULT..EXTENDED)
        final double minIn = Math.min(Position.DEFAULT.inches(), Position.EXTENDED.inches());
        final double maxIn = Math.max(Position.DEFAULT.inches(), Position.EXTENDED.inches());

        final Timer timer = new Timer();

        final double startIn = getExtendoInches();
        final double totalDist = Math.max(1e-6, Math.abs(goalInches - startIn));

        return Commands.sequence(
            Commands.runOnce(() -> { timer.reset(); timer.start(); }),

            // Runs forever until interrupted/canceled
            Commands.run(() -> {
                final double curIn = getExtendoInches();
                final double remaining = Math.abs(goalInches - curIn);
                final boolean atGoal = remaining <= kPositionTolerance.in(Inches);

                // Two modes:
                //  - approaching: taper amplitude down with distance (plus optional minimum)
                //0.40
                // 0.10
                // 0.30
                // 0.05
                // 0.20
                // 0.02
                //  - holding: constant small amplitude around the goal
                //0
                // 0.10
                // 0
                // 0.10
                // 0
                // 0.10
                double amp;
                if (atGoal) {
                    amp = holdAmpIn;
                } else {
                    amp = approachAmp * (remaining / totalDist);
                    amp = Math.max(amp, minAmpIn);
                    amp = Math.min(amp, remaining); // don't bounce past the goal near the end
                }

                // final double jiggle = (freqHz > 0.0)
                //     ? amp * Math.abs(Math.sin(2.0 * Math.PI * freqHz * timer.get()))
                //     : 0.0;

                double jiggle;
                if (freqHz > 0.0) {
                    double phase = 2.0 * Math.PI * freqHz * timer.get();
                    double oscillation = Math.abs(Math.sin(phase));
                    jiggle = amp * oscillation;
                } else {
                    jiggle = 0.0;
                }

                double setpoint = goalInches + jiggle;

                // Clamp to enum travel range
                setpoint = Math.max(minIn, Math.min(maxIn, setpoint));

                setExtendoInches(setpoint);
            })
            // Optional: ensure motors stop if you cancel the command
            .finallyDo(interrupted -> timer.stop())
        );
    }

    public void setExtendoZero(){
        ExtendoMotor.setPosition(0.0);
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