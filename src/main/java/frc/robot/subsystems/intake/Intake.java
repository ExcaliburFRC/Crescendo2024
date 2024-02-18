package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.intake.IntakeState.IntakeAngle;
import frc.robot.subsystems.shooter.ShooterState;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming;
import static frc.lib.Color.Colors.ORANGE;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;

public class Intake extends SubsystemBase implements Logged {
    private final Neo intakeMotor = new Neo(INTAKE_MOTOR_ID);
    private final Neo angleMotor = new Neo(ANGLE_MOTOR_ID);

    private final DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(ENCODER_PORT);

    @Log.NT
    private final DigitalInput intakeBeambreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger hasNoteTrigger = new Trigger(() -> !intakeBeambreak.get()).debounce(0.2);

    private final PIDController anglePIDcontroller = new PIDController(INTAKE_GAINS.kp, INTAKE_GAINS.ki, INTAKE_GAINS.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(INTAKE_GAINS.ks, INTAKE_GAINS.kg, INTAKE_GAINS.kv, INTAKE_GAINS.ka);

    @Log.NT
    public IntakeAngle setpoint;

    @Log.NT
    public final Trigger atSetpointTrigger = new Trigger(anglePIDcontroller::atSetpoint).debounce(0.2);
    public final Trigger atShooterTrigger = atSetpointTrigger.and(() -> setpoint.equals(IntakeAngle.SHOOTER));

    public final Trigger intakingTrigger = new Trigger(() -> getCurrentCommand() != null && getCurrentCommand().equals("intakeCommand"));

    private final LEDs leds = LEDs.getInstance();

    public Intake() {
        intakeMotor.setIdleMode(IdleMode.kCoast);

        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(40);
        angleMotor.setInverted(true);
        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setOpenLoopRampRate(0.35);

        angleMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false);
        angleMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false);

        intakeEncoder.setDistancePerRotation(360);
        intakeEncoder.setPositionOffset(INTAKE_ENCODER_OFFSET_POSITION);

        anglePIDcontroller.setTolerance(INTAKE_TOLERANCE);

        setDefaultCommand(intakeIdleCommand());
    }

    @Log.NT(key = "intakeAngle")
    public double getAngle() {
        double angle = -intakeEncoder.getDistance();
        while (angle < -100) angle += 360;
        return angle;
    }

    @Log.NT(key = "intakeVelocity")
    private double getIntakeVel() {
        return intakeMotor.getVelocity();
    }

    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    private void setIntakeAngle(IntakeAngle angle) {
        setpoint = angle;

        double pid = anglePIDcontroller.calculate(getAngle() + INTAKE_READING_OFFSET, angle.angle);
        double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0) / 60.0;

        angleMotor.setVoltage(pid + ff);
    }

    private void stopMotors() {
        intakeMotor.stopMotor();
        angleMotor.stopMotor();
    }

    private Command setIntakeCommand(IntakeState intakeState) {
        return this.runEnd(() -> {
            leds.setPattern(BLINKING, ORANGE.color);

            setIntakeAngle(intakeState.angle);

            if (!intakeState.waitForAngle) setIntakeSpeed(intakeState.intakeDC);
            else if (atSetpointTrigger.getAsBoolean()) setIntakeSpeed(intakeState.intakeDC);
            else setIntakeSpeed(0);

        }, this::stopMotors);
    }

    public Command intakeFromAngleCommand(IntakeAngle angle, Command vibrateCommand) {
        return setIntakeCommand(new IntakeState(0.35, angle, true)).until(hasNoteTrigger.debounce(0.15)).andThen(vibrateCommand::schedule); // schedule it instead of composing it to free up the intake requirement immediately
    }

    public Command shootToAmpCommand() {
        return setIntakeCommand(new IntakeState(AMP_SHOOTER_SPEED, IntakeAngle.AMP, true)).until(hasNoteTrigger.negate());
    }

    public Command transportToShooterCommand(Supplier<ShooterState> state) {
        return setIntakeCommand(new IntakeState(state.get().isSameVel() ? -0.75 : -0.35, IntakeAngle.SHOOTER, true))
                .until(hasNoteTrigger.negate().debounce(0.75));
    }

    public Command intakeIdleCommand() {
        return new SequentialCommandGroup(
                setIntakeCommand(new IntakeState(0, IntakeAngle.SHOOTER, false)).until(atShooterTrigger),
                new ConditionalCommand(pumpNoteCommand(), new InstantCommand(() -> {}), hasNoteTrigger),
                Commands.idle());
    }

    public Command pumpNoteCommand() {
        return new SequentialCommandGroup(
                this.runEnd(() -> intakeMotor.set(-0.2), intakeMotor::stopMotor).withTimeout(0.1),
                new WaitCommand(0.25),
                this.runEnd(() -> intakeMotor.set(0.2), intakeMotor::stopMotor)
                        .withInterruptBehavior(kCancelIncoming)
                        .withTimeout(0.25));
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> angleMotor.setIdleMode(IdleMode.kCoast),
                () -> angleMotor.setIdleMode(IdleMode.kBrake)).ignoringDisable(true);
    }

    // SysId stuff
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> degrees = mutable(Degrees.of(0));

    private final MutableMeasure<Velocity<Angle>> velocity = mutable(DegreesPerSecond.of(0));

    private final SysIdRoutine angleSysid = new SysIdRoutine(
            sysidConfig,
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> angleMotor.setVoltage(volts.in(Volts)),
                    log -> log.motor("angleMotor")
                            .voltage(appliedVoltage.mut_replace(
                                    angleMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(degrees.mut_replace(getAngle(), Degrees))
                            .angularVelocity(velocity.mut_replace(angleMotor.getVelocity(), RPM)),
                    this
            ));

    public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
        return angleSysid.quasistatic(direction);
    }

    public Command sysidDynamic(SysIdRoutine.Direction direction) {
        return angleSysid.dynamic(direction);
    }
}