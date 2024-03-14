package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.lib.Neo.Model;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.intake.IntakeState.IntakeAngle;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming;
import static frc.lib.Color.Colors.GREEN;
import static frc.lib.Color.Colors.ORANGE;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;


public class Intake extends SubsystemBase implements Logged {
    private final Neo intakeMotor = new Neo(INTAKE_MOTOR_ID, Model.SparkMax);
    private final Neo angleMotor = new Neo(ANGLE_MOTOR_ID, Model.SparkMax);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ENCODER_PORT);

    @Log.NT
    public final DigitalInput beambreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger hasNoteTrigger = new Trigger(() -> !beambreak.get()).debounce(0.2);

    private final PIDController anglePIDcontroller = new PIDController(INTAKE_GAINS.kp, INTAKE_GAINS.ki, INTAKE_GAINS.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(INTAKE_GAINS.ks, INTAKE_GAINS.kg, INTAKE_GAINS.kv, INTAKE_GAINS.ka);

    @Log.NT
    public IntakeAngle setpoint = IntakeAngle.SHOOTER;

    @Log.NT
    public final Trigger atSetpointTrigger = new Trigger(anglePIDcontroller::atSetpoint).debounce(0.2);
    public final Trigger atShooterTrigger = atSetpointTrigger.and(() -> setpoint.equals(IntakeAngle.SHOOTER));

    public final Trigger intakingTrigger = new Trigger(() -> getCurrentCommand() != null && getCurrentCommand().getName().equals("intakeCommand"));

    private final LEDs leds = LEDs.getInstance();


    public Intake() {
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setSmartCurrentLimit(50);

        angleMotor.setConversionFactors(ANGLE_MOTOR_CONVERSION_FACTOR);
        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(40);
        angleMotor.setInverted(true);
        angleMotor.setOpenLoopRampRate(0.35);

        anglePIDcontroller.setTolerance(INTAKE_TOLERANCE);

        encoder.setPositionOffset(INTAKE_ENCODER_OFFSET);
        encoder.setDistancePerRotation(360);

        initShuffleboard();
        setDefaultCommand(setIntakeCommand(new IntakeState(0, IntakeAngle.SHOOTER, false)));
    }

    @Log.NT
    public double rawEncoderVal(){
        return 360 - encoder.getDistance();
    }

    @Log.NT(key = "intakeAngle")
    public double getAngle() {
        double val = 360 - encoder.getDistance();

        while (val < -50) val += 360;
        while (val > 200) val -= 360;

        return val;
    }

    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    double encoderAngle;
    private void setIntakeAngle(IntakeAngle angle) {
        setpoint = angle;
        encoderAngle = getAngle();

        double pid = anglePIDcontroller.calculate(encoderAngle + INTAKE_READING_OFFSET, angle.angle);
        double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0) / 60.0;

        if (encoderAngle > 200 || encoderAngle < -50) DriverStation.reportError("intake encoder malfunctions", false);
        else angleMotor.setVoltage(pid + ff);
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
        return new SequentialCommandGroup(
                setIntakeCommand(new IntakeState(0.35, angle, true)).until(hasNoteTrigger.debounce(0.15)),
                new InstantCommand(vibrateCommand::schedule),
                setIntakeCommand(new IntakeState(0, IntakeAngle.SHOOTER, false)).until(atShooterTrigger),
                pumpNoteCommand().unless(DriverStation::isAutonomous)).withName("intakeCommand")
                .deadlineWith(new StartEndCommand(
                        () -> leds.setPattern(BLINKING, ORANGE.color).schedule(),
                        () -> leds.setPattern(SOLID, GREEN.color).withTimeout(1.5).schedule()
                ));
    }

    public Command halfIntakeFromGround() {
        return setIntakeCommand(new IntakeState(0.35, IntakeAngle.GROUND, false)).until(hasNoteTrigger);
    }

    public Command closeIntakeCommand() {
        return setIntakeCommand(new IntakeState(0, IntakeAngle.SHOOTER, false)).until(atShooterTrigger);
    }

    public Command shootToAmpCommand() {
        return setIntakeCommand(new IntakeState(AMP_SHOOTER_SPEED, IntakeAngle.AMP, true)).until(hasNoteTrigger.negate().debounce(0.3));
    }

    public Command transportToShooterCommand(BooleanSupplier toAmp) {
        return setIntakeCommand(new IntakeState(toAmp.getAsBoolean() ? -0.6 : -0.75, IntakeAngle.SHOOTER, true))
                .until(hasNoteTrigger.negate().debounce(0.75))
                .deadlineWith(leds.setPattern(SOLID, GREEN.color));
    }

    public Command pumpNoteCommand() {
        return new SequentialCommandGroup(
                this.runEnd(() -> intakeMotor.set(-0.2), intakeMotor::stopMotor).withTimeout(0.1),
                new WaitCommand(0.25),
                this.runEnd(() -> intakeMotor.set(0.2), intakeMotor::stopMotor).withTimeout(0.25))
                .withInterruptBehavior(kCancelIncoming);
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> angleMotor.setIdleMode(IdleMode.kCoast),
                () -> angleMotor.setIdleMode(IdleMode.kBrake)).ignoringDisable(true);
    }

    private void initShuffleboard() {
        RobotContainer.robotData.addBoolean("intake beambreak", hasNoteTrigger);
        RobotContainer.robotData.addDouble("intake angle", this::getAngle).withSize(2, 2);
    }

    @Log.NT
    private boolean intakingTrigger() {
        return intakingTrigger.getAsBoolean();
    }

    @Log.NT
    private double getIntakeVel() {
        return intakeMotor.getVelocity();
    }

    @Log.NT
    private double getAngleMotorOutputDC() {
        return angleMotor.getAppliedOutput();
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