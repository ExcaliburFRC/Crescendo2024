package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.intake.IntakeState.intakeAngle;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.lib.Color.Colors.ORANGE;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;

public class Intake extends SubsystemBase implements Logged {
    private final Neo intakeMotor = new Neo(INTAKE_MOTOR_ID);
    private final Neo angleMotor = new Neo(ANGLE_MOTOR_ID);

    private final DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(ENCODER_PORT);

    private final DigitalInput beamBreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger hasNoteTrigger = new Trigger(() -> !beamBreak.get()).debounce(0.2);

    private final PIDController anglePIDcontroller = new PIDController(INTAKE_GAINS.kp, INTAKE_GAINS.ki, INTAKE_GAINS.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(INTAKE_GAINS.ks, INTAKE_GAINS.kg, INTAKE_GAINS.kv, INTAKE_GAINS.ka);

    public final Trigger atSetpointTrigger = new Trigger(anglePIDcontroller::atSetpoint).debounce(0.2);
    public final Trigger atShooterTrigger =
            new Trigger(()-> MathUtil.isNear(intakeAngle.SHOOTER.angle, getAngle(), INTAKE_TOLERANCE));

    public final Trigger intakingTrigger = new Trigger(()-> {
        if (getCurrentCommand()!= null && getCurrentCommand().equals("intakeCommand")) return true;
        return false;
    });

    private final LEDs leds = LEDs.getInstance();

    public Intake() {
        intakeMotor.setConversionFactors(INTAKE_MOTOR_CONVERSION_FACTOR);

        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(60);
        angleMotor.setInverted(true);
        angleMotor.setOpenLoopRampRate(0);

        angleMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false);
        angleMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false);

        intakeEncoder.setDistancePerRotation(360);
        intakeEncoder.setPositionOffset(INTAKE_ENCODER_OFFSET_POSITION);

        anglePIDcontroller.enableContinuousInput(0, 360);
        anglePIDcontroller.setTolerance(INTAKE_TOLERANCE);
//        setDefaultCommand(intakeIdleCommand());

    }

    @Log.NT (key = "intakeAngle")
    private double getAngle() {
        return MathUtil.inputModulus(180 - intakeEncoder.getDistance(), 0, 360);
    }

    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    private void setIntakeAngle(intakeAngle angle) {
        double pid = anglePIDcontroller.calculate(getAngle(), angle.angle);
        double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0) / 60.0;

        System.out.println(anglePIDcontroller.getPositionError());

        angleMotor.setVoltage(pid + ff);
    }

    private void stopMotors(){
        intakeMotor.stopMotor();
        angleMotor.stopMotor();
    }


    private Command setIntakeCommand(IntakeState intakeState) {
        return this.runEnd(()-> {
//            leds.applyPatternCommand(BLINKING, ORANGE.color);

            setIntakeAngle(intakeState.angle);

            if (!intakeState.waitForAngle) setIntakeSpeed(intakeState.intakeDC);
            else if (atSetpointTrigger.getAsBoolean()) setIntakeSpeed(intakeState.intakeDC);
            else {
                setIntakeSpeed(0);
                System.out.println("not at setpoint");
            }

        }, this::stopMotors);
    }

    public Command intakeFromAngleCommand(intakeAngle angle) {
        return setIntakeCommand(new IntakeState(0.5, angle, true)).until(hasNoteTrigger).withName("intakeCommand");
    }

    public Command intakeFromAngleCommand(intakeAngle angle, Command vibrateCommand) {
        return intakeFromAngleCommand(angle).andThen(vibrateCommand::schedule); // schedule it instead of composing it to free up the intake requirement immediately
    }

    public Command shootToAmpCommand() {
        return setIntakeCommand(new IntakeState(AMP_SHOOTER_SPEED, intakeAngle.AMP, true)).until(hasNoteTrigger.negate());
    }

    public Command transportToShooterCommand() {
        return setIntakeCommand(new IntakeState(-0.5, intakeAngle.SHOOTER, true)).until(hasNoteTrigger.negate());
    }

    public Command intakeIdleCommand() {
        return setIntakeCommand(new IntakeState(STALL_DC, intakeAngle.SHOOTER, false));
    }

    public Command manualCommand(DoubleSupplier angle, BooleanSupplier intake, BooleanSupplier outake) {
        return this.runEnd(
                () -> {
                    if (intake.getAsBoolean()) intakeMotor.set(0.5);
                    else if (outake.getAsBoolean()) intakeMotor.set(-0.35);
                    else intakeMotor.stopMotor();
                    angleMotor.set(angle.getAsDouble() / 3);
                },
                () -> {
                    intakeMotor.stopMotor();
                    angleMotor.stopMotor();
                }
        );
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

    @Override
    public void periodic() {
        angleMotor.setPosition(getAngle());
    }
}