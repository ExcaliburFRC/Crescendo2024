package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.intake.IntakeState.intakeAngle;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.lib.Color.Colors.ORANGE;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;

public class Intake extends SubsystemBase {
    private final Neo intakeMotor = new Neo(INTAKE_MOTOR_ID);
    private final Neo angleMotor = new Neo(ANGLE_MOTOR_ID);

    private final DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(ENCODER_PORT);

    private ShuffleboardTab intakeTab = Shuffleboard.getTab("IntakeTab");
    private final GenericEntry intakeSpeed = intakeTab.add("intakeSpeed", 0).getEntry();

    private intakeAngle setpoint = intakeAngle.SHOOTER;
    private final DigitalInput beamBreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger hasNoteTrigger = new Trigger(beamBreak::get).debounce(0.2);

    private final PIDController anglePIDcontroller = new PIDController(PID_GAINS.kp, PID_GAINS.ki, PID_GAINS.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(FF_ANGLE_GAINS.ks, FF_ANGLE_GAINS.kg, FF_ANGLE_GAINS.kv);

    public final Trigger atSetpointTrigger = new Trigger(() -> Math.abs(getAngle() - setpoint.angle) < ANGLE_THRESHOLD).debounce(0.2);
    public final Trigger atShooterTrigger = atSetpointTrigger.and(()-> setpoint == intakeAngle.SHOOTER);

    public final Trigger intakingTrigger = new Trigger(()-> {
        if (getCurrentCommand()!= null && getCurrentCommand().equals("intakeCommand")) return true;
        return false;
    });

    private final LEDs leds = LEDs.getInstance();

    public Intake() {
        intakeMotor.setConversionFactors(INTAKE_MOTOR_POSITION_CONVERSION_FACTOR, INTAKE_MOTOR_VELOCITY_CONVERSION_FACTOR);

        intakeEncoder.setDistancePerRotation(360);
        intakeEncoder.setPositionOffset(INTAKE_ENCODER_OFFSET_POSITION);

//        setDefaultCommand(intakeIdleCommand());
    }

    private double getAngle() {
        return intakeEncoder.getDistance();
    }

    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    private void setIntakeAngle(intakeAngle angle) {
        this.setpoint = angle;

        double pid = anglePIDcontroller.calculate(getAngle(), angle.angle);
        double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0);

        angleMotor.setVoltage(pid + ff);
    }

    private void stopMotors(){
        intakeMotor.stopMotor();
        angleMotor.stopMotor();
    }


    private Command setIntakeCommand(IntakeState intakeState) {
        return this.runEnd(()-> {
            leds.applyPatternCommand(BLINKING, ORANGE.color);

            setIntakeAngle(intakeState.angle);

            if (!intakeState.waitForAngle) setIntakeSpeed(intakeState.intakeDC);
            else if (atSetpointTrigger.getAsBoolean()) setIntakeSpeed(intakeState.intakeDC);
            else setIntakeSpeed(0);

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
                    if (intake.getAsBoolean()) intakeMotor.set(0.3);
                    else if (outake.getAsBoolean()) intakeMotor.set(-0.3);
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
                () -> angleMotor.setIdleMode(IdleMode.kBrake));
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
                                    angleMotor.get() * RobotController.getBatteryVoltage(), Volts))
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