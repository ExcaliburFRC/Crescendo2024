package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final Neo intakeMotor = new Neo(INTAKE_MOTOR_ID);
    private final Neo angleMotor = new Neo(ANGLE_MOTOR_ID);

    private final DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(ENCODER_PORT);

    private final DigitalInput beamBreak = new DigitalInput(BEAMBREAK_PORT);
    public final Trigger hasNoteTrigger = new Trigger(beamBreak::get).debounce(0.2);

    private final PIDController anglePIDcontroller = new PIDController(PID_GAINS.kp, PID_GAINS.ki, PID_GAINS.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(FF_ANGLE_GAINS.ks, FF_ANGLE_GAINS.kg, FF_ANGLE_GAINS.kv);

    public final Trigger isAtShooterTrigger = new Trigger(() -> Math.abs(getAngle() - INTAKE_ANGLE.SHOOTER.angle) < SHOOTER_ANGLE_THRESHOLD).debounce(0.2);

    public Intake() {
        intakeMotor.setConversionFactors(INTAKE_MOTOR_POSITION_CONVERSION_FACTOR, INTAKE_MOTOR_VELOCITY_CONVERSION_FACTOR);

        intakeEncoder.setDistancePerRotation(360);
        intakeEncoder.setPositionOffset(INTAKE_ENCODER_OFFSET_POSITION);

        setDefaultCommand(intakeIdleCommand());
    }

    private double getAngle() {
        return intakeEncoder.getDistance();
    }

    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    private void setIntakeAngle(INTAKE_ANGLE angle) {
        double pid = anglePIDcontroller.calculate(getAngle(), angle.angle);
        double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0);

        angleMotor.setVoltage(pid + ff);
    }

    private void stopMotors(){
        intakeMotor.stopMotor();
        angleMotor.stopMotor();
    }

    private void stallIntakeMotor() {
        if (hasNoteTrigger.getAsBoolean()) intakeMotor.set(STALL_DC);
        else intakeMotor.stopMotor();
    }

    private Command setIntakeCommand(double speed, INTAKE_ANGLE angle) {
        return this.runEnd(()-> {
            setIntakeAngle(angle);
            setIntakeSpeed(speed);
        }, this::stopMotors);
    }

    public Command intakeFromAngleCommand(INTAKE_ANGLE angle) {
        return setIntakeCommand(0.5, angle).until(hasNoteTrigger).withName("intakeCommand");
    }

    public Command intakeFromAngleCommand(INTAKE_ANGLE angle, Command vibrateCommand) {
        return intakeFromAngleCommand(angle).andThen(vibrateCommand::schedule); // schedule it instead of composing it to free up the intake requirement immediately
    }

    public Command shootToAmpCommand() {
        return setIntakeCommand(AMP_SHOOTER_SPEED, INTAKE_ANGLE.AMP).until(hasNoteTrigger.negate());
    }

    public Command transportToShooterCommand() {
        return new SequentialCommandGroup(
                setIntakeCommand(0, INTAKE_ANGLE.SHOOTER).until(isAtShooterTrigger),
                setIntakeCommand(-0.5, INTAKE_ANGLE.SHOOTER).until(hasNoteTrigger.negate()));
    }

    public Command intakeIdleCommand() {
        return this.run(
                ()-> {
                    setIntakeAngle(INTAKE_ANGLE.SHOOTER);
                    stallIntakeMotor();
                });
    }

    public Command manualCommand(DoubleSupplier speed, DoubleSupplier angle) {
        return this.runEnd(
                () -> {
                    intakeMotor.set(speed.getAsDouble());
                    angleMotor.set(angle.getAsDouble());
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

    @Override
    public String getName() {
        return "Intake";
    }
}