package frc.robot.subsystems;

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

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.intakeConstants.*;

public class Intake extends SubsystemBase {
    private final Neo intakeMotor = new Neo(INTAKE_MOTOR_ID);
    private final Neo angleMotor = new Neo(ANGLE_MOTOR_ID);

    private final DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(INTAKE_ENCODER_ID);

    private final DigitalInput beamBreak = new DigitalInput(0);
    public final Trigger hasNoteTrigger = new Trigger(beamBreak::get);

    private final PIDController anglePIDcontroller = new PIDController(PID_GAINS.kp, PID_GAINS.ki, PID_GAINS.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(FF_ANGLE_GAINS.ks, FF_ANGLE_GAINS.kg, FF_ANGLE_GAINS.kv);

    public final Trigger isAtShooterTrigger = new Trigger(() -> Math.abs(getAngle() - INTAKE_ANGLE.SHOOTER.angle) < SHOOTER_ANGLE_THRESHOLD).debounce(0.2);

    public Intake() {
        intakeMotor.setConversionFactors(INTAKE_MOTOR_POSITION_CONVERSION_FACTOR, INTAKE_MOTOR_VELOCITY_CONVERSION_FACTOR);

        intakeEncoder.setDistancePerRotation(360);
        intakeEncoder.setPositionOffset(INTAKE_ENCODER_OFFSET_POSITION);

        setDefaultCommand(resetIntakeCommand());
    }

    private double getAngle() {
        return intakeEncoder.getDistance();
    }

    private Command setRollerSpeedCommand(double speed) {
        return new RunCommand(() -> intakeMotor.set(speed));
    }

    private Command ejectNoteCommand() {
        return setRollerSpeedCommand(-0.5);
    }

    public Command stallIntakeMotor() {
        return new RunCommand(() -> {
            if (hasNoteTrigger.getAsBoolean()) intakeMotor.set(STALL_DC);
            else intakeMotor.stopMotor();
        });
    }

    private Command setIntakeAngleCommand(INTAKE_ANGLE angle) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    double pid = anglePIDcontroller.calculate(getAngle(), angle.angle);
                    double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0);
                    double output = pid + (ff / 60.0);

                    angleMotor.setVoltage(output);
                },
                (__) -> angleMotor.stopMotor(),
                () -> false
        );
    }

    public Command intakeFromAngleCommand(INTAKE_ANGLE angle) {
        return new ParallelCommandGroup(
                setIntakeAngleCommand(angle),
                setRollerSpeedCommand(0.5),
                requirement())
                .until(hasNoteTrigger);
    }

    public Command shootToAmpCommand() {
        return new ParallelCommandGroup(
                setIntakeAngleCommand(INTAKE_ANGLE.AMP),
                setRollerSpeedCommand(AMP_SHOOTER_DC),
                requirement())
                .until(hasNoteTrigger.negate());
    }

    public Command transportToShooterCommand() {
        return new ConditionalCommand(
                ejectNoteCommand(),
                Commands.none(),
                isAtShooterTrigger).repeatedly().until(hasNoteTrigger.negate());
    }

    public Command resetIntakeCommand() {
        return new ParallelCommandGroup(
                setIntakeAngleCommand(INTAKE_ANGLE.SHOOTER),
                stallIntakeMotor(),
                requirement());
    }

    private Command requirement() {
        return this.run(() -> {
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return angleSysid.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return angleSysid.dynamic(direction);
    }

    @Override
    public String getName() {
        return "Intake";
    }
}