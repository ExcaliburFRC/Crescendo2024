package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.robot.Constants.FieldConstants.FieldLocations;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ContinuouslyConditionalCommand;

import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RPM;
import static frc.lib.Color.Colors.GREEN;
import static frc.robot.Constants.FieldConstants.FieldLocations.AMPLIFIER;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;
import static frc.robot.subsystems.shooter.ShooterState.*;

public class Shooter extends SubsystemBase {
    private final Neo upperShooter = new Neo(UPPER_SHOOTER_MOTOR_ID);
    private final Neo lowerShooter = new Neo(LOWER_SHOOTER_MOTOR_ID);
    private final Neo shooterAngleMotor = new Neo(ANGLE_SHOOTER_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final BooleanEvent noteShotTrigger =
            new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), beamBreak::get).falling().debounce(0.2);
    private final DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(SHOOTER_ENCODER_CHANNEL);

    private final ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();
    private final GenericEntry shooterAngle = shooterTab.add("shooterAngle", 0/*TODO: find default value*/).getEntry();

    private static final InterpolatingDoubleTreeMap metersToRPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap metersToShooterAngle = new InterpolatingDoubleTreeMap();

    private final LEDs leds = LEDs.getInstance();

    public Trigger shooterReadyTrigger = new Trigger(ShooterState::atSetpoint)
            .onTrue(leds.applyPatternCommand(SOLID, GREEN.color))
            .onFalse(leds.restoreLEDs());

    public Shooter() {
        upperShooter.setIdleMode(kCoast);
        upperShooter.setConversionFactors(ROT_TO_DEGREES, RPM_TO_DEG_PER_SEC);
        upperShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

        lowerShooter.setIdleMode(kCoast);
        lowerShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

        shooterAngleMotor.setIdleMode(kBrake);

        metersToRPM.put(0.0, 0.0);
        metersToShooterAngle.put(0.0, 0.0);
    }

    private void stopShootMotors() {
        upperShooter.stopMotor();
        lowerShooter.stopMotor();
    }

    private Command setShooterStateCommand(ShooterState state) {
        return this.runEnd(
                () -> {
                    state.setVelocities(upperShooter.getVelocity(), lowerShooter.getVelocity());
                    state.setAngleVelocity(shooterAngleMotor.getVelocity());

                    upperShooter.set(state.upperVelocity);
                    lowerShooter.set(state.lowerVelocity);
                    shooterAngleMotor.set(state.angleVelocity);
                }, this::stopShootMotors).until(noteShotTrigger);
    }

    public Command shootToAmpCommand() {
        return setShooterStateCommand(new ShooterState(AMP_UPPER_SHOOTER_RPM, AMP_LOWER_SHOOTER_RPM, AMP_ANGLE));
    }

    public Command shootFromWooferCommand() {
        return setShooterStateCommand(new ShooterState(WOOFER_RPM, WOOFER_ANGLE));
    }

    public Command shootToLocationCommand(FieldLocations locations) {
        return new ConditionalCommand(
                shootToAmpCommand(),
                shootFromWooferCommand(),
                () -> locations.equals(AMPLIFIER));
    }

    public Command shootFromDistanceCommand(DoubleSupplier distance) {
        return setShooterStateCommand(new ShooterState(metersToRPM.get(distance.getAsDouble()), metersToShooterAngle.get(distance.getAsDouble())));
    }

    public Command intakeFromShooterCommand() {
        return this.runEnd(() -> setShooterStateCommand(new ShooterState(INTAKE_FROM_SHOOTER_RPM, INTAKE_FROM_SHOOTER_ANGLE)),
                this::stopShootMotors).until(beamBreak::get);
    }

    public Command prepShooterCommand(Trigger isAtSpeakerRadius, Intake intake) {
        return new ContinuouslyConditionalCommand(
                prepShooterCommand().alongWith(leds.applyPatternCommand(BLINKING, GREEN.color)),
                new RunCommand(this::stopShootMotors, this),
                isAtSpeakerRadius.and(intake.atShooterTrigger).and(intake.hasNoteTrigger)
        );
    }

    public Command prepShooterCommand() {
        return new RunCommand(() -> upperShooter.set(SPEAKER_PREP_DC), this);
    }

    public Command manualShooterCommand() {
        return new RunCommand(() -> {
            upperShooter.set(shooterSpeed.getDouble(0));
            shooterAngleMotor.set(anglePIDcontroller.calculate(shooterEncoder.get(), shooterAngle.getDouble(0)) +
                    angleFFcontroller.calculate(Math.toRadians(shooterAngle.getDouble(0)), 0));
        }, this);
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> {
                    upperShooter.setIdleMode(kCoast);
                    shooterAngleMotor.setIdleMode(kCoast);
                },
                () -> {
                    upperShooter.setIdleMode(kBrake);
                    shooterAngleMotor.setIdleMode(kBrake);
                }
        );
    }

    // sysid stuff
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(DegreesPerSecond.of(0));

    private final MutableMeasure<Angle> degrees = mutable(Degrees.of(0));

    private final SysIdRoutine shooterSysid = new SysIdRoutine(
            sysidConfig,
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> upperShooter.setVoltage(volts.in(Volts)),
                    log -> log.motor("shooterMotor")
                            .voltage(appliedVoltage.mut_replace(
                                    upperShooter.get() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(degrees.mut_replace(upperShooter.getPosition(), Degrees))
                            .angularVelocity(velocity.mut_replace(upperShooter.getVelocity(), RPM)),

                    this
            ));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterSysid.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterSysid.dynamic(direction);
    }
}