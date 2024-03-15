package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.lib.Neo.Model;
import frc.robot.Constants.FieldConstants.FieldLocations;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.lib.Color.Colors.GREEN;
import static frc.lib.Color.Colors.RED;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class Shooter extends SubsystemBase implements Logged {
    private final Neo upperShooter = new Neo(UPPER_SHOOTER_MOTOR_ID, Model.SparkFlex);
    private final Neo lowerShooter = new Neo(LOWER_SHOOTER_MOTOR_ID, Model.SparkFlex);

    private ShooterState currentState = new ShooterState(0);

    public GenericEntry lowerSpeed = Shuffleboard.getTab("match").add("lower speed", SPEAKER_DC * 100)
            .withSize(2, 2).withPosition(14, 5).getEntry();
    public GenericEntry upperSpeed = Shuffleboard.getTab("match").add("upper speed", SPEAKER_DC * 100)
            .withSize(2, 2).withPosition(14, 3).getEntry();

    @Log.NT
    private final DigitalInput shooterBeambreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);

    public final BooleanEvent noteShotTrigger = new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> !shooterBeambreak.get()).falling();
    public final Trigger hasNoteTrigger = new Trigger(() -> !shooterBeambreak.get()).debounce(0.1);
    public final Trigger shooterSpins = new Trigger(() -> upperShooter.getVelocity() > 50).debounce(0.05);

    private final LEDs leds = LEDs.getInstance();

    @Log.NT
    public Trigger shooterReadyTrigger = currentState.stateReady;

    public Shooter() {
        upperShooter.setIdleMode(kCoast);
        upperShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        upperShooter.setInverted(false);
        upperShooter.setPosition(0);

        lowerShooter.setIdleMode(kCoast);
        lowerShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        lowerShooter.setInverted(false);
        lowerShooter.setPosition(0);

        RobotContainer.robotData.addBoolean("shooter beambreak", hasNoteTrigger);
    }

    private void stopMotors() {
        upperShooter.stopMotor();
        lowerShooter.stopMotor();
    }

    public ShooterState getCurrentState() {
        return this.currentState;
    }

    public Command setShootercommand(ShooterState state, Trigger intakeTrigger) {
        return new FunctionalCommand(
                () -> currentState = state,
                () -> {
                    state.setVelocities(upperShooter.getVelocity(), lowerShooter.getVelocity());

                    upperShooter.setVoltage(state.upperVoltage);
                    lowerShooter.setVoltage(state.lowerVoltage);
                },
                (__) -> {
                    stopMotors();
                    currentState = new ShooterState(0);
                },
                intakeTrigger.negate().debounce(1), this);
    }

    public Command shootToAmpCommand(Trigger intakeTrigger) {
        return setShootercommand(new ShooterState(AMP_UPPER_SHOOTER_RPM, AMP_LOWER_SHOOTER_RPM), intakeTrigger);
    }

//    public Command shootFromWooferCommand() {
//        return setShootercommand(new ShooterState(WOOFER_RPM), );
//    }

    public Command prepShooterCommand() {
        return new RunCommand(() -> {
            upperShooter.set(SPEAKER_DC);
            lowerShooter.set(SPEAKER_DC);
        }, this);
    }

    public Command shootToSpeakerManualCommand(Trigger intakeTrigger) {
        return new FunctionalCommand(
                () -> this.currentState = new ShooterState(WOOFER_RPM),
                () -> {
                    upperShooter.set(upperSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                    lowerShooter.set(lowerSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                },
                (__) -> this.stopMotors(),
                intakeTrigger.negate().debounce(0.1),
                this);
//        ).alongWith(leds.setPattern(BLINKING, RED.color));
    }

    public Command shootToAmpManualCommand(Trigger intakeTrigger) {
        return this.runEnd(
                () -> {
//                    upperShooter.setVoltage(4.24);
//                    lowerShooter.setVoltage(6.25);

                    upperShooter.set(upperSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                    lowerShooter.set(lowerSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                },
                this::stopMotors).until(intakeTrigger.negate().debounce(1));
    }

    public Command farShooterCommand(DoubleSupplier distMeters, Trigger intakeTrigger) {
        return this.runEnd(
                () -> {
                    ShooterState st = new ShooterState(distMeters.getAsDouble());

                    System.out.println("dist: " + distMeters.getAsDouble());
                    System.out.println("upper: " + st.upperDC);
                    System.out.println("lower: " + st.lowerDC);

                    upperShooter.set(st.upperDC);
                    lowerShooter.set(st.lowerDC);
                },
                this::stopMotors).until(intakeTrigger.negate().debounce(0.25));
    }

public Command manualShooter(double upper, double lower, Trigger intakeTrigger) {
    return this.runEnd(
            () -> {
                upperShooter.set(upper);
                lowerShooter.set(lower);
            },
            () -> {
            }).until(intakeTrigger.negate().debounce(1));
}

public Command stopShooterCommand() {
    return new InstantCommand(() -> {
        upperShooter.set(0);
        lowerShooter.set(0);
    });
}

public Command toggleIdleModeCommand() {
    return new StartEndCommand(
            () -> upperShooter.setIdleMode(kCoast),
            () -> upperShooter.setIdleMode(kBrake))
            .ignoringDisable(true);
}

@Log.NT
private boolean noteShotTrigger() {
    return noteShotTrigger.getAsBoolean();
}

@Log.NT
private double getUpperRPM() {
    return upperShooter.getVelocity();
}

@Log.NT
private double getUpperSetpoint() {
    return currentState.upperRPMsetpoint;
}

@Log.NT
private double getLowerSetpoint() {
    return currentState.lowerRPMsetpoint;
}

@Log.NT
private double getLowerRPM() {
    return lowerShooter.getVelocity();
}


// sysid stuff
private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
private final MutableMeasure<Angle> degrees = mutable(Rotations.of(0));

private final SysIdRoutine shooterSysid = new SysIdRoutine(
        sysidConfig,
        new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> upperShooter.setVoltage(volts.in(Volts)),
                log -> log.motor("shooterMotor")
                        .voltage(appliedVoltage.mut_replace(
                                upperShooter.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(degrees.mut_replace(upperShooter.getPosition(), Rotations))
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