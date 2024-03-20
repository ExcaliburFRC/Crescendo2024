package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Color;
import frc.lib.Neo;
import frc.lib.Neo.Model;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDPattern;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.lib.Color.Colors.RED;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase implements Logged {
    private final Neo upperShooter = new Neo(UPPER_SHOOTER_MOTOR_ID, Model.SparkFlex);
    private final Neo lowerShooter = new Neo(LOWER_SHOOTER_MOTOR_ID, Model.SparkFlex);

    private ShooterState currentState = new ShooterState(0);

    private final Trigger intakeTrigger;

    public GenericEntry lowerSpeed = Shuffleboard.getTab("match").add("lower speed", SPEAKER_DC * 100)
            .withSize(2, 2).withPosition(14, 5).getEntry();
    public GenericEntry upperSpeed = Shuffleboard.getTab("match").add("upper speed", SPEAKER_DC * 100)
            .withSize(2, 2).withPosition(14, 3).getEntry();

    private final LEDs leds = LEDs.getInstance();

    public Shooter(Trigger intakeTrigger) {
        upperShooter.setIdleMode(kCoast);
        upperShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        upperShooter.setInverted(false);
        upperShooter.setPosition(0);

        lowerShooter.setIdleMode(kCoast);
        lowerShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        lowerShooter.setInverted(false);
        lowerShooter.setPosition(0);

        this.intakeTrigger = intakeTrigger;
    }

    private void stopMotors() {
        upperShooter.stopMotor();
        lowerShooter.stopMotor();
    }

    public Command setShooterCommand(ShooterState state) {
        return new FunctionalCommand(
                () -> currentState = state,
                () -> {
                    state.setVelocities(upperShooter.getVelocity(), lowerShooter.getVelocity());

                    upperShooter.setVoltage(state.upperVoltage);
                    lowerShooter.setVoltage(state.lowerVoltage);
                },
                (__) -> {
                    stopMotors();
                    currentState = new ShooterState(0, 0);
                },
                intakeTrigger.negate().debounce(0.25),
                this);
    }

    public Command shootToAmpCommand() {
        return setShooterCommand(new ShooterState(AMP_UPPER_SHOOTER_RPM, AMP_LOWER_SHOOTER_RPM));
    }

    public Command shootToSpeakerCommand() {
        return setShooterCommand(new ShooterState(SPEAKER_UPPER_SHOOTER_RPM, SPEAKER_LOWER_SHOOTER_RPM));
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

    public Command shootToSpeakerManualCommand() {
        return new FunctionalCommand(
                () -> this.currentState = new ShooterState(0, 0),
                () -> {
                    upperShooter.set(upperSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                    lowerShooter.set(lowerSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                },
                (__) -> this.stopMotors(),
                intakeTrigger.negate().debounce(0.1),
                this);
//        ).alongWith(leds.setPattern(BLINKING, RED.color));
    }

    public Command prepShooterCommand() {
        return new RunCommand(() -> {
            upperShooter.set(SPEAKER_DC);
            lowerShooter.set(SPEAKER_DC);
        }, this);
    }

    ShooterState prepState;

    public Command prepFarShooter(DoubleSupplier distMeters) {
        return new FunctionalCommand(
                () -> prepState = new ShooterState(distMeters),
                () -> {
                    prepState.setVelocities(upperShooter.getVelocity(), lowerShooter.getVelocity());

                    upperShooter.setVoltage(prepState.upperVoltage);
                    lowerShooter.setVoltage(prepState.lowerVoltage);
                },
                (__) -> {
                },
                () -> false,
                this);
    }

    public Command stopShooterCommand() {
        return new InstantCommand(() -> {
            upperShooter.set(0);
            lowerShooter.set(0);
        });
    }

    public Command manualShooter(double upper, double lower) {
        return this.runEnd(
                () -> {
                    upperShooter.set(upper);
                    lowerShooter.set(lower);
                },
                this::stopMotors).until(intakeTrigger.negate().debounce(0.25));
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> upperShooter.setIdleMode(kCoast),
                () -> upperShooter.setIdleMode(kBrake))
                .ignoringDisable(true);
    }

    public ShooterState getCurrentState() {
        return this.currentState;
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
}