package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Neo;
import frc.robot.util.Neo.Model;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase implements Logged {
    private final Neo upperShooter = new Neo(UPPER_SHOOTER_MOTOR_ID, Model.SparkFlex);
    private final Neo lowerShooter = new Neo(LOWER_SHOOTER_MOTOR_ID, Model.SparkFlex);

    private ShooterVelocity currentVelocity = new ShooterVelocity(0);

    private final Trigger intakeTrigger;

    public GenericEntry lowerSpeed = Shuffleboard.getTab("match").add("lower speed", SPEAKER_DC * 100)
            .withSize(2, 2).withPosition(14, 5).getEntry();
    public GenericEntry upperSpeed = Shuffleboard.getTab("match").add("upper speed", SPEAKER_DC * 100)
            .withSize(2, 2).withPosition(14, 3).getEntry();

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

    public ShooterVelocity getCurrentVelocity() {
        return this.currentVelocity;
    }

    public Command setShooterCommand(ShooterVelocity vel) {
        return new FunctionalCommand(
                () -> currentVelocity = vel,
                () -> {
                    vel.setVelocities(upperShooter.getVelocity(), lowerShooter.getVelocity());

                    upperShooter.setVoltage(vel.getUpperVoltage());
                    lowerShooter.setVoltage(vel.getLowerVoltage());
                },
                (__) -> {
                    stopMotors();
                    currentVelocity = new ShooterVelocity(0, 0);
                },
                intakeTrigger.negate().debounce(0.25),
                this);
    }

    public Command shootToAmpCommand() {
        return setShooterCommand(new ShooterVelocity(AMP_UPPER_SHOOTER_RPM, AMP_LOWER_SHOOTER_RPM));
    }

    public Command shootToSpeakerCommand() {
        return setShooterCommand(new ShooterVelocity(SPEAKER_UPPER_SHOOTER_RPM, SPEAKER_LOWER_SHOOTER_RPM));
    }

    public Command shootToSpeakerManualCommand() {
        return this.runEnd(
                () -> {
                    upperShooter.set(upperSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                    lowerShooter.set(lowerSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                },
                this::stopMotors).until(intakeTrigger.negate().debounce(1));
    }

    public Command forceShootCommand(){
        return new FunctionalCommand(
                () -> this.currentVelocity = new ShooterVelocity(0, 0),
                () -> {
                    upperShooter.set(upperSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                    lowerShooter.set(lowerSpeed.getDouble(SPEAKER_DC * 100) / 100.0);
                },
                (__) -> this.stopMotors(),
                ()-> false,
                this).withTimeout(2);
    }

    public Command prepShooterCommand() {
        return new RunCommand(() -> {
            upperShooter.set(SPEAKER_DC);
            lowerShooter.set(SPEAKER_DC);
        }, this);
    }

    ShooterVelocity prepVelocity;

    public Command prepFarShooter(DoubleSupplier distMeters) {
        return new FunctionalCommand(
                () -> prepVelocity = new ShooterVelocity(distMeters),
                () -> {
                    prepVelocity.setVelocities(upperShooter.getVelocity(), lowerShooter.getVelocity());

                    upperShooter.setVoltage(prepVelocity.upperVoltage);
                    lowerShooter.setVoltage(prepVelocity.lowerVoltage);
                },
                (__) -> {
                },
                () -> false,
                this);
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

    @Log.NT
    private double getUpperRPM() {
        return upperShooter.getVelocity();
    }

    @Log.NT
    private double getUpperSetpoint() {
        return currentVelocity.upperSetpoint;
    }

    @Log.NT
    private double getLowerSetpoint() {
        return currentVelocity.lowerSetpoint;
    }

    @Log.NT
    private double getLowerRPM() {
        return lowerShooter.getVelocity();
    }
}