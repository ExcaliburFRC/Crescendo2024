package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import frc.robot.subsystems.shooter.ShooterState.LinearState;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(SHOOTER_LEADER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(SHOOTER_FOLLOWER_MOTOR_ID);

    private final Neo linear = new Neo(LINEAR_LEADER_MOTOR_ID);
    private final Neo linearFollower = new Neo(LINEAR_FOLLOWER_MOTOR_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final Trigger noteTrigger = new Trigger(beamBreak::get);

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);

    private final PIDController linearPID = new PIDController(LINEAR_PID.kp, LINEAR_PID.ki, LINEAR_PID.kd);

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private Trigger shooterAtSetpoint = new Trigger(shooterPID::atSetpoint);
    private Trigger linearAtSetpoint = new Trigger(linearPID::atSetpoint);

    public Trigger shooterReady = shooterAtSetpoint.and(linearAtSetpoint);

    public Shooter() {
        shooterFollower.follow(shooter, true);
        linearFollower.follow(linear);

        shooterPID.setTolerance(SHOOTER_PID_TOLERANCE);
        linearPID.setTolerance(LINEAR_PID_TOLERANCE);

        shooter.setVoltage(MAX_VOLTAGE_SHOOTER);
        linear.setVoltage(MAX_VOLTAGE_LINEAR);

    }

    private void setShooterRPM(double setpoint) {
        double pid = shooterPID.calculate(shooter.getVelocity(), setpoint);
        double ff = shooterFF.calculate(setpoint, 0);

        shooter.set(pid + ff);
    }

    private void setLinearSetpoint(LinearState state) {
        linear.set(linearPID.calculate(state.length, linear.getVelocity()));
    }

    public Command setShooterState(ShooterState state) {
        return this.runEnd(
                ()-> {
                    setShooterRPM(state.RPM);
                    setLinearSetpoint(state.linearState);
                }, shooter::stopMotor);
    }

    public Command closeLinearCommand() {
        return this.runEnd(() -> setLinearSetpoint(LinearState.CLOSE), linear::stopMotor).until(linearAtSetpoint);
    }
    public Command shootToAmpCommand() {
        return setShooterState(new ShooterState(AMP_RPM, true)).andThen(closeLinearCommand());
    }

    public Command shootFromWooferCommand() {
        return setShooterState(new ShooterState(WOOFER_RPM, false));
    }

    public Command shootFromDistanceCommand(double distance) {
        return setShooterState(new ShooterState(distance));
    }

    public Command prepShooterCommand(BooleanSupplier amp, BooleanSupplier speaker, BooleanSupplier hasNote) {
        return new RunCommand(() -> {
            if (hasNote.getAsBoolean()) {
                if (amp.getAsBoolean()) {
                    shooter.set(AMP_PREP_DC);
                } else if (speaker.getAsBoolean()) {
                    shooter.set(SPEAKER_PREP_DC);
                } else {
                    shooter.stopMotor();
                }
            } else shooter.stopMotor();
        }, this);
    };

    public Command manualShooterCommand(DoubleSupplier speed) {
        return new RunCommand(() -> {
            linear.set(speed.getAsDouble());
            shooter.set(shooterSpeed.getDouble(0));
        }, this);
    }
}