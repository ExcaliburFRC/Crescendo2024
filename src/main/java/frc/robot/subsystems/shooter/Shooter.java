package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterState.LinearState;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(SHOOTER_LEADER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(SHOOTER_FOLLOWER_MOTOR_ID);

    private final Neo linear = new Neo(LINEAR_LEADER_MOTOR_ID);
    private final Neo linearFollower = new Neo(LINEAR_FOLLOWER_MOTOR_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final BooleanEvent noteShotTrigger =
            new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), beamBreak::get).falling().debounce(0.2);

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

        shooter.setIdleMode(kCoast);
        shooterFollower.setIdleMode(kCoast);

        shooterPID.setTolerance(SHOOTER_PID_TOLERANCE);
        linearPID.setTolerance(LINEAR_PID_TOLERANCE);

        shooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        linear.setSmartCurrentLimit(LINEAR_CURRENT_LIMIT);
    }

    private void setShooterRPM(double setpoint) {
        double pid = shooterPID.calculate(shooter.getVelocity(), setpoint);
        double ff = shooterFF.calculate(setpoint, 0);

        shooter.set(pid + ff);
    }

    private void setLinearSetpoint(LinearState state) {
        linear.set(linearPID.calculate(state.length, linear.getVelocity()));
    }

    private Command setShooterState(ShooterState state) {
        return this.runEnd(
                () -> {
                    setShooterRPM(state.RPM);
                    setLinearSetpoint(state.linearState);
                }, shooter::stopMotor).until(noteShotTrigger);
    }

    public Command closeLinearCommand() {
        return this.runEnd(() -> setLinearSetpoint(LinearState.CLOSE), linear::stopMotor).until(linearAtSetpoint);
    }

    public Command shootToAmpCommand() {
        return setShooterState(new ShooterState(AMP_RPM)).andThen(closeLinearCommand());
    }

    public Command shootFromWooferCommand() {
        return setShooterState(new ShooterState(WOOFER_RPM));
    }

    public Command shootFromDistanceCommand(DoubleSupplier distance) {
        return this.runEnd(()-> setShooterRPM(new ShooterState(distance.getAsDouble()).RPM), shooter::stopMotor);
    }

    public Command prepShooterCommand(Trigger isAtSpeakerRadius, Intake intake) {
        return new ConditionalCommand(
                prepShooterCommand(),
                Commands.runOnce(shooter::stopMotor, this),
                isAtSpeakerRadius.and(intake.isAtShooterTrigger).and(intake.hasNoteTrigger))
                .repeatedly();
    }

    public Command prepShooterCommand() {
        return this.runOnce(()-> shooter.set(SPEAKER_PREP_DC));
    }

    public Command manualShooterCommand(DoubleSupplier speed) {
        return new RunCommand(() -> {
            linear.set(speed.getAsDouble());
            shooter.set(shooterSpeed.getDouble(0));
        }, this);
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> {
                    linear.setIdleMode(kCoast);
                    linearFollower.setIdleMode(kCoast);
                    shooter.setIdleMode(kCoast);
                    shooterFollower.setIdleMode(kCoast);
                },
                () -> {
                    linear.setIdleMode(kBrake);
                    linearFollower.setIdleMode(kBrake);
                    shooter.setIdleMode(kBrake);
                    shooterFollower.setIdleMode(kBrake);
                }
        );
    }
}