package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import frc.robot.subsystems.Shooter.ShooterState.LinearState;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);
    private final Neo linearFollower = new Neo(FOLLOWER_LINEAR_MOTOR_ID);
    private final Neo linear = new Neo(LEADER_LINEAR_MOTOR_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final Trigger noteTrigger = new Trigger(beamBreak::get);

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);

    private final PIDController linearPID = new PIDController(LINEAR_PID.kp, LINEAR_PID.ki, LINEAR_PID.kd);

    private Trigger shooterAtSetpoint = new Trigger(shooterPID::atSetpoint);
    private Trigger linearAtSetpoint = new Trigger(linearPID::atSetpoint);

    public Trigger shooterReady = shooterAtSetpoint.and(linearAtSetpoint);

    public Shooter() {
        shooterFollower.follow(shooter, true);
        linearFollower.follow(linear);

        shooterPID.setTolerance(SHOOTER_PID_TOLERANCE);
        linearPID.setTolerance(LINEAR_PID_TOLERANCE);
    }

    private void setShooterPID(double setpoint) {
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
                    setShooterPID(state.RPM);
                    setLinearSetpoint(state.linearState);
                }, shooter::stopMotor);
    }

    public Command AMPShooterCommand() {
        return setShooterState(new ShooterState(AMP_RPM, true));
    }

    public Command speakerShootFromWooferCommand() {
        return setShooterState(new ShooterState(SPEAKER_RPM, false));
    }

    public Command shootFromDistanceCommand(double distance) {
        return setShooterState(new ShooterState(distance));
    }
}