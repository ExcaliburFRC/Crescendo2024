package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import java.util.function.DoubleSupplier;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);

    private final Neo linearFollower = new Neo(FOLLOWER_LINEAR_MOTOR_ID);
    private final Neo linear = new Neo(LEADER_LINEAR_MOTOR_ID);
    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final Trigger noteTrigger = new Trigger(beamBreak::get);

    public ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);
    private final PIDController linearPID = new PIDController(LINEAR_PID.kp, LINEAR_PID.ki, LINEAR_PID.kd);
    private final InterpolatingDoubleTreeMap interpolate = new InterpolatingDoubleTreeMap();


    public Shooter(){
        shooterFollower.follow(shooter, true);
        Neo linearFollower = new Neo(FOLLOWER_LINEAR_MOTOR_ID);
        linearFollower.follow(linear);
        interpolate.put(MIN_SHOOTING_DISTANCE, MIN_SHOOTING_RPM);
        interpolate.put(MAX_SHOOTING_DISTANCE, MAX_SHOOTING_RPM);
    }
    private double setShooterPID(double setpoint) {
        double pid = shooterPID.calculate(setpoint, shooter.getVelocity());
        double ff = shooterFF.calculate(setpoint, 0);
        double output = pid + ff;
        shooter.set(output);
        return pid;
    }
    private void setLinearPID(double setpoint){
        double pid = linearPID.calculate(setpoint, linear.getVelocity());
        linear.set(pid);
    }

    public Command AMPShooter(){
        return this.runEnd(
                ()->{
                    setLinearPID(LINEAR_SETPOINT);
                    setShooterPID(SHOOTER_AMP_SPEED);
                },
                shooter::stopMotor)
                .andThen(
                new InstantCommand(()->setLinearPID(0)).until(noteTrigger.negate()));
    }

    public Command SpeakerShootWithControlCommand(){
        return this.runEnd(
                ()-> setShooterPID(SETPOINT_SHOOT_SPEAKER),
                shooter::stopMotor).until(noteTrigger.negate());
    }



    public Command StartLinearMotorCommand(DoubleSupplier speed){
        return new RunCommand(() -> setLinearPID(speed.getAsDouble()));
    }

    public Command ManualShooterCommand() {
        return runEnd(
                ()-> shooter.set(shooterSpeed.getDouble(0)),
                shooter::stopMotor).until(noteTrigger.negate());
    }
}

// Path: src/main/java/frc/robot/subsystems/Shooter.java
