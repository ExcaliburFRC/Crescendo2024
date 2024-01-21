package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Neo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);
    private final Neo linearFollower = new Neo(FOLLOWER_LINEAR_MOTOR_ID);
    private final Neo linear = new Neo(LEADER_LINEAR_MOTOR_ID);


    public ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);
    private final PIDController linearPID = new PIDController(LINEAR_PID.kp, LINEAR_PID.ki, LINEAR_PID.kd);

    public Shooter() {
        linearFollower.follow(linear);
    }

    private void setShooterRPM(double setpoint) {
        double pid = shooterPID.calculate(setpoint, shooter.getVelocity());
        double ff = shooterFF.calculate(setpoint, 0);
        double output = pid + ff;
        shooter.set(output);
    }

    private void setLinearPID(double setpoint) {
        double pid = linearPID.calculate(setpoint, linear.getVelocity());
        linear.set(pid);

    }

//Waiting for LinearState To Be Coded By Ori
//private void setLinearState(LinearState){pid(state.length)}

    public Command ManualShooterCommand(DoubleSupplier DutyCycle) {
        return new RunCommand(() -> {
            linear.set(DutyCycle.getAsDouble());
            shooter.set(shooterSpeed.getDouble(0));
        }, this);
    };

    public Command PrepShooterCommand(BooleanSupplier amp, BooleanSupplier speaker) {
        return new RunCommand(() -> {
            if (amp.getAsBoolean()) {
                shooter.set(PREP_SHOOTER_SPEED_FOR_AMP);
            } else if (speaker.getAsBoolean()) {
                shooter.set(PREP_SHOOTER_SPEED_FOR_SPEAKER);
            } else {
                setShooterRPM(0);
            }
        }, this);
    };

}

// Path: src/main/java/frc/robot/subsystems/Shooter.java

