package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Neo;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo LeaderMotor = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo FollowerMotor = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);
    private final Neo LinearMotorLeader = new Neo(LEADER_LINEAR_SHOOTER_MOTOR_ID);
    private final Neo LinearMotorFollower = new Neo(FOLLOWER_LINEAR_SHOOTER_MOTOR_ID);

    private final DigitalInput BeamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);

    public Shooter(){
        FollowerMotor.follow(LeaderMotor, true);
        LinearMotorFollower.follow(LinearMotorLeader);
    }

    private final void setLinear(double speed){
        LinearMotorFollower.set(speed);
    }

    public Command LinearStart(DoubleSupplier speed){
        return new RunCommand(() -> setLinear(speed.getAsDouble()));
    }



}
