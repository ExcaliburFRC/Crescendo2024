package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooterMotors = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo followerShooterMotor = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);
    
    private final Neo linearLeader = new Neo(LEADER_LINEAR_MOTOR_ID);
    private final Neo linearFollowerMotor = new Neo(FOLLOWER_LINEAR_MOTOR_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final Trigger noteTrigger = new Trigger(beamBreak::get);

    public ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    public Shooter(){
        followerShooterMotor.follow(shooterMotors, true);
        linearFollowerMotor.follow(linearLeader);
    }

    private final void setLinear(double speed){
        linearLeader.set(speed);
    }

    public Command StartLinearMotor(DoubleSupplier speed){
        return new RunCommand(() -> setLinear(speed.getAsDouble()));
    }

    public Command ManualShooterCommand() {
        return new RunCommand(()-> shooterMotors.set(shooterSpeed.getDouble(0)), this).until(noteTrigger);
    }
}
