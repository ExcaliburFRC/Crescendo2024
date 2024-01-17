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

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.SwerveConstants.DRIVE_SPEED_PERCENTAGE;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo followerMotor = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);
    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final Trigger beamBreakTrigger = new Trigger(()-> beamBreak.get());

    private ShuffleboardTab shooterTab;
    private GenericEntry shooterSpeed = Shuffleboard.getTab("Shooter").add("shootSpeedPercent", 0).getEntry();


    public Shooter(){
        followerMotor.follow(shooter, true);
    }

    public Command ManualShooterCommand() {
        return new RunCommand(()-> shooter.set(shooterSpeed.getDouble(DRIVE_SPEED_PERCENTAGE)), this).until(beamBreakTrigger);
    }

}
