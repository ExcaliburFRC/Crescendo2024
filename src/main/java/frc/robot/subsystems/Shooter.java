package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Neo;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo LeaderMotor = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo FollowerMotor = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);
    private final DigitalInput BeamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);

    Shooter(){
        FollowerMotor.follow(LeaderMotor, true);
    }



}
