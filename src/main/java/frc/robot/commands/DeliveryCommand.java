package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.shooter.Shooter;

import static frc.robot.Constants.ShooterConstants.SPEAKER_DC;

public class DeliveryCommand extends StartEndCommand {
    public DeliveryCommand(Shooter shooter){
        super(
                ()-> {
                    shooter.upperSpeed.setDouble(80);
                    shooter.lowerSpeed.setDouble(100);
                },
                ()-> {
                    shooter.upperSpeed.setDouble(SPEAKER_DC * 100);
                    shooter.lowerSpeed.setDouble(SPEAKER_DC * 100);
                }
        );

        super.setName("Delivery");
        super.ignoringDisable(true);
    }
}
