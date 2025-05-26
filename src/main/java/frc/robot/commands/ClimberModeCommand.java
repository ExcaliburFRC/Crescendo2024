package frc.robot.commands;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.util.Color.Colors.*;
import static frc.robot.Constants.SwerveConstants.DRIVE_SPEED_PERCENTAGE;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;
import static frc.robot.subsystems.LEDs.LEDPattern.TRAIN;

public class ClimberModeCommand extends StartEndCommand {
    public ClimberModeCommand(EventLoop climberLoop, Swerve swerve) {
        super(() -> {
                    CommandScheduler.getInstance().setActiveButtonLoop(climberLoop);
                    swerve.maxSpeed.setDouble(20);
                    LEDs.getInstance().setPattern(SOLID, CYAN.color).schedule();
                },
                () -> {
                    final CommandScheduler cs = CommandScheduler.getInstance();
                    cs.setActiveButtonLoop(cs.getDefaultButtonLoop());
                    swerve.maxSpeed.setDouble(DRIVE_SPEED_PERCENTAGE);
                    LEDs.getInstance().setPattern(TRAIN, TEAM_BLUE.color, TEAM_GOLD.color).schedule();
                });

        super.setName("ClimberMode");
        super.ignoringDisable(true);
    }
}