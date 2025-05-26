package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;

public class VibrateControllerCommand extends Command{
    private final XboxController controller;
    private final double seconds, intensity;
    private final Timer timer = new Timer();

    public VibrateControllerCommand(XboxController controller, double seconds, int intensity){
        this.controller = controller;
        this.seconds = seconds;
        this.intensity = intensity;
        super.ignoringDisable(true);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        controller.setRumble(kBothRumble, intensity / 100.0);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(kBothRumble, 0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }
}
