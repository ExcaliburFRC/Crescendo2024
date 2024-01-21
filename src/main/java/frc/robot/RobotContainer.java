package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.Constants.FieldConstants.FieldLocations.*;
import static frc.robot.Constants.FieldConstants.FieldLocations.HM_RIGHT;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class RobotContainer {
    // subsystems
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();
    private final Shooter shooter = new Shooter();

    // controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(0);
    private final CommandPS4Controller controllerOperator = new CommandPS4Controller(1);

    public final SendableChooser<Command> shouldDriveToCenterLineChooser = new SendableChooser<>();

    public RobotContainer(){
        configureBindings();
        initSendableChoosers();
    }

    // bindings
    private void configureBindings() {
        swerve.setDefaultCommand(
                swerve.driveSwerveCommand(
                        () -> applyDeadband(-controller.getLeftY(), 0.07),
                        () -> applyDeadband(-controller.getLeftX(), 0.07),
                        () -> applyDeadband(-controller.getRightX(), 0.07),
                        controller.L2().negate(),
                        controller::getR2Axis));

        controller.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.applyPatternCommand(SOLID, WHITE.color)));
        controller.PS().onTrue(swerve.setOdometryPositionCommand(new Pose2d(0, 0, new Rotation2d(0))));

        // teleop path's

        // speaker pathfinding
        controller.povLeft().whileTrue(swerve.pathFindToLocation(SPEAKER_TOP));
        controller.povUp().whileTrue(swerve.pathFindToLocation(SPEAKER_CENTER));
        controller.povRight().whileTrue(swerve.pathFindToLocation(SPEAKER_BOTTOM));

        // Human player pathfinding
        controller.square().whileTrue(swerve.pathFindToLocation(HM_LEFT));
        controller.triangle().whileTrue(swerve.pathFindToLocation(HM_CENTER));
        controller.circle().whileTrue(swerve.pathFindToLocation(HM_RIGHT));

        controllerOperator.triangle().onTrue(shooter.shootToAmpCommand());
    }

    // methods
    public Command toggleMotorsIdleMode() {
        return new ParallelCommandGroup(
                swerve.toggleIdleModeCommand()
                // add other subsystems here
        );
    }

    private void initSendableChoosers(){
        shouldDriveToCenterLineChooser.setDefaultOption("don't Drive", Commands.none());
        shouldDriveToCenterLineChooser.addOption("drive", Commands.idle()); // this is my commandddd!!
    }

    public Command getAutonomousCommand(){
        return Commands.none();
    }
}
