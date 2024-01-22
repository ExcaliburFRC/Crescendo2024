package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.Constants.IntakeConstants.INTAKE_ANGLE.GROUND;
import static frc.robot.Constants.IntakeConstants.INTAKE_ANGLE.HUMAN_PLAYER;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class RobotContainer {
    // subsystems
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    // controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(0);
    private final CommandPS4Controller operator = new CommandPS4Controller(1);

    public final SendableChooser<Command> shouldDriveToCenterLineChooser = new SendableChooser<>();
    public boolean shooterWorks = true;

    public ShuffleboardTab matchTab = Shuffleboard.getTab("Match settings");

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

        operator.triangle().onTrue(shooter.shootToAmpCommand());

        operator.square().onTrue(intake.intakeFromAngleCommand(HUMAN_PLAYER));
        operator.cross().onTrue(intake.intakeFromAngleCommand(GROUND));

        operator.square().onTrue(new ConditionalCommand(
                        shooter.shootToAmpCommand(),
                        intake.shootToAmpCommand(),
                        ()-> shooterWorks)
        );
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

        matchTab.addBoolean("Shooter works", ()-> shooterWorks);
    }
    public Command ScoreNoteCommand(Command shooterCommand){
        return new ParallelCommandGroup(
                shooterCommand,
                new WaitUntilCommand(shooter.shooterReady).andThen(intake.transportToShooterCommand()));
    }

    public Command getAutonomousCommand(){
        return Commands.none();
    }
}
