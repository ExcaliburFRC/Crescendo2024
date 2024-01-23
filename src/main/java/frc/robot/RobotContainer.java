package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.Constants.IntakeConstants.INTAKE_ANGLE.*;
import static frc.robot.Constants.ShooterConstants.SPEAKER_PREP_RADIUS;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;
import static frc.robot.Constants.FieldConstants.FieldLocations.*;

public class RobotContainer {
    // subsystems
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    // controllers
    private final CommandPS4Controller driver = new CommandPS4Controller(0);
    private final CommandPS4Controller operator = new CommandPS4Controller(1);
    private final CommandPS4Controller sysidController = new CommandPS4Controller(2);

    public final SendableChooser<Command> shouldDriveToCenterLineChooser = new SendableChooser<>();

    public boolean shooterWorks = true;
    public final Trigger isAtSpeakerRadius = new Trigger(()-> swerve.getDistanceFromPose(SPEAKER_CENTER.pose.get()) < SPEAKER_PREP_RADIUS);

    public ShuffleboardTab matchTab = Shuffleboard.getTab("Match settings");

    public RobotContainer(){
        configureBindings();
        initSendableChoosers();
    }

    // bindings
    private void configureBindings() {
        swerve.setDefaultCommand(
                swerve.driveSwerveCommand(
                        () -> applyDeadband(-driver.getLeftY(), 0.07),
                        () -> applyDeadband(-driver.getLeftX(), 0.07),
                        () -> applyDeadband(-driver.getRightX(), 0.07),
                        driver.L2().negate(),
                        driver::getR2Axis));

        driver.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.applyPatternCommand(SOLID, WHITE.color)));
        driver.PS().onTrue(swerve.resetOdometryAngleCommand());

        shooter.setDefaultCommand(shooter.prepShooterCommand(isAtSpeakerRadius, intake));

        // manual actions
        // if the shooter doesn't work, we shoot the note from the intake
        operator.circle().toggleOnTrue(new ConditionalCommand(
                scoreNoteCommand(shooter.shootToAmpCommand()),
                intake.shootToAmpCommand(),
                ()-> shooterWorks)
        );
        operator.triangle().toggleOnTrue(shooter.shootFromWooferCommand());
        operator.square().toggleOnTrue(intake.intakeFromAngleCommand(HUMAN_PLAYER));
        operator.cross().toggleOnTrue(intake.intakeFromAngleCommand(GROUND));

        // automated actions
        // auto drive to subwoofer and shoot
        driver.povUp().whileTrue(scoreNoteCommand(
                swerve.pathFindToLocation(SPEAKER_CENTER),
                shooter.shootFromWooferCommand()));

        // auto drive to amp and score
        driver.povDown().whileTrue(scoreNoteCommand(
                swerve.pathFindToLocation(AMPLIFIER),
                shooter.shootToAmpCommand()
        ));

        // auto aim to speaker and shoot with auto calculated RPM
        driver.povRight().toggleOnTrue(scoreNoteCommand(
                swerve.turnToLocationCommand(SPEAKER),
                shooter.shootFromDistanceCommand(()-> swerve.getDistanceFromPose(SPEAKER.pose.get()))));

        // sysid
        sysidController.circle().whileTrue(intake.sysidQuasistatic(kForward));
        sysidController.cross().whileTrue(intake.sysidQuasistatic(kReverse));
        sysidController.triangle().whileTrue(intake.sysidDynamic(kForward));
        sysidController.square().whileTrue(intake.sysidDynamic(kReverse));
    }

    // methods
    public Command matchPrepCommand() {
        return new SequentialCommandGroup(
                swerve.straightenModulesCommand(),
                shooter.closeLinearCommand(),
                intake.setIntakeAngleCommand(SHOOTER)
        );
    }

    public Command scoreNoteCommand(Command shooterCommand){
        return new ParallelCommandGroup(
                shooterCommand,
                new WaitUntilCommand(shooter.shooterReady).andThen(intake.transportToShooterCommand()));
    }

    public Command scoreNoteCommand(Command swerveCommand, Command shooterCommand){
        return new SequentialCommandGroup(
                swerveCommand.deadlineWith(shooter.prepShooterCommand()),
                scoreNoteCommand(shooterCommand)
        );
    }

    public Command toggleMotorsIdleMode() {
        return new ParallelCommandGroup(
                swerve.toggleIdleModeCommand(),
                shooter.toggleIdleModeCommand(),
                intake.toggleIdleModeCommand()
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