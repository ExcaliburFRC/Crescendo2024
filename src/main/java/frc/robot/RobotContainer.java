package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.AllianceUtils;
import monologue.Logged;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelSelf;
import static frc.lib.Color.Colors.*;
import static frc.robot.subsystems.LEDs.LEDPattern.*;
import static frc.robot.subsystems.intake.IntakeState.IntakeAngle.*;

public class RobotContainer implements Logged {
    // subsystems
    public final Shooter shooter = new Shooter();
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();

    // controllers
    private final CommandPS5Controller driver = new CommandPS5Controller(0);
//    private final CommandPS5Controller sysid = new CommandPS5Controller(1);
    private final XboxController driverVibration = new XboxController(5);

    public static ShuffleboardTab matchTab = Shuffleboard.getTab("match");
    public static ShuffleboardTab pitTab = Shuffleboard.getTab("pit");
    public static ShuffleboardTab robotData = Shuffleboard.getTab("RobotData");

    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static EventLoop climberLoop = new EventLoop();
    // swerve
    boolean robotRelativeDrive = false;
    final Pose2d emptyPose = new Pose2d();
    private boolean climberMode = false;

    private final Command climberModeCommand = new InstantCommand(()-> {
        final CommandScheduler cs = CommandScheduler.getInstance();
        climberMode = !climberMode;

        if (climberMode) {
            cs.setActiveButtonLoop(climberLoop);
            swerve.maxSpeed.setDouble(20);
        } else {
            cs.setActiveButtonLoop(cs.getDefaultButtonLoop());
            swerve.maxSpeed.setDouble(Constants.SwerveConstants.DRIVE_SPEED_PERCENTAGE);
        }
    }).withName("ClimberMode");

    Command intakeVibrate = vibrateControllerCommand(50, 0.25);

    public RobotContainer() {
        init();
        configureBindings();
    }

    // bindings
    private void configureBindings() {
        // swerve
        swerve.setDefaultCommand(
                swerve.driveSwerveCommand(
                        () -> applyDeadband(-driver.getLeftY(), 0.07),
                        () -> applyDeadband(-driver.getLeftX(), 0.07),
                        () -> applyDeadband(-driver.getRightX(), 0.07),
                        () -> !robotRelativeDrive,
                        driver::getL2Axis, // decelerator
                        driver.L1().and(()-> !climberMode),
                        () -> emptyPose)
        );

        driver.options().whileTrue(toggleMotorsIdleMode().alongWith(leds.setPattern(SOLID, WHITE.color)));
        driver.PS().onTrue(swerve.resetOdometryAngleCommand());

        // manual actions
        climber.setDefaultCommand(climber.manualCommand(driver.L1(climberLoop), driver.R1(climberLoop), driver.L2(climberLoop), driver.R2(climberLoop)));

        // intake
        driver.circle().toggleOnTrue(intake.intakeFromAngleCommand(HUMAN_PLAYER_BACKWARD, intakeVibrate));
        driver.cross().toggleOnTrue(intake.intakeFromAngleCommand(GROUND, intakeVibrate));

        driver.options().onTrue(intake.toggleIdleModeCommand());

        driver.touchpad().onTrue(intake.pumpNoteCommand());

        // shooter
        driver.square().and(intake.intakingTrigger.negate()).toggleOnTrue(scoreNoteCommand(shooter.shootToAmpCommand(), driver.R1(), true));
        driver.triangle().and(intake.intakingTrigger.negate()).toggleOnTrue(scoreNoteCommand(shooter.shootToSpeakerManualCommand(), driver.R1(), false));

        driver.povLeft().toggleOnTrue(intake.shootToAmpCommand().alongWith(
                new RunCommand(()-> swerve.driveRobotRelative(new ChassisSpeeds(-0.75, 0, 0))).withTimeout(0.1)));
        driver.povUp().onTrue(climberModeCommand);
    }

    // triangle - shoot to speaker
    // square - shoot to amp
    // circle - intake from ground
    // cross - intake from HP

    // methods
    private Command scoreNoteCommand(Command shooterCommand, Trigger release, boolean toAmp) {
        return shooterCommand.alongWith(
                new WaitUntilCommand(release).andThen(intake.transportToShooterCommand(() -> toAmp))
        );
    }

    public Command matchPrepCommand() {
        return new InstantCommand(()-> {});
//        return swerve.straightenModulesCommand().andThen(climber.autoCloseCommand());
    }

    private Command systemTesterCommand() {
        return new SequentialCommandGroup(
                swerve.driveSwerveCommand(() -> 0, () -> 0, () -> 0.75, () -> false).withTimeout(5),
                swerve.straightenModulesCommand(),
                intake.intakeFromAngleCommand(HUMAN_PLAYER_BACKWARD, intakeVibrate),
                new WaitCommand(1),
                scoreNoteCommand(shooter.shootToAmpCommand(), new Trigger(() -> true), true)
//                new WaitCommand(1),
//                climber.manualCommand(()-> true, ()-> true, ()-> false, ()-> false).withTimeout(1.5),
//                new WaitCommand(1),
//                climber.manualCommand(()-> false, ()-> false, ()-> true, ()-> true).withTimeout(1.5),
//                climber.manualCommand(()-> false, ()-> false, ()-> false, ()-> false).withTimeout(0.5)
                );
    }

    private Command vibrateControllerCommand(int intensity, double seconds) {
        return Commands.runEnd(
                        () -> driverVibration.setRumble(kBothRumble, intensity / 100.0),
                        () -> driverVibration.setRumble(kBothRumble, 0))
                .withTimeout(seconds).ignoringDisable(true);
    }

    public Command dynamicIntakeCommand() {
        return new SequentialCommandGroup(
                // intake from both the intake and the shooter
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(intake.hasNoteTrigger.or(shooter.hasNoteTrigger)),
//                        intake.intakeFromAngleCommand(HUMAN_PLAYER_BACKWARD, intakeVibrate),
                        shooter.intakeFromShooterCommand()),

                new ConditionalCommand(
                        // if the note is in the shooter, transport it to the intake
                        new ParallelCommandGroup(
//                                intake.intakeFromAngleCommand(SHOOTER, intakeVibrate),
                                new WaitCommand(0.75).andThen(shooter.transportToIntakeCommand()).until(intake.hasNoteTrigger)
                        ),
                        // if it's in the intake, do nothing
                        Commands.none(),
                        shooter.hasNoteTrigger),

                // pump the note (just to make sure)
                intake.pumpNoteCommand()
        );
    }

    public Command toggleMotorsIdleMode() {
        return new ParallelCommandGroup(
                swerve.toggleIdleModeCommand(),
                shooter.toggleIdleModeCommand(),
                intake.toggleIdleModeCommand(),
                climber.toggleIdleModeCommand()
        );
    }

    private void init() {
        NamedCommands.registerCommand("shootToSpeakerCommand", scoreNoteCommand(shooter.shootToSpeakerManualCommand(), new Trigger(() -> true), false));
        NamedCommands.registerCommand("prepShooterCommand", shooter.prepShooterCommand());
        NamedCommands.registerCommand("shootToAmpCommand", scoreNoteCommand(shooter.shootToAmpCommand(), shooter.shooterReadyTrigger, false));

        NamedCommands.registerCommand("intakeFromGround", intake.halfIntakeFromGround());
        NamedCommands.registerCommand("closeIntake", intake.closeIntakeCommand());
        NamedCommands.registerCommand("pumpNote", intake.pumpNoteCommand());

        pitTab.add("Match prep", matchPrepCommand().withName("MatchPrep")).withSize(2, 2);
        pitTab.add("System tester", systemTesterCommand().withName("SystemTest")).withSize(2, 2);

        matchTab.add("pumpNote", intake.pumpNoteCommand().withName("PumpNote")).withPosition(15, 1).withSize(4, 4);
        matchTab.addBoolean("intakeBeambreak", ()-> !intake.intakeBeambreak.get()).withPosition(19, 1).withSize(4, 4);
        matchTab.addBoolean("shooterWorks", shooter.shooterSpins).withPosition(19, 5).withSize(4, 4);
//        matchTab.addCamera("intakeCam", "", "").withPosition(0, 1).withSize(13, 10);

        matchTab.add("climberMode", climberModeCommand).withPosition(15, 5).withSize(4, 4);

        autoChooser.setDefaultOption("none", new InstantCommand(()-> {}));
        autoChooser.addOption("123", swerve.runAuto("123"));
        autoChooser.addOption("321", swerve.runAuto("321"));
        autoChooser.addOption("14", swerve.runAuto("14"));
        autoChooser.addOption("shoot", swerve.runAuto("shoot"));
        autoChooser.addOption("leaveFromBottom", swerve.runAuto("shootAndLeave"));

        Shuffleboard.getTab("Auto").add(autoChooser);
    }

    public Command getAutonomousCommand() {
        // Pose 1
//        Pose2d startingPose = new Pose2d(0.94, 6.48, Rotation2d.fromDegrees(60));
        // Pose 2
//        Pose2d startingPose = new Pose2d(1.3, 5.57, new Rotation2d());
        // Pose 3
//        Pose2d startingPose = AllianceUtils.mirrorAlliance(new Pose2d(0.74, 4.48, Rotation2d.fromDegrees(-60)));

        return new ProxyCommand(()-> autoChooser.getSelected());
    }
}