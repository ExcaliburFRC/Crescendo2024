package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.PDH;
import monologue.Logged;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static frc.lib.Color.Colors.*;
import static frc.robot.Constants.FieldConstants.FieldLocations.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.*;
import static frc.robot.subsystems.intake.IntakeState.IntakeAngle.*;

public class RobotContainer implements Logged {
    // subsystems
    public final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    public final Shooter shooter = new Shooter(intake.hasNoteTrigger);
    private final Climber climber = new Climber();
    private final LEDs leds = LEDs.getInstance();

    // controllers
    private final CommandPS5Controller driver = new CommandPS5Controller(0);
    private final CommandPS5Controller sysid = new CommandPS5Controller(1);
    private final XboxController driverVibration = new XboxController(5);

    public static ShuffleboardTab matchTab = Shuffleboard.getTab("match");
    public static ShuffleboardTab pitTab = Shuffleboard.getTab("pit");
    public static ShuffleboardTab robotData = Shuffleboard.getTab("RobotData");

    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static EventLoop climberLoop = new EventLoop();
    // swerve
    final Translation2d emptyPose = new Translation2d();

    private boolean climberMode = false;
    private boolean farShooter = false;

    Timer timer = new Timer();
    Trigger timerTrigger = new Trigger(() -> timer.get() > 0.25);

    private final Command climberModeCommand = new InstantCommand(() -> {
        final CommandScheduler cs = CommandScheduler.getInstance();
        climberMode = !climberMode;

        if (climberMode) {
            cs.setActiveButtonLoop(climberLoop);
            swerve.maxSpeed.setDouble(20);
            leds.setPattern(SOLID, CYAN.color).schedule();
        } else {
            cs.setActiveButtonLoop(cs.getDefaultButtonLoop());
            swerve.maxSpeed.setDouble(Constants.SwerveConstants.DRIVE_SPEED_PERCENTAGE);
            leds.setPattern(TRAIN, TEAM_BLUE.color, TEAM_GOLD.color).schedule();
        }
    }).withName("ClimberMode").ignoringDisable(true);


    private final Command shooterDistanceCommand = new InstantCommand(() -> {
        farShooter = !farShooter;

        if (farShooter) {
            shooter.upperSpeed.setDouble(80);
            shooter.lowerSpeed.setDouble(100);
        } else {
            shooter.upperSpeed.setDouble(SPEAKER_DC * 100);
            shooter.lowerSpeed.setDouble(SPEAKER_DC * 100);
        }
    }).ignoringDisable(true).withName("Delivery");


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
                        () -> true,
                        driver::getL2Axis, // decelerator
                        driver.L1().and(() -> !climberMode),
                        () -> emptyPose)
        );

        driver.options().whileTrue(toggleMotorsIdleMode().alongWith(leds.setPattern(SOLID, WHITE.color)));
        driver.PS().onTrue(swerve.resetOdometryAngleCommand());

        driver.L1().whileTrue(leds.deadLineLEDcommand(leds.setPattern(BLINKING, RED.color)));

        // manual actions
        climber.setDefaultCommand(climber.manualCommand(driver.L1(climberLoop), driver.R1(climberLoop), driver.L2(climberLoop), driver.R2(climberLoop)));

        // intake
        driver.circle().toggleOnTrue(intake.intakeFromAngleCommand(HUMAN_PLAYER_BACKWARD, intakeVibrate));
        driver.cross().toggleOnTrue(intake.intakeFromAngleCommand(GROUND, intakeVibrate));

        driver.options().onTrue(intake.toggleIdleModeCommand());

        driver.touchpad().onTrue(intake.pumpNoteCommand());

        // shooter
        driver.square().and(intake.intakingTrigger.negate()).toggleOnTrue(scoreNoteCommand(shooter.shootToAmpCommand(), driver.R1(), true));
        driver.triangle().and(intake.intakingTrigger.negate()).toggleOnTrue(
                scoreNoteCommand(shooter.shootToSpeakerManualCommand(), driver.R1(), false));

        driver.R2().and(intake.intakingTrigger.negate()).onTrue(shooter.prepFarShooter(() -> swerve.getDistanceFromPose(SPEAKER.pose.get())));
        driver.R2().onFalse(
                swerve.turnToLocationCommand(SPEAKER).alongWith(
                        scoreNoteCommand(shooter.setShooterCommand(new ShooterState(() -> swerve.getDistanceFromPose(SPEAKER.pose.get()))), swerve.atAngleTrigger.and(shooter.getCurrentState().stateReady).and(() -> swerve.angleHeartbeat >= 25), false)));

        driver.povUp().toggleOnTrue(new ParallelDeadlineGroup(
                scoreNoteCommand(shooter.manualShooter(1, 0.7), driver.R1(), false),
                swerve.turnToLocationCommand(SHOOTING_LOCATION)));

        driver.povLeft().toggleOnTrue(new ParallelCommandGroup(
                new ProxyCommand(() -> swerve.pathFindToPose(HP_RIGHT.pose.get(), new PathConstraints(2, 2, Math.PI, Math.PI), 0)),
                intake.intakeFromAngleCommand(HUMAN_PLAYER_BACKWARD, new InstantCommand(() -> {
                }))));

        driver.povRight().toggleOnTrue(new ParallelCommandGroup(
                new ProxyCommand(() -> swerve.pathFindToPose(AMPLIFIER.pose.get(), new PathConstraints(1.5, 2, Math.PI, Math.PI), 0)),
                scoreNoteCommand(shooter.shootToAmpCommand(), driver.R1(), true)));

        driver.povDown().onTrue(shooter.forceShootCommand().alongWith(new WaitCommand(0.75).andThen(intake.forceTransport(() -> false))));

        driver.create().onTrue(intake.shootToAmpCommand());

        //up - full field shot to speaker
        //right - drive to amp
        //left - drive to HP
        //down - force shoot command
    }

    // methods
    private Command scoreNoteCommand(Command shooterCommand, Trigger release, boolean toAmp) {
        return new ParallelCommandGroup(
                shooterCommand,
                leds.scheduleLEDcommand(leds.setPattern(SOLID, ORANGE.color)),
                new SequentialCommandGroup(
                        new WaitUntilCommand(release),
                        new ParallelDeadlineGroup(
                                intake.transportToShooterCommand(() -> toAmp),
                                leds.deadLineLEDcommand(leds.setPattern(SOLID, GREEN.color).withTimeout(0.25))
                        )));
    }

    private Command systemTesterCommand() {
        return new SequentialCommandGroup(
                swerve.driveSwerveCommand(() -> 0, () -> 0, () -> 0.75, () -> false).withTimeout(5),
                swerve.straightenModulesCommand(),
                intake.intakeFromAngleCommand(HUMAN_PLAYER_BACKWARD, intakeVibrate),
                new WaitCommand(1),
                scoreNoteCommand(shooter.shootToAmpManualCommand(intake.hasNoteTrigger), new Trigger(() -> true), true)
        );
    }

    private Command vibrateControllerCommand(int intensity, double seconds) {
        return Commands.runEnd(
                        () -> driverVibration.setRumble(kBothRumble, intensity / 100.0),
                        () -> driverVibration.setRumble(kBothRumble, 0))
                .withTimeout(seconds).ignoringDisable(true);
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

        NamedCommands.registerCommand("intakeFromGround", intake.halfIntakeFromGround());
        NamedCommands.registerCommand("closeIntake", intake.closeIntakeCommand());
        NamedCommands.registerCommand("pumpNote", intake.pumpNoteCommand());

        NamedCommands.registerCommand("setSwerveIdle", swerve.setDriveIdleMode(CANSparkBase.IdleMode.kCoast));

        NamedCommands.registerCommand("forceShoot", shooter.forceShootCommand().alongWith(new WaitCommand(0.75).andThen(intake.forceTransport(() -> false))));

        NamedCommands.registerCommand("farShooter", scoreNoteCommand(shooter.manualShooter(1, 0.6), new Trigger(() -> true), false));
        NamedCommands.registerCommand("prepFarShooter", shooter.prepFarShooter(() -> swerve.getDistanceFromPose(SPEAKER.pose.get())));
        NamedCommands.registerCommand("shootFromDistance",
                new SequentialCommandGroup(
                        new InstantCommand(timer::restart),
                        scoreNoteCommand(shooter.setShooterCommand(new ShooterState(() -> swerve.getDistanceFromPose(SPEAKER.pose.get()))), shooter.getCurrentState().stateReady.and(timerTrigger), false)));

        pitTab.add("System tester", systemTesterCommand().withName("SystemTest")).withSize(2, 2);

        matchTab.addBoolean("intake note", intake.hasNoteTrigger).withPosition(19, 0).withSize(4, 3);
        matchTab.add("delivery", shooterDistanceCommand).withPosition(16, 1).withSize(3, 2);
        matchTab.add("climberMode", climberModeCommand).withPosition(16, 3).withSize(3, 2);

        autoChooser.setDefaultOption("none", new InstantCommand(() -> {
        }));
        autoChooser.addOption("123", swerve.runAuto("123"));
        autoChooser.addOption("321", swerve.runAuto("321"));

        autoChooser.addOption("3214", swerve.runAuto("3214"));
        autoChooser.addOption("73", swerve.runAuto("73"));

        autoChooser.addOption("shoot", shooter.forceShootCommand().alongWith(new WaitCommand(0.75).andThen(intake.forceTransport(() -> false))));
        autoChooser.addOption("leaveFromBottom", swerve.runAuto("shootAndLeave"));

        Shuffleboard.getTab("Auto").add(autoChooser);
    }

    public Command stopSwerveCommand() {
        return swerve.driveSwerveCommand(() -> 0, () -> 0, () -> 0, () -> false).withTimeout(0.1);
    }

    public Command getAutonomousCommand() {
        // Pose 1
//        Pose2d(0.94, 6.48, Rotation2d.fromDegrees(60));
        // Pose 2
//        Pose2d(1.3, 5.57, new Rotation2d());
        // Pose 3
//        new Pose2d(0.74, 4.48, Rotation2d.fromDegrees(-60)));

        return new ProxyCommand(() -> autoChooser.getSelected());
    }
}