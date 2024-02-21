package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants.FieldLocations;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.swerve.Swerve;
import monologue.Logged;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelSelf;
import static frc.lib.Color.Colors.*;
import static frc.robot.Constants.FieldConstants.FieldLocations.*;
import static frc.robot.Constants.ShooterConstants.SPEAKER_PREP_RADIUS;
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
        private final CommandPS5Controller sysid = new CommandPS5Controller(1);
    private final XboxController driverVibration = new XboxController(5);

    public ShuffleboardTab matchTab = Shuffleboard.getTab("match");
    public ShuffleboardTab pitTab = Shuffleboard.getTab("pit");

    // swerve
    boolean robotRelativeDrive = false;
    final Pose2d emptyPose = new Pose2d();

    FieldLocations HP_Station = HP_CENTER;
    FieldLocations shooter_Location = AMPLIFIER;

    EventLoop climberLoop = new EventLoop();

    // TODO: find leftX & leftY axis indexes
    final Trigger terminateAutoTrigger = new Trigger(driver.axisGreaterThan(0, 0.5).or(driver.axisGreaterThan(0, 0.5)));

    // shooter
    boolean shooterWorks = true;
    final Trigger isAtSpeakerRadius = new Trigger(() -> swerve.getDistanceFromPose(SPEAKER_CENTER.pose.get()) < SPEAKER_PREP_RADIUS);

    // intake
    boolean intakeWorks = true;

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
                        () -> emptyPose)
        );

        driver.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.setPattern(SOLID, WHITE.color)));
        driver.PS().onTrue(swerve.resetOdometryAngleCommand());

        // manual actions
        climber.setDefaultCommand(climber.manualCommand(driver.L1(climberLoop), driver.R1(climberLoop), driver.L2(climberLoop), driver.R2(climberLoop)));

        // intake
        driver.circle().toggleOnTrue(dynamicIntakeCommand());
        driver.cross().toggleOnTrue(intake.intakeFromAngleCommand(GROUND, intakeVibrate));

        driver.options().onTrue(intake.pumpNoteCommand());

        driver.create().onTrue(new InstantCommand(()-> CommandScheduler.getInstance().setActiveButtonLoop(climberLoop)));
        climberLoop.bind(()-> driver.create().onTrue(new InstantCommand(()-> CommandScheduler.getInstance().setActiveButtonLoop(CommandScheduler.getInstance().getDefaultButtonLoop()))));

        // shooter
        driver.square().toggleOnTrue(scoreNoteCommand(shooter.shootToAmpCommand(), driver.R1(), true));
        driver.triangle().toggleOnTrue(scoreNoteCommand(shooter.shootToSpeakerManualCommand(), driver.R1(), false));

        sysid.L1().toggleOnTrue(shooter.setShootercommand(new ShooterState(2500)));

//        // susid
//        sysid.circle().toggleOnTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        sysid.square().toggleOnTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        sysid.triangle().toggleOnTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        sysid.cross().toggleOnTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    // triangle - shoot to speaker
    // square - shoot to amp
    // circle - intake from ground
    // cross - intake from HP

    // methods
    private Command scoreNoteCommand(Command shooterCommand, Trigger release, boolean toAmp) {
        return shooterCommand.alongWith(
                new WaitUntilCommand(release).andThen(intake.transportToShooterCommand(()-> toAmp))
        );
    }

    public Command matchPrepCommand() {
        return new SequentialCommandGroup(
                swerve.straightenModulesCommand(),
                climber.manualCommand(() -> false, () -> false, () -> true, () -> true)
        );
    }

//    private Command systemTesterCommand() {
//        return new SequentialCommandGroup(
//                swerve.driveSwerveCommand(() -> 0.25, () -> 0, () -> 0.25, () -> false).withTimeout(5),
//                intake.intakeFromAngleCommand(HUMAN_PLAYER),
//                scoreNoteCommand(shooter.shootToAmpCommand())
    /// TODO: add climber test
//        );
//    }

    private Command vibrateControllerCommand(int intensity, double seconds) {
        return Commands.runEnd(
                        () -> driverVibration.setRumble(kBothRumble, intensity / 100.0),
                        () -> driverVibration.setRumble(kBothRumble, 0))
                .withTimeout(seconds).ignoringDisable(true);
    }

    public Command dynamicIntakeCommand(){
        return new ParallelDeadlineGroup(
                new WaitUntilCommand(intake.hasNoteTrigger.or(shooter.hasNoteTrigger)),
                intake.intakeFromAngleCommand(HUMAN_PLAYER_BACKWARD, vibrateControllerCommand(50, 0.25)),
                shooter.intakeFromShooterCommand())
                .andThen(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                intake.intakeFromAngleCommand(SHOOTER, intakeVibrate),
                                new WaitCommand(0.75).andThen(shooter.transportToIntakeCommand()).until(intake.hasNoteTrigger)
                        ),
                        new InstantCommand(()-> {}),
//                        intake.getDefaultCommand().until(intake.atShooterTrigger).andThen(intake.pumpNoteCommand()),
                        shooter.hasNoteTrigger));
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
        NamedCommands.registerCommand("shootToSpeakerCommand", scoreNoteCommand(shooter.shootToSpeakerManualCommand(), new Trigger(()-> true), false));
        NamedCommands.registerCommand("prepShooterCommand", shooter.prepShooterCommand());
        NamedCommands.registerCommand("shootToAmpCommand", scoreNoteCommand(shooter.shootToAmpCommand(), shooter.shooterReadyTrigger, false));

        NamedCommands.registerCommand("intakeFromGround", intake.halfIntakeFromGround());
        NamedCommands.registerCommand("closeIntake", intake.closeIntakeCommand());

        pitTab.add("Match prep", matchPrepCommand());
//        pitTab.add("System tester", systemTesterCommand());

        matchTab.add("HP_Left", new InstantCommand(() -> HP_Station = HP_LEFT));
        matchTab.add("HP_Center", new InstantCommand(() -> HP_Station = HP_CENTER));
        matchTab.add("HP_Right", new InstantCommand(() -> HP_Station = HP_RIGHT));

        matchTab.add("Amplifier", new InstantCommand(() -> shooter_Location = AMPLIFIER));
        matchTab.add("Speaker", new InstantCommand(() -> shooter_Location = SPEAKER));

        matchTab.add("toggleShooterWorks", new InstantCommand(() -> shooterWorks = !shooterWorks)); // TODO: display as toggle (not as button) in shuffleboard
        matchTab.add("toggleIntakeWorks", new InstantCommand(() -> intakeWorks = !intakeWorks));

        matchTab.add("prepShooter", shooter.prepShooterCommand());
    }

    public Command getAutonomousCommand() {
//        Pose2d startingPose = new Pose2d(1.25, 5.57, new Rotation2d());
        Pose2d startingPose = new Pose2d(0.94, 6.48, Rotation2d.fromDegrees(60));


        return swerve.setOdometryPositionCommand(startingPose).andThen(
                swerve.runAuto("note14"));

//        return swerve.setOdometryPositionCommand(new Pose2d()).andThen(swerve.runPath("PID Test"));
    }

}