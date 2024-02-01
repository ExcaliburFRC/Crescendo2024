package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.Constants.FieldConstants.FieldLocations.*;
import static frc.robot.Constants.IntakeConstants.INTAKE_ANGLE.GROUND;
import static frc.robot.Constants.IntakeConstants.INTAKE_ANGLE.HUMAN_PLAYER;
import static frc.robot.Constants.ShooterConstants.SPEAKER_PREP_RADIUS;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class RobotContainer {
    // subsystems
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    // controllers
    private final CommandPS5Controller driver = new CommandPS5Controller(0);
    private final XboxController driverVibration = new XboxController(4);

    public final SendableChooser<Command> shouldDriveToCenterLineChooser = new SendableChooser<>();

    public boolean shooterWorks = true;
    public final Trigger isAtSpeakerRadius = new Trigger(()-> swerve.getDistanceFromPose(SPEAKER_CENTER.pose.get()) < SPEAKER_PREP_RADIUS);

    // TODO: find leftX & leftY axis indexes
    public final Trigger terminatePathTrigger = new Trigger(driver.axisGreaterThan(0, 0.5).or(driver.axisGreaterThan(0, 0.5)));

    private final Pose2d emptyPose = new Pose2d();

    public ShuffleboardTab matchTab = Shuffleboard.getTab("match");
    public ShuffleboardTab pitTab = Shuffleboard.getTab("pit");

    public RobotContainer(){
        configureBindings();
        initShuffleBoard();
    }

    // bindings
    private void configureBindings() {
        swerve.setDefaultCommand(
                swerve.driveSwerveCommand(
                        () -> applyDeadband(-driver.getLeftY(), 0.07),
                        () -> applyDeadband(-driver.getLeftX(), 0.07),
                        () -> applyDeadband(-driver.getRightX(), 0.07),
                        driver.R3().and(()-> intake.getCurrentCommand().getName().equals("intakeCommand")), // activate robot oriented only while intaking
                        driver::getL2Axis,
                        ()-> driver.R1().getAsBoolean()? SPEAKER.pose.get() : emptyPose)
        );

        shooter.setDefaultCommand(shooter.prepShooterCommand(isAtSpeakerRadius, intake));

        driver.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.applyPatternCommand(SOLID, WHITE.color)));
        driver.PS().onTrue(swerve.resetOdometryAngleCommand());

        // show intake cam while pressing R3
        driver.R3().whileTrue(Commands.startEnd(()-> Shuffleboard.selectTab("intakeCam"), ()-> Shuffleboard.selectTab("match")));

        // if R1 is pressed and the robot is stationary, shoot to speaker
        driver.R1().and(()-> Math.max(swerve.getRobotRelativeSpeeds().vxMetersPerSecond, swerve.getRobotRelativeSpeeds().vyMetersPerSecond) < 0.2)
        .whileTrue(scoreNoteCommand(shooter.shootFromWooferCommand()));

        // manual actions
        // if the shooter doesn't work, we shoot the note from the intake
        driver.square().toggleOnTrue(new ConditionalCommand(
                scoreNoteCommand(shooter.shootToAmpCommand()),
                intake.shootToAmpCommand(),
                ()-> shooterWorks)
        );
        driver.triangle().toggleOnTrue(shooter.shootFromWooferCommand());
        driver.cross().toggleOnTrue(intake.intakeFromAngleCommand(HUMAN_PLAYER));
        driver.circle().toggleOnTrue(intake.intakeFromAngleCommand(GROUND));

        // autonomous intake from HP stations
        driver.povRight().onTrue(swerve.pathFindToLocation(HP_RIGHT).alongWith(intake.intakeFromAngleCommand(HUMAN_PLAYER)).until(terminatePathTrigger));
        driver.povUp().onTrue(swerve.pathFindToLocation(HP_CENTER).alongWith(intake.intakeFromAngleCommand(HUMAN_PLAYER)).until(terminatePathTrigger));
        driver.povLeft().onTrue(swerve.pathFindToLocation(HP_LEFT).alongWith(intake.intakeFromAngleCommand(HUMAN_PLAYER)).until(terminatePathTrigger));

        // auto drive to amp and score
        driver.povDown().whileTrue(scoreNoteCommand(
                swerve.pathFindToLocation(AMPLIFIER),
                shooter.shootToAmpCommand()
        ));

        // vibrate driver controller after note intake
        new Trigger(intake.noteIntakedTrigger.and(()-> intake.getCurrentCommand().getName().equals("intakeCommand")))
                .onTrue(vibrateControllerCommand(50, 0.5));
    }

    // methods
    public Command matchPrepCommand() {
        return new SequentialCommandGroup(
                swerve.straightenModulesCommand(),
                intake.intakeIdleCommand()
                // TODO: add close climber telescopic arms
        );
    }

    private Command scoreNoteCommand(Command shooterCommand){
        return new ParallelCommandGroup(
                shooterCommand,
                new WaitUntilCommand(shooter.shooterReadyTrigger).andThen(intake.transportToShooterCommand()));
    }

    private Command scoreNoteCommand(Command swerveCommand, Command shooterCommand){
        return new SequentialCommandGroup(
                swerveCommand.deadlineWith(shooter.prepShooterCommand()),
                scoreNoteCommand(shooterCommand)
        );
    }

    private Command systemTesterCommand(){
        return new SequentialCommandGroup(
                swerve.driveSwerveCommand(()-> 0.25, ()-> 0, ()-> 0.25, ()-> false).withTimeout(5),
                intake.intakeFromAngleCommand(HUMAN_PLAYER),
                new WaitUntilCommand(intake.isAtShooterTrigger),
                scoreNoteCommand(shooter.shootToAmpCommand())
                /// TODO: add climber test
        );
    }

    private Command vibrateControllerCommand(int intensity, double seconds){
        return Commands.runEnd(
                ()-> driverVibration.setRumble(kBothRumble, intensity / 100.0),
                ()-> driverVibration.setRumble(kBothRumble, 0))
                .withTimeout(seconds).ignoringDisable(true);
    }

    public Command toggleMotorsIdleMode() {
        return new ParallelCommandGroup(
                swerve.toggleIdleModeCommand(),
                shooter.toggleIdleModeCommand(),
                intake.toggleIdleModeCommand()
        );
    }

    private void initShuffleBoard(){
        shouldDriveToCenterLineChooser.setDefaultOption("don't Drive", Commands.none());
        shouldDriveToCenterLineChooser.addOption("drive", Commands.idle()); // this is my commandddd!!

        pitTab.add("Match prep", matchPrepCommand());
        pitTab.add("System tester", systemTesterCommand());

        matchTab.add(new InstantCommand(()-> shooterWorks = !shooterWorks));
    }

    public Command getAutonomousCommand(){
        return Commands.none();
    }
}