package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
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

import java.util.function.BooleanSupplier;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static frc.lib.Color.Colors.*;
import static frc.robot.Constants.FieldConstants.FieldLocations.*;
import static frc.robot.Constants.ShooterConstants.SPEAKER_PREP_RADIUS;
import static frc.robot.subsystems.LEDs.LEDPattern.*;
import static frc.robot.subsystems.intake.IntakeState.IntakeAngle.*;

public class RobotContainer implements Logged {
    // subsystems
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();
    private final Shooter shooter = new Shooter();
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

    // TODO: find leftX & leftY axis indexes
    final Trigger terminateAutoTrigger = new Trigger(driver.axisGreaterThan(0, 0.5).or(driver.axisGreaterThan(0, 0.5)));

    // shooter
    boolean shooterWorks = true;
    final Trigger isAtSpeakerRadius = new Trigger(() -> swerve.getDistanceFromPose(SPEAKER_CENTER.pose.get()) < SPEAKER_PREP_RADIUS);

    // intake
    boolean intakeWorks = true;

    public RobotContainer() {
        init();
        configureBindings();
    }

    // bindings
    private void configureBindings() {
        swerve.setDefaultCommand(
                swerve.driveSwerveCommand(
                        () -> applyDeadband(-driver.getLeftY(), 0.07),
                        () -> applyDeadband(-driver.getLeftX(), 0.07),
                        () -> applyDeadband(-driver.getRightX(), 0.07),
                        () -> robotRelativeDrive,
                        driver::getL2Axis, // decelerator
                        () -> emptyPose) // if R1 pressed, turn swerve to Speaker
//                        () -> driver.L1().getAsBoolean() ? SPEAKER.pose.get() : emptyPose) // if R1 pressed, turn swerve to Speaker
        );

        driver.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.applyPatternCommand(SOLID, WHITE.color)));
        driver.PS().onTrue(swerve.resetOdometryAngleCommand());

//         if R1 is pressed and the robot is stationary, shoot to speaker
//        driver.R1().and(() -> Math.max(swerve.getRobotRelativeSpeeds().vxMetersPerSecond, swerve.getRobotRelativeSpeeds().vyMetersPerSecond) < 0.2)
//                .whileTrue(scoreNoteCommand(shooter.shootFromWooferCommand()));

        // manual actions
        // if the shooter doesn't work, we shoot the note from the intake
        driver.square().toggleOnTrue(
                new ConditionalCommand(
                        scoreNoteCommand(shooter.shootToAmpManualCommand(), () -> driver.R1().getAsBoolean()),
                        intake.shootToAmpCommand(),
                        () -> shooterWorks)
        );
        // if the intake doesn't work, we intake the note from the shooter
        driver.cross().toggleOnTrue(new ConditionalCommand(
                intake.intakeFromAngleCommand(HUMAN_PLAYER, vibrateControllerCommand(50, 0.5)),
                shooter.intakeFromShooterCommand(),
                () -> intakeWorks));

        driver.triangle().toggleOnTrue(scoreNoteCommand(shooter.shootToSpeakerManualCommand(), () -> driver.R1().getAsBoolean()));
        driver.circle().toggleOnTrue(intake.intakeFromAngleCommand(GROUND, vibrateControllerCommand(50, 0.5)));

        climber.setDefaultCommand(climber.manualCommand(driver.L1(), driver.R1(), driver.L2(), driver.R2()));

        sysid.circle().toggleOnTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysid.square().toggleOnTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysid.triangle().toggleOnTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysid.cross().toggleOnTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    // triangle - shoot to speaker
    // square - shoot to amp
    // circle - intake from ground
    // cross - intake from HP

    // methods
    public Command matchPrepCommand() {
        return new SequentialCommandGroup(
                swerve.straightenModulesCommand(),
                intake.intakeIdleCommand()
                // TODO: add close climber telescopic arms
        );
    }

    private Command scoreNoteCommand(Command shooterCommand, BooleanSupplier release) {
        return new ParallelCommandGroup(
                shooterCommand,
                new SequentialCommandGroup(
                        new WaitUntilCommand(release),
                        new ParallelDeadlineGroup(
                                intake.transportToShooterCommand(shooter::getCurrentState),
                                leds.applyPatternCommand(SOLID, RED.color)))
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

    public Command toggleMotorsIdleMode() {
        return new ParallelCommandGroup(
                swerve.toggleIdleModeCommand(),
                shooter.toggleIdleModeCommand(),
                intake.toggleIdleModeCommand()
        );
    }

    private void init() {
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
        return Commands.none();
    }
}