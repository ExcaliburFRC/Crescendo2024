// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import monologue.Logged;
import monologue.Monologue;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.Constants.FieldConstants.FieldLocations.*;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements Logged {
  // subsystems
  private final Swerve swerve = new Swerve();
  private final LEDs leds = LEDs.getInstance();
  private final Shooter shooter = new Shooter();
  // controllers
  private final CommandPS4Controller controller = new CommandPS4Controller(0);
  private final CommandPS4Controller controllerArms = new CommandPS4Controller(1);


  // commands
  private final Command m_autonomousCommand = Commands.none(); // insert autonomous command here

  private final SendableChooser<Command> shouldDriveToCenterLineChooser = new SendableChooser<>();


  public Robot(){}


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

    shooter.setDefaultCommand(shooter.ManualShooterCommand());

    shooter.setDefaultCommand(shooter.LinearStart(()-> controller.getLeftY()));
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

  // Robot methods
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    configureBindings();

    boolean fileOnly = false;
    boolean lazyLogging = false;
    Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

      // setFileOnly is used to shut off NetworkTables broadcasting for most logging calls.
      // Basing this condition on the connected state of the FMS is a suggestion only.
      Monologue.setFileOnly(DriverStation.isFMSAttached());
      Monologue.updateAll();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    NamedCommands.registerCommand("ShouldDriveToCenterLine", shouldDriveToCenterLineChooser.getSelected());
    m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_autonomousCommand.cancel();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}