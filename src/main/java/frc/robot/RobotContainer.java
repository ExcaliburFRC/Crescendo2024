// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // subsystems
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();

    // controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(0);

    public RobotContainer() {
        configureBindings();
    }

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
    }

    public Command toggleMotorsIdleMode() {
        return new ParallelCommandGroup(
                swerve.toggleIdleModeCommand()
                // add other subsystems here
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}