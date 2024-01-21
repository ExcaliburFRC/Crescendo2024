package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);
    private final Neo linearFollower = new Neo(FOLLOWER_LINEAR_MOTOR_ID);
    private final Neo linear = new Neo(LEADER_LINEAR_MOTOR_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    private final Trigger noteTrigger = new Trigger(beamBreak::get);

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);

    private final PIDController linearPID = new PIDController(LINEAR_PID.kp, LINEAR_PID.ki, LINEAR_PID.kd);
    private final InterpolatingDoubleTreeMap interpolate = new InterpolatingDoubleTreeMap();

    public Shooter() {
        shooterFollower.follow(shooter, true);
        linearFollower.follow(linear);
        interpolate.put(MIN_SHOOTING_DISTANCE, MIN_SHOOTING_RPM);
        interpolate.put(MAX_SHOOTING_DISTANCE, MAX_SHOOTING_RPM);
    }

    private Command setShooterPID(double setpoint) {
        double pid = shooterPID.calculate(setpoint, shooter.getVelocity());
        double ff = shooterFF.calculate(setpoint, 0);
        return new RunCommand(() -> shooter.set(pid + ff));
    }

    private Command setLinearCommand(double setpoint) {
        double pid = linearPID.calculate(setpoint, linear.getVelocity());
        return new RunCommand(() -> linear.set(pid));
    }

    public Command setShooterState(ShooterState state) {
        return new ParallelCommandGroup(
              setShooterPID(state.getRPM()),
              new ConditionalCommand(
                     setLinearCommand(LINEAR_LENGTH),
                     new InstantCommand(),
                      state::isLinearOpen
              )
        );
    }

    public Command AMPShooterCommand() {
        return setShooterState(new ShooterState(SHOOT_AMP_RPM, true));
    }

    public Command speakerShootFromWooferCommand() {
        return setShooterState(new ShooterState(SHOOT_AMP_RPM, false));
    }

    public Command shootFromDistanceCommand(double distance) {
        return setShooterState(new ShooterState(distance));
    }
}


// Path: src/main/java/frc/robot/subsystems/Shooter.java

