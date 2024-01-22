package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.robot.subsystems.shooter.ShooterState.LinearState;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(SHOOTER_LEADER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(SHOOTER_FOLLOWER_MOTOR_ID);

    private final Neo linear = new Neo(LINEAR_LEADER_MOTOR_ID);
    private final Neo linearFollower = new Neo(LINEAR_FOLLOWER_MOTOR_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final Trigger noteTrigger = new Trigger(beamBreak::get);

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);

    private final PIDController linearPID = new PIDController(LINEAR_PID.kp, LINEAR_PID.ki, LINEAR_PID.kd);

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private Trigger shooterAtSetpoint = new Trigger(shooterPID::atSetpoint);
    private Trigger linearAtSetpoint = new Trigger(linearPID::atSetpoint);

    public Trigger shooterReady = shooterAtSetpoint.and(linearAtSetpoint);

    public Shooter() {
        shooterFollower.follow(shooter, true);
        linearFollower.follow(linear);

        shooterPID.setTolerance(SHOOTER_PID_TOLERANCE);
        linearPID.setTolerance(LINEAR_PID_TOLERANCE);
    }

    private void setShooterRPM(double setpoint) {
        double pid = shooterPID.calculate(shooter.getVelocity(), setpoint);
        double ff = shooterFF.calculate(setpoint, 0);

        shooter.set(pid + ff);
    }

    private void setLinearSetpoint(LinearState state) {
        linear.set(linearPID.calculate(state.length, linear.getVelocity()));
    }

    public Command setShooterState(ShooterState state) {
        return this.runEnd(
                ()-> {
                    setShooterRPM(state.RPM);
                    setLinearSetpoint(state.linearState);
                }, shooter::stopMotor);
    }

    public Command shootToAmpCommand() {
        return setShooterState(new ShooterState(AMP_RPM, true));
    }

    public Command shootFromWooferCommand() {
        return setShooterState(new ShooterState(WOOFER_RPM, false));
    }

    public Command shootFromDistanceCommand(double distance) {
        return setShooterState(new ShooterState(distance));
    }

    public Command prepShooterCommand(BooleanSupplier amp, BooleanSupplier speaker, BooleanSupplier hasNote) {
        return new RunCommand(() -> {
            if (hasNote.getAsBoolean()) {
                if (amp.getAsBoolean()) {
                    shooter.set(AMP_PREP_DC);
                } else if (speaker.getAsBoolean()) {
                    shooter.set(SPEAKER_PREP_DC);
                } else {
                    shooter.stopMotor();
                }
            } else shooter.stopMotor();
        }, this);
    };

    public Command manualShooterCommand(DoubleSupplier speed) {
        return new RunCommand(() -> {
            linear.set(speed.getAsDouble());
            shooter.set(shooterSpeed.getDouble(0));
        }, this);
    }

    // sysid stuff
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(DegreesPerSecond.of(0));

    private final SysIdRoutine shooterSysid = new SysIdRoutine(
            sysidConfig,
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> shooter.setVoltage(volts.in(Volts)),
                    log -> log.motor("shooterMotor")
                            .voltage(appliedVoltage.mut_replace(
                                    shooter.get() * RobotController.getBatteryVoltage(), Volts))
                            .angularVelocity(velocity.mut_replace(shooter.getVelocity(), RPM)),
                    this
            ));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterSysid.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterSysid.dynamic(direction);
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}