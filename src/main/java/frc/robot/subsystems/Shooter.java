package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Neo;
import frc.robot.Constants.FieldConstants.FieldLocations;

import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(SHOOTER_LEADER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(SHOOTER_FOLLOWER_MOTOR_ID);

    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final BooleanEvent noteShotTrigger =
            new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), beamBreak::get).falling().debounce(0.2);

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);
    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private static InterpolatingDoubleTreeMap metersToRPM = new InterpolatingDoubleTreeMap();

    public Trigger shooterReadyTrigger = new Trigger(shooterPID::atSetpoint);

    public Shooter() {
        shooter.setIdleMode(kCoast);
        shooter.setConversionFactors(ROT_TO_DEGREES, RPM_TO_DEG_PER_SEC);
        shooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

        shooterFollower.follow(shooter, false);
        shooterFollower.setIdleMode(kCoast);
        shooterFollower.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

        shooterPID.setTolerance(SHOOTER_PID_TOLERANCE);

        metersToRPM.put(0.0, 0.0);
    }

    private double getPIDtoSetpoint(double RPM){
        double pid = shooterPID.calculate(shooter.getVelocity(), RPM);
        double ff = shooterFF.calculate(RPM, 0);

        return pid + ff;
    }

    private Command setShooterRPMcommand(double RPM) {
        return this.runEnd(
                () -> shooter.set(getPIDtoSetpoint(RPM)), shooter::stopMotor).until(noteShotTrigger);
    }

    public Command shootToAmpCommand() {
        return setShooterRPMcommand(AMP_RPM);
    }

    public Command shootFromWooferCommand() {
        return setShooterRPMcommand(WOOFER_RPM);
    }

    public Command shootToLocationCommand(FieldLocations locations){
        return new ConditionalCommand(
                shootToAmpCommand(),
                shootFromWooferCommand(),
                () -> locations.equals(FieldLocations.AMPLIFIER));
    }

    public Command shootFromDistanceCommand(DoubleSupplier distance) {
        return this.runEnd(
                () -> shooter.set(getPIDtoSetpoint(metersToRPM.get(distance.getAsDouble()))), shooter::stopMotor)
                .until(noteShotTrigger);
    }

    public Command intakeFromShooterCommand() {
        return this.runEnd(()-> shooter.set(-0.5), shooter::stopMotor).until(beamBreak::get);
    }

    public Command prepShooterCommand(Trigger isAtSpeakerRadius, Intake intake) {
        return new ConditionalCommand(
                prepShooterCommand(),
                Commands.runOnce(shooter::stopMotor, this),
                isAtSpeakerRadius.and(intake.isAtShooterTrigger).and(intake.hasNoteTrigger))
                .repeatedly();
    }

    public Command prepShooterCommand() {
        return this.runOnce(()-> shooter.set(SPEAKER_PREP_DC));
    }

    public Command manualShooterCommand() {
        return new RunCommand(() -> shooter.set(shooterSpeed.getDouble(0)), this);
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> shooter.setIdleMode(kCoast),
                () -> shooter.setIdleMode(kBrake)
        );
    }
    // sysid stuff
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(DegreesPerSecond.of(0));

    private final MutableMeasure<Angle> degrees = mutable(Degrees.of(0));

    private final SysIdRoutine shooterSysid = new SysIdRoutine(
            sysidConfig,
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> shooter.setVoltage(volts.in(Volts)),
                    log -> log.motor("shooterMotor")
                            .voltage(appliedVoltage.mut_replace(
                                    shooter.get() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(degrees.mut_replace(shooter.getPosition(), Degrees))
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