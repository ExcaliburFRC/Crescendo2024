package frc.robot.subsystems.shooter;

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
import frc.robot.util.ContinuouslyConditionalCommand;
import frc.lib.Neo;
import frc.robot.Constants.FieldConstants.FieldLocations;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.intake.Intake;
import monologue.Annotations.Log;
import monologue.Logged;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RPM;
import static frc.lib.Color.Colors.GREEN;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class Shooter extends SubsystemBase implements Logged {
    private final Neo upperShooter = new Neo(UPPER_SHOOTER_MOTOR_ID);
    private final Neo lowerShooter = new Neo(LOWER_SHOOTER_MOTOR_ID);

    private ShooterState currentState = new ShooterState(0);

    @Log.NT
    private final DigitalInput shooterBeambreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);

    public final BooleanEvent noteShotTrigger = new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> !shooterBeambreak.get()).falling();
    public final Trigger hasNoteTrigger = new Trigger(() -> !shooterBeambreak.get()).debounce(0.05);

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry upperShooterVel = shooterTab.add("upperShooter", 0).getEntry();
    private final GenericEntry lowerShooterVel = shooterTab.add("lowerShooter", 0).getEntry();
    private final GenericEntry bothShootersVel = shooterTab.add("bothShooters", 0).getEntry();

    private static InterpolatingDoubleTreeMap metersToRPM = new InterpolatingDoubleTreeMap();

    private final LEDs leds = LEDs.getInstance();

    @Log.NT
    public Trigger shooterReadyTrigger = new Trigger(ShooterState::atSetpoint)
            .onTrue(leds.setPattern(SOLID, GREEN.color))
            .onFalse(leds.restoreLEDs());

    public Shooter() {
        upperShooter.setIdleMode(kCoast);
        upperShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        upperShooter.setInverted(true);
        upperShooter.setPosition(0);

        lowerShooter.setIdleMode(kCoast);
        lowerShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        lowerShooter.setInverted(true);
        lowerShooter.setPosition(0);

        metersToRPM.put(0.0, 0.0);
    }

    private void stopMotors(){
        upperShooter.stopMotor();
        lowerShooter.stopMotor();
    }

    public ShooterState getCurrentState() {
        return this.currentState;
    }

    public Command setShootercommand(ShooterState state) {
        return new FunctionalCommand(
                ()-> currentState = state,
                ()-> {
                    state.setVelocities(upperShooter.getVelocity(), lowerShooter.getVelocity());

                    upperShooter.setVoltage(state.upperVoltage);
                    lowerShooter.setVoltage(state.lowerVoltage);
                },
                (__)-> {
                    stopMotors();
                    currentState = new ShooterState(0);
                },
                noteShotTrigger, this);
    }

    public Command shootToAmpCommand() {
        return setShootercommand(new ShooterState(AMP_UPPER_SHOOTER_RPM, AMP_LOWER_SHOOTER_RPM));
    }

    public Command shootFromWooferCommand() {
        return setShootercommand(new ShooterState(WOOFER_RPM));
    }

    public Command shootToLocationCommand(FieldLocations locations) {
        return new ConditionalCommand(
                shootToAmpCommand(),
                shootFromWooferCommand(),
                () -> locations.equals(FieldLocations.AMPLIFIER));
    }

    public Command intakeFromShooterCommand() {
        return this.runEnd(() -> {
            upperShooter.set(-0.15);
            lowerShooter.set(-0.15);
        }, ()-> {
            upperShooter.stopMotor();
            lowerShooter.stopMotor();
        }).until(hasNoteTrigger);
    }

    public Command transportToIntakeCommand(){
        return this.runEnd(() -> {
            upperShooter.set(-1);
            lowerShooter.set(-1);
        }, ()-> {
            upperShooter.stopMotor();
            lowerShooter.stopMotor();
        }).until(hasNoteTrigger.negate());
    }

    public Command prepShooterCommand(Trigger isAtSpeakerRadius, Intake intake) {
        return new ContinuouslyConditionalCommand(
                prepShooterCommand().alongWith(leds.setPattern(BLINKING, GREEN.color)),
                new RunCommand(upperShooter::stopMotor, this),
                isAtSpeakerRadius.and(intake.atShooterTrigger).and(intake.hasNoteTrigger)
        );
    }

    public Command prepShooterCommand() {
        return new RunCommand(() -> upperShooter.set(SPEAKER_PREP_DC), this);
    }

    public Command manualShooterCommand() {
        return new StartEndCommand(
                () -> {
                    if (bothShootersVel.getDouble(0) == 0) {
                        upperShooter.set(upperShooterVel.getDouble(0) / 100);
                        lowerShooter.set(lowerShooterVel.getDouble(0) / 100);
                    } else {
                        upperShooter.set(bothShootersVel.getDouble(0));
                        lowerShooter.set(bothShootersVel.getDouble(0));
                    }
                },
                () -> {
                    upperShooter.stopMotor();
                    lowerShooter.stopMotor();
                },
                this);
    }

    public Command shootToAmpManualCommand() {
        return this.runEnd(
                ()-> {
                    this.currentState = new ShooterState(AMP_UPPER_SHOOTER_RPM, AMP_LOWER_SHOOTER_RPM);
                    upperShooter.set(0.25);
                    lowerShooter.set(0.4);
                },
                this::stopMotors)
                .until(noteShotTrigger);
    }

    public Command shootToSpeakerManualCommand() {
        return this.runEnd(
                ()-> {
                    this.currentState = new ShooterState(WOOFER_RPM);
                    upperShooter.set(0.8);
                    lowerShooter.set(0.8);
                },
                this::stopMotors).until(noteShotTrigger);
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> upperShooter.setIdleMode(kCoast),
                () -> upperShooter.setIdleMode(kBrake)
        ).ignoringDisable(true);
    }

    @Log.NT
    private boolean noteShotTrigger(){
        return noteShotTrigger.getAsBoolean();
    }

    @Log.NT
    private double getUpperRPM(){
        return upperShooter.getVelocity();
    }

    @Log.NT
    private double getLowerRPM(){
        return lowerShooter.getVelocity();
    }

    @Log.NT
    private double getUpperSetpoint(){
        return this.currentState.upperRPMsetpoint;
    }

    @Log.NT
    private double getShooterPos(){
        return lowerShooter.getPosition();
    }

    @Log.NT
    private double getLowerSetpoint(){
        return this.currentState.lowerRPMsetpoint;
    }

    // sysid stuff
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
    private final MutableMeasure<Angle> degrees = mutable(Rotations.of(0));

    private final SysIdRoutine shooterSysid = new SysIdRoutine(
            sysidConfig,
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> lowerShooter.setVoltage(volts.in(Volts)),
                    log -> log.motor("shooterMotor")
                            .voltage(appliedVoltage.mut_replace(
                                    lowerShooter.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(degrees.mut_replace(lowerShooter.getPosition(), Rotations))
                            .angularVelocity(velocity.mut_replace(lowerShooter.getVelocity(), RPM)),
                    this
            ));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterSysid.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterSysid.dynamic(direction);
    }
}