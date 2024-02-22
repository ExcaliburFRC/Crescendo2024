package frc.robot.subsystems.shooter;

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
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;
import monologue.Annotations.Log;
import monologue.Logged;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkBase.IdleMode.kCoast;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RPM;
import static frc.lib.Color.Colors.GREEN;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class Shooter extends SubsystemBase implements Logged {
    private final Neo upperShooter = new Neo(UPPER_SHOOTER_MOTOR_ID);
    private final Neo lowerShooter = new Neo(LOWER_SHOOTER_MOTOR_ID);

    private ShooterState currentState = new ShooterState(0);

    @Log.NT
    private final DigitalInput shooterBeambreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);

    public final BooleanEvent noteShotTrigger = new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> !shooterBeambreak.get()).falling();
    public final Trigger beambreakTrigger = new Trigger(() -> !shooterBeambreak.get());
    public final Trigger hasNoteTrigger = new Trigger(() -> !shooterBeambreak.get()).debounce(0.05);


    private final LEDs leds = LEDs.getInstance();

    @Log.NT
    public Trigger shooterReadyTrigger = currentState.stateReady
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

        RobotContainer.robotData.add("beambreak", beambreakTrigger.getAsBoolean());
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
                    System.out.println(__);
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

//    public Command prepShooterCommand(Trigger isAtSpeakerRadius, Intake intake) {
//        return new ContinuouslyConditionalCommand(
//                prepShooterCommand().alongWith(leds.setPattern(BLINKING, GREEN.color)),
//                new RunCommand(upperShooter::stopMotor, this),
//                isAtSpeakerRadius.and(intake.atShooterTrigger).and(intake.hasNoteTrigger)
//        );
//    }

    public Command prepShooterCommand() {
        return new RunCommand(() -> {
            upperShooter.set(SPEAKER_DC);
            lowerShooter.set(SPEAKER_DC);
        }, this);
    }

    public Command shootToSpeakerManualCommand() {
        return this.runEnd(
                ()-> {
                    this.currentState = new ShooterState(WOOFER_RPM);
                    upperShooter.set(SPEAKER_DC);
                    lowerShooter.set(SPEAKER_DC);
                },
                this::stopMotors).until(noteShotTrigger);
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> upperShooter.setIdleMode(kCoast),
                () -> upperShooter.setIdleMode(kBrake))
                .ignoringDisable(true);
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
    private double getUpperSetpoint(){
        return currentState.upperRPMsetpoint;
    }

    @Log.NT
    private double getLowerSetpoint(){
        return currentState.lowerRPMsetpoint;
    }

    @Log.NT
    private double getLowerRPM(){
        return lowerShooter.getVelocity();
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