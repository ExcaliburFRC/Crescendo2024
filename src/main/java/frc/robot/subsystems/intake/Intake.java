package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import frc.lib.Neo.Model;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.intake.IntakeState.IntakeAngle;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming;
import static frc.lib.Color.Colors.GREEN;
import static frc.lib.Color.Colors.ORANGE;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;


public class Intake extends SubsystemBase implements Logged {
    private final Neo intakeMotor = new Neo(INTAKE_MOTOR_ID, Model.SparkMax);
    private final Neo angleMotor = new Neo(ANGLE_MOTOR_ID, Model.SparkMax);

    private boolean shouldPump = true;
    private boolean useColorSensor = false;

    @Log.NT
    public final DigitalInput beambreak = new DigitalInput(BEAMBREAK_PORT);
    public final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public final Trigger hasNoteTrigger = new Trigger(() -> useColorSensor? (colorSensor.getProximity() > 80) : !beambreak.get()).debounce(0.2);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ENCODER_PORT);

    private final PIDController anglePIDcontroller = new PIDController(INTAKE_GAINS.kp, INTAKE_GAINS.ki, INTAKE_GAINS.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(INTAKE_GAINS.ks, INTAKE_GAINS.kg, INTAKE_GAINS.kv, INTAKE_GAINS.ka);

    public IntakeAngle setpoint = IntakeAngle.SHOOTER;

    public final Trigger atSetpointTrigger = new Trigger(anglePIDcontroller::atSetpoint).debounce(0.2);
    public final Trigger atShooterTrigger = atSetpointTrigger.and(() -> setpoint.equals(IntakeAngle.SHOOTER));

    public final Trigger intakingTrigger = new Trigger(() -> getCurrentCommand() != null && getCurrentCommand().getName().equals("intakeCommand"));

    private final LEDs leds = LEDs.getInstance();

    public Intake() {
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setSmartCurrentLimit(50);

        angleMotor.setConversionFactors(ANGLE_MOTOR_CONVERSION_FACTOR);
        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(40);
        angleMotor.setInverted(true);
        angleMotor.setOpenLoopRampRate(0.35);

        anglePIDcontroller.setTolerance(INTAKE_TOLERANCE);

        encoder.setPositionOffset(INTAKE_ENCODER_OFFSET);
        encoder.setDistancePerRotation(360);

        initShuffleboard();
        setDefaultCommand(setIntakeCommand(new IntakeState(0, IntakeAngle.SHOOTER, false)));
    }

    @Log.NT(key = "intakeAngle")
    public double getAngle() {
        double val = 360 - encoder.getDistance();

        while (val < -50) val += 360;
        while (val > 200) val -= 360;

        return val;
    }

    @Log.NT
    public double getProximity() {
        return colorSensor.getProximity();
    }

    private void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    double encoderAngle;

    private void setIntakeAngle(IntakeAngle angle) {
        setpoint = angle;
        encoderAngle = getAngle();

        double pid = anglePIDcontroller.calculate(encoderAngle + INTAKE_READING_OFFSET, angle.angle);
        double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0) / 60.0;

        if (encoderAngle > 200 || encoderAngle < -50)
            DriverStation.reportError("intake encoder malfunctions: " + encoderAngle, false);
        else angleMotor.setVoltage(pid + ff);
    }

    private void stopMotors() {
        intakeMotor.stopMotor();
        angleMotor.stopMotor();
    }

    private Command setIntakeCommand(IntakeState intakeState) {
        return this.runEnd(() -> {
            setIntakeAngle(intakeState.angle);

            if (!intakeState.waitForAngle) setIntakeSpeed(intakeState.intakeDC);
            else if (atSetpointTrigger.getAsBoolean()) setIntakeSpeed(intakeState.intakeDC);
            else setIntakeSpeed(0);

        }, this::stopMotors);
    }

    public Command intakeFromAngleCommand(IntakeAngle angle, Command vibrateCommand) {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        setIntakeCommand(new IntakeState(0.35, angle, true)).until(hasNoteTrigger.debounce(0.1)),
                        new InstantCommand(vibrateCommand::schedule),
                        setIntakeCommand(new IntakeState(0, IntakeAngle.SHOOTER, false)).until(atShooterTrigger),
                        pumpNoteCommand().unless(() -> DriverStation.isAutonomous() || !shouldPump)).withName("intakeCommand"),
                new SequentialCommandGroup(
                        leds.scheduleLEDcommand(leds.setPattern(BLINKING, ORANGE.color)),
                        new WaitUntilCommand(hasNoteTrigger),
                        leds.scheduleLEDcommand(leds.setPattern(SOLID, GREEN.color).withTimeout(2))));
    }

    public Command halfIntakeFromGround() {
        return setIntakeCommand(new IntakeState(0.35, IntakeAngle.GROUND, false)).until(hasNoteTrigger);
    }

    public Command closeIntakeCommand() {
        return setIntakeCommand(new IntakeState(0, IntakeAngle.SHOOTER, false)).until(atShooterTrigger);
    }

    public Command shootToAmpCommand() {
        return setIntakeCommand(new IntakeState(AMP_SHOOTER_SPEED, IntakeAngle.AMP, true)).until(hasNoteTrigger.negate().debounce(0.3));
    }

    public Command transportToShooterCommand(BooleanSupplier toAmp) {
        return setIntakeCommand(new IntakeState(toAmp.getAsBoolean() ? -0.6 : -0.75, IntakeAngle.SHOOTER, true))
                .until(hasNoteTrigger.negate().debounce(0.2));
    }

    public Command forceTransport(BooleanSupplier toAmp){
        return setIntakeCommand(new IntakeState(toAmp.getAsBoolean() ? -0.6 : -0.75, IntakeAngle.SHOOTER, true)).withTimeout(0.35);
    }

    public Command pumpNoteCommand() {
        return new SequentialCommandGroup(
                this.runEnd(() -> intakeMotor.set(-0.2), intakeMotor::stopMotor).withTimeout(0.1),
                new WaitCommand(0.25),
                this.runEnd(() -> intakeMotor.set(0.2), intakeMotor::stopMotor).withTimeout(0.25))
                .withInterruptBehavior(kCancelIncoming);
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> angleMotor.setIdleMode(IdleMode.kCoast),
                () -> angleMotor.setIdleMode(IdleMode.kBrake)).ignoringDisable(true);
    }

    private void initShuffleboard() {
        RobotContainer.robotData.addBoolean("intake beambreak", hasNoteTrigger);
        RobotContainer.robotData.addDouble("intake angle", this::getAngle).withSize(2, 2);

        RobotContainer.matchTab.add(new InstantCommand(()-> shouldPump = !shouldPump).ignoringDisable(true).withName("togglePump")).withSize(3, 2).withPosition(16, 5);
        RobotContainer.matchTab.add(new InstantCommand(()-> useColorSensor = !useColorSensor).ignoringDisable(true).withName("toggleSensor")).withSize(3, 2).withPosition(16, 7);
    }
}