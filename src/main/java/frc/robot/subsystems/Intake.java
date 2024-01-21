package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    public int targetPos = INTAKE_ANGLE.SHOOTER.angle;

    private final PIDController anglePIDcontroller = new PIDController(PIDGains.kp, PIDGains.ki, PIDGains.kd);
    private final ArmFeedforward angleFFcontroller = new ArmFeedforward(FFangleGains.ks, FFangleGains.kg, FFangleGains.kv);

    private final Neo intakeMotor = new Neo(0);
    private final Neo angleMotor = new Neo(0);

    private final DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(0);

    private final DigitalInput beamBreak = new DigitalInput(0);
    private final DigitalInput limitSwitch = new DigitalInput(0);
    private final Trigger beamBreakTrigger = new Trigger(beamBreak::get);

    public Intake() {
        intakeMotor.setConversionFactors(INTAKE_MOTOR_POSITION_CONVERSION_FACTOR, INTAKE_MOTOR_VELOCITY_CONVERSION_FACTOR);

        intakeEncoder.setDistancePerRotation(360);
        intakeEncoder.setPositionOffset(INTAKE_ENCODER_OFFSET_POSITION);
        setDefaultCommand(resetIntakeCommand());
    }

    private Command ejecetNodeCommand(){
       return  setRollerSpeedCommand(()-> -0.5);
    }

    private Command setRollerSpeedCommand(DoubleSupplier speed) {
        return new RunCommand(() -> intakeMotor.set(speed.getAsDouble()));
    }

    private Command stopRollersCommand() {
        return new RunCommand(() -> intakeMotor.set(0));
    }

    private double getAngle() {
        return intakeEncoder.getDistance();
    }


    private Command angleManualCommand(DoubleSupplier joystickValue) {
        return new RunCommand(() ->
          angleMotor.set(joystickValue.getAsDouble()));
    }

    public Command manualIntakeCommand(DoubleSupplier speed, DoubleSupplier joystick) {
        return new ParallelCommandGroup(setRollerSpeedCommand(speed), angleManualCommand(joystick), requirement());
    }

    private Command setIntakeAngleCommand(INTAKE_ANGLE angle) {
        return new FunctionalCommand(
          () -> targetPos = angle.angle,
          () -> {
              double pid = anglePIDcontroller.calculate(getAngle(), angle.angle);
              double ff = angleFFcontroller.calculate(Math.toRadians(angle.angle), 0);
              double output = pid + (ff / 60.0);

              angleMotor.setVoltage(output);
          },
          (__) -> angleMotor.stopMotor(),
          () -> false
        );
    }

    public Command resetIntakeCommand() {
        return new ParallelCommandGroup(
          setIntakeAngleCommand(INTAKE_ANGLE.SHOOTER),
          stopRollersCommand(),
          requirement());
    }

    private Command requirement() {
        return new RunCommand(() -> {
        }, this);
    }
}