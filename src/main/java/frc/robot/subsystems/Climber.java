package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Gains;
import frc.lib.Neo;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {

    private final Neo rightMotor = new Neo(0);
    private final Neo leftMotor = new Neo(0);
    public static final Climber climber = new Climber();
    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0));
    private final Timer trapozoidTimer = new Timer();

    public Climber() {

        rightMotor.setConversionFactors(0, 0);
        leftMotor.setConversionFactors(0, 0);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.initPIDcontroller(Constants.ClimberConstants.RIGHT_GAINS);
        leftMotor.initPIDcontroller(Constants.ClimberConstants.LEFT_GAINS);
        System.out.print("hello");
        System.out.print(" ");
        System.out.print("world");
        System.out.print("!");
    }


    public Command manualCommand(DoubleSupplier leftVal, DoubleSupplier rightVal) {
        return new RunCommand(
                () -> {
                    rightMotor.set(rightVal.getAsDouble());
                    leftMotor.set(leftVal.getAsDouble());
                }, this);
    }
/*
    private Command goToLength(double length, Neo motor){
        return new FunctionalCommand(
                ()-> {
                    trapozoidTimer.restart();
                    trapezoidProfile = new TrapezoidProfile(
                            new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration));
                },
                ()->{
                    TrapezoidProfile.State currentState = new TrapezoidProfile.State(motor.getPosition(),motor.getVelocity());
                    TrapezoidProfile.State finalState = new TrapezoidProfile.State(length,0);

                    TrapezoidProfile.State desiredCurrentState = trapezoidProfile.calculate(trapozoidTimer.get(), new TrapezoidProfile.State())
                }
        )
    }*/
/*
* public Command moveToLengthCommand(Translation2d setpoint) {
    return new FunctionalCommand(
            ()-> {
              trapozoidTimer.restart();
              trapezoidProfile = new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration),
                      new TrapezoidProfile.State(setpoint.getNorm(), 0),
                      new TrapezoidProfile.State(lengthEncoder.getPosition(), lengthEncoder.getVelocity()));
            },
            ()-> {
              TrapezoidProfile.State state = trapezoidProfile.calculate(trapozoidTimer.get());
              double feedforward = kS_LENGTH * Math.signum(state.velocity)
                        + kG_LENGTH * Math.sin(Units.degreesToRadians(getArmAngle()))
                        + kV_LENGTH * state.velocity;

              lengthController.setReference(
                      state.position, CANSparkMax.ControlType.kPosition, 0,
                      feedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
                },
            (__)-> trapozoidTimer.stop(),
            ()-> trapozoidTimer.hasElapsed(trapezoidProfile.totalTime()));
  }
* */




    public Command openArms() {
        return Commands.runEnd(
                () -> {
                    rightMotor.set(0.3);
                    leftMotor.set(0.3);
                },
                () -> {
                    rightMotor.set(0);
                    leftMotor.set(0);
                }, this);
    }

    public Command closeArms() {
        return Commands.runEnd(
                () -> {
                    rightMotor.set(-0.6);
                    leftMotor.set(-0.6);
                },
                () -> {
                    rightMotor.set(0);
                    leftMotor.set(0);
                }, this);
    }


}