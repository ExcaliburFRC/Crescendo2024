package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Neo;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

    private final Neo rightMotor = new Neo(0, RIGHT_GAINS);
    private final Neo leftMotor = new Neo(0,LEFT_GAINS);
    public static final Climber climber = new Climber();
    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0));
    private final Timer trapozoidTimer = new Timer();

    public Climber() {

        rightMotor.setConversionFactors(0, 0);
        leftMotor.setConversionFactors(0, 0);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        System.out.print("hello");
        System.out.print(" ");
        System.out.print("world");
        System.out.print("!");
    }
    //stage 1

    public Command manualCommand(DoubleSupplier leftVal, DoubleSupplier rightVal) {
        return new RunCommand(
                () -> {
                    rightMotor.set(rightVal.getAsDouble());
                    leftMotor.set(leftVal.getAsDouble());
                }, this);
    }

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

    // stage 2
    private Command goToLength(double length, Neo motor){
        return new FunctionalCommand(
                ()-> {
                    trapozoidTimer.restart();
                    trapezoidProfile = new TrapezoidProfile(
                            new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration));
                },
                ()-> {
                    TrapezoidProfile.State currentState = new TrapezoidProfile.State(motor.getPosition(),motor.getVelocity());
                    TrapezoidProfile.State goalState = new TrapezoidProfile.State(length,0);

                    TrapezoidProfile.State setPointState = trapezoidProfile.calculate(trapozoidTimer.get(),currentState,goalState);
                    double feedForward =
                            Math.signum(setPointState.velocity) * motor.getGains().ks +
                                    motor.getGains().kg +
                                    motor.getGains().kv * currentState.velocity;
                    motor.setReference(setPointState.position, CANSparkBase.ControlType.kPosition,0,feedForward);
                },
                (__)-> trapozoidTimer.stop(),
                ()-> trapozoidTimer.hasElapsed(trapezoidProfile.totalTime())
        );
    }
}