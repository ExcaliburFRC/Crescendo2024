package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Gains;
import frc.lib.Neo;
import frc.lib.Neo.Model;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ClimberConstants.MAX_LINEAR_ACCELERATION;

class ClimberSide implements Logged {
    private final Neo motor;
    @Log.NT
    private double liftingForce = 0;
    private final Timer autoCloseTimer = new Timer();

    public ClimberSide(Gains motorGains, int motorID) {
        motor = new Neo(motorID, motorGains, Model.SparkMax);
//        motor.setConversionFactors(ROT_TO_METR, ROT_TO_METER_PER_SEC);
    }

    public Command setLengthCommand(double length) {
        final TrapezoidProfile trapezoidProfile =
                new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_LINEAR_VELOCITY, MAX_LINEAR_ACCELERATION));
        ElevatorFeedforward feedforwardController =
                new ElevatorFeedforward(motor.getGains().ks,motor.getGains().kg,motor.getGains().kv);
        final Timer timer = new Timer();
        //returns a command that sets the arm to a desired height
        return new FunctionalCommand(
                //initialization
                timer::restart,
                () -> {
                    TrapezoidProfile.State currentState = new TrapezoidProfile.State(motor.getPosition(), motor.getVelocity());
                    TrapezoidProfile.State goalState = new TrapezoidProfile.State(length, 0);
                    TrapezoidProfile.State setpoint = trapezoidProfile.calculate(timer.get(), currentState, goalState);

                    //calculate the feed forward
                    double feedForward = feedforwardController.calculate(currentState.velocity) + liftingForce;
                    //set motor reference with the current setpoint and ff
                    motor.setReference(setpoint.position, CANSparkBase.ControlType.kPosition, 0, feedForward);
                },
                (__) -> timer.stop(), //stop timer at the end
                //a check if the time to open the arm ended
                () -> timer.hasElapsed(trapezoidProfile.totalTime())
        );
    }


    public void setLiftingForce(double liftingForce) {
        //sets the T force required to lift the robot at the time in this arm
        this.liftingForce = liftingForce;
    }

    @Log.NT (key = "climberHeight")
    public double getHeight() {
        return motor.getPosition();
    }

    public Command manualCommand(BooleanSupplier rise, BooleanSupplier lower){
        return Commands.runEnd(
                ()-> {
                    if (rise.getAsBoolean()) motor.set(0.85);
                    else if (lower.getAsBoolean()) motor.set(-0.5);
                    else motor.stopMotor();
                },
                motor::stopMotor
        );
    }

    public Command autoClose(){
        return new FunctionalCommand(
                autoCloseTimer::restart,
                ()-> motor.set(-0.2),
                (__)-> motor.stopMotor(),
                ()-> (autoCloseTimer.get() > 0.25 && Math.abs(motor.getVelocity()) < 0.01)
        );
    }

    public Command toggleCoastCommand(){
        return new StartEndCommand(
                ()-> motor.setIdleMode(CANSparkBase.IdleMode.kCoast),
                ()-> motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
                ).ignoringDisable(true);
    }
}
