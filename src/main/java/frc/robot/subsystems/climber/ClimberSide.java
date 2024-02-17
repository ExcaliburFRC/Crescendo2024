package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.Gains;
import frc.lib.Neo;
import monologue.Annotations;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ClimberConstants.MAX_LINEAR_ACCELERATION;

class ClimberSide implements Logged {
    private final Neo motor;
    @Log.NT
    private double liftingForce = 0;

    public ClimberSide(Gains motorGains, int motorID) {
        motor = new Neo(motorID, motorGains);
        motor.setConversionFactors(ROT_TO_METR, ROT_TO_METER_PER_SEC);
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
                    if (rise.getAsBoolean()) motor.set(0.5);
                    else if (lower.getAsBoolean()) motor.set(-0.5);
                    else motor.stopMotor();
                },
                motor::stopMotor
        );
    }
}
