package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.Gains;
import frc.lib.Neo;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ClimberConstants.kMaxLinearAcceleration;

public class ClimberSide {
    private final Neo motor;
    private double liftingForce = 0;

    public ClimberSide(Gains motorGains, int motorID) {
        motor = new Neo(motorID, motorGains);
        motor.setConversionFactors(kRotToMeter, kRotToMeterPerSec);
    }

    public Command setLengthCommand(double length) {
        final TrapezoidProfile trapezoidProfile =
                new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration));
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
    public double getHeight() {
        return motor.getPosition();
    }
}
