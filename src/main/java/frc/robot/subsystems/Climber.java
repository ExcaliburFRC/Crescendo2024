package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Gains;
import frc.lib.Neo;

import java.util.function.Supplier;

import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    ClimberSide leftSide;
    ClimberSide rightSide;

    public Climber() {
        leftSide = new ClimberSide(LEFT_GAINS, leftID);
        leftSide = new ClimberSide(RIGHT_GAINS, rightID);
    }

    public Command setClimbHeightsCommand(double leftLength, double rightLength) {
        return new ParallelCommandGroup(
                leftSide.setLengthCommand(leftLength),
                rightSide.setLengthCommand(rightLength)
        );
    }

    public Command smartOpenCommand(Supplier<Pose3d> robotPoseSupplier){
        return setClimbHeightsCommand(
                getChainHeight(robotPoseSupplier.get(), LEFT_ARM_LOCATION) + EXTRA_SAFETY_DISTANCE,
                getChainHeight(robotPoseSupplier.get(), RIGHT_ARM_LOCATION) + EXTRA_SAFETY_DISTANCE);
    }
    public Command smartCloseCommand(Supplier<Pose3d> robotPoseSupplier){
        return new RunCommand(() -> setLiftForces(getChainRoll(robotPoseSupplier.get())))
                .alongWith(
                        setClimbHeightsCommand(
                                leftSide.getHeight() - getDesiredHeight(),
                                rightSide.getHeight() - getDesiredHeight()));
    }

    private double getChainRoll(Pose3d pose) {
        return 0;//TODO: calculate the robot roll relative to the chain
    }

    private void setLiftForces(double chainRoll) {
        //TODO: add pid calculation and ff to each side lift force
    }

    private double getChainHeight(Pose3d pose, double armLoc) {
        return 0;/*:TODO implement the next algorithm:
        1. recognise chain based on alliance color and robot angle from center of stage
        2. find projection robot on the chain axis
        3. add to the projection the arm location
        4. find the height of chain in this point
        */
    }
    private double getDesiredHeight(){
        return Math.min(leftSide.getHeight(), rightSide.getHeight()) - MINIMAL_HEIGHT;
    }

    private class ClimberSide {
        private Neo motor;
        private TrapezoidProfile trapezoidProfile;
        private Timer timer;
        private double liftingForce = 0;

        private ClimberSide(Gains motorGains, int motorID) {
            motor = new Neo(motorID, motorGains);
            motor.setConversionFactors(kRotToMeter, kRotToMeterPerSec);
        }

        private Command setLengthCommand(double length) {

            return new FunctionalCommand(
                    () -> {
                        timer.restart();
                        trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxLinearVelocity, kMaxLinearAcceleration));
                    },
                    () -> {
                        TrapezoidProfile.State currentState = new TrapezoidProfile.State(motor.getPosition(), motor.getVelocity());
                        TrapezoidProfile.State goalState = new TrapezoidProfile.State(length, 0);

                        TrapezoidProfile.State setpoint = trapezoidProfile.calculate(timer.get(), currentState, goalState);
                        double feedForward = Math.signum(setpoint.velocity) * motor.getGains().ks +
                                motor.getGains().kv * currentState.velocity +
                                motor.getGains().kg
                                + liftingForce;

                        motor.setReference(setpoint.position, CANSparkBase.ControlType.kPosition, 0, feedForward);
                    },
                    (__) -> timer.stop(),
                    () -> timer.hasElapsed(trapezoidProfile.totalTime())
            );
        }

        private void setLiftingForce(double liftingForce) {
            this.liftingForce = liftingForce;
        }
        private double getHeight(){
            return motor.getPosition();
        }
    }
}