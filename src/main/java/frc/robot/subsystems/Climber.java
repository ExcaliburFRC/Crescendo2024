package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Gains;
import frc.lib.Neo;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.util.AllianceUtils.isBlueAlliance;

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

    public Command smartOpenCommand(Translation2d robotTranslation) {
        return setClimbHeightsCommand(
                getChainHeight(robotTranslation, LEFT_ARM_LOCATION) + EXTRA_SAFETY_DISTANCE,
                getChainHeight(robotTranslation, RIGHT_ARM_LOCATION) + EXTRA_SAFETY_DISTANCE);
    }

    public Command smartCloseCommand(DoubleSupplier rollSupplier) {
        return new RunCommand(() -> setLiftForces(rollSupplier.getAsDouble()))
                .alongWith(
                        setClimbHeightsCommand(
                                leftSide.getHeight() - getDesiredHeight(),
                                rightSide.getHeight() - getDesiredHeight()));
    }

    private void setLiftForces(double chainRoll) {
        //TODO: add pid calculation and ff to each side lift force
    }

    private double getChainHeight(Translation2d robotTranslation, double armLoc) {
        Chain chain;

        if (isBlueAlliance()) {
            if (Chain.CHAIN_0.inRange(robotTranslation)) chain = Chain.CHAIN_0;
            else if (Chain.CHAIN_120.inRange(robotTranslation)) chain = Chain.CHAIN_120;
            else chain = Chain.CHAIN_240;
        } else {
            if (Chain.CHAIN_60.inRange(robotTranslation)) chain = Chain.CHAIN_60;
            else if (Chain.CHAIN_180.inRange(robotTranslation)) chain = Chain.CHAIN_180;
            else chain = Chain.CHAIN_300;
        }

        Translation2d projection = Chain.getProjection(robotTranslation, chain);
        double armProjection = CHAIN_LENGTH_IN_XY_METERS / 2 - chain.getPosEdge().getDistance(projection) + armLoc;
        return CHAIN_PARABOLA_PARAMETER * Math.pow(armProjection,2) + MINIMAL_CHAIN_HEIGHT_METERS;
    }



    private double getDesiredHeight() {
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
                                motor.getGains().kv * currentState.velocity + motor.getGains().kg + liftingForce;

                        motor.setReference(setpoint.position, CANSparkBase.ControlType.kPosition, 0, feedForward);
                    },
                    (__) -> timer.stop(),
                    () -> timer.hasElapsed(trapezoidProfile.totalTime())
            );
        }

        private void setLiftingForce(double liftingForce) {
            this.liftingForce = liftingForce;
        }

        private double getHeight() {
            return motor.getPosition();
        }
    }
}