package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.SwerveConstants.TRACK_WIDTH;

public class Climber extends SubsystemBase {
    private final ClimberSide leftSide;
    private final ClimberSide rightSide;


    public Climber() {
        leftSide = new ClimberSide(LEFT_GAINS, LEFT_MOTOR_ID);
        rightSide = new ClimberSide(RIGHT_GAINS, RIGHT_MOTOR_ID);
    }

    public Command setClimbHeightsCommand(double leftLength, double rightLength) {
        return new ParallelCommandGroup(
                leftSide.setLengthCommand(leftLength),
                rightSide.setLengthCommand(rightLength)
        );
    }


    public Command smartOpenCommand(Translation2d robotTranslation) {
        //returns a command that opens each arm to the ideal
        //height that is just above the chain based on robot translation
        // so both arms will connect to the chain at the same time and allows us to climb faster
        return setClimbHeightsCommand(
                getChainHeight(robotTranslation, LEFT_ARM_LOCATION) + EXTRA_SAFETY_DISTANCE,
                getChainHeight(robotTranslation, RIGHT_ARM_LOCATION) + EXTRA_SAFETY_DISTANCE);
    }

    public Command smartCloseCommand(DoubleSupplier rollSupplier) {
        //returns a command that lifts the robot of the ground
        //while making sure it is balanced with regard of the forces applied to each arm separately
        return new RunCommand(() -> setLiftForces(rollSupplier.getAsDouble()))
                .alongWith(setClimbHeightsCommand(
                        getErrorHeight(leftSide),
                        getErrorHeight(rightSide)));
    }

    //Resnick did some physics stuff for us
    private void setLiftForces(double chainRollRad) {
        if (chainRollRad > 0) {
            leftSide.setLiftingForce(liftForcesFormula(-1, rightSide, chainRollRad));

            rightSide.setLiftingForce(kG + liftForcesFormula(1, rightSide, chainRollRad));
        } else {
            leftSide.setLiftingForce(kG + liftForcesFormula(1, leftSide, -chainRollRad));

            rightSide.setLiftingForce(liftForcesFormula(-1, leftSide, -chainRollRad));
        }
    }

    public double liftForcesFormula(int toggleKG, ClimberSide climberSide, double chainRollRad) {
        return (kG * toggleKG) * ((TRACK_WIDTH / 2) * Math.cos(chainRollRad) - climberSide.getHeight() * Math.sin(chainRollRad)) /
                ((rightSide.getHeight() + leftSide.getHeight()) * Math.sin(chainRollRad));
    }

    private double getChainHeight(Translation2d robotTranslation, double armLoc) {
        Chain chain = Chain.getBestChain(robotTranslation);
        //find the robot projection on the chain
        //same as where in the chain we want to climb
        Translation2d projection = chain.getProjection(robotTranslation);
        //calculate where on the chain the robot arm supposed to be
        double armProjection = CHAIN_LENGTH_IN_XY_METERS / 2 - chain.posEdge.getDistance(projection) + armLoc;
        //return the height in this place
        return CHAIN_PARABOLA_PARAMETER * Math.pow(armProjection, 2) + MINIMAL_CHAIN_HEIGHT_METERS;
    }


    private double getErrorHeight(ClimberSide climberSide) {
        //return the height the robot will climb above the ground
        return climberSide.getHeight() - Math.min(leftSide.getHeight(), rightSide.getHeight()) - MINIMAL_HEIGHT;
    }

    public Command manualCommand(BooleanSupplier leftRise, BooleanSupplier rightRise, BooleanSupplier leftLower, BooleanSupplier rightLower){
        return new ParallelCommandGroup(
                leftSide.manualCommand(leftRise, leftLower),
                rightSide.manualCommand(rightRise, rightLower),
                new RunCommand(()-> {}, this));
    }

    public Command toggleCoastCommand(){
        return leftSide.toggleCoastCommand().alongWith(rightSide.toggleCoastCommand());
    }
}
