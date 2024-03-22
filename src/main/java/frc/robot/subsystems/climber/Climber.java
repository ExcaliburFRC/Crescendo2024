package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ClimberConstants.LEFT_MOTOR_ID;
import static frc.robot.Constants.ClimberConstants.RIGHT_MOTOR_ID;

public class Climber extends SubsystemBase {
    private final ClimberSide leftSide;
    private final ClimberSide rightSide;

    public Climber() {
        leftSide = new ClimberSide(LEFT_MOTOR_ID);
        rightSide = new ClimberSide(RIGHT_MOTOR_ID);
    }

    public Command manualCommand(BooleanSupplier leftRise, BooleanSupplier rightRise, BooleanSupplier leftLower, BooleanSupplier rightLower){
        return new ParallelCommandGroup(
                leftSide.manualCommand(leftRise, leftLower),
                rightSide.manualCommand(rightRise, rightLower),
                new RunCommand(()-> {}, this));
    }

    public Command toggleIdleModeCommand(){
        return leftSide.toggleCoastCommand().alongWith(rightSide.toggleCoastCommand());
    }
}
