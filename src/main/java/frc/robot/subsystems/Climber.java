package frc.robot.subsystems;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Neo;

import java.util.function.BooleanSupplier;

public class Climber extends SubsystemBase {

   private final Neo rightMotor;
    private final Neo leftMotor;
    public Climber() {
     rightMotor = new Neo(0);
     leftMotor = new Neo(0);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public Command manualCommand(
            BooleanSupplier leftUp, BooleanSupplier leftDown,
            BooleanSupplier rightUp, BooleanSupplier rightDown){
        return new RunCommand(
                ()-> {
                    if (leftUp.getAsBoolean()) leftMotor.set(0.5);
                    else if (leftDown.getAsBoolean()) leftMotor.set(-0.5);
                    else leftMotor.set(0);
                    if (rightUp.getAsBoolean()) rightMotor.set(0.5);
                    else if (rightDown.getAsBoolean()) rightMotor.set(-0.5);
                    else rightMotor.set(0);

                }, this);
    }
}