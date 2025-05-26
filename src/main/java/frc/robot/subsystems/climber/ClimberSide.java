package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.Neo;
import frc.robot.util.Neo.Model;
import monologue.Logged;

import java.util.function.BooleanSupplier;

class ClimberSide implements Logged {
    private final Neo motor;

    public ClimberSide(int motorID) {
        motor = new Neo(motorID, Model.SparkMax);

        motor.setSmartCurrentLimit(40);
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

    public Command toggleCoastCommand(){
        return new StartEndCommand(
                ()-> motor.setIdleMode(CANSparkBase.IdleMode.kCoast),
                ()-> motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
                ).ignoringDisable(true);
    }
}
