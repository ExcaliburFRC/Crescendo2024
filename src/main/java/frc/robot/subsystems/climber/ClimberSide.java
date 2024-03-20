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
