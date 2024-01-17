package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.Neo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {

    private final Neo rightMotor = new Neo(0);
    private final Neo leftMotor = new Neo(0);
    public static final Climber climber = new Climber();

    public Climber() {

        rightMotor.setConversionFactors(0, 0);
        leftMotor.setConversionFactors(0, 0);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        System.out.print("hello");
        System.out.print(" ");
        System.out.print("world");
        System.out.print("!");
    }


    public Command manualCommand(DoubleSupplier leftVal, DoubleSupplier rightVal) {
        return new RunCommand(
                () -> {
                    rightMotor.set(rightVal.getAsDouble());
                    leftMotor.set(leftVal.getAsDouble());
                }, this);
    }

    public Command openArms() {
        return Commands.runEnd(
                () -> {
                    rightMotor.set(0.3);
                    leftMotor.set(0.3);
                },
                () -> {
                    rightMotor.set(0);
                    leftMotor.set(0);
                }, this);
    }

    public Command closeArms() {
        return Commands.runEnd(
                () -> {
                    rightMotor.set(-0.6);
                    leftMotor.set(-0.6);
                },
                () -> {
                    rightMotor.set(0);
                    leftMotor.set(0);
                }, this);
    }


}