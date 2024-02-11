package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterState {
    private double upperRPM, lowerRPM;
    public double upperVoltage, lowerVoltage;

    private static final PIDController upperPIDcontroller = new PIDController(UPPER_GAINS.kp, UPPER_GAINS.ki, UPPER_GAINS.kd);
    private static final PIDController lowerPIDcontroller = new PIDController(LOWER_GAINS.kp, LOWER_GAINS.ki, LOWER_GAINS.kd);

    private static final SimpleMotorFeedforward upperFFcontroller = new SimpleMotorFeedforward(UPPER_GAINS.ks, UPPER_GAINS.kv, UPPER_GAINS.ka);
    private static final SimpleMotorFeedforward lowerFFcontroller = new SimpleMotorFeedforward(LOWER_GAINS.ks, LOWER_GAINS.kv, LOWER_GAINS.ka);


    public ShooterState(double upperRPM, double lowerRPM) {
        this.upperRPM = upperRPM;
        this.lowerRPM = lowerRPM;

        upperPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);
        lowerPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);
    }

    public ShooterState(double RPM) {
        this(RPM, RPM);
    }

    public void setVelocities(double upperMeasurement, double lowerMeasurement){
        upperVoltage = upperPIDcontroller.calculate(upperMeasurement, upperRPM) + upperFFcontroller.calculate(upperRPM);
        lowerVoltage = lowerPIDcontroller.calculate(lowerMeasurement, lowerRPM) + lowerFFcontroller.calculate(lowerRPM);
    }

    public boolean isSameVel(){
        return upperRPM == lowerRPM;
    }

    public static boolean atSetpoint(){
        return upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint();
    }
}
