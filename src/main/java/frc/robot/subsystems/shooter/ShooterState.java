package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterState {
    public double upperRPMsetpoint, lowerRPMsetpoint;
    public double upperVoltage, lowerVoltage;

    private static final PIDController upperPIDcontroller = new PIDController(UPPER_GAINS.kp, UPPER_GAINS.ki, UPPER_GAINS.kd);
    private static final PIDController lowerPIDcontroller = new PIDController(LOWER_GAINS.kp, LOWER_GAINS.ki, LOWER_GAINS.kd);

    private static final SimpleMotorFeedforward upperFFcontroller = new SimpleMotorFeedforward(UPPER_GAINS.ks, UPPER_GAINS.kv, UPPER_GAINS.ka);
    private static final SimpleMotorFeedforward lowerFFcontroller = new SimpleMotorFeedforward(LOWER_GAINS.ks, LOWER_GAINS.kv, LOWER_GAINS.ka);


    public ShooterState(double uppersetpoint, double lowersetpoint) {
        this.upperRPMsetpoint = uppersetpoint;
        this.lowerRPMsetpoint = lowersetpoint;

        upperPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);
        lowerPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);
    }

    public ShooterState(double setpoint) {
        this(setpoint, setpoint);
    }

    public void setVelocities(double upperMeasurement, double lowerMeasurement){
        upperVoltage = upperFFcontroller.calculate(upperRPMsetpoint) / 60 + upperPIDcontroller.calculate(upperMeasurement, upperRPMsetpoint);
        lowerVoltage = lowerFFcontroller.calculate(lowerRPMsetpoint) / 60 + lowerPIDcontroller.calculate(lowerMeasurement, lowerRPMsetpoint);
    }

    public boolean isSameVel(){
        return upperRPMsetpoint == lowerRPMsetpoint;
    }

    public static boolean atSetpoint(){
        return upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint();
    }

    @Override
    public String toString() {
        return "upper: " + upperRPMsetpoint + ", lower: " + lowerRPMsetpoint;
    }
}
