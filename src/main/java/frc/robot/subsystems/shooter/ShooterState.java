package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterState {
    public double upperVoltage, lowerVoltage;
    public double upperDC, lowerDC;
    public double upperRPMsetpoint, lowerRPMsetpoint;

    private static final PIDController upperPIDcontroller = new PIDController(UPPER_GAINS.kp, UPPER_GAINS.ki, UPPER_GAINS.kd);
    private static final PIDController lowerPIDcontroller = new PIDController(LOWER_GAINS.kp, LOWER_GAINS.ki, LOWER_GAINS.kd);

    private static final SimpleMotorFeedforward upperFFcontroller = new SimpleMotorFeedforward(UPPER_GAINS.ks, UPPER_GAINS.kv, UPPER_GAINS.ka);
    private static final SimpleMotorFeedforward lowerFFcontroller = new SimpleMotorFeedforward(LOWER_GAINS.ks, LOWER_GAINS.kv, LOWER_GAINS.ka);

    private static InterpolatingDoubleTreeMap upperInterolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lowerInterolation = new InterpolatingDoubleTreeMap();
    static {
        upperInterolation.put(1.32, 0.8);
        lowerInterolation.put(1.32, 0.8);

        upperInterolation.put(1.6, 0.8);
        lowerInterolation.put(1.6, 0.7);

        upperInterolation.put(2.1, 0.8);
        lowerInterolation.put(2.1, 0.6);

        upperInterolation.put(2.39, 0.95);
        lowerInterolation.put(2.39, 0.54);

        upperInterolation.put(2.5, 1.0);
        lowerInterolation.put(2.5, 0.53);
    }

    public final Trigger stateReady = new Trigger(()-> upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint());

    public ShooterState(double uppersetpoint, double lowersetpoint) {
        this.upperRPMsetpoint = uppersetpoint;
        this.lowerRPMsetpoint = lowersetpoint;

        upperPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);
        lowerPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);
    }

    public ShooterState(double distMeters){
        this.upperDC = upperInterolation.get(distMeters);
        this.lowerDC = lowerInterolation.get(distMeters);
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
