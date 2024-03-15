package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        upperPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);
        lowerPIDcontroller.setTolerance(SHOOTER_PID_TOLERANCE);

        upperInterolation.put(1.32, 4300.0);
        lowerInterolation.put(1.32, 4300.0);

        upperInterolation.put(1.6, 4300.0);
        lowerInterolation.put(1.6, 4300.0);

        upperInterolation.put(2.0, 5000.0);
        lowerInterolation.put(2.0, 3500.0);

        upperInterolation.put(2.4, 5500.0);
        lowerInterolation.put(2.4, 3500.0);
    }

    public final Trigger stateReady = new Trigger(()-> upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint());

    public ShooterState(double uppersetpoint, double lowersetpoint) {
        this.upperRPMsetpoint = uppersetpoint;
        this.lowerRPMsetpoint = lowersetpoint;
    }

    public ShooterState(double distMeters){
        this(upperInterolation.get(distMeters), lowerInterolation.get(distMeters));
    }

    public void setVelocities(double upperMeasurement, double lowerMeasurement){
        upperVoltage = upperFFcontroller.calculate(upperRPMsetpoint) / 60.0 + upperPIDcontroller.calculate(upperMeasurement, upperRPMsetpoint);
        lowerVoltage = lowerFFcontroller.calculate(lowerRPMsetpoint) / 60.0 + lowerPIDcontroller.calculate(lowerMeasurement, lowerRPMsetpoint);
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
