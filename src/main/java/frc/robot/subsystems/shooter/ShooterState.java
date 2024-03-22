package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterState {
    public double upperVoltage, lowerVoltage;
    public double upperRPMsetpoint, lowerRPMsetpoint;

    public boolean interpolate;
    private DoubleSupplier distMeters;

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

        upperInterolation.put(2.2, 5450.0);
        lowerInterolation.put(2.2, 3200.0);

        upperInterolation.put(2.5, 6120.0);
        lowerInterolation.put(2.5, 2750.0);
    }

    public final Trigger stateReady = new Trigger(()-> upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint()).debounce(0.2);

    public ShooterState(double uppersetpoint, double lowersetpoint, boolean interpolate) {
        this.upperRPMsetpoint = uppersetpoint;
        this.lowerRPMsetpoint = lowersetpoint;

        this.interpolate = interpolate;
    }

    public ShooterState(double upperSetpoint, double lowerSetpoint){
        this(upperSetpoint, lowerSetpoint, false);
    }

    public ShooterState(DoubleSupplier upperSetpoint, DoubleSupplier lowerSetpoint){
        this(upperSetpoint.getAsDouble(), lowerSetpoint.getAsDouble());
    }

    public ShooterState(DoubleSupplier distMeters){
        this.distMeters = distMeters;
        this.interpolate = true;
    }

    public ShooterState(double setpoint){
        this(setpoint, setpoint);
    }

    public void setVelocities(double upperMeasurement, double lowerMeasurement){
        if (this.interpolate){
            double upperRPM = upperInterolation.get(distMeters.getAsDouble());
            double lowerRPM = lowerInterolation.get(distMeters.getAsDouble());

            upperVoltage = upperFFcontroller.calculate(upperRPM) / 60.0 + upperPIDcontroller.calculate(upperMeasurement, upperRPM);
            lowerVoltage = lowerFFcontroller.calculate(lowerRPM) / 60.0 + lowerPIDcontroller.calculate(lowerMeasurement, lowerRPM);

            this.upperRPMsetpoint = upperRPM;
            this.lowerRPMsetpoint = lowerRPM;
        } else {
            upperVoltage = upperFFcontroller.calculate(upperRPMsetpoint) / 60.0 + upperPIDcontroller.calculate(upperMeasurement, upperRPMsetpoint);
            lowerVoltage = lowerFFcontroller.calculate(lowerRPMsetpoint) / 60.0 + lowerPIDcontroller.calculate(lowerMeasurement, lowerRPMsetpoint);
        }
    }

    public boolean isSameVel(){
        return upperRPMsetpoint == lowerRPMsetpoint;
    }

    public static boolean atSetpoint(){
        return upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint();
    }

    double x = 1.3;
    public Command printInterpolation(){
        return new RunCommand(()-> {
            System.out.println(x + ", " + upperInterolation.get(x));
            x += 0.02;
        }).until(()-> x > 3);
    }

    @Override
    public String toString() {
        return "upper: " + upperRPMsetpoint + ", lower: " + lowerRPMsetpoint;
    }
}
