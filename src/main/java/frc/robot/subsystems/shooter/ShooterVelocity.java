package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterVelocity {
    public double upperVoltage, lowerVoltage;
    public double upperSetpoint, lowerSetpoint;

    public boolean interpolate;
    private DoubleSupplier distMeters;

    private static final PIDController upperPIDcontroller = new PIDController(UPPER_GAINS.kp, UPPER_GAINS.ki, UPPER_GAINS.kd);
    private static final PIDController lowerPIDcontroller = new PIDController(LOWER_GAINS.kp, LOWER_GAINS.ki, LOWER_GAINS.kd);

    private static final SimpleMotorFeedforward upperFFcontroller = new SimpleMotorFeedforward(UPPER_GAINS.ks, UPPER_GAINS.kv, UPPER_GAINS.ka);
    private static final SimpleMotorFeedforward lowerFFcontroller = new SimpleMotorFeedforward(LOWER_GAINS.ks, LOWER_GAINS.kv, LOWER_GAINS.ka);

    private static InterpolatingDoubleTreeMap upperInterolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap lowerInterolation = new InterpolatingDoubleTreeMap();

    public final Trigger velocitiesReady = new Trigger(() -> upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint()).debounce(0.2);

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

    public ShooterVelocity(double upperSetpoint, double lowerSetpoint) {
        this.upperSetpoint = upperSetpoint;
        this.lowerSetpoint = lowerSetpoint;
        this.interpolate = false;
    }

    public ShooterVelocity(DoubleSupplier distMeters) {
        this.distMeters = distMeters;
        this.interpolate = true;
    }

    public ShooterVelocity(double setpoint) {
        this(setpoint, setpoint);
    }

    public void setVelocities(double upperVel, double lowerVel) {
        if (this.interpolate) {
            this.upperSetpoint = upperInterolation.get(distMeters.getAsDouble());
            this.lowerSetpoint = lowerInterolation.get(distMeters.getAsDouble());
        }

        upperVoltage = upperFFcontroller.calculate(upperSetpoint) / 60.0 + upperPIDcontroller.calculate(upperVel, upperSetpoint);
        lowerVoltage = lowerFFcontroller.calculate(lowerSetpoint) / 60.0 + lowerPIDcontroller.calculate(lowerVel, lowerSetpoint);
    }

    public double getUpperVoltage() {
        return this.upperVoltage;
    }

    public double getLowerVoltage() {
        return lowerVoltage;
    }

    public boolean isSameVel() {
        return upperSetpoint == lowerSetpoint;
    }
}
