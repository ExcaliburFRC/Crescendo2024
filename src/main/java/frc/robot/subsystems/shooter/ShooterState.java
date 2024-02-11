package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterState {
    public double upperRPM, lowerRPM;
    public double upperVelocity, lowerVelocity, shooterAngle;
    public double angle;

    private static final PIDController upperPIDcontroller = new PIDController(UPPER_GAINS.kp, UPPER_GAINS.ki, UPPER_GAINS.kd);
    private static final PIDController lowerPIDcontroller = new PIDController(LOWER_GAINS.kp, LOWER_GAINS.ki, LOWER_GAINS.kd);
    private static final PIDController anglePIDcontroller = new PIDController(ANGLE_GAINS.kp, ANGLE_GAINS.ki, ANGLE_GAINS.kd);

    private static final SimpleMotorFeedforward upperFFcontroller = new SimpleMotorFeedforward(UPPER_GAINS.ks, UPPER_GAINS.kv, UPPER_GAINS.ka);
    private static final SimpleMotorFeedforward lowerFFcontroller = new SimpleMotorFeedforward(LOWER_GAINS.ks, LOWER_GAINS.kv, LOWER_GAINS.ka);
    private static final ArmFeedforward angleFFcontroller = new ArmFeedforward(ANGLE_GAINS.ks, ANGLE_GAINS.kg, ANGLE_GAINS.kv, ANGLE_GAINS.ka);

    private static final InterpolatingDoubleTreeMap metersToRPM = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap metersToShooterAngle = new InterpolatingDoubleTreeMap();

    static {
        metersToRPM.put(0.0, 0.0);
        metersToShooterAngle.put(0.0, 0.0);
    }

    public ShooterState(double upperRPM, double lowerRPM, double angle) {
        this.upperRPM = upperRPM;
        this.lowerRPM = lowerRPM;
        this.angle = angle;

        upperPIDcontroller.setTolerance(SHOOTER_PID_VELOCITY_TOLERANCE);
        lowerPIDcontroller.setTolerance(SHOOTER_PID_VELOCITY_TOLERANCE);
        anglePIDcontroller.setTolerance(SHOOTER_PID_ANGLE_TOLERANCE);
    }

    public ShooterState(double RPM, double angle) {
        this(RPM, RPM, angle);
    }

    public ShooterState(double meters){
        this(metersToRPM.get(meters), metersToShooterAngle.get(meters));
    }

    public void setVelocities(double upperMeasurement, double lowerMeasurement) {
        upperVelocity = upperPIDcontroller.calculate(upperMeasurement, upperRPM) + upperFFcontroller.calculate(upperRPM);
        lowerVelocity = lowerPIDcontroller.calculate(lowerMeasurement, lowerRPM) + lowerFFcontroller.calculate(lowerRPM);
    }

    public void setShooterAngle(double angleMeasurement) {
        shooterAngle = anglePIDcontroller.calculate(angleMeasurement, angle) + angleFFcontroller.calculate(Math.toRadians(angle), 0);
    }

    public boolean isSameVel() {
        return upperRPM == lowerRPM;
    }

    public static boolean atSetpoint() {
        return upperPIDcontroller.atSetpoint() && lowerPIDcontroller.atSetpoint();
    }
}
