package frc.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Neo extends CANSparkMax{
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;
    private final Gains gains;

/**
 * constructor for the Neo class
 * @param motorID id of the motor
 * @param gains the gains of the motor
 */
    public Neo(int motorID, Gains gains){
        super(motorID, MotorType.kBrushless);

        this.encoder = this.getEncoder();
        this.pidController = this.getPIDController();
        this.gains = gains;

        this.setIdleMode(IdleMode.kBrake); // motors default to brake mode
        initPIDcontroller(gains);
    }

    public Neo(int motorID){
        this(motorID, new Gains());
    }

    public void setConversionFactors(double positionFactor, double velocityFactor) {
        this.encoder.setPositionConversionFactor(positionFactor);
        this.encoder.setVelocityConversionFactor(velocityFactor);
    }

    public void setPosition(double position){
        encoder.setPosition(position);
    }

    public void setReference(double value, CANSparkBase.ControlType ctrl, int pidSlot, double arbFeedforward){
        pidController.setReference(value, ctrl, pidSlot, arbFeedforward);
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    public Gains getGains(){
        return new Gains(gains);
    }

    public void initPIDcontroller(Gains gains){
        pidController.setP(gains.kp);
        pidController.setI(gains.ki);
        pidController.setD(gains.kd);
    }
}