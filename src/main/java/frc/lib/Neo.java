package frc.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Neo extends CANSparkMax{
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;

    public Neo(int motorID){
        super(motorID, MotorType.kBrushless);
        this.encoder = this.getEncoder();
        this.pidController = this.getPIDController();
        setBrake(true); // motors default to brake mode
    }

    public void setConversionFactors(double positionFactor, double velocityFactor) {
        this.encoder.setPositionConversionFactor(positionFactor);
        this.encoder.setVelocityConversionFactor(velocityFactor);
    }

    public void setBrake(boolean isBrake){
        if (isBrake) this.setIdleMode(IdleMode.kBrake);
        else this.setIdleMode(IdleMode.kCoast);
    }

    public void initPIDcontroller(Gains gains){
        pidController.setP(gains.kp);
        pidController.setI(gains.ki);
        pidController.setD(gains.kd);
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public void setPosition(double position){
        encoder.setPosition(position);
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }
    public void setReference(double value, CANSparkBase.ControlType ctrl, int pidSlot, double arbFeedforward){
        pidController.setReference(value, ctrl, pidSlot, arbFeedforward);
    }
}
