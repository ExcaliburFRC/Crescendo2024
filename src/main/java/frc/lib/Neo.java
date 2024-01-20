package frc.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Neo extends CANSparkMax{
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;
    private final Gains neoGains;
/**
 * constructor for the Neo class
 * @param motorID id of the motor
 * @param gains the gains of the motor, in case there is no gains, enter Gains.Empty_Gains
 */
    public Neo(int motorID, Gains gains){
        super(motorID, MotorType.kBrushless);
        this.encoder = this.getEncoder();
        this.pidController = this.getPIDController();
        setBrake(true); // motors default to brake mode
        neoGains = Gains.copyGains(gains);
        initPIDcontroller();
    }
    public Neo(int motorID){
        this(motorID, Gains.EMPTY_GAINS);
    }

    public void setConversionFactors(double positionFactor, double velocityFactor) {
        this.encoder.setPositionConversionFactor(positionFactor);
        this.encoder.setVelocityConversionFactor(velocityFactor);
    }

    public void setBrake(boolean isBrake){
        if (isBrake) this.setIdleMode(IdleMode.kBrake);
        else this.setIdleMode(IdleMode.kCoast);
    }

    public void initPIDcontroller(){
        pidController.setP(neoGains.kp);
        pidController.setI(neoGains.ki);
        pidController.setD(neoGains.kd);
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
    public Gains getGains(){
        return Gains.copyGains(neoGains);
    }
}
