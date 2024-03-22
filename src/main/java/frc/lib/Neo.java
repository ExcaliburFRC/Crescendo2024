package frc.lib;

import com.revrobotics.CANSparkBase;
import com.revrobotics.*;

public class Neo extends CANSparkBase {
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;
    private final Gains gains;

    public enum Model{
        SparkMax(SparkModel.SparkMax),
        SparkFlex(SparkModel.SparkFlex);

        SparkModel model;

        Model(SparkModel model){
            this.model = model;
        }
    }

    /**
     * constructor for the Neo class
     *
     * @param motorID id of the motor
     * @param gains   the gains of the motor
     */
    public Neo(int motorID, Gains gains, Model model) {
        super(motorID, MotorType.kBrushless, model.model);
        this.encoder = this.getEncoder();
        this.pidController = this.getPIDController();
        this.gains = gains;

        this.setIdleMode(IdleMode.kBrake); // motors default to brake mode
        this.initPIDcontroller(gains);
    }

    public Neo(int motorID, Model model) {
        this(motorID, new Gains(), model);
    }

    public void setConversionFactors(double positionFactor, double velocityFactor) {
        this.encoder.setPositionConversionFactor(positionFactor);
        this.encoder.setVelocityConversionFactor(velocityFactor);
    }

    public void setConversionFactors(double conversionFactor) {
        this.encoder.setPositionConversionFactor(conversionFactor);
        this.encoder.setVelocityConversionFactor(conversionFactor);
    }

    public void initPIDcontroller(Gains gains) {
        pidController.setP(gains.kp);
        pidController.setI(gains.ki);
        pidController.setD(gains.kd);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    public void setReference(double value, CANSparkBase.ControlType ctrl, int pidSlot, double arbFeedforward) {
        pidController.setReference(value, ctrl, pidSlot, arbFeedforward);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public Gains getGains() {
        return new Gains(gains);
    }
    }