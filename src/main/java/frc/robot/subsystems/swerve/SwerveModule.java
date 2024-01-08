package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import frc.robot.Constants;

import static frc.robot.Constants.ModuleConstants.*;
import static java.lang.Math.PI;

public class SwerveModule implements Sendable {
  //create the module's motors
  private final Neo _driveMotor;
  private final Neo _angleMotor;

  //create the module's encoders
  private final DutyCycleEncoder _absEncoder;

  private final double _resetOffset;
  private final double _absEncoderOffsetRad;
  //a pid controller for the angle of the module
  private final PIDController _spinningPIDController;

  public Trigger isReset = new Trigger(()-> Math.abs(getResetRad()) < TOLERANCE).debounce(0.1);

  // construct the class
  public SwerveModule(
          int driveMotorId,
          int spinningMotorId,
          boolean driveMotorReversed,
          boolean spinningMotorReversed,
          int absEncoderChannel,
          double offsetAngle) {
    _absEncoder = new DutyCycleEncoder(absEncoderChannel);
    _absEncoderOffsetRad = offsetAngle * 2 * PI;
    _resetOffset = _absEncoderOffsetRad - PI;

    _driveMotor = new Neo(driveMotorId);
    _angleMotor = new Neo(spinningMotorId);

    _driveMotor.setInverted(driveMotorReversed);
    _angleMotor.setInverted(spinningMotorReversed);

    _driveMotor.setBrake(true);
    _angleMotor.setBrake(true);

    _driveMotor.clearFaults();
    _angleMotor.clearFaults();

    _angleMotor.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
    _driveMotor.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);

    _driveMotor.setConversionFactors(kDriveEncoderRotationsToMeters, kDriveEncoderRPMToMeterPerSec);
    _angleMotor.setConversionFactors(kTurningEncoderRotationsToRadians, kTurningEncoderRPMToRadiansPerSec);

    _spinningPIDController = new PIDController(MODULE_ANGLE_GAINS.kp, MODULE_ANGLE_GAINS.ki, MODULE_ANGLE_GAINS.kd);
    _spinningPIDController.enableContinuousInput(-PI, PI);

    resetEncoders();
  }

  // return the module angle between -PI to PI
  public double getResetRad() {
    double angle = _absEncoder.getAbsolutePosition();
    angle = angle * 2 * PI -PI;
    angle -= _resetOffset;
    angle = angle < -PI ? 2 * PI + angle : angle;
    angle = angle > PI ? angle - (2 * PI) : angle;
    return angle;
  }

  // return the module angle between 0 to 2PI
  public double getAbsEncoderRad() {
    double angle = _absEncoder.getAbsolutePosition();
    angle = angle * 2 * PI;
    angle -= _absEncoderOffsetRad;
    angle = angle < 0 ? 2 * PI + angle : angle;
    return angle;
  }

  public void resetEncoders() {
//    _driveEncoder.setPosition(0);
    _angleMotor.setPosition(getAbsEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(_driveMotor.getVelocity(), new Rotation2d(_angleMotor.getPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(_driveMotor.getPosition(), Rotation2d.fromRadians(_angleMotor.getPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stopModule();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);

    _driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.MAX_VELOCITY_METER_PER_SECOND);
    _angleMotor.set(_spinningPIDController.calculate(_angleMotor.getPosition(), state.angle.getRadians()));
  }

  public void spinTo(double setpoint){
    if (Math.abs(getResetRad() - setpoint) > TOLERANCE) {
      _angleMotor.set(-_spinningPIDController.calculate(setpoint, getResetRad()));
    }
    else {
      _angleMotor.set(0);
    }
  }

  public double getAbsPos(){
    return _absEncoder.getAbsolutePosition();
  }

  public void stopModule() {
    _driveMotor.set(0);
    _angleMotor.set(0);
  }

  public void setIdleModeCoast(){
    _driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    _angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void setIdleModebreak(){
    _driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    _angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", () -> Math.toDegrees(getAbsEncoderRad()), null);
    builder.addDoubleProperty("absEncoderPos", this::getAbsPos, null);
    builder.addDoubleProperty("drive output current", _driveMotor::getOutputCurrent, null);
    builder.addDoubleProperty("drive dc output", _driveMotor::getAppliedOutput, null);
  }
}
