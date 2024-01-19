package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Neo;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import java.util.function.DoubleSupplier;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final Neo shooter = new Neo(LEADER_SHOOTER_MOTOR_ID);
    private final Neo shooterFollower = new Neo(FOLLOWER_SHOOTER_MOTOR_ID);

    private final Neo linearFollower = new Neo(FOLLOWER_LINEAR_MOTOR_ID);
    private final Neo linear = new Neo(LEADER_LINEAR_MOTOR_ID);
    private final DigitalInput beamBreak = new DigitalInput(SHOOTER_BEAMBREAK_CHANNEL);
    public final Trigger noteTrigger = new Trigger(beamBreak::get);

    public ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    private final GenericEntry shooterSpeed = shooterTab.add("shootSpeedPercent", 0).getEntry();

    private final PIDController shooterPID = new PIDController(SHOOTER_PID.kp, SHOOTER_PID.ki, SHOOTER_PID.kd);

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(SHOOTER_FF.ks, SHOOTER_FF.kv, SHOOTER_FF.ka);
    private final PIDController linearPID = new PIDController(LINEAR_PID.kp, LINEAR_PID.ki, LINEAR_PID.kd);
    private final InterpolatingTreeMap interpolate = new InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble());




    public Shooter(){
        shooterFollower.follow(shooter, true);
        Neo linearFollower = new Neo(FOLLOWER_LINEAR_MOTOR_ID);
        linearFollower.follow(linear);
    }


    private void setLinear(double speed){
        linear.set(speed);
    }

    public Command AMPShooter(){
        return new RunCommand(
                ()->{
                    double LinearAmpPID = linearPID.calculate(LINEAR_SETPOINT, linear.getVelocity());
                    linear.set(LinearAmpPID);

                    double ShooterAmpPid = shooterPID.calculate(SHOOTER_AMP_SPEED, shooter.getVelocity());
                    double ShooterAmpFF = shooterFF.calculate(SHOOTER_AMP_SPEED, 0);
                    double output = ShooterAmpPid + ShooterAmpFF;
                    shooter.set(output);

                    LinearAmpPID = linearPID.calculate(LINEAR_SETPOINT_BACK, linear.getVelocity());
                    linear.set(LinearAmpPID);

                },
        this).until(noteTrigger).andThen(( () -> StopMotorCommand(shooter)));
    }

    public Command SpeakerShotWithControlCommand(double setPoint){
        return new RunCommand(
                ()-> {
                    double pid = shooterPID.calculate(setPoint, shooter.getVelocity());
                    double ff = shooterFF.calculate(setPoint, 0);
                    double output = pid + ff;
                    shooter.set(output);
                    },
                this).until(noteTrigger.negate()).andThen( () -> StopMotorCommand(shooter));
    }

    public Command StartLinearMotorCommand(DoubleSupplier speed){
        return new RunCommand(() -> setLinear(speed.getAsDouble()));
    }


    public Command ManualShooterCommand() {
        return new RunCommand(()-> shooter.set(shooterSpeed.getDouble(0)), this).until(noteTrigger.negate()).andThen(() -> StopMotorCommand(shooter));
    }

    public Command StopMotorCommand(Neo motor){
        return new RunCommand(()-> motor.stopMotor(),this);
    }






    public Command ShootFromDistanceCommand(DoubleSupplier distance)  {
        return new RunCommand(
                ()-> {
                    interpolate.put(MIN_SHOOTING_DISTANCE, MIN_SHOOTING_SPEED);
                    interpolate.put(MAX_SHOOTING_DISTANCE, MIN_SHOOTING_SPEED);
            double pid  = shooterPID.calculate(distance.getAsDouble(), shooter.getVelocity());
            double ff = shooterFF.calculate(distance.getAsDouble(), 0);
            shooter.set((double)interpolate.get(SET_SHOOTING_RANGE)+pid+ff);
            interpolate.get(distance.getAsDouble());

        },
                this).until(noteTrigger.negate()).andThen( () -> StopMotorCommand(shooter));

    }

}
// Path: src/main/java/frc/robot/subsystems/Shooter.java

