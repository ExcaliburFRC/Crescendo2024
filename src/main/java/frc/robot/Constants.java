// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Gains;
import frc.robot.util.AllianceUtils.AlliancePose;
import frc.robot.util.SysIdConfig;

import static frc.robot.util.AllianceUtils.isBlueAlliance;
import static java.lang.Math.PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {
        public enum Modules {
            // drive ID, spin ID, abs encoder channel, offset angle, drive reversed, angle reversed
            FL(18, 17, 7, 0.872, true, true),
            FR(12, 11, 3, 0.071, true, true),
            BL(16, 15, 4, 0.595, true, true),
            BR(14, 13, 2, 0.296, true, true);


            public int DRIVE_MOTOR_ID, SPIN_MOTOR_ID, ABS_ENCODER_CHANNEL;
            public double OFFSET_ANGLE;
            public boolean DRIVE_MOTOR_REVERSED, SPIN_MOTOR_REVERSED;

            public static final int FRONT_LEFT = 0;
            public static final int FRONT_RIGHT = 1;
            public static final int BACK_LEFT = 2;
            public static final int BACK_RIGHT = 3;

            Modules(int DRIVE_MOTOR_ID,
                    int SPIN_MOTOR_ID,
                    int ABS_ENCODER_CHANNEL,
                    double OFFSET_ANGLE,
                    boolean DRIVE_MOTOR_REVERSED,
                    boolean SPIN_MOTOR_REVERSED) {
                this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
                this.SPIN_MOTOR_ID = SPIN_MOTOR_ID;
                this.ABS_ENCODER_CHANNEL = ABS_ENCODER_CHANNEL;
                this.OFFSET_ANGLE = OFFSET_ANGLE;
                this.DRIVE_MOTOR_REVERSED = DRIVE_MOTOR_REVERSED;
                this.SPIN_MOTOR_REVERSED = SPIN_MOTOR_REVERSED;
            }
        }

        public static final int PIGEON_ID = 19;

        public static final double TRACK_WIDTH = 0.51; // m
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
                new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2));

        public static final double MAX_VELOCITY_METER_PER_SECOND = Units.feetToMeters(19.3);
        public static final double MAX_VELOCITY_ACCELERATION_METER_PER_SECOND = 6; //TODO: find values

        public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SECOND = 2 * PI; //TODO: find values
        public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SECOND = 2 * 2 * PI; //TODO: find values

        // intentional limitations
        public static final double DRIVE_SPEED_PERCENTAGE = 20; // %

        // autonomous constants
        public static final Gains ANGLE_GAINS = new Gains(0.12, 0, 0.00015);
        public static final Gains TRANSLATION_GAINS = new Gains(0, 0, 0);

        public static final Gains PATHPLANNER_ANGLE_GAINS = new Gains(3.5, 0, 0);
        public static final Gains PATHPLANNER_TRANSLATION_GAINS = new Gains(2.5, 0, 0);

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MAX_VELOCITY_METER_PER_SECOND, MAX_VELOCITY_ACCELERATION_METER_PER_SECOND,
                MAX_ANGULAR_VELOCITY_RAD_PER_SECOND, MAX_ANGULAR_ACCELERATION_RAD_PER_SECOND);
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.12;
        public static final double kDriveEncoderRotationsToMeters = kDriveMotorGearRatio * PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotationsToMeters / 60;
        public static final double kTurningMotorGearRatio = 1 / 21.4285714;
        public static final double kTurningEncoderRotationsToRadians = kTurningMotorGearRatio * 2 * PI;
        public static final double kTurningEncoderRPMToRadiansPerSec = kTurningEncoderRotationsToRadians / 60;

        public static final Gains MODULE_ANGLE_GAINS = new Gains(0.75, 0, 0);
        public static final double TOLERANCE = 0.05;

        public static final int DRIVE_CURRENT_LIMIT = 65;
        public static final int ANGLE_CURRENT_LIMIT = 25;
    }

    public static final class ShooterConstants {
        public static final int UPPER_SHOOTER_MOTOR_ID = 41;
        public static final int LOWER_SHOOTER_MOTOR_ID = 42;

        public static final int SHOOTER_CURRENT_LIMIT = 70;

        public static final int SHOOTER_BEAMBREAK_CHANNEL = 8;

        public static final Gains UPPER_GAINS = new Gains(new Gains(1.96155E-4, 0, 0), new Gains(0.30647001, 0, 0.11433157, 0.011852)); //TODO: find value
        public static final Gains LOWER_GAINS = new Gains(new Gains(1.96155E-4, 0, 0), new Gains(0.30647001, 0, 0.11433157, 0.011852)); //TODO: find value

        public static final double AMP_UPPER_SHOOTER_RPM = 1300;
        public static final double AMP_LOWER_SHOOTER_RPM = 1700;
        public static final double WOOFER_RPM = 0; //TODO: find value

        public static final double SHOOTER_PID_TOLERANCE = 300;

        public static final double SPEAKER_PREP_DC = 0.8; //TODO: find value
        public static final double SPEAKER_PREP_RADIUS = 0;//TODO: find value

        // sysid
        public static final SysIdRoutine.Config sysidConfig = new SysIdConfig(0.5, 6, 30);
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 21;
        public static final int ANGLE_MOTOR_ID = 22;

        public static final int ENCODER_PORT = 1;
        public static final int BEAMBREAK_PORT = 0;

        public static final Gains INTAKE_GAINS = new Gains(0.058037 * 2 , 0.0, 0.011109 / 4,0.038684, 0.12578, 0.022038, 54.356);
        public static final double INTAKE_TOLERANCE = 10;

        public static final double INTAKE_MOTOR_CONVERSION_FACTOR = 1 / 10.0 * 1 / 3.0 * 16 / 40;

        public static final double INTAKE_ENCODER_OFFSET_POSITION = 0.2293;
        public static final int INTAKE_READING_OFFSET = 50; // deg
        public static final int MINIMAL_INTAKE_ANGLE = -15;

        public static final double AMP_SHOOTER_SPEED = -0.5;//TODO: find value
        public static final double STALL_DC = 0.1;

        //sysid
        public static final SysIdRoutine.Config sysidConfig = new SysIdConfig(0.5, 3, 10);
    }

    public static final class ClimberConstants {
        public static final int LEFT_MOTOR_ID = 32;
        public static final int RIGHT_MOTOR_ID = 31;

        //gains
        public static final Gains LEFT_GAINS = new Gains(0, 0, 0); //TODO
        public static final Gains RIGHT_GAINS = new Gains(0, 0, 0);//TODO

        //movement limitation
        public static final double MAX_LINEAR_VELOCITY = 0;//TODO
        public static final double MAX_LINEAR_ACCELERATION = 0;//TODO

        public static final double GEARING = 0;//TODO
        public static final double DRIVE_WHEEL_RADIUS = 0;//TODO

        public static final double ROT_TO_METR = 2 * PI * DRIVE_WHEEL_RADIUS * GEARING;//TODO
        public static final double ROT_TO_METER_PER_SEC = ROT_TO_METR / 60;//TODO

        public static final double kG = 0;//TODO
        //representing the location of the arms on an axis
        // that is parallel to the middle of the robot (the 0 point is the middle of the robot)
        public static final double LEFT_ARM_LOCATION = 0;//TODO
        public static final double RIGHT_ARM_LOCATION = 0;//TODO

        public static final double MINIMAL_HEIGHT = 0;//TODO
        public static final double EXTRA_SAFETY_DISTANCE = 0.1;//TODO

        public enum Chain {
            CHAIN_0(new Translation2d(0, 0), new Translation2d(0, 0), new Translation2d(0, 0)),//TODO
            CHAIN_120(new Translation2d(0, 0), new Translation2d(0, 0), new Translation2d(0, 0)),//TODO
            CHAIN_240(new Translation2d(0, 0), new Translation2d(0, 0), new Translation2d(0, 0)),//TODO
            CHAIN_60(new Translation2d(0, 0), new Translation2d(0, 0), new Translation2d(0, 0)),//TODO
            CHAIN_180(new Translation2d(0, 0), new Translation2d(0, 0), new Translation2d(0, 0)),//TODO
            CHAIN_300(new Translation2d(0, 0), new Translation2d(0, 0), new Translation2d(0, 0));//TODO

            public final Translation2d negEdge;
            public final Translation2d posEdge;
            public final Translation2d centerStage;

            Chain(Translation2d negEdge, Translation2d posEdge, Translation2d centerStage) {
                this.negEdge = negEdge;
                this.posEdge = posEdge;
                this.centerStage = centerStage;
            }

            //gets the robot translation, returns true if the robot's angle from
            //the center of the stage is in the range of angles defined for this chain
            public boolean inRange(Translation2d robotTranslation) {
                //calculate the robot's angle from the center of the stage (relative to the x axis)
                double robotStageAngle =
                        Math.atan(robotTranslation.minus(centerStage).getX() / robotTranslation.minus(centerStage).getY());
                //calculate the same thing for the neg and pos edges
                double negAngle = Math.atan(negEdge.minus(centerStage).getX() / negEdge.minus(centerStage).getY());
                double posAngle = Math.atan(posEdge.minus(centerStage).getX() / posEdge.minus(centerStage).getY());
                //returns true if the robot angle is between the neg and pos angles
                if (posAngle > negAngle) return robotStageAngle < posAngle && robotStageAngle > negAngle;
                return (robotStageAngle > posAngle && robotStageAngle > negAngle)
                        || (robotStageAngle < posAngle && robotStageAngle < negAngle);
            }

            //this function returns the chain that is both in
            //the robot's color in returns true to the inRange function
            public static Chain getBestChain(Translation2d robotTranslation) {
                //recognise which chain we want to climb on
                if (isBlueAlliance()) {
                    if (Chain.CHAIN_0.inRange(robotTranslation)) return Chain.CHAIN_0;
                    else if (Chain.CHAIN_120.inRange(robotTranslation)) return Chain.CHAIN_120;
                    return Chain.CHAIN_240;
                }

                if (Chain.CHAIN_60.inRange(robotTranslation)) return Chain.CHAIN_60;
                else if (Chain.CHAIN_180.inRange(robotTranslation)) return Chain.CHAIN_180;
                return Chain.CHAIN_300;
            }

            //this function returns the projection point of the robot's translation on the
            //line that connects negEdge and posEdge
            public Translation2d getProjection(Translation2d robotTranslation) {
                //calculate m and b for the y = mx + b expression that represents the
                //linear function that is on both posEdge and negEdge
                double m1 = (this.negEdge.getY() - this.posEdge.getY()) /
                        (this.negEdge.getX() - this.posEdge.getX());
                double b1 = this.negEdge.getY() - m1 * this.negEdge.getX();
                //calculate m and b for the y = mx + b expression that represents the
                //linear function that is on both robotTranslation and the projection point
                double m2 = -1 / m1;
                double b2 = robotTranslation.getY() - m2 * robotTranslation.getX();

                //calculate the x and y values of the point the two lines cross aka the projection point
                double projectionX = (b1 - b2) / (m2 - m1);
                double projectionY = m1 * projectionX + b1;

                return new Translation2d(projectionX, projectionY);
            }
        }

        public static final double MINIMAL_CHAIN_HEIGHT_METERS = 0.72;
        public static final double CHAIN_LENGTH_IN_XY_METERS = 2.51;
        public static final double CHAIN_PARABOLA_PARAMETER = 0.3174;
    }


    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.02;

        public enum FieldLocations {
            // Human player locations
            HP_LEFT("HMLeft", new AlliancePose(16.01, 1.21, -60)),
            HP_CENTER("HMCenter", new AlliancePose(15.45, 0.9, -60)),
            HP_RIGHT("HMRight", new AlliancePose(14.95, 0.56, -60)),

            // Speaker locations,
            SPEAKER_TOP("SpeakerTop", new AlliancePose(0.82, 6.61, 60)),
            SPEAKER_CENTER("SpeakerCenter", new AlliancePose(1.32, 5.6, 0)),
            SPEAKER_BOTTOM("SpeakerBottom", new AlliancePose(0.71, 4.51, 120)),

            AMPLIFIER("Amp", new AlliancePose(1.86, 7.68, 90)),
            SPEAKER("", new AlliancePose(0, 5.56, 0)),
            PODIUM("Podium", new AlliancePose(2.86, 4.09, -28));

            public String pathName;
            public AlliancePose pose;

            FieldLocations(String pathName, AlliancePose pose) {
                this.pathName = pathName;
                this.pose = pose;
            }
        }
    }

    public static class LedsConstants {
        public static final int LEDS_PORT = 4; // pwm
        public static final int LENGTH = 50;
    }
}