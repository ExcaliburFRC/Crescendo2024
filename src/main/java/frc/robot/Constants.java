// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.Gains;

import static frc.robot.util.PositionUtils.getPose;
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
            FL(18, 17, 5, 0.842, true, true),
            FR(12, 11, 9, 0.122, true, true),
            BL(16, 15, 4, 0.55, true, true),
            BR(14, 13, 8, 0.3339, true, true);


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

        public static final double TRACK_WIDTH = 0.56665; // m
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
                new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2));

        public static final double MAX_VELOCITY_METER_PER_SECOND = Units.feetToMeters(12); //TODO: find values
        public static final double MAX_VELOCITY_ACCELERATION_METER_PER_SECOND = 3; //TODO: find values

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
        public static final double kDriveMotorGearRatio = 1 / 8.14;
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

    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.02;

        public enum FieldLocations {
            // Human player locations
            HM_LEFT("HMLeft", getPose(16.01, 1.21, -60)),
            HM_CENTER("HMCenter", getPose(15.45, 0.9, -60)),
            HM_RIGHT("HMRight", getPose(14.95, 0.56, -60)),

            // Speaker locations,
            SPEAKER_TOP("SpeakerTop", getPose(0.82, 6.61, 60)),
            SPEAKER_CENTER("SpeakerCenter", getPose(1.32, 5.6, 0)),
            SPEAKER_BOTTOM("SpeakerBottom", getPose(0.71, 4.51, 120)),

            AMPLIFIER("Amp", getPose(1.86, 7.68, 90)),
            PODIUM("Podium", getPose(2.86, 4.09, -28));

            public String pathName;
            public Pose2d pose2d;

            FieldLocations(String pathName, Pose2d pose){
                this.pathName = pathName;
                this.pose2d = pose;

            }
        }
    }

    public static class LedsConstants {

        public static final int LEDS_PORT = 0; // pwm
        public static final int LENGTH = 0;

    }
    public static final class ShooterConstants {
        public static final int LEADER_SHOOTER_MOTOR_ID = 31;
        public static final int FOLLOWER_SHOOTER_MOTOR_ID = 32;
        public static final int LEADER_LINEAR_MOTOR_ID = 33;
        public static final int FOLLOWER_LINEAR_MOTOR_ID = 34;
        public static final int SHOOTER_BEAMBREAK_CHANNEL = 0;

        public static final Gains SHOOTER_PID = new Gains(0,0,0);
        public static final Gains SHOOTER_FF = new Gains(0,0,0,0);
        public static final Gains LINEAR_PID = new Gains(0,0,0);
        public static final double LINEAR_SETPOINT = 0;
        public static final double LINEAR_SETPOINT_BACK = 0;
        public static final double SHOOTER_AMP_SPEED = 0.75;
        public static final double MIN_SHOOTING_RPM = 0;
        public static final double MAX_SHOOTING_RPM = 0;
        public static final double SETPOINT_SHOOT_SPEAKER = 0;
        public static final double SETPOINT_SHOOT_AMP = 0;
        public static final double SETPOINT_LINEAR_AMP = 0;

        public static final double SETPOINT_SHOT_SPEAKER = 0;
        public static final double MIN_SHOOTING_DISTANCE = 0.0;//TODO: Find Values
        public static final double MAX_SHOOTING_DISTANCE = 0.0; //TODO: Find Values
        public static final double SET_SHOOTING_RANGE = 0.0; //TODO: find values!
        public static final double SHOOTER_SPEED_PERCENT = 0.0; //TODO: find values!

        public static final double PREP_SHOOTER_SPEED_FOR_AMP = 0.0; //TODO: find values!
        public static final double PREP_SHOOTER_SPEED_FOR_SPEAKER = 0.0; //TODO: find values!
    }
}