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
            FL(18, 17, 2, 0.045, true, true),
            FR(12, 11, 3, 0.071, true, true),
            BL(16, 15, 4, 0.595, true, true),
            BR(14, 13, 1, 0.127, true, true);


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
        public static final double MAX_VELOCITY_ACCELERATION_METER_PER_SECOND = 6;

        public static final double MAX_ANGULAR_VELOCITY_PER_SECOND = 180;
        public static final double MAX_ANGULAR_ACCELERATION_PER_SECOND = 360;

        // intentional limitations
        public static final double DRIVE_SPEED_PERCENTAGE = 80; // %
        public static final double BOOST_SPEED_PERCENTAGE = 100; // %

        // autonomous constants
        public static final Gains ANGLE_GAINS = new Gains(0.175, 0, 0.007);
        public static final Gains TRANSLATION_GAINS = new Gains(0, 0, 0);

        public static final Gains PATHPLANNER_ANGLE_GAINS = new Gains(7, 0, 0);
        public static final Gains PATHPLANNER_TRANSLATION_GAINS = new Gains(5.5, 0, 0);

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MAX_VELOCITY_METER_PER_SECOND, MAX_VELOCITY_ACCELERATION_METER_PER_SECOND,
                MAX_ANGULAR_VELOCITY_PER_SECOND, MAX_ANGULAR_ACCELERATION_PER_SECOND);
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4 * (4.65 / 4.9));
        public static final double kDriveMotorGearRatio = 1 / 6.12;
        public static final double kDriveEncoderRotationsToMeters = kDriveMotorGearRatio * PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPMToMeterPerSec = kDriveEncoderRotationsToMeters / 60;
        public static final double kTurningMotorGearRatio = 1 / 21.4285714;
        public static final double kTurningEncoderRotationsToRadians = kTurningMotorGearRatio * 2 * PI;
        public static final double kTurningEncoderRPMToRadiansPerSec = kTurningEncoderRotationsToRadians / 60;

        public static final Gains MODULE_ANGLE_GAINS = new Gains(0.785, 0, 0);
        public static final double TOLERANCE = 0.15;

        public static final int DRIVE_CURRENT_LIMIT = 60;
        public static final int ANGLE_CURRENT_LIMIT = 25;
    }

    public static final class ShooterConstants {
        public static final int UPPER_SHOOTER_MOTOR_ID = 42;
        public static final int LOWER_SHOOTER_MOTOR_ID = 41;

        public static final int SHOOTER_CURRENT_LIMIT = 70;

        public static final Gains UPPER_GAINS = new Gains(new Gains(0.00028558, 0, 0), new Gains(14.5152, 0, 0.11033, 0));
        public static final Gains LOWER_GAINS = new Gains(new Gains(0.0006, 0, 0), new Gains(15.5466, 0, 0.1059890436, 0));

        public static final double AMP_UPPER_SHOOTER_RPM = 1000; //930
        public static final double AMP_LOWER_SHOOTER_RPM = 2250; //2580

        public static final double SPEAKER_SHOOTER_RPM = 5300;

        public static final double SPEAKER_UPPER_SHOOTER_RPM = 5700;
        public static final double SPEAKER_LOWER_SHOOTER_RPM = 3100;

        public static final double SHOOTER_PID_TOLERANCE = 100;

        public static final double MAX_SHOOTER_DISTANCE = 2.4;

        public static final double SPEAKER_DC = 0.8;

        // sysid
        public static final SysIdRoutine.Config sysidConfig = new SysIdConfig(0.5, 7, 30);
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 21;
        public static final int ANGLE_MOTOR_ID = 22;

        public static final int ENCODER_PORT = 6;
        public static final int BEAMBREAK_PORT = 0;

        public static final Gains INTAKE_GAINS = new Gains(0.058037 * 2 , 0.0, 0.011109 / 4,0.038684, 0.12578, 0.022038, 54.356);
        public static final double INTAKE_TOLERANCE = 7.5;

        public static final double ANGLE_MOTOR_CONVERSION_FACTOR = (1 / 10.0) * (1 / 3.0) * (16.0 / 40.0);

        public static final double INTAKE_ENCODER_OFFSET = 0.235;
        public static final int INTAKE_READING_OFFSET = 50; // deg

        public static final double AMP_SHOOTER_SPEED = -1;

        //sysid
        public static final SysIdRoutine.Config sysidConfig = new SysIdConfig(0.5, 3, 10);
    }

    public static final class ClimberConstants {
        public static final int LEFT_MOTOR_ID = 32;
        public static final int RIGHT_MOTOR_ID = 31;
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.02;

        public enum FieldLocations {
            // Human player locations
            HP_LEFT(new AlliancePose(16.01, 1.21, -60)),
            HP_CENTER(new AlliancePose(15.45, 0.9, -60)),
            HP_RIGHT(new AlliancePose(14.82, 0.66, 120)),

            // Speaker locations,
            SPEAKER_TOP(new AlliancePose(0.82, 6.61, 60)),
            SPEAKER_CENTER(new AlliancePose(1.32, 5.6, 0)),
            SPEAKER_BOTTOM(new AlliancePose(0.71, 4.51, 120)),

            BLUE_TRAP(new AlliancePose(4.65, 4.47, -60)),
            SHOOTING_LOCATION(new AlliancePose(1.2, 7.15, 0)),

            AMPLIFIER(new AlliancePose(1.88, 7.76, -90)),
            SPEAKER(new AlliancePose(0, 5.56, 0)),
            PODIUM(new AlliancePose(2.86, 4.09, -28));

            public AlliancePose pose;

            FieldLocations(AlliancePose pose) {
                this.pose = pose;
            }
        }
    }

    public static class LedsConstants {
        public static final int LEDS_PORT = 4; // pwm
        public static final int LENGTH = 50;
    }
}