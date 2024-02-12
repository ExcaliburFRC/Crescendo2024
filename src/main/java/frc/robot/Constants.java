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

        public static final int SHOOTER_BEAMBREAK_CHANNEL = 0; //TODO: find value

        public static final Gains UPPER_GAINS = new Gains(new Gains(0, 0, 0), new Gains(0, 0, 0, 0)); //TODO: find value
        public static final Gains LOWER_GAINS = new Gains(new Gains(0, 0, 0), new Gains(0, 0, 0, 0)); //TODO: find value

        public static final double AMP_LOWER_SHOOTER_RPM = 0; //TODO: find value
        public static final double AMP_UPPER_SHOOTER_RPM = 0; //TODO: find value
        public static final double WOOFER_RPM = 0; //TODO: find value

        public static final double SHOOTER_PID_TOLERANCE = 100;

        public static final double SPEAKER_PREP_DC = 0; //TODO: find value
        public static final double SPEAKER_PREP_RADIUS = 0;//TODO: find value

        public static final double ROT_TO_DEGREES = 0; //TODO: find value
        public static final double RPM_TO_DEG_PER_SEC = 0;//TODO: find value

        // sysid
        private static final double RAMP_RATE = 3;
        private static final double STEP_VOLTAGE = 0.5;
        private static final double TIMEOUT = 10;

        public static final SysIdRoutine.Config sysidConfig = new SysIdConfig(RAMP_RATE, STEP_VOLTAGE, TIMEOUT);
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 0; //TODO: find value
        public static final int ANGLE_MOTOR_ID = 0;//TODO: find value

        public static final int ENCODER_PORT = 0; //TODO: find value
        public static final int BEAMBREAK_PORT = 0;//TODO: find value

        public static final Gains PID_GAINS = new Gains(0, 0, 0);//TODO: find value
        public static final Gains FF_ANGLE_GAINS = new Gains(0, 0, 0);//TODO: find value

        public static final double INTAKE_MOTOR_POSITION_CONVERSION_FACTOR = 0;//TODO: find value
        public static final double INTAKE_MOTOR_VELOCITY_CONVERSION_FACTOR = 0;//TODO: find value

        public static final double INTAKE_ENCODER_OFFSET_POSITION = 0;//TODO: find value
        public static final int ANGLE_THRESHOLD = 2;//TODO: find value
        public static final int MINIMAL_INTAKE_ANGLE = 2;//TODO: find value

        public static final double AMP_SHOOTER_SPEED = -0.5;//TODO: find value
        public static final double STALL_DC = 0.1;

        //sysid
        private static final double RAMP_RATE = 3;
        private static final double STEP_VOLTAGE = 0.5;
        private static final double TIMEOUT = 10;

        public static final SysIdRoutine.Config sysidConfig = new SysIdConfig(RAMP_RATE, STEP_VOLTAGE, TIMEOUT);
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
        public static final int LEDS_PORT = 0; // pwm //TODO: find value
        public static final int LENGTH = 0; //TODO: find value
    }
}