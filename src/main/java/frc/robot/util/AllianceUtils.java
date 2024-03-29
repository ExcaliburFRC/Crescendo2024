package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.FieldConstants.*;

public class AllianceUtils {
    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) return alliance.get().equals(Blue);
        DriverStation.reportError("DS alliance empty!", false);
        return true;
    }

    public static boolean isRedAlliance() {
        return !isBlueAlliance();
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param pose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d pose) {
        if (isBlueAlliance()) return pose;
        return mirrorAlliance(pose);
    }

    public static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(), FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromDegrees(180))
        );
    }

    public static Pose2d mirrorAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    public static class AlliancePose {
        private Pose2d pose;

        public AlliancePose(double x, double y, double degrees){
            this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        }

        public AlliancePose(double degrees){
            this.pose = new Pose2d(0, 0, Rotation2d.fromDegrees(degrees));
        }

        public Pose2d get() {
            return toAlliancePose(pose);
        }
    }
}
