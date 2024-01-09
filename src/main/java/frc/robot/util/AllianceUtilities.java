package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.SwerveConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.SwerveConstants.FIELD_WIDTH_METERS;

public class AllianceUtilities {
    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) return alliance.get().equals(Blue);
        DriverStation.reportError("DS alliance empty!", false);
        return true;
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param pose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d pose) {
        if (isBlueAlliance()) {
            System.out.println("blue alliance");
            return pose;
        }
        System.out.println("red alliance");
        return switchAlliance(pose);
    }

    private static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(), FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromDegrees(180))
        );
    }
}
