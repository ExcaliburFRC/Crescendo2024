package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionUtils {

    public static Pose2d getPose(double x, double y, double degrees){
        return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    }
}
