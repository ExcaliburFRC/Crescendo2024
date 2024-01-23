package frc.lib;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

import java.util.List;

public class AutoAlignment {

    private final Translation2d notePos = new Translation2d(0, 0);
    private final Rotation2d noteAngle = new Rotation2d(0);


    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(notePos, noteAngle)
    );

    // Create the path using the bezier points created above
    PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
                new PathConstraints(Constants.IntakeConstants.MAX_VELOCITY_MPS_PP, Constants.IntakeConstants.MAX_ACCELERATION_MPS_SQ_PP,
                        Constants.IntakeConstants.MAX_ANGULAR_VELOCITY_RPS_PP * Math.PI,
                        Constants.IntakeConstants.MAX_ANGULAR_ACCELERATION_RPS_SQ_PP * Math.PI),

            new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
    );


}
