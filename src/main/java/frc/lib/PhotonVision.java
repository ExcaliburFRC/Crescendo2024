package frc.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class PhotonVision {
    private static final double THATS_WAY_TOO_FUCKING_FAR = 3.5;
    private static final int THATS_WAY_TOO_FUCKING_LONG = 5;

    PhotonCamera camera;
    AprilTagFieldLayout fieldLayout;
    Timer seenTagTimer = new Timer();
    Trigger seenTagTrigger = new Trigger(()-> !seenTagTimer.hasElapsed(THATS_WAY_TOO_FUCKING_LONG));

    private PhotonPoseEstimator photonPoseEstimator;

    private final Transform3d robotToCamera = new Transform3d(-0.384279,0,0.46352,new Rotation3d(0,2.573, 0));

    public PhotonVision() {
        camera = new PhotonCamera("OV9281");
        camera.setDriverMode(false);

        fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        seenTagTimer.start();

        RobotContainer.matchTab.addBoolean("seen Tag", seenTagTrigger).withSize(4, 4);
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public void setDriverMode(boolean isDriverMode){
        camera.setDriverMode(isDriverMode);
    }

    public void setPipeline(int index){
        camera.setPipelineIndex(index);
    }

    public PhotonPipelineResult getLatestResualt(){
        var result = camera.getLatestResult();

        if (result != null) return result;
        return new PhotonPipelineResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.2) {
            Translation2d targetTranslation = result.getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d();

            if (targetTranslation.getDistance(new Translation2d(0, 0)) < THATS_WAY_TOO_FUCKING_FAR) {
                seenTagTimer.restart();
                photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                return photonPoseEstimator.update();
            }
        }

        return Optional.empty();
    }

    public boolean updateFromAprilTagPose(BiConsumer<Pose2d, Double> toUpdate) {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return false;

        var id = result.getBestTarget().getFiducialId();
        if (id == -1) return false;

        var tag = fieldLayout.getTagPose(id);
        if (tag.isEmpty()) return false;

        toUpdate.accept(tag.get().plus(result.getBestTarget().getBestCameraToTarget()).toPose2d(), result.getTimestampSeconds());
        return true;
    }
}

