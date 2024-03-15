package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.PhotonVision;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.FieldLocations;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceUtils;
import frc.robot.util.AllianceUtils.AlliancePose;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.SwerveConstants.Modules.*;

public class Swerve extends SubsystemBase implements Logged {
    private final SwerveModule[] swerveModules = {
            new SwerveModule(
                    Modules.FL.DRIVE_MOTOR_ID,
                    Modules.FL.SPIN_MOTOR_ID,
                    Modules.FL.DRIVE_MOTOR_REVERSED,
                    Modules.FL.SPIN_MOTOR_REVERSED,
                    Modules.FL.ABS_ENCODER_CHANNEL,
                    Modules.FL.OFFSET_ANGLE,
                    "FL",
                    0),
            new SwerveModule(
                    Modules.FR.DRIVE_MOTOR_ID,
                    Modules.FR.SPIN_MOTOR_ID,
                    Modules.FR.DRIVE_MOTOR_REVERSED,
                    Modules.FR.SPIN_MOTOR_REVERSED,
                    Modules.FR.ABS_ENCODER_CHANNEL,
                    Modules.FR.OFFSET_ANGLE,
                    "FR",
                    0),
            new SwerveModule(
                    Modules.BL.DRIVE_MOTOR_ID,
                    Modules.BL.SPIN_MOTOR_ID,
                    Modules.BL.DRIVE_MOTOR_REVERSED,
                    Modules.BL.SPIN_MOTOR_REVERSED,
                    Modules.BL.ABS_ENCODER_CHANNEL,
                    Modules.BL.OFFSET_ANGLE,
                    "BL",
                    0),
            new SwerveModule(
                    Modules.BR.DRIVE_MOTOR_ID,
                    Modules.BR.SPIN_MOTOR_ID,
                    Modules.BR.DRIVE_MOTOR_REVERSED,
                    Modules.BR.SPIN_MOTOR_REVERSED,
                    Modules.BR.ABS_ENCODER_CHANNEL,
                    Modules.BR.OFFSET_ANGLE,
                    "BR",
                    0),
    };

    private final Pigeon2 pigeon = new Pigeon2(Constants.SwerveConstants.PIGEON_ID);

    private final ProfiledPIDController anglePIDcontroller = new ProfiledPIDController(
            ANGLE_GAINS.kp, ANGLE_GAINS.ki, ANGLE_GAINS.kd,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY_PER_SECOND, MAX_ANGULAR_ACCELERATION_PER_SECOND));

    private final PIDController xTranslationPIDcontroller = new PIDController(PATHPLANNER_TRANSLATION_GAINS.kp, PATHPLANNER_TRANSLATION_GAINS.ki, PATHPLANNER_TRANSLATION_GAINS.kd);
    private final PIDController yTranslationPIDcontroller = new PIDController(PATHPLANNER_TRANSLATION_GAINS.kp, PATHPLANNER_TRANSLATION_GAINS.ki, PATHPLANNER_TRANSLATION_GAINS.kd);

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            getGyroRotation2d(),
            getModulesPositions(),
            new Pose2d());

    private final SwerveDriveOdometry relativeOdometry = new SwerveDriveOdometry(kSwerveKinematics, getGyroRotation2d(), getModulesPositions(), new Pose2d());

    private final Field2d field = new Field2d();

    private final PhotonVision photonVision = PhotonVision.INSTANCE;

    public GenericEntry maxSpeed = Shuffleboard.getTab("Swerve").add("speedPercent", DRIVE_SPEED_PERCENTAGE).withPosition(2, 0).withSize(2, 2).getEntry();

    private final InterpolatingDoubleTreeMap interpolate = new InterpolatingDoubleTreeMap();

    public boolean estimatePose = true;
    public boolean manualStraighten = false;

    public Swerve() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        resetGyroHardware();

        odometry.resetPosition(getGyroRotation2d(), getModulesPositions(), new Pose2d(0, 0, new Rotation2d()));

        anglePIDcontroller.enableContinuousInput(0, 360);
        anglePIDcontroller.setTolerance(3);

        interpolate.put(1.0, 0.4);
        interpolate.put(-1.0, 1.0);

        initAutoBuilder();
        initShuffleboardData();

        RobotContainer.matchTab.add(setOdometryPositionCommand(new Pose2d(1.3, 5.57, new Rotation2d())));
    }

    // gyro getters and setters
    public void resetGyroHardware() {
        pigeon.reset();
    }

    @Log.NT
    private double getPigeonDegrees() {
        return pigeon.getAngle();
    }

    // odometry getters and setters
    private void setPose2d(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getModulesPositions(), pose);
    }

    public double getDistanceFromPose(Pose2d pose) {
        return getPose2d().getTranslation().getDistance(pose.getTranslation());
    }

    @Log.NT(key = "robotPose")
    public Pose2d getPose2d() {
        return odometry.getEstimatedPosition();
    }

    @Log.NT
    public Rotation2d getGyroRotation2d() {
        return pigeon.getRotation2d();
    }

    public Rotation2d getOdometryRotation2d() {
        return odometry.getEstimatedPosition().getRotation();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kSwerveKinematics.toChassisSpeeds(getModulesStates());
    }

    public Command setOdometryPositionCommand(Pose2d pose) {
        return new InstantCommand(() -> setPose2d(pose)).ignoringDisable(true);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        setModulesStates(kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public Command setOdometryAngleCommand(AlliancePose angle) {
        return new ProxyCommand(() ->
                setOdometryPositionCommand(new Pose2d(
                        odometry.getEstimatedPosition().getTranslation(),
                        angle.get().getRotation())).ignoringDisable(true));
    }

    public Command resetOdometryAngleCommand() {
        return setOdometryAngleCommand(new AlliancePose(0));
    }

    // drive commands
    public Command driveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier spinningSpeedSupplier,
            BooleanSupplier fieldOriented,
            DoubleSupplier decelerator, // credit to @oh_fear (discord) from Trigon 5990
            BooleanSupplier boost,
            Supplier<Pose2d> turnToPose
    ) {
        return straightenModulesCommand().andThen(this.run(
                () -> {
                    int allianceMultiplier = AllianceUtils.isRedAlliance() ? -1 : 1;
                    double speedLimit = boost.getAsBoolean() ? BOOST_SPEED_PERCENTAGE : maxSpeed.getDouble(DRIVE_SPEED_PERCENTAGE);

                    //create the speeds for x,y and spin
                    double xSpeed = xSpeedSupplier.getAsDouble() * MAX_VELOCITY_METER_PER_SECOND / 100 * speedLimit * interpolate.get(decelerator.getAsDouble()) * allianceMultiplier;
                    double ySpeed = ySpeedSupplier.getAsDouble() * MAX_VELOCITY_METER_PER_SECOND / 100 * speedLimit * interpolate.get(decelerator.getAsDouble()) * allianceMultiplier;
                    double spinningSpeed = spinningSpeedSupplier.getAsDouble() * MAX_VELOCITY_METER_PER_SECOND / 100 * speedLimit * interpolate.get(decelerator.getAsDouble());

                    // if a pose was supplied, replace the driver input with a pid calculated value for turning to the given pose
                    if (!turnToPose.get().equals(new Pose2d()))
                        spinningSpeed = anglePIDcontroller.calculate(getPose2d().minus(turnToPose.get()).getRotation().getDegrees());

                    // create a CassisSpeeds object and apply it the speeds
                    ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getOdometryRotation2d()) :
                            new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

                    //use the ChassisSpeedsObject to create an array of SwerveModuleStates
                    SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

                    //apply the array to the swerve modules of the robot
                    setModulesStates(moduleStates);
                }
        ));
    }

    public Command driveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier spinningSpeedSupplier,
            BooleanSupplier fieldOriented) {
        Pose2d emptyPose = new Pose2d();
        return driveSwerveCommand(xSpeedSupplier, ySpeedSupplier, spinningSpeedSupplier, fieldOriented, () -> 1, () -> false, () -> emptyPose);
    }

    public Command straightenModulesCommand() {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    swerveModules[FRONT_LEFT].spinTo(0);
                    swerveModules[FRONT_RIGHT].spinTo(0);
                    swerveModules[BACK_LEFT].spinTo(0);
                    swerveModules[BACK_RIGHT].spinTo(0);
                },
                (__) -> {
                    stopModules();
//                    resetAngleEncoders();
                },
                swerveModules[FRONT_LEFT].isReset
                        .and(swerveModules[FRONT_RIGHT].isReset)
                        .and(swerveModules[BACK_LEFT].isReset)
                        .and(swerveModules[BACK_RIGHT].isReset)
                        .or(new Trigger(() -> manualStraighten)),
                this);
    }

    public Command turnToAngleCommand(double degrees) {
        return new FunctionalCommand(
                () -> relativeOdometry.resetPosition(getGyroRotation2d(), getModulesPositions(), new Pose2d(getPose2d().getTranslation(), new Rotation2d())),
                () -> {
                    relativeOdometry.update(getGyroRotation2d(), getModulesPositions());
                    driveRobotRelative(new ChassisSpeeds(
                            0, 0,
                            anglePIDcontroller.calculate(relativeOdometry.getPoseMeters().getRotation().getDegrees(), -degrees)));
                },
                (__) -> driveRobotRelative(new ChassisSpeeds()),
                new Trigger(anglePIDcontroller::atGoal).debounce(0.2),
                this);
    }

    public Rotation2d getRobotAngleToTranslation(Translation2d translation) {
        return getOdometryRotation2d().minus(translation.minus(getPose2d().getTranslation()).getAngle()).plus(Rotation2d.fromDegrees(180));
    }

    public Command turnToLocationCommand(FieldLocations location) {
        return new ProxyCommand(() -> turnToAngleCommand(getRobotAngleToTranslation(location.pose.get().getTranslation()).getDegrees()));
    }

    // other methods
    public void resetAngleEncoders() {
        foreachModule(SwerveModule::resetEncoders);
    }

    private void stopModules() {
        foreachModule(SwerveModule::stopModule);
    }

    public void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METER_PER_SECOND);
        swerveModules[FRONT_LEFT].setDesiredState(states[FRONT_LEFT]);
        swerveModules[FRONT_RIGHT].setDesiredState(states[FRONT_RIGHT]);
        swerveModules[BACK_LEFT].setDesiredState(states[BACK_LEFT]);
        swerveModules[BACK_RIGHT].setDesiredState(states[BACK_RIGHT]);
    }

    @Log.NT
    public SwerveModulePosition[] getModulesPositions() {
        return new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition(),
        };
    }

    @Log.NT
    public SwerveModuleState[] getModulesStates() {
        return new SwerveModuleState[]{
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState(),
        };
    }

    private void foreachModule(Consumer<SwerveModule> module) {
        for (int i = 0; i < swerveModules.length; i++) {
            module.accept(swerveModules[i]);
        }
    }

    public Command toggleIdleModeCommand() {
        return new StartEndCommand(
                () -> foreachModule(SwerveModule::setIdleModeCoast),
                () -> foreachModule(SwerveModule::setIdleModebreak))
                .ignoringDisable(true);
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getModulesPositions());

        //      localization with PhotonPoseEstimator
        if (estimatePose) {
            Optional<EstimatedRobotPose> pose = photonVision.getEstimatedGlobalPose(odometry.getEstimatedPosition());
            if (pose.isPresent())
                odometry.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);//
        }

        field.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putData(field);
    }

    // on-the-fly auto generation functions
    // drive the robot from the current location to a given Pose2d
    public Command pathPlannerToPose(Pose2d position, double endVel) {
        return new ProxyCommand(() ->
                followPath(endVel, position.getRotation().getDegrees(), getStraightLinePoses(position.getTranslation()))
        );
    }

    private Command followPath(double endVel, double endDegrees, Pose2d... positions) {
        return AutoBuilder.followPath(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(getAlliancePositions(positions)),
                        PATH_CONSTRAINTS,
                        new GoalEndState(endVel, Rotation2d.fromDegrees(endDegrees)))
        );
    }

    private Pose2d[] getAlliancePositions(Pose2d... poses) {
        for (int i = 0; i < poses.length; i++) poses[i] = AllianceUtils.toAlliancePose(poses[i]);
        return poses;
    }

    private Pose2d[] getStraightLinePoses(Translation2d setpoint) {
        Translation2d current = odometry.getEstimatedPosition().getTranslation();
        Rotation2d directionOfTravel = setpoint.minus(current).getAngle();

        return new Pose2d[]{
                new Pose2d(current, directionOfTravel),
                new Pose2d(setpoint, directionOfTravel)
        };
    }

    public Command pidToPose(Pose2d setpoint) {
        return new RunCommand(() -> driveRobotRelative(
                ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                                xTranslationPIDcontroller.calculate(getPose2d().getX(), setpoint.getX()),
                                yTranslationPIDcontroller.calculate(getPose2d().getY(), setpoint.getY()),
                                anglePIDcontroller.calculate(getPose2d().getRotation().getDegrees(), setpoint.getRotation().getDegrees())),
                        getOdometryRotation2d())), this);
    }

    public Command pathFindToLocation(FieldLocations location) {
        return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(location.pathName), PATH_CONSTRAINTS);
    }

    public Command pathFindToPose(Pose2d pose, PathConstraints constraints, double goalEndVel) {
        return AutoBuilder.pathfindToPose(pose, constraints, goalEndVel);
    }

    public Command pathFindToPose(Pose2d pose) {
        return pathFindToPose(pose, PATH_CONSTRAINTS, 0);
    }

    public Command shootInMotionCommand() {
        PathPlannerPath topPath = PathPlannerPath.fromPathFile("ShootInMotionTop");
        PathPlannerPath bottomPath = PathPlannerPath.fromPathFile("ShootInMotionBottom");

        return new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(topPath, PATH_CONSTRAINTS),
                AutoBuilder.pathfindThenFollowPath(bottomPath, PATH_CONSTRAINTS),
                () -> getDistanceFromPose(topPath.getStartingDifferentialPose()) < getDistanceFromPose(bottomPath.getStartingDifferentialPose())
        );
    }

    public Command runAuto(String autoName) {
        return new SequentialCommandGroup(
                straightenModulesCommand(),
                setOdometryPositionCommand(PathPlannerAuto.getStaringPoseFromAutoFile(autoName)),
                new PathPlannerAuto(autoName));
    }

    public Command pathFindThenFollowPath(String pathName) {
        return AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(pathName),
                new PathConstraints(0, 0, 0, 0));
    }

    public Command estimatePoseCommand() {
        return new RunCommand(() -> {
            Optional<EstimatedRobotPose> pose = photonVision.getEstimatedGlobalPose(odometry.getEstimatedPosition());
            if (pose.isPresent())
                odometry.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
        });
    }
    // ----------

    private void initShuffleboardData() {
        var swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.add("FL", swerveModules[FRONT_LEFT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(4, 0).withSize(4, 4);
        swerveTab.add("FR", swerveModules[FRONT_RIGHT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(8, 0).withSize(4, 4);
        swerveTab.add("BL", swerveModules[BACK_LEFT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(4, 4).withSize(4, 4);
        swerveTab.add("BR", swerveModules[BACK_RIGHT]).withWidget(BuiltInWidgets.kGyro)
                .withPosition(8, 4).withSize(4, 4);
        swerveTab.addDouble("SwerveAngle", () -> getOdometryRotation2d().getDegrees()).withWidget(BuiltInWidgets.kGyro)
                .withPosition(0, 2).withSize(4, 4);
        swerveTab.add("Field2d", field).withSize(9, 5).withPosition(12, 0);
        swerveTab.add("manual straighten", new InstantCommand(() -> manualStraighten = !manualStraighten));
    }

    private void initAutoBuilder() {
        AutoBuilder.configureHolonomic(
                this::getPose2d,
                (pose) -> {},
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(PATHPLANNER_TRANSLATION_GAINS.kp, 0.0, PATHPLANNER_TRANSLATION_GAINS.kd),
                        new PIDConstants(PATHPLANNER_ANGLE_GAINS.kp, 0.0, PATHPLANNER_ANGLE_GAINS.kd),
                        MAX_VELOCITY_METER_PER_SECOND,
                        Math.sqrt(2) * (TRACK_WIDTH / 2), // needs to change for a non-square swerve
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().get().equals(Red), this
        );
    }

//    public Command adjustTo(FieldParts fieldPart){
//        int id = AllianceUtils.isRedAlliance()? fieldPart.redID : fieldPart.blueID;
//        PhotonPipelineResult results = camera.getLatestResult();
//        if (!results.hasTargets()) return new PrintCommand("target "+ id +"not found");
//        List<PhotonTrackedTarget> targets = results.getTargets();
//        PhotonTrackedTarget desiredTarget = null;
//        for (PhotonTrackedTarget target : targets){
//            if (target.getFiducialId() == id) {
//                desiredTarget = target;
//                break;
//            }
//        }
//        Transform3d cameraToTarget = desiredTarget.getBestCameraToTarget();
//        Transform3d TargetToRobot;
//        return null;//TODO get the diffrence between the required transform to the given transform and driving it using pp
//    }
//    private Transform3d targetToRobot(Transform3d cameraToTarget){
//        return null;//TODO write a function that receives the transform of the camera to a target, returns the transform of the robot to the target
//    }
}
