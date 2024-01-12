package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.util.AllianceUtilities;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.SwerveConstants.Modules.*;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveModules = {
            new SwerveModule(
                    Modules.FL.DRIVE_MOTOR_ID,
                    Modules.FL.SPIN_MOTOR_ID,
                    Modules.FL.DRIVE_MOTOR_REVERSED,
                    Modules.FL.SPIN_MOTOR_REVERSED,
                    Modules.FL.ABS_ENCODER_CHANNEL,
                    Modules.FL.OFFSET_ANGLE),
            new SwerveModule(
                    Modules.FR.DRIVE_MOTOR_ID,
                    Modules.FR.SPIN_MOTOR_ID,
                    Modules.FR.DRIVE_MOTOR_REVERSED,
                    Modules.FR.SPIN_MOTOR_REVERSED,
                    Modules.FR.ABS_ENCODER_CHANNEL,
                    Modules.FR.OFFSET_ANGLE),
            new SwerveModule(
                    Modules.BL.DRIVE_MOTOR_ID,
                    Modules.BL.SPIN_MOTOR_ID,
                    Modules.BL.DRIVE_MOTOR_REVERSED,
                    Modules.BL.SPIN_MOTOR_REVERSED,
                    Modules.BL.ABS_ENCODER_CHANNEL,
                    Modules.BL.OFFSET_ANGLE),
            new SwerveModule(
                    Modules.BR.DRIVE_MOTOR_ID,
                    Modules.BR.SPIN_MOTOR_ID,
                    Modules.BR.DRIVE_MOTOR_REVERSED,
                    Modules.BR.SPIN_MOTOR_REVERSED,
                    Modules.BR.ABS_ENCODER_CHANNEL,
                    Modules.BR.OFFSET_ANGLE)};

    private boolean hasStraighten = false;

    private final AHRS _gyro = new AHRS(SPI.Port.kMXP);

    private final ProfiledPIDController anglePIDcontroller = new ProfiledPIDController(
            ANGLE_GAINS.kp, ANGLE_GAINS.ki, ANGLE_GAINS.kd,
            new TrapezoidProfile.Constraints(MAX_VELOCITY_METER_PER_SECOND, MAX_VELOCITY_ACCELERATION_METER_PER_SECOND));

    private final PIDController xTranslationPIDcontroller = new PIDController(PATHPLANNER_TRANSLATION_GAINS.kp, PATHPLANNER_TRANSLATION_GAINS.ki, PATHPLANNER_TRANSLATION_GAINS.kd);
    private final PIDController yTranslationPIDcontroller = new PIDController(PATHPLANNER_TRANSLATION_GAINS.kp, PATHPLANNER_TRANSLATION_GAINS.ki, PATHPLANNER_TRANSLATION_GAINS.kd);

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            kSwerveKinematics,
            getGyroRotation2d(),
            getModulesPositions(),
            new Pose2d());

    private final Field2d field = new Field2d();

//    private final Limelight limelight = Limelight.INSTANCE;

    private GenericEntry maxSpeed = Shuffleboard.getTab("Swerve").add("speedPercent", DRIVE_SPEED_PERCENTAGE).withPosition(2, 0).withSize(2, 2).getEntry();

    private final InterpolatingTreeMap interpolate = new InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble());

    private boolean isClosedloop = false;

    public Swerve() {
        resetGyroHardware();

        odometry.resetPosition(getGyroRotation2d(), getModulesPositions(), new Pose2d(0, 0, new Rotation2d()));
        resetOdometryAngleCommand();

        anglePIDcontroller.enableContinuousInput(0, 360);
        anglePIDcontroller.setTolerance(1);

        interpolate.put(1.0, 0.2);
        interpolate.put(-1.0, 1.0);

        initAutoBuilder();
        initShuffleboardData();
    }

    // gyro getters and setters
    public void resetGyroHardware() {
        _gyro.reset();
    }

    private double getGyroDegrees() {
        return Math.IEEEremainder(_gyro.getAngle(), 360);
    }

    public double getRobotPitch() {
        return _gyro.getRoll() - 0.46;
    }

    // odometry getters and setters
    private void setPose2d(Pose2d pose) {
        odometry.resetPosition(getGyroRotation2d(), getModulesPositions(), pose);
    }

    public Pose2d getPose2d() {
        return odometry.getEstimatedPosition();
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getGyroDegrees()).times(-1);
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

    private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        setModulesStates(kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public Command setOdometryAngleCommand(double angle) {
        return new ProxyCommand(() ->
                setOdometryPositionCommand(
                        new Pose2d(
                                odometry.getEstimatedPosition().getTranslation(),
                        Rotation2d.fromDegrees(angle)))
                        .ignoringDisable(true));
    }

    public Command resetOdometryAngleCommand() {
        return setOdometryAngleCommand(0);
    }

    // drive commands
    public Command driveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier spinningSpeedSupplier,
            BooleanSupplier fieldOriented,
            DoubleSupplier decelerator) {

        return new ConditionalCommand(new InstantCommand(), straightenModulesCommand(), () -> hasStraighten)
                .andThen(
                        this.runEnd(
                                () -> {
                                    double xSpeed = xSpeedSupplier.getAsDouble(), ySpeed = ySpeedSupplier.getAsDouble(), spinningSpeed = spinningSpeedSupplier.getAsDouble();

                                    if (!isClosedloop) {
                                        //create the speeds for x,y and spin
                                        xSpeed = xSpeedSupplier.getAsDouble() * MAX_VELOCITY_METER_PER_SECOND / 100 * maxSpeed.getDouble(DRIVE_SPEED_PERCENTAGE) * (double) interpolate.get(decelerator.getAsDouble());
                                        ySpeed = ySpeedSupplier.getAsDouble() * MAX_VELOCITY_METER_PER_SECOND / 100 * maxSpeed.getDouble(DRIVE_SPEED_PERCENTAGE) * (double) interpolate.get(decelerator.getAsDouble());
                                        spinningSpeed = spinningSpeedSupplier.getAsDouble() * MAX_VELOCITY_METER_PER_SECOND / 100 * maxSpeed.getDouble(DRIVE_SPEED_PERCENTAGE) * (double) interpolate.get(decelerator.getAsDouble());
                                    }

                                    // create a CassisSpeeds object and apply it the speeds
                                    ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                                            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getOdometryRotation2d()) :
                                            new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

                                    //use the ChassisSpeedsObject to create an array of SwerveModuleStates
                                    SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

                                    //apply the array to the swerve modules of the robot
                                    setModulesStates(moduleStates);
                                },
                                this::stopModules
                        ));
    }

    public Command driveSwerveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier spinningSpeedSupplier,
            BooleanSupplier fieldOriented) {
        return driveSwerveCommand(xSpeedSupplier, ySpeedSupplier, spinningSpeedSupplier, fieldOriented, () -> 1);
    }

    public Command straightenModulesCommand() {
        return new FunctionalCommand(
                () -> hasStraighten = true,
                () -> {
                    swerveModules[FRONT_LEFT].spinTo(0);
                    swerveModules[FRONT_RIGHT].spinTo(0);
                    swerveModules[BACK_LEFT].spinTo(0);
                    swerveModules[BACK_RIGHT].spinTo(0);
                },
                (__) -> {
                    stopModules();
                    resetAngleEncoders();
                },
                swerveModules[FRONT_LEFT].isReset
                        .and(swerveModules[FRONT_RIGHT].isReset)
                        .and(swerveModules[BACK_LEFT].isReset)
                        .and(swerveModules[BACK_RIGHT].isReset),
                this);
    }

    public Command turnToAngleCommand(double setpoint) {
        return setClosedLoop(true).andThen(
                driveSwerveCommand(
                        () -> 0, () -> 0,
                        () -> anglePIDcontroller.calculate(getOdometryRotation2d().getDegrees(), setpoint),
                        () -> false).until(new Trigger(anglePIDcontroller::atSetpoint).debounce(0.1)),
                setClosedLoop(false));
    }

    // other methods
    private void resetAngleEncoders() {
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

    public SwerveModulePosition[] getModulesPositions() {
        return new SwerveModulePosition[]{
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition(),
        };
    }

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

    public Command setClosedLoop(boolean isCloseLoop) {
        return new InstantCommand(() -> this.isClosedloop = isCloseLoop);
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getModulesPositions());

        // localization with PhotonPoseEstimator
//        Optional<EstimatedRobotPose> pose = limelight.getEstimatedGlobalPose(odometry.getEstimatedPosition());
//        if (pose.isPresent()) odometry.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);

        // localization with SwervePoseEstimator
//        if (limelight.getLatestResualt().hasTargets()) limelight.updateFromAprilTagPose(odometry::addVisionMeasurement);

        field.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putData(field);
    }

    // on-the-fly auto generation functions
    public Command followPath(double endVel, double endDegrees, Pose2d... positions) {
        return AutoBuilder.followPath(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(getAlliancePositions(positions)),
                        PATH_CONSTRAINTS,
                        new GoalEndState(endVel, Rotation2d.fromDegrees(endDegrees)))
        );
    }

    // drives the robot from current location to a given Pose2d
    public Command pathPlannerToPose(Pose2d position, double endVel) {
        return new ProxyCommand(() ->
                followPath(endVel, position.getRotation().getDegrees(),
                        getStraightLinePoses(position.getTranslation()))
        );
    }

    private Pose2d[] getAlliancePositions(Pose2d... poses) {
        for (int i = 0; i < poses.length; i++) {
            poses[i] = AllianceUtilities.toAlliancePose(poses[i]);
            System.out.println(poses[i]);
        }
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
        Trigger atSetpointTrigger = new Trigger(() ->
                getPose2d().getTranslation().getDistance(setpoint.getTranslation()) < 0.05 &&
                        Math.abs(getPose2d().getRotation().minus(setpoint.getRotation()).getDegrees()) < 3).debounce(0.25);
        double maxVel = 0.5;

        return new RunCommand(() -> driveRobotRelative(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                                MathUtil.clamp(xTranslationPIDcontroller.calculate(getPose2d().getX(), setpoint.getX()), -maxVel, maxVel),
                                MathUtil.clamp(yTranslationPIDcontroller.calculate(getPose2d().getY(), setpoint.getY()), -maxVel, maxVel),
                                MathUtil.clamp(anglePIDcontroller.calculate(getPose2d().getRotation().getDegrees(), setpoint.getRotation().getDegrees()), -maxVel, maxVel)),
                        getGyroRotation2d())),
                this).until(atSetpointTrigger);
    }

    public Command pathFindToLocation(FieldLocations location){
        return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(location.pathName), PATH_CONSTRAINTS);
    }

    public Command pathFindToPose(Pose2d pose, PathConstraints constraints, double goalEndVel){
        return AutoBuilder.pathfindToPose(pose, constraints, goalEndVel);
    }

    public Command pathFindToPose(Pose2d pose){
        return pathFindToPose(pose, PATH_CONSTRAINTS, 0);
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
        swerveTab.addDouble("robotPitch", this::getRobotPitch);
    }

    private void initAutoBuilder(){
        AutoBuilder.configureHolonomic(
                this::getPose2d,
                this::setPose2d,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(PATHPLANNER_TRANSLATION_GAINS.kp, 0.0, PATHPLANNER_TRANSLATION_GAINS.kd),
                        new PIDConstants(PATHPLANNER_ANGLE_GAINS.kp, 0.0, PATHPLANNER_ANGLE_GAINS.kd),
                        MAX_VELOCITY_METER_PER_SECOND,
                        Math.sqrt(2) * (TRACK_WIDTH / 2), // needs to change for a non-square swerve
                        new ReplanningConfig()),
                AllianceUtilities::isRedAlliance, this
        );
    }
}
