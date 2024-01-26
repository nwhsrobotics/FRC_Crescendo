package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the swerve drive subsystem, managing four swerve modules and handling overall robot control.
 */
public class SwerveSubsystem extends SubsystemBase {
    // when the robot is set to "field relative,"
    // linear movement will be relative to the field.
    //
    // for example, even if the robot is oriented towards the driver station,
    // holding forwards will move the robot away from the driver station,
    // because that is the forwards direction relative to the field.
    public boolean isFieldRelative = true;
    public String lastPath;
    public Pose2d lastPose2d;
    public String lastPathType;

    public PenguinLogistics autonavigator;

    // 4 instances of SwerveModule to represent each wheel module with the constants
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    // array of SwerveModules for convenience in accessing all modules
    public final SwerveModule[] swerveMods = {frontLeft, frontRight, backLeft, backRight};

    // create an AHRS object for gyro
    public final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

    // create a SwerveDriveOdometry object to handle odometry calculations
    /*private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getHeading()), getModulePositions());*/

    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getHeading()),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(18)),
        VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(162)));

    private final PathConstraints kPathfindingConstraints = new PathConstraints(
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 8.0,
        AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared / 2.0);

    /**
     * Constructor for the SwerveSubsystem class.
     * Configures the AutoBuilder for holonomic/swerve path planning and initializes the gyro.
     */
    public SwerveSubsystem() {
        // Configure AutoBuilder for holonomic/swerve path planning & paths
        AutoBuilder.configureHolonomic(
                this::getPose,               // Supplier for getting the robot's pose
                this::resetOdometry,         // Runnable for resetting odometry
                this::getSpeeds,             // Supplier for getting the robot's speeds
                this::driveRobotRelative,    // Consumer for driving the robot relative to its orientation
                AutoConstants.pathFollowerConfig, // Path follower configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );

        this.autonavigator = new PenguinLogistics(this);

        // Pause for 500 milliseconds to allow the gyro to stabilize.
        // Set the yaw of the gyro to 0 afterwards.
        Commands.waitSeconds(0.5)
                .andThen(new RunCommand(() -> m_gyro.zeroYaw()));
    }

    /**
     * Set the speed of the robot in the x direction.
     *
     * @param xSpeed The desired speed in the x direction.
     */
    public void setSpeed(double xSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0.0, 0.0);
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Get the heading (yaw) of the robot.
     *
     * @return The heading of the robot in degrees.
     */
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * -1;
    }

    /**
     * Get the pitch of the robot in degrees.
     *
     * @return The pitch of the robot in degrees.
     */
    public double getPitchDeg() {
        return -m_gyro.getPitch();
    }

    /**
     * Drive the robot relative to its current orientation.
     *
     * @param robotRelativeSpeeds The desired robot-relative speeds.
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /**
     * Reset the heading (yaw) and the odometry pose of the robot.
     */
    public void resetHeadingAndPose() {
        m_gyro.zeroYaw(); // Reset the yaw angle
        resetOdometry(new Pose2d()); // Reset the robot's odometry pose
    }

    /**
     * Switch between field-relative and robot-relative driving.
     */
    public void switchFR() {
        isFieldRelative = !isFieldRelative; // switch between field-relative and robot-relative driving
    }

    /**
     * Get the speeds of the robot.
     *
     * @return The speeds of the robot.
     */
    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Get the current pose of the robot in meters.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return odometer.getEstimatedPosition(); // get the robot's current pose from the odometry system
    }

    /**
     * Reset the odometry with the specified pose.
     *
     * @param pose The desired pose for resetting odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // Reset the drive encoder positions on all four swerve modules
        // TODO: Remove this.
        //
        // why?
        // well, re-enabling a second time resets odometry to (0, 0).
        // these two lines break odometry testing.
        //
        // resetPosition is passed module positions, it already compensates for the reset.
        /*
        for (SwerveModule sModule : swerveMods)
            sModule.driveEncoder.setPosition(0);
        */

        // Reset the odometry system using the current heading, module positions, and specified pose
        odometer.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), pose);
    }

    /**
     * Straighten the orientation of each swerve module.
     */
    public void straighten() {
        // Turn each swerve module to point straight ahead
        for (SwerveModule s_mod : swerveMods) {
            s_mod.turningMotor.set(s_mod.turningPidController.calculate(s_mod.getAbsoluteEncoderRad(), 0)); // use PID control to turn to the desired angle
            s_mod.turningMotor.set(0); // Stop turning once the desired angle is reached
        }
    }

    /**
     * Get the states of all swerve modules.
     *
     * @return The states of all swerve modules.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < swerveMods.length; i++) {
            states[i] = swerveMods[i].getState();
        }
        return states;
    }

    /**
     * Get the positions of all swerve modules.
     *
     * @return The positions of all swerve modules.
     */
    public SwerveModulePosition[] getModulePositions() {
        // Get the current position of each swerve module
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveMods.length; i++) {
            positions[i] = swerveMods[i].getPosition();
        }
        return positions;
    }

    /**
     * Finds a path and follows it based on the specified path name.
     * Loads the path from a file, sets constraints, and uses AutoBuilder to create a pathfinding command.
     *
     * @param pathName The name of the path file to load and follow.
     */
    public void pathFindThenFollowPath(String pathName) {
        //lastPath = pathName;
        //lastPathType = "Path";

        // Load the path we want to pathfind to and follow
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 8.0,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared / 2.0);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // What pathfinding does is pathfind to the start of a path and then continue along that path.
        // If you don't want to continue along the path, you can make it pathfind to a specific location.
        /*
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                2.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        pathfindingCommand.schedule();
        */
    }

    /**
     * Run pathfinding to given position.
     * 
     * @param position - position to pathfind to.
     * @return - scheduled pathfinding command.
     */
    public Command pathfindToPosition(Pose2d position) {
        Command command = AutoBuilder.pathfindToPose(
            position,
            this.kPathfindingConstraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        command.schedule();

        return command;
    }

    // This method is called periodically to update the robot's state and log data
    @Override
    public void periodic() {
        // Update the robot's odometer
        //odometer.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());
        odometer.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(getHeading()), getModulePositions());
        //var alliance = DriverStation.getAlliance();
        /*if (alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red){
                odometer.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiRed("Limelight"), Timer.getFPGATimestamp());
            } else {
                odometer.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue("Limelight"), Timer.getFPGATimestamp());
            }
        }
        //stddevs should be scaled to improve accuracy https://www.chiefdelphi.com/t/poseestimators-and-limelight-botpose/430334/3
        //https://tools.limelightvision.io/map-builder
        //https://www.chiefdelphi.com/t/is-there-a-2024-limelight-release/449411/11
        //upload the april tag build map to limelight (unofficial) https://www.chiefdelphi.com/uploads/short-url/1j60M9TmbpUb74e8YjAdMxyc2ew.zip (.fmap in it) also have uploaded it in robot directly
        LimelightHelpers.Results result =
        LimelightHelpers.getLatestResults("limelight").targetingResults;
        if (!(result.botpose[0] == 0 && result.botpose[1] == 0) && alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                //LimelightHelpers.toPose2D(result.botpose_wpiblue)
            odometer.addVisionMeasurement(
                result.getBotPose2d_wpiBlue(),
                Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) - (result.latency_pipeline / 1000.0));
            } else if (alliance.get() == Alliance.Red) {
            // double[] botpose = LimelightHelpers.getBotPose_wpiRed("limelight");
            odometer.addVisionMeasurement(
                LimelightHelpers.toPose2D(result.botpose_wpired),
                Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) - (result.latency_pipeline / 1000.0));
            }
        }*/

        // Log position of robot.
        Logger.recordOutput("swerve.pose", getPose());

        // Log whether robot is driving in field relative mode.
        Logger.recordOutput("swerve.isfieldrelative", isFieldRelative);

        // Log pitch and heading of IMU.
        Logger.recordOutput("swerve.pitch", getPitchDeg());
        Logger.recordOutput("swerve.heading", getHeading());

        // Log steering direction.
        Logger.recordOutput("swerve.steer.front.left.abs", frontLeft.getAbsoluteEncoderRad());
        Logger.recordOutput("swerve.steer.front.right.abs", frontRight.getAbsoluteEncoderRad());
        Logger.recordOutput("swerve.steer.back.left.abs", backLeft.getAbsoluteEncoderRad());
        Logger.recordOutput("swerve.steer.back.right.abs", backRight.getAbsoluteEncoderRad());

        // Log travel velocity.
        Logger.recordOutput("swerve.drive.front.left.velocity", frontLeft.getDriveVelocity());
        Logger.recordOutput("swerve.drive.front.right.velocity", frontRight.getDriveVelocity());
        Logger.recordOutput("swerve.drive.back.left.velocity", backLeft.getDriveVelocity());
        Logger.recordOutput("swerve.drive.back.right.velocity", backRight.getDriveVelocity());
    }

    /**
     * Stops all of the robot's swerve modules.
     * Iterates through each swerve module and stops its motion.
     */
    public void stopModules() {
        for (SwerveModule sMod : swerveMods) {
            sMod.stop();
        }
    }

    /**
     * Sets the states of the robot's swerve modules based on the desired states.
     * Scales all speeds down instead of truncating them if they exceed the max speed.
     *
     * @param desiredStates An array of SwerveModuleState representing the desired states for each swerve module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Scales all speeds down instead of truncating them if they exceed the max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Check if the robot is in a clover state.
     *
     * @return True if the robot is in a cloverleaf configuration, false otherwise.
     */
    public boolean isClover() {
        return frontLeft.getTurningPositionWrapped() > Math.PI / 4 - Math.toRadians(1.0) && frontLeft.getTurningPositionWrapped() < Math.PI / 4 + Math.toRadians(1.0)
                && frontRight.getTurningPositionWrapped() > -Math.PI / 4 - Math.toRadians(1.0) && frontRight.getTurningPositionWrapped() < -Math.PI / 4 + Math.toRadians(1.0)
                && backLeft.getTurningPositionWrapped() > -Math.PI / 4 - Math.toRadians(1.0) && backLeft.getTurningPositionWrapped() < -Math.PI / 4 + Math.toRadians(1.0)
                && backRight.getTurningPositionWrapped() > Math.PI / 4 - Math.toRadians(1.0) && backRight.getTurningPositionWrapped() < Math.PI / 4 + Math.toRadians(1.0);
    }

    /**
     * Orient wheels into a "clover" formation, in order to brake.
     */
    public void brake() {
        stopModules();

        // Sets the turning motors to their braking positions
        frontLeft.turningMotor.set(frontLeft.turningPidController.calculate(frontLeft.getTurningPosition(), Math.PI / 4));
        frontRight.turningMotor.set(frontRight.turningPidController.calculate(frontRight.getTurningPosition(), -Math.PI / 4));
        backLeft.turningMotor.set(backLeft.turningPidController.calculate(backLeft.getTurningPosition(), -Math.PI / 4));
        backRight.turningMotor.set(backRight.turningPidController.calculate(backRight.getTurningPosition(), Math.PI / 4));
    }
}