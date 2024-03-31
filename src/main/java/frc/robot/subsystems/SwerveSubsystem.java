package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.limelight.LimelightHelpers;

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

    public AutoNavigation autonavigator;

    // 4 instances of SwerveModule to represent each wheel module with the constants
    public final SwerveModule frontLeft = new SwerveModule(
            CANAssignments.FRONT_LEFT_DRIVE_MOTOR_ID,
            CANAssignments.FRONT_LEFT_STEER_MOTOR_ID,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            CANAssignments.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule frontRight = new SwerveModule(
            CANAssignments.FRONT_RIGHT_DRIVE_MOTOR_ID,
            CANAssignments.FRONT_RIGHT_STEER_MOTOR_ID,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            CANAssignments.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            CANAssignments.BACK_LEFT_DRIVE_MOTOR_ID,
            CANAssignments.BACK_LEFT_STEER_MOTOR_ID,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            CANAssignments.BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule backRight = new SwerveModule(
            CANAssignments.BACK_RIGHT_DRIVE_MOTOR_ID,
            CANAssignments.BACK_RIGHT_STEER_MOTOR_ID,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            CANAssignments.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    // array of SwerveModules for convenience in accessing all modules
    public final SwerveModule[] swerveMods = {frontLeft, frontRight, backLeft, backRight};

    // create an AHRS object for gyro
    public final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    //odometry is a system to keep track of robots current position and rotation on the fields based on the coordinate system
    public final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getHeading()),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5)));
    //The default standard deviations of the module states are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading. 
    //The default standard deviations of the vision measurements are 0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
    //Decrease standard deviations to trust the data more (right now the vision is mostly insignificant compared to module state)

    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double prevTime = WPIUtilJNI.now() * 1e-6;

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

        this.autonavigator = new AutoNavigation(this);

        // Pause for 500 milliseconds to allow the gyro to stabilize.
        // Set the yaw of the gyro to 0 afterwards.
        // TODO: Do we need this anymore? Might cause conflicts with path planner
        //gyro.reset();
        Commands.waitUntil(() -> !gyro.isCalibrating()).andThen(new InstantCommand(() -> gyro.zeroYaw()));
        /*Commands.waitSeconds(0.5)
                .andThen(new RunCommand(() -> gyro.zeroYaw()));*/
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
        return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
    }

    /**
     * Get the pitch of the robot in degrees.
     *
     * @return The pitch of the robot in degrees.
     */
    public double getPitchDeg() {
        return -gyro.getPitch();
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
    public void resetHeading() {
        gyro.zeroYaw(); // Reset the yaw angle
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
     * MAKE SURE TO FLIP POSTION IF NEEDED GeometryUtil.flipFieldPose
     * <p>
     * Reset the odometry with the specified pose.
     *
     * @param pose The desired pose for resetting odometry.
     */
    public void resetOdometry(Pose2d pose) {
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
        Command pathfindingCommand;
        // Load the path we want to pathfind to and follow
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 8.0,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared / 2.0);

        // What pathfinding does is pathfind to the start of a path and then continue along that path.
        // If you don't want to continue along the path, you can make it pathfind to a specific location.
        
        pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        pathfindingCommand.schedule();
        
    }

    /**
     * Run pathfinding to given position.
     *
     * @param position - position to pathfind to.
     * @return - scheduled pathfinding command.
     */
    public Command pathfindToPosition(Pose2d position) {
        Command command = AutoBuilder.pathfindToPoseFlipped(
                position,
                OIConstants.kPathfindingConstraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        command.addRequirements(this);
        command.schedule();

        return command;
    }

    // This method is called periodically to update the robot's state and log data
    @Override
    public void periodic() {
        updateOdometry();

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

        Logger.recordOutput("swerve.steer.front.left.absraw", frontLeft.getAbsoluteEncoderRadRaw());
        Logger.recordOutput("swerve.steer.front.right.absraw", frontRight.getAbsoluteEncoderRadRaw());
        Logger.recordOutput("swerve.steer.back.left.absraw", backLeft.getAbsoluteEncoderRadRaw());
        Logger.recordOutput("swerve.steer.back.right.absraw", backRight.getAbsoluteEncoderRadRaw());

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
     * Orient wheels into a X (clover) position to prevent movement
     */
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Update odometry periodically with vision and internal measurements
     */
    public void updateOdometry() {
        // Update the robot's odometer
        //odometer.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(getHeading()), getModulePositions());
        odometer.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());
        //stddevs should be scaled to improve accuracy https://www.chiefdelphi.com/t/poseestimators-and-limelight-botpose/430334/3
        //LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        double poseDifference = odometer.getEstimatedPosition().getTranslation().getDistance(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose.getTranslation());
        double dist = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").avgTagDist;
        double xyStds = 0.9;
        double degStds = 0.9;
        if (limelightMeasurement.tagCount >= 2) {
            if(dist < 1.0){
                xyStds = 0.1;
                degStds = 1;
            } else if (dist < 2.0){
                xyStds = 0.2;
                degStds = 2;
            } else if (dist < 3.0){
                xyStds = 0.3;
                degStds = 3;
            } else if (dist < 4.0){
                xyStds = 0.4;
                degStds = 4;
            } else {
                xyStds = 0.5;
                degStds = 6;
            }
        } else {
            if (limelightMeasurement.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            } else if (limelightMeasurement.avgTagArea > 0.1 && poseDifference < 0.3) {
                // 1 target farther away and estimated pose is close
                xyStds = 2.0;
                degStds = 30;
            }
        }
        //odometer.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        odometer.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
        odometer.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds);
        Logger.recordOutput("swerve.odometer", odometer.getEstimatedPosition());
        Logger.recordOutput("swerve.odometer.xCoordinate", odometer.getEstimatedPosition().getX());
        Logger.recordOutput("swerve.odometer.yCoordinate", odometer.getEstimatedPosition().getY());
        Logger.recordOutput("swerve.odometer.rotation", odometer.getEstimatedPosition().getRotation().getDegrees());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
        // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        if (currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
        } else {
            directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        }
        

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - prevTime;
        double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
        if (angleDif < 0.45*Math.PI) {
            currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
        else if (angleDif > 0.85*Math.PI) {
            if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
            // keep currentTranslationDir unchanged
            currentTranslationMag = magLimiter.calculate(0.0);
            }
            else {
            currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
            currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            }
        }
        else {
            currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            currentTranslationMag = magLimiter.calculate(0.0);
        }
        prevTime = currentTime;
        
        xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
        ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
        currentRotation = rotLimiter.calculate(rot);


        } else {
        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;
        currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double rotDelivered = currentRotation * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, odometer.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Update odometry using precise vision measurements with high accuracy.
     * <p>
     * It is RECOMMENDED to stand still and be close to the April tag when resetting this way as it solely relies on vision
     */
    public void resetOdometryWithVision() {
        int pipeline = (int) LimelightHelpers.getCurrentPipelineIndex("limelight");
        //set the pipeline index to the high resolution april tag (less fps but high accuracy)
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        odometer.setVisionMeasurementStdDevs(VecBuilder.fill(0, 0, Units.degreesToRadians(0)));
        odometer.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        //set back to normal april tag pipeline
        LimelightHelpers.setPipelineIndex("limelight", pipeline);
        //use this logger key to log important evvents 
        Logger.recordOutput("robot.events", "ResetOdometryWithVision");
    }

}