package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/**
 * Represents the swerve drive subsystem, managing four swerve modules and handling overall robot control.
 */
public class SwerveSubsystem extends SubsystemBase {

    // boolean variable to indicate if the robot is Field Relative
    public boolean isFR = true;
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
    public final SwerveModule[] swerveMods = { frontLeft, frontRight, backLeft, backRight };

    // create an AHRS object for gyro
    public final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

    // create a SwerveDriveOdometry object to handle odometry calculations
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getHeading()), getModulePositions());

    // create a Logger object for logging
    //public Logger logger = Logger.getInstance();

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
            Constants.pathFollowerConfig, // Path follower configuration
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

        // Pause for 500 milliseconds to allow the gyro to stabilize
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            // Handle interrupted exception
        }

        // Set the yaw of the gyro to 0
        m_gyro.zeroYaw();
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
        isFR = !isFR; // switch between field-relative and robot-relative driving
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
        return odometer.getPoseMeters(); // get the robot's current pose from the odometry system
    }

    /**
     * Reset the odometry with the specified pose.
     *
     * @param pose The desired pose for resetting odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // Reset the drive encoder positions on all four swerve modules
        for (SwerveModule sModule : swerveMods)
            sModule.driveEncoder.setPosition(0);

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
     * Get a trajectory command for the specified trajectory.
     *
     * @param traj The trajectory to follow.
     * @return The swerve controller command for the trajectory.
     */
    public SwerveControllerCommand getTrajCmd(Trajectory traj) {
        // Create a PID controller for the heading (yaw) error
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create a new swerve controller command using the specified trajectory, odometry, and controllers
        return new SwerveControllerCommand(
                traj,
                this::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                this::setModuleStates,
                this);
    }

    // This method is called periodically to update the robot's state and log data
    @Override
    public void periodic() {
        // Log whether the robot is field-relative or not
        SmartDashboard.putBoolean("FIELD RELATIVE?", isFR);
        SmartDashboard.putNumber("FL abs", frontLeft.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("FR ABS", frontRight.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("BL ABS", backLeft.getAbsoluteEncoderRadRaw());
        SmartDashboard.putNumber("BR ABS", backRight.getAbsoluteEncoderRadRaw());

        // Update the robot's odometer
        odometer.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        // Log various data points
        /*
        logger.recordOutput("swerve.pitch", getPitchDeg());
        logger.recordOutput("swerve.steer.front.left.abs", frontLeft.getAbsoluteEncoderRad());
        logger.recordOutput("swerve.steer.front.right.abs", frontRight.getAbsoluteEncoderRad());
        logger.recordOutput("swerve.steer.back.left.abs", backLeft.getAbsoluteEncoderRad());
        logger.recordOutput("swerve.steer.back.right.abs", backRight.getAbsoluteEncoderRad());
        logger.recordOutput("swerve.pose", getPose());
        logger.recordOutput("swerve.heading", getHeading());
        logger.recordOutput("swerve.drive.front.left.velocity", frontLeft.getDriveVelocity());
        logger.recordOutput("swerve.drive.front.right.velocity", frontRight.getDriveVelocity());
        logger.recordOutput("swerve.drive.back.left.velocity", backLeft.getDriveVelocity());
        logger.recordOutput("swerve.drive.back.right.velocity", backRight.getDriveVelocity());*/
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
    public boolean isClover(){
        return frontLeft.getTurningPositionWrapped() > Math.PI/4 - Math.toRadians(1.0) && frontLeft.getTurningPositionWrapped() < Math.PI/4 + Math.toRadians(1.0)
        && frontLeft.getTurningPositionWrapped() > Math.PI/4 - Math.toRadians(1.0) && frontLeft.getTurningPositionWrapped() < Math.PI/4 + Math.toRadians(1.0)
        && frontRight.getTurningPositionWrapped() > -Math.PI/4 - Math.toRadians(1.0) && frontRight.getTurningPositionWrapped() < -Math.PI/4 + Math.toRadians(1.0)
        && backLeft.getTurningPositionWrapped() > -Math.PI/4 - Math.toRadians(1.0) && backLeft.getTurningPositionWrapped() < -Math.PI/4 + Math.toRadians(1.0)
        && backRight.getTurningPositionWrapped() > Math.PI/4 - Math.toRadians(1.0) && backRight.getTurningPositionWrapped() < Math.PI/4 + Math.toRadians(1.0);
    }

    /**
     * Activate the robot's brakes/clover.
     */
    public void brake() {
        // Stops all swerve modules
        for (SwerveModule sMod : swerveMods) {
            sMod.stop();
        }

        System.out.println("======================");
        System.out.println(frontLeft.getTurningPositionWrapped() + " " + Math.PI / 4);
        System.out.println(frontRight.getTurningPositionWrapped() + " " + -Math.PI / 4);
        System.out.println(backLeft.getTurningPositionWrapped() + " " + -Math.PI / 4);
        System.out.println(backRight.getTurningPositionWrapped() + " " + Math.PI / 4);

        // Sets the turning motors to their braking positions
        frontLeft.turningMotor.set(frontLeft.turningPidController.calculate(frontLeft.getTurningPosition(), Math.PI / 4));
        frontRight.turningMotor.set(frontRight.turningPidController.calculate(frontRight.getTurningPosition(), -Math.PI / 4));
        backLeft.turningMotor.set(backLeft.turningPidController.calculate(backLeft.getTurningPosition(), -Math.PI / 4));
        backRight.turningMotor.set(backRight.turningPidController.calculate(backRight.getTurningPosition(), Math.PI / 4));
    }
}