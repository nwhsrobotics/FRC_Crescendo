package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class SwerveSubsystem extends SubsystemBase {

  // boolean variable to indicate if the robot is Field Relative
  public boolean isFR = true;
  private SwerveDriveKinematics kinematics;
  public String lastPath;
  public Pose2d lastPose2d;
  public String lastPathType;
  public Command pathfindingCommand;

  // 4 instances of SwerveModule to represent each wheel module
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

  public SwerveSubsystem() {
    //pause for 500 milliseconds
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
    }

    // set the yaw of the gyro to 0
    m_gyro.zeroYaw();
    ;
      AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      Constants.pathFollowerConfig,
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
  }

  // set the speed of the robot in the x direction
  public void setSpeed(double xSpeed) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0.0, 0.0);
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }


  // get the heading (yaw) of the robot
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * -1;
  }

  // get the pitch of the robot in degrees
  public double getPitchDeg() {
    return -m_gyro.getPitch();
  }

  // reset the heading (yaw) and the odometry pose of the robot
  public void resetHeadingAndPose() {
    m_gyro.zeroYaw(); // reset the yaw angle
    resetOdometry(new Pose2d()); // reset the robot's odometry pose
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void switchFR() {
    isFR = !isFR; // switch between field-relative and robot-relative driving
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters(); // get the robot's current pose from the odometry system
  }

  public void resetOdometry(Pose2d pose) {
    // reset the drive encoder positions on all four swerve modules
    for (SwerveModule sModule : swerveMods)
      sModule.driveEncoder.setPosition(0);

    // reset the odometry system using the current heading, module positions, and specified pose
    odometer.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < swerveMods.length; i++) {
      states[i] = swerveMods[i].getState();
    }
    return states;
  }

  public void straighten() {
    // turn each swerve module to point straight ahead
    for (SwerveModule s_mod : swerveMods) {
      s_mod.turningMotor.set(s_mod.turningPidController.calculate(s_mod.getAbsoluteEncoderRad(), 0)); // use PID control to turn to the desired angle
      s_mod.turningMotor.set(0); // stop turning once the desired angle is reached
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    // get the current position of each swerve module
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < swerveMods.length; i++) {
      positions[i] = swerveMods[i].getPosition();
    }
    return positions;
  }

  public SwerveControllerCommand getTrajCmd(Trajectory traj) {
    // create a PID controller for the heading (yaw) error
    var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // create a new swerve controller command using the specified trajectory, odometry, and controllers
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

  public void pathFindThenFollowPath(String pathName){
    lastPath = pathName;
    lastPathType = "Path";
        // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
      kPhysicalMaxSpeedMetersPerSecond / 4.0, kMaxAccelerationMetersPerSecondSquared / 2.0,
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared / 2.0);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    //what pathfinding basically does is path find to start of a path and then continue in that path, if you wanna not follow the path after make it pathfind to specific location
    pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints,
            2.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  public void pathFindToPos(Pose2d coords){
    // Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation
    lastPose2d = coords;
    lastPathType = "Pos";

// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            kPhysicalMaxSpeedMetersPerSecond / 4.0, kMaxAccelerationMetersPerSecondSquared / 2.0,
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared / 2.0);

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand = AutoBuilder.pathfindToPose(
            coords,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  // This method is called periodically to update the robot's state and log data
  @Override
  public void periodic() {
    // Log whether the robot is field-relative or not
    SmartDashboard.putBoolean("FIELD RELATIVE?", isFR);

    // Update the robot's odometer
    odometer.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

    // Log various data points
    Logger.recordOutput("swerve.pitch", getPitchDeg());
    Logger.recordOutput("swerve.steer.front.left.abs", frontLeft.getAbsoluteEncoderRad());
    Logger.recordOutput("swerve.steer.front.right.abs", frontRight.getAbsoluteEncoderRad());
    Logger.recordOutput("swerve.steer.back.left.abs", backLeft.getAbsoluteEncoderRad());
    Logger.recordOutput("swerve.steer.back.right.abs", backRight.getAbsoluteEncoderRad());
    Logger.recordOutput("swerve.pose", getPose());
    Logger.recordOutput("swerve.heading", getHeading());
    Logger.recordOutput("swerve.drive.front.left.velocity", frontLeft.getDriveVelocity());
    Logger.recordOutput("swerve.drive.front.right.velocity", frontRight.getDriveVelocity());
    Logger.recordOutput("swerve.drive.back.left.velocity", backLeft.getDriveVelocity());
    Logger.recordOutput("swerve.drive.back.right.velocity", backRight.getDriveVelocity());
  }

  // Stops all of the robot's swerve modules
  public void stopModules() {
    for (SwerveModule sMod : swerveMods) {
      sMod.stop();
    }
  }

  // Sets the states of the robot's swerve modules based on the desired states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Scales all speeds down instead of truncating them if they exceed the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  // Activates the robot's brakes
  public void brake() {
    // Stops all swerve modules
    for (SwerveModule sMod : swerveMods) {
      sMod.stop();
    }

    // Sets the turning motors to their braking positions
    frontLeft.turningMotor.set(frontLeft.turningPidController.calculate(frontLeft.getTurningPosition(), Math.PI / 4));
    frontRight.turningMotor.set(frontRight.turningPidController.calculate(frontRight.getTurningPosition(), -Math.PI / 4));
    backLeft.turningMotor.set(backLeft.turningPidController.calculate(backLeft.getTurningPosition(), -Math.PI / 4));
    backRight.turningMotor.set(backRight.turningPidController.calculate(backLeft.getTurningPosition(), Math.PI / 4));
  }

  
}