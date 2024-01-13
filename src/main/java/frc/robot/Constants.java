package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

// IMPORTANT: SET UP FOR NEOS for both driving and turning
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//Most values are from SDS SwerveLib setup for MK4_L2
public final class Constants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.10033; // set up for MK4(i)
        public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // (set up for MK4(i) L2)
        public static final double kTurningMotorGearRatio = (15.0 / 32.0) * (10.0 / 60.0); // (set up for MK4 L2)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .5; // P constant for turning
        //public static final double kPTolerance = 2.5 * (Math.PI/180);
        public static final double kITurning = 0.;
    }

    public static final class DriveConstants {
        // left-to-right distance between the drivetrain wheels, should be measured from center to center AND IN METERS
        public static final double kTrackWidth = 0.4445;
        // front-back distance between drivetrain wheels, should be measured from center to center AND IN METERS 
        public static final double kWheelBase = 0.5715;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //backright

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 4;
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 10;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 21;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
        public static final int kBackRightDriveAbsoluteEncoderPort = 23;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //FOR ALL OFFSETS: turn wheels until they become straight, replace with the value of encoderss
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =  5.201;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.61;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 5.23;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.64;//5.20 - Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for NEOs to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.25;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 0.25;
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.0;

        public static final TrajectoryConfig autoTrajectoryConfig =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.DriveConstants.kDriveKinematics);
        
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 0.70;

        //creates a TrapezoidProfile to determine setpoints for autonomous
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double kXYDeadband = 0.05;
        public static final double kZDeadband = 0.05;
        public static final int kJoystickPort = 2;
        public static final double kPreciseSpdMetersPerSecond = 0.5;
    }

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(1.0, 0, 0), // Translation constants 
      new PIDConstants(1.0, 0, 0), // Rotation constants 
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4.0, 
      0.36200569746898, // or Math.sqrt(kTrackWidth*kTrackWidth + kWheelBase*kWheelBase) Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
    

    public static enum RuntimeEnvironment {
        /** Running on physical robot. */
        REAL,
        /** Running on simulated robot. */
        SIMULATION,
        /** Replaying robot from log file. */
        REPLAY
    }
    public static final class LoggerConstants {
        public static final RuntimeEnvironment MODE = RuntimeEnvironment.REAL;
        public static final String RUNNING_UNDER = "2023.q1";
        // SET TO FALSE IF WE'RE RUNNING OUT OF BANDWIDTH.
        public static final boolean SILENT_NT4 = false;
    }
}