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
        public static final double kTrackWidth = 0.52;
        // front-back distance between drivetrain wheels, should be measured from center to center AND IN METERS 
        public static final double kWheelBase = 0.655;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //backright

        public static final int kFrontLeftDriveMotorPort = 6;
        public static final int kBackLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 10;
        public static final int kBackRightDriveMotorPort = 3;

        public static final int kFrontLeftTurningMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 11;
        public static final int kBackRightTurningMotorPort = 4;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 22;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 21;
        public static final int kBackRightDriveAbsoluteEncoderPort = 23;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //FOR ALL OFFSETS: turn wheels until they become straight, replace with the value of encoders
        //THE BLACK GEAR SHOULD BE ON THE OUTSIDE FOR ALL WHEELS, regardless of side
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.523087448669004 - Math.PI;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.733709237713651 + Math.PI;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.637068448076855 + Math.PI;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 5.781573589540982 - Math.PI;//5.20 - Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for NEOs to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    }

    public static final class ShoulderConstants {
        public static final double kp = 0.5;
        public static final double absOffset = 0.095;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double kIz = 0.0;
        public static final double kFFz = 0.0;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final int LeftShoulderCanID = 8;
        public static final int RightShoulderCanID = 6;
        public static final double kAngleRange = 55.0;
    }

    public static final class ExtendArmConstants {
        public static final double EXTEND_SPEED_IPS = 20.0;
        public static final double ACCEL_MAX_V_IPS = 60.0;
        public static final double ACCEL_MAX_A_IPS2 = 60.0; //about 0.2gs @ 80 in/s
        public static final double SECONDS_PER_TICK = 0.02;
        public static final double kp = 0.2;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final int ExtendArmCanID24 = 24;
        public static final int ExtendArmCanID25 = 25;
        public static final double MAX_EXTEND_INCH = 20.0;
        public static final double MIN_VEL_IPS = 4;
        public static final double MIN_X_INCH = 1;
    }

    public static final class GrabberConstants {
        public static final int forwardChannel = 1;
        public static final int reverseChannel = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

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

    public static final class WristConstants {
        public static final double kp = 0.05;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double kIz = 0.0;
        public static final double kFFz = 0.0;
        public static final double absAOffset = 0.638;
        public static final double absBOffset = 0.99;
        public static final double kMaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final double kMaxRoll = 115.0;
        public static final double kMinRoll = -115.0;
        public static final double kMaxPitch = 130.0;
        public static final double kMinPitch = -90.0;
        public static final int WristCanIDA = 9;
        public static final int WristCanIDB = 13;
        public static final int AbsoluteEncoderAChannel = 2;
        public static final int AbsoluteEncoderBChannel = 1;
        public static final double WRIST_GEAR_RATIO_RIGHT = 100.0;
        public static final double WRIST_GEAR_RATIO_LEFT = 50.0;
        public static final double REVS_PER_OUTPUT_DEGREE_RIGHT = WRIST_GEAR_RATIO_RIGHT / 360.0;
        public static final double REVS_PER_OUTPUT_DEGREE_LEFT = WRIST_GEAR_RATIO_LEFT / 360.0;
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(AutoConstants.kPXController, 0, 0), // Translation constants 
      new PIDConstants(AutoConstants.kPThetaController, 0, 0), // Rotation constants 
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4.0, 
      Math.sqrt(DriveConstants.kTrackWidth*DriveConstants.kTrackWidth + DriveConstants.kWheelBase*DriveConstants.kWheelBase) / 2.0,// Drive base radius (distance from center to furthest module) 
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
        public static final String RUNNING_UNDER = "2024.q1";
        // SET TO FALSE IF WE'RE RUNNING OUT OF BANDWIDTH.
        public static final boolean SILENT_NT4 = false;
    }
}
