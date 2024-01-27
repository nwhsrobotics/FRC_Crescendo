package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

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
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //back right

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
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.523087448669004 + Math.PI / 2;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.733709237713651 + Math.PI / 2;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.637068448076855 + Math.PI / 2;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 5.781573589540982 + Math.PI / 2;//5.20 - Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for NEOs to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS
    }

    public static final class ClimbConstants {
    }

    public static final class ArmConstants {
    }

    public static final class ShooterConstants {
    }

    public static final class IndexConstants {
    }

    public static final class IntakeConstants {
    }

    public static final class SheildConstants {
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = 5;
        public static final double kPYController = 5;
        public static final double kPThetaController = 5;

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(AutoConstants.kPXController, 0, 0), // Translation constants
                new PIDConstants(AutoConstants.kPThetaController, 0, 0), // Rotation constants
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2).getNorm(),// Drive base radius (distance from center to furthest module)
                new ReplanningConfig()
        );
    }

    public static final class OIConstants {
        public static final double kXYDeadband = 0.05;
        public static final double kZDeadband = 0.05;
        public static final int kJoystickPort = 2;
        public static final boolean isLefty = true;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.6;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * 0.6;
        
        public static final PathConstraints kPathfindingConstraints = new PathConstraints(
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 6.0,
            AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class FavoritePositions {
        // TODO: confirm robot size; affects location coordinates.
        public static final Pose2d SOURCE = new Pose2d(15.45, 0.87, Rotation2d.fromDegrees(-60.00));
        public static final Pose2d AMP = new Pose2d(1.97, 7.80, Rotation2d.fromDegrees(90.00));

        // TODO: consider other stage locations.
        public static final Pose2d STAGE = new Pose2d(5.84, 3.99, Rotation2d.fromDegrees(-180.00));
    }

    public enum RuntimeEnvironment {
        /**
         * Running on physical robot.
         */
        REAL,
        /**
         * Running on simulated robot.
         */
        SIMULATION,
        /**
         * Replaying robot from log file.
         */
        REPLAY
    }

    public static final class LoggerConstants {
        public static final RuntimeEnvironment MODE = RuntimeEnvironment.REAL;
        public static final String RUNNING_UNDER = "2024.pre";
        public static final ModuleType PDU_TYPE = ModuleType.kRev;

        // SET TO FALSE IF WE'RE RUNNING OUT OF BANDWIDTH.
        public static final boolean SILENT_NT4 = false;
    }
}
