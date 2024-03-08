package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.HashMap;

public final class Constants {
    public static final class CANAssignments {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 6;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 10;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 3;

        public static final int FRONT_LEFT_STEER_MOTOR_ID = 8;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 11;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 4;

        public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 22;
        public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 20;
        public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 21;
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 23;

        // TODO verify all IDs other than drive after hardware assembly.
        public static final int CLIMB_LEFT_MOTOR_ID = 120;
        public static final int CLIMB_RIGHT_MOTOR_ID = 130;

        public static final int FLYWHEEL_MOTOR_ID = 140;
        public static final int INDEX_MOTOR_ID = 150;

        public static final int INTAKE_MOTOR_ID = 160;

        public static final int SHOULDER_MOTOR_ID = 80;

        public static final int WRIST_MOTOR_ID = 140;

        public static final int WRIST_INTAKE_ID = 160;

        /**
         * Check for duplicate CAN assignments,
         * declared under the class this method is defined in.
         * <p>
         * If an assignment cannot be loaded,
         * or a duplicate assignment is found,
         * a message will be printed in the console.
         *
         * @return - true if duplicate assignment is found, otherwise false.
         */
        public static boolean checkAssignments() {
            Field[] constants = CANAssignments.class.getFields();
            HashMap<Integer, String> tracker = new HashMap<>();
            boolean dupeFound = false;

            for (Field field : constants) {
                field.setAccessible(true);

                int workingId;
                try {
                    workingId = field.getInt(CANAssignments.class);
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    System.out.println("Achtung! Checking CAN assignment for " + field.getName() + " failed!");
                    continue;
                }

                if (tracker.put(workingId, field.getName()) != null) {  // this also adds the field to the tracker.
                    System.out.println("Fehler! Duplicate CAN assignment on " +
                            workingId +
                            " for " +
                            field.getName() +
                            " already used by " +
                            tracker.get(workingId) +
                            "!");

                    dupeFound = true;
                }
            }

            Logger.recordOutput("canassignmentsok", dupeFound);

            return dupeFound;
        }
    }


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
        public static final double kTrackWidth = 0.597;
        // front-back distance between drivetrain wheels, should be measured from center to center AND IN METERS 
        public static final double kWheelBase = 0.546;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //back right

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

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

        public static final double SHOULDER_PID_P = 0.5;
        public static final double SHOULDER_GEAR_RATIO = 1; //placeholder number
    }

    public static final class WristConstants {

        public static final double WRIST_PID_P = 0.5;
        public static final double WRIST_GEAR_RATIO = 1; //placeholder number
    }

    public static final class ShooterConstants {
        /**
         * This adds +/- tolerance to the target RPM for the flywheel.
         * <p>
         * If the flywheel spins within the tolerance,
         * then it is considered up-to-speed for firing.
         */
        public static final double FLYWHEEL_TARGET_RPM_TOLERANCE = 10.0;

        public static final double FLYWHEEL_PID_P = 0.5;
        public static final double INDEX_PID_P = 0.5;

        private static final double INDEX_WHEEL_DIAMETER_INCHES = 4;
        private static final double NOTE_DIAMETER_INCHES = 12;

        public static final double FLYWHEEL_SPEAKER_RPM = 50.0; //TODO FIX THIS
        public static final double FLYWHEEL_AMP_RPM = 25.0; //TODO FIX THIS


        // do not edit manually; change diameter measurements instead.
        public static final double INDEX_STEP_ROTATIONS = INDEX_WHEEL_DIAMETER_INCHES / NOTE_DIAMETER_INCHES;  // pi cancels out.
    }

    public static final class IntakeConstants {
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
        public static final double scaleFactor = 0.6;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * scaleFactor;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * scaleFactor;

        //TODO: Better constraints 
        public static final PathConstraints kPathfindingConstraints = new PathConstraints(
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 6.0,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 1.2);
        public static final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 1.2);
        public static final SlewRateLimiter zLimiter = new SlewRateLimiter(DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * 0.85);
    }

    public static final class FavoritePositions {
        // TODO: confirm robot size; affects location coordinates.
        public static final Pose2d SOURCE = new Pose2d(15.39, 0.94, Rotation2d.fromDegrees(-60.00));
        public static final Pose2d AMP = new Pose2d(1.83, 7.68, Rotation2d.fromDegrees(90.00));
        public static final Pose2d SPEAKER = new Pose2d(1.34, 5.54, Rotation2d.fromDegrees(0.00));

        public static final Pose2d MIDSTAGE = new Pose2d(5.85, 4.12, Rotation2d.fromDegrees(-180.00));
        //not near amp
        public static final Pose2d BOTTOMSTAGE = new Pose2d(4.37, 3.27, Rotation2d.fromDegrees(60.00));
        //same area as amp
        public static final Pose2d TOPSTAGE = new Pose2d(4.38, 4.89, Rotation2d.fromDegrees(-60.00));
    }

    public static final class LimelightConstants {
        public static final double mountHeight = 1.0; //in meters
        public static final double mountAngle = -10.0; //in degrees with straight being 0 up being 90 and down being -90
        public static final double horizontalOffset = -0.5; //in meters, this offset is how far left or right LL3 is mounted from center (negative is left, positive right)
        //this might not be needed but doesn't hurt us
        public static final double distanceFromCenter = 0.3; //in meters, straight distance to the camera from middle
        public static final double hypotenuseDistance = Math.hypot(horizontalOffset, distanceFromCenter); // actual distance in 2d from middle
        public static final double thethaFromCenter = -25.0; //this might be needed for angle offset

        //TODO: Move everything here finish this
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
