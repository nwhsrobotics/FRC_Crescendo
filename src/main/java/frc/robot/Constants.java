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
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

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

        public static final int CLIMB_LEFT_MOTOR_ID = 32;
        public static final int CLIMB_RIGHT_MOTOR_ID = 30;

        public static final int FLYWHEEL_MOTOR_ID = 7;
        public static final int INDEX_MOTOR_ID = 13;
        public static final int SECONDARY_FLYWHEEL_MOTOR_ID = 41;
        public static final int SECONDARY_INDEX_MOTOR_ID = 42;

        public static final int INTAKE_MOTOR_ID = 15;

        // shoulder/wrist not used.
        // DO NOT INITIALIZE THEIR SUBSYSTEMS.
        public static final int RIGHT_SHOULDER_MOTOR_ID = 19;
        public static final int LEFT_SHOULDER_MOTOR_ID = 17;
        public static final int WRIST_MOTOR_ID = 16;
        public static final int WRIST_INTAKE_ID = 18;

        public static final int PDU_ID = 24;

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

            Logger.recordOutput("canassignmentsok", !dupeFound);

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

        public static final double kDirectionSlewRate = 0.9; // radians per second
        public static final double kMagnitudeSlewRate = 1.35; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 1.5; // percent per second (1 = 100%)

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //FOR ALL OFFSETS: turn wheels until they become straight, replace with the value of encoders
        //THE BLACK GEAR SHOULD BE ON THE OUTSIDE FOR ALL WHEELS, regardless of side
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.064582666792635;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.202796411403781;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.064738140494073;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.066272121281959;//5.20 - Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for NEOs to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS
    }

    public static final class ClimbConstants {
        private static final double SPOOL_DIAMETER_METERS = 0.787 * 0.0254;
        private static final double GEAR_RATIO = Math.pow(7, 2);
        public static final double CLIMB_DISTANCE_PER_ROTATION = SPOOL_DIAMETER_METERS / GEAR_RATIO;

        public static final double CLIMB_TRAVEL_DISTANCE_METERS = 12 * 0.0254;
        public static final double TOLERANCE_DISTANCE_METERS = 0.25 * 0.0254;
    }

    public static final class ArmConstants {
        public static final double SHOULDER_PID_P = 0.1;
        public static final double SHOULDER_GEAR_RATIO = 112.0;
        public static final double SHOULDER_ABS_ENCODER_ROTATION_OFFSET = -0.161;
        public static final double SHOULDER_OUTPUT_LIMIT = 0.2;

        public static final int ABSOLUTE_ENCODER_DIO_CHANNEL = 0;
    }

    public static final class WristConstants {
        public static final double WRIST_PID_P = 0.5;
        public static final double WRIST_GEAR_RATIO = 100.0;
        public static final double absOffset = 0;

        public static final int ABSOLUTE_ENCODER_DIO_CHANNEL = 1;
    }

    public static final class ScoringConstants {
        /**
         * This adds +/- tolerance to the target RPM for the flywheel.
         * <p>
         * If the flywheel spins within the tolerance,
         * then it is considered up-to-speed for firing.
         */
        public static final double FLYWHEEL_TARGET_RPM_TOLERANCE = 600.0;

        public static final double FLYWHEEL_PID_FF = 0.0005;
        public static final double FLYWHEEL_PID_P = 0.00001;
        public static final double INDEX_PID_FF = 0.0005;
        public static final double INTAKE_PID_FF = 0.0005;

        /**
         * This controls the intake RPM when it is activated.
         *
         * <p>
         * <p>
         * If the intake is told to be driven backwards,
         * this value still is applied, and simply made negative.
         */
        public static final double INTAKE_RPM = 1650;  // TODO requires tuning.
        /**
         * This controls the indexer RPM when the intake is activated in the forwards direction.
         *
         * <p>
         * <p>
         * The indexer aids in ingesting the game piece,
         * elastically deforming it to be pressed firmly against the flywheel.
         */
        public static final double INDEX_INTAKE_COOP_RPM = 0;
        /**
         * This controls the indexer RPM when the intake is activated in the reverse direction.
         *
         * <p>
         * <p>
         * The indexer is responsible for initially dislodging the game piece
         * from its resting position against the flywheel.
         * Once displaced far enough, the intake wheels can engage with the game piece
         * and eject it completely out of the scoring assembly.
         */
        public static final double INDEX_INTAKE_UNLOAD_RPM = 800; // TODO requires tuning.

        public static final double FLYWHEEL_SPEAKER_RPM = 4750.0; //it was 4250 during second comp, increased it because does it even matter anymore?
        public static final double FLYWHEEL_AMP_RPM = 450.0;
        public static final double FLYWHEEL_IDLE_RPM = 0;
        public static final double INDEX_FLYWHEEL_COOP_RPM = 4750.0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = 5;
        public static final double kPThetaController = 5;

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(AutoConstants.kPXController, 0, 0), // Translation constants
                new PIDConstants(AutoConstants.kPThetaController, 0, 0), // Rotation constants
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
                new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2).getNorm(),// Drive base radius (distance from center to furthest module)
                new ReplanningConfig()
        );

        public static final PathConstraints kPathfindingConstraints = new PathConstraints(
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.8, AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.75,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double scaleFactor = 0.6;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * scaleFactor;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * scaleFactor;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class Positions {
        public static final Pose2d SOURCE = new Pose2d(15.39, 0.94, Rotation2d.fromDegrees(-60.00));
        public static final Pose2d AMP = new Pose2d(1.83, 7.70, Rotation2d.fromDegrees(-90.00));
        public static final Pose2d SPEAKER = new Pose2d(1.54, 5.54, Rotation2d.fromDegrees(0.00));

        public static final Pose2d MIDSTAGE = new Pose2d(5.85, 4.12, Rotation2d.fromDegrees(-180.00));
        //not near amp
        public static final Pose2d BOTTOMSTAGE = new Pose2d(4.37, 3.27, Rotation2d.fromDegrees(60.00));
        //same area as amp
        public static final Pose2d TOPSTAGE = new Pose2d(4.38, 4.89, Rotation2d.fromDegrees(-60.00));
        //0.49, 7.69 for odometry reset corner
        //same area as amp
        public static final Pose2d TOPSPEAKER = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(60));
        //away from amp
        public static final Pose2d BOTTOMSPEAKER = new Pose2d(0.71, 4.285, Rotation2d.fromDegrees(-60.00));
        //list of all positions (POI) to pathfind to allow closest pathfinding
        public static final List<Pose2d> allPoses = new ArrayList<Pose2d>(List.of(SOURCE, AMP, SPEAKER, MIDSTAGE, BOTTOMSTAGE, TOPSTAGE, TOPSPEAKER, BOTTOMSPEAKER));

        public static final Pose2d BACKRIGHT = new Pose2d(2.90, 4.10, Rotation2d.fromDegrees(0.00));
        public static final Pose2d BACKCENTER = new Pose2d(2.90, 5.55, Rotation2d.fromDegrees(0.00));
        public static final Pose2d BACKLEFT = new Pose2d(2.90, 7.00, Rotation2d.fromDegrees(0.00));
        public static final Pose2d FRONTRIGHTMOST = new Pose2d(8.29, 7.44, Rotation2d.fromDegrees(0.00));
        public static final Pose2d FRONTRIGHT = new Pose2d(8.29, 5.78, Rotation2d.fromDegrees(0.00));
        public static final Pose2d FRONTCENTER = new Pose2d(8.29, 4.10, Rotation2d.fromDegrees(0.00));
        public static final Pose2d FRONTLEFTMOST = new Pose2d(8.29, 2.44, Rotation2d.fromDegrees(0.00));
        public static final Pose2d FRONTLEFT = new Pose2d(8.29, 0.77, Rotation2d.fromDegrees(0.00));
        public static final List<Pose2d> allNotes = new ArrayList<Pose2d>(List.of(BACKRIGHT, BACKCENTER, BACKLEFT, FRONTRIGHTMOST, FRONTRIGHT, FRONTCENTER, FRONTLEFT, FRONTLEFTMOST));
    }

    public static final class LimelightConstants {
        public static final double mountHeight = 1.0; //in meters
        public static final double mountAngle = -10.0; //in degrees with straight being 0 up being 90 and down being -90
        public static final double horizontalOffset = -0.5; //in meters, this offset is how far left or right LL3 is mounted from center (negative is left, positive right)
        //this might not be needed but doesn't hurt us
        public static final double distanceFromCenter = 0.3; //in meters, straight distance to the camera from middle
        public static final double hypotenuseDistance = Math.hypot(horizontalOffset, distanceFromCenter); // actual distance in 2d from middle
        public static final double thethaFromCenter = -5.0; //this might be needed for angle offset
        public static String llObjectDetectionName = "limelight";
        public static String llLocalizationName = "limelightLoc";
        public static double targetHeight;
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

    public static final ModuleType PDU_TYPE = ModuleType.kRev;

    public static final class LoggerConstants {
        public static final RuntimeEnvironment MODE = RuntimeEnvironment.REAL;
        public static final String RUNNING_UNDER = "2024.q1";

        // SET TO FALSE IF WE'RE RUNNING OUT OF BANDWIDTH.
        public static final boolean SILENT_NT4 = false;
    }
}
