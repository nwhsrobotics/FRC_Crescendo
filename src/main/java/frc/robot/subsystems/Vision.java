package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * This class uses a Limelight camera to track targets, measure distance, and aid in aiming.
 * Each method returns simple, direct values to help you steer and drive toward a target.
 */
public class Vision {

    public static Pose2d visionTargetLocation = new Pose2d();
    public static double tagDist;
    public static ArrayList<Integer> tagIds = new ArrayList<>();

    /**
     * This returns an angular velocity based on how far off-center the target is in the camera view.
     * It uses proportional control. If the target appears to the right, the robot turns right, and vice versa.
     *
     * @param limelightName The name of the limelight.
     * @return Angular velocity in radians per second, proportional to horizontal angle error.
     */
    public static double limelight_aim_proportional(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double kP = 0.035;
            double angleErrorDegrees = LimelightHelpers.getTX(limelightName) + LimelightConstants.thethaFromCenter;
            double targetingAngularVelocity = angleErrorDegrees * kP;
            targetingAngularVelocity *= 0.5; 
            targetingAngularVelocity *= -1.0;
            return targetingAngularVelocity;
        }
        return 0.0;
    }

    /**
     * This returns a forward speed command based on how far away the target is.
     * If using an AprilTag pipeline, it uses known 3D data for higher accuracy.
     * If using a standard pipeline, it estimates distance using vertical angle.
     *
     * @param limelightName The name of the limelight.
     * @return Forward speed (m/s) proportional to distance error.
     */
    public static double limelight_range_proportional(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double kP = 0.1;
            if (isAprilTagPipeline(limelightName)) {
                var fiducials = LimelightHelpers.getRawFiducials(limelightName);
                if (fiducials.length > 0) {
                    double distToRobot = fiducials[0].distToRobot; 
                    // Negative sign to move forward (assuming positive dist is forward)
                    return -distToRobot * kP; 
                } else {
                    // If no fiducials, fallback to a safe default
                    return 0.0;
                }
            } else {
                // For non-AprilTag pipelines, estimate distance using the camera angle.
                // Negative sign to move forward as angle increases
                double dist = verticalPlaneDistance(limelightName);
                return -dist * kP;
            }
        }
        return 0.0;
    }

    /**
     * Calculates the distance from the Limelight to the target based on vertical angle.
     * Uses geometry: distance = (targetHeight - lensHeight) / tan(mountAngle + ty).
     *
     * @param limelightName The name of the limelight.
     * @return Distance (meters) from the Limelight lens to the target.
     */
    public static double verticalPlaneDistance(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double limelightMountAngleDegrees = LimelightConstants.mountAngle;
            double limelightLensHeightMeters = LimelightConstants.mountHeight;
            double goalHeightMeters = LimelightConstants.targetHeight;
            double angleToGoalDegrees = limelightMountAngleDegrees + LimelightHelpers.getTY(limelightName);
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            return (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        }
        return 0.0;
    }

    /**
     * Calculates the straight-line distance to the target considering horizontal offset.
     * This uses the leg length from distanceFromLimelight and the horizontal angle to find the actual line of sight distance.
     *
     * @param limelightName The name of the limelight.
     * @return Hypotenuse distance (meters) to the target.
     */
    public static double lineOfSightDistance(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double legLength = verticalPlaneDistance(limelightName);
            double txRadians = Math.toRadians(LimelightHelpers.getTX(limelightName));
            double cosTheta = Math.cos(txRadians);
            if (cosTheta == 0.0) {
                return 0.0;
            }
            return legLength / cosTheta;
        }
        return 0.0;
    }

    /**
     * Calculates the full 3D distance (hypotenuse) to the target.
     *
     * @param limelightName The name of the limelight.
     * @return Full 3D distance (meters) from the Limelight lens to the target.
     */
    public static double full3DDistance(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double verticalDistance = verticalPlaneDistance(limelightName);
            double horizontalDistance = horizontalOffestDistance(limelightName);
            double zOffset = LimelightConstants.targetHeight - LimelightConstants.mountHeight;
            return Math.sqrt(Math.pow(verticalDistance, 2) + Math.pow(horizontalDistance, 2) + Math.pow(zOffset, 2));
        }
        return 0.0;
    }

    
    /**
     * Calculates how far left or right the target is from the camera's forward line.
     *
     * @param limelightName The name of the limelight.
     * @return Horizontal offset distance (meters) from the target.
     */
    public static double horizontalOffestDistance(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double legLength = verticalPlaneDistance(limelightName);
            double txRadians = Math.toRadians(LimelightHelpers.getTX(limelightName));
            double tanTheta = Math.tan(txRadians);
            return legLength * tanTheta;
        }
        return 0.0;
    }

    /**
     * Checks if we are currently using an AprilTag pipeline.
     *
     * @param limelightName The name of the limelight.
     * @return True if AprilTag pipeline is active, else false.
     */
    private static boolean isAprilTagPipeline(String limelightName) {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == 0.0;
    }

    /**
     * Transforms the given pose based on the camera's detected angle and distance to the target.
     * This tries to place the target on the field based on the robot's known position and camera angle.
     *
     * @param pos           The current pose.
     * @param limelightName The name of the limelight.
     * @return A new Pose2d representing the transformed target location.
     */
    public static Pose2d transformTargetLocation(Pose2d pos, String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double distance = Math.abs(lineOfSightDistance(limelightName));
            Translation2d translation = pos.getTranslation();
            Rotation2d targetRotation = pos.getRotation().plus(Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)));

            double cosVal = Math.cos(targetRotation.getRadians());
            double sinVal = Math.sin(targetRotation.getRadians());

            double actualX = translation.getX() + (distance * cosVal - (LimelightConstants.horizontalOffset * sinVal));
            double actualY = translation.getY() + (distance * sinVal - (LimelightConstants.horizontalOffset * cosVal));

            Translation2d actualTranslation = new Translation2d(actualX, actualY);
            Pose2d targetPose2d = new Pose2d(actualTranslation, targetRotation);
            Logger.recordOutput("limelight.objectPos", targetPose2d);
            return targetPose2d;
        }
        return pos;
    }

    /**
     * Switches to the next pipeline. If at the end, it cycles back.
     *
     * @param limelightName The name of the limelight.
     */
    public static void nextPipeline(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        if (currentIndex < 1) {
            LimelightHelpers.setPipelineIndex(limelightName, currentIndex + 1);
        } else {
            LimelightHelpers.setPipelineIndex(limelightName, 0);
        }
    }

    /**
     * Returns the name of the current pipeline based on its index.
     *
     * @param limelightName The name of the limelight.
     * @return The pipeline name.
     */
    public static String getPipelineName(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        switch (currentIndex) {
            case 0:
                return "AprilTag";
            case 1:
                return "Note";
            default:
                return "None";
        }
    }
}
