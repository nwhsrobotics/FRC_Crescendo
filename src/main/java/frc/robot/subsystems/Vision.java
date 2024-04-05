// This class provides implementations for utilizing a Limelight camera for various robot control tasks.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.HashSet;

import org.littletonrobotics.junction.Logger;

/**
 * This class contains implementations for using Limelight camera for aiming, ranging, and transforming target location.
 */
public class Vision{

    public static Pose2d visionTargetLocation = new Pose2d();
    public static double tagDist;
    public static ArrayList<Integer> tagIds = new ArrayList<>();
    /**
     * Implements simple proportional turning control with the Limelight.
     * Proportional control adjusts output based on error, here the difference between target's angle and camera's angle.
     *
     * @return The angular velocity proportional to the horizontal angle error (tx) detected by the Limelight.
     */
    public static double limelight_aim_proportional(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            // Proportional constant determining the aggressiveness of turning.
            double kP = .035;

            // Calculate targeting angular velocity based on horizontal angle error (tx) from Limelight.
            // like maybe thethaFromCenter add to offset
            double targetingAngularVelocity = (LimelightHelpers.getTX(limelightName) + LimelightConstants.thethaFromCenter) * kP;

            //targetingAngularVelocity *= Constants.DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
            // Convert angular velocity to radians per second.
            targetingAngularVelocity *= 0.5;
            // Invert the velocity since Limelight's tx is positive when the target is to the right of the crosshair.
            targetingAngularVelocity *= -1.0;

            return targetingAngularVelocity;
        }
        return 0.0;
    }

    /**
     * Implements simple proportional ranging control using Limelight's "ty" value.
     * Adjusts the robot's forward speed based on the vertical angle (ty) detected by the Limelight.
     *
     * @return The forward speed proportional to the vertical angle error (ty) from Limelight.
     */
    public static double limelight_range_proportional(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double targetingForwardSpeed = 0.0;
            if (isAprilTagPipeline(limelightName)) {
                // Calculate targeting forward speed based on target's distance (TZ) in 3D space.
                targetingForwardSpeed = LimelightHelpers.getCameraPose_TargetSpace(limelightName)[2];
                //first april tag
                // TODO: USE distanceToCamera OR distanceToRobot now BELOW
                //double dist = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).rawFiducials[0].distToRobot;
                //targetingForwardSpeed *= Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
                targetingForwardSpeed *= 0.345;
                targetingForwardSpeed *= -1.0;
            } else {
                double kP = .1;
                //double targetingForwardSpeed = LimelightHelpers.getTY(limelightName) * kP;
                //double targetingForwardSpeed = 2 / (LimelightHelpers.getTY(limelightName) * 0.1 * kP);

                //if(Double.isInfinite(targetingForwardSpeed)) return 0;

                //targetingForwardSpeed = -distanceFromLimelight(LimelightHelpers.getTY(limelightName) * 0.5 * kP);
                // Calculate targeting forward speed based on vertical angle error (ty) from Limelight.
                targetingForwardSpeed = -hypotenuseLength(limelightName) * kP;
            }
            return targetingForwardSpeed;
        }
        return 0.0;
    }

    /**
     * Calculates the distance from the Limelight to the target.
     *
     * @return The calculated distance from the Limelight to the target.
     */
    public static double distanceFromLimelight(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            // Angle at which the Limelight is mounted (degrees).
            double limelightMountAngleDegrees = LimelightConstants.mountAngle;
            // Height of Limelight lens above the floor (meters).
            double limelightLensHeightMeters = LimelightConstants.mountHeight;
            // Height of target above the floor (meters).
            double goalHeightMeters = 0.0;

            // Calculate angle to the target.
            double angleToGoalDegrees = limelightMountAngleDegrees + LimelightHelpers.getTY(limelightName);
            double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

            // Calculate distance to the target.
            return (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        }
        return 0.0;
    }

    /**
     * Calculates the hypotenuse length between the Limelight and the target.
     *
     * @return The calculated hypotenuse length (aka actual length to object) between the Limelight and the target.
     */
    public static double hypotenuseLength(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double legLength = distanceFromLimelight(limelightName);
            double cosTheta = Math.cos(LimelightHelpers.getTX(limelightName) / 180 * Math.PI);
            return legLength / cosTheta;
        }
        return 0.0;
    }

    /**
     * Calculates the horizontal offset distance from the target.
     *
     * @return The calculated horizontal offset distance from the target.
     */
    public static double horizontalOffestDistance(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double legLength = distanceFromLimelight(limelightName);
            double tanTheta = Math.tan(LimelightHelpers.getTX(limelightName) / 180 * Math.PI);
            return legLength * tanTheta;
        }
        return 0.0;
    }

    /**
     * Checks if the AprilTag pipeline is being used.
     *
     * @return True if the AprilTag pipeline is being used, false otherwise.
     */
    private static boolean isAprilTagPipeline(String limelightName) {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == 0.0;
    }

    /**
     * Transforms the target's location based on Limelight's data.
     *
     * @param pos The current position of the target.
     * @return The transformed position of the target.
     */
    public static Pose2d transformTargetLocation(Pose2d pos, String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double distance = Math.abs(hypotenuseLength(limelightName));

            Translation2d translation = pos.getTranslation();

            // Calculate the rotation of the target based on Limelight's horizontal angle (tx).
            // Note: We add the rotation of the Limelight to the target's existing rotation.
            Rotation2d targetRotation = pos.getRotation().plus(Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)));

            // Calculate the actual X-coordinate after rotation
            double actualX = translation.getX() + (distance * Math.cos(targetRotation.getRadians()) - (LimelightConstants.horizontalOffset * Math.sin(targetRotation.getRadians())));

            // Calculate the actual Y-coordinate after rotation
            double actualY = translation.getY() + ((distance * Math.sin(targetRotation.getRadians())) - (LimelightConstants.horizontalOffset * Math.cos(targetRotation.getRadians())));

            Translation2d actualTranslation = new Translation2d(actualX, actualY);

            Pose2d targetPose2d = new Pose2d(actualTranslation, targetRotation);
            Logger.recordOutput("limelight.objectPos", targetPose2d);
            return targetPose2d;
        }

        return pos;
    }

    public static void nextPipeline(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        if (currentIndex < 1.0) {
            LimelightHelpers.setPipelineIndex(limelightName, currentIndex + 1);
        } else {
            LimelightHelpers.setPipelineIndex(limelightName, 0);
        }
    }

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


    //Remove a lot of redudant methods like getting sin of distance vs getting tan of a leg is same thing but its being repeated
}
