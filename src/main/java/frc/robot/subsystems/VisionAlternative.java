package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class VisionAlternative {

    public static Pose2d visionTargetLocation = new Pose2d();
    public static double tagDist;
    public static ArrayList<Integer> tagIds = new ArrayList<>();

    /**
     * Implements simple proportional turning control with the Limelight.
     * Adjusts output based on the horizontal alignment error.
     *
     * @return The angular velocity proportional to the horizontal angle error.
     */
    public static double limelight_aim_proportional(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            double kP = 0.035;
            double horizontalError = LimelightHelpers.getLatestResults(limelightName)
                    .targets_Retro[0].getTargetPose_CameraSpace2D().getRotation().getDegrees();
            return -kP * horizontalError;
        }
        return 0.0;
    }

    /**
     * Implements simple proportional ranging control using the Limelight.
     * Adjusts the robot's forward speed based on the distance to the target.
     *
     * @return The forward speed proportional to the target's distance.
     */
    public static double limelight_range_proportional(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            if (isAprilTagPipeline(limelightName)) {
                // Use distance from AprilTag estimation
                double distanceToTarget = LimelightHelpers.getLatestResults(limelightName)
                    .targets_Fiducials[0].getTargetPose_CameraSpace().getTranslation().getZ();
                return -distanceToTarget * 0.345; 
            } else {
                // Use Z-axis distance from retro-reflective target
                double zDistance = LimelightHelpers.getLatestResults(limelightName)
                        .targets_Retro[0].getTargetPose_CameraSpace().getTranslation().getZ();
                return -zDistance * 0.1;
            }
        }
        return 0.0;
    }

    /**
     * Calculates the distance from the Limelight to the target using prebuilt methods.
     *
     * @return The calculated distance from the Limelight to the target.
     */
    public static double distanceFromLimelight(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            // Use Z-axis distance to the target
            return LimelightHelpers.getLatestResults(limelightName)
                    .targets_Retro[0].getTargetPose_CameraSpace().getTranslation().getZ();
        }
        return 0.0;
    }

    /**
     * Calculates the hypotenuse length between the Limelight and the target using prebuilt methods.
     *
     * @return The calculated hypotenuse length (actual distance) to the target.
     */
    public static double hypotenuseLength(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            Pose2d cameraPose = LimelightHelpers.getLatestResults(limelightName)
                    .targets_Retro[0].getTargetPose_CameraSpace2D();
            return cameraPose.getTranslation().getNorm();
        }
        return 0.0;
    }

    /**
     * Calculates the horizontal offset distance from the target.
     *
     * @return The calculated horizontal offset distance from the target.
     */
    public static double horizontalOffsetDistance(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            Translation2d cameraTranslation = LimelightHelpers.getLatestResults(limelightName)
                    .targets_Retro[0].getTargetPose_CameraSpace2D().getTranslation();
            return cameraTranslation.getX();
        }
        return 0.0;
    }

    /**
     * Transforms the target's location based on Limelight's data.
     *
     * @param pos           The current position of the target.
     * @param limelightName Name of the Limelight.
     * @return The transformed position of the target.
     */
    public static Pose2d transformTargetLocation(Pose2d pos, String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            Pose2d cameraPose = LimelightHelpers.getLatestResults(limelightName)
                    .targets_Retro[0].getTargetPose_CameraSpace2D();
            double actualX = pos.getX() + cameraPose.getTranslation().getX();
            double actualY = pos.getY() + cameraPose.getTranslation().getY();

            Pose2d transformedPose = new Pose2d(new Translation2d(actualX, actualY),
                    pos.getRotation().plus(cameraPose.getRotation()));
            Logger.recordOutput("limelight.objectPos", transformedPose);
            return transformedPose;
        }
        return pos;
    }

    /**
     * Switches to the next pipeline.
     */
    public static void nextPipeline(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        int newIndex = (currentIndex + 1) % 2; // Assuming two pipelines (AprilTag and retro-reflective)
        LimelightHelpers.setPipelineIndex(limelightName, newIndex);
    }

    /**
     * Gets the name of the current pipeline.
     *
     * @return Name of the active pipeline.
     */
    public static String getPipelineName(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        return currentIndex == 0 ? "AprilTag" : "RetroReflective";
    }

    /**
     * Checks if the AprilTag pipeline is being used.
     *
     * @return True if the AprilTag pipeline is active.
     */
    private static boolean isAprilTagPipeline(String limelightName) {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == 0;
    }
}
