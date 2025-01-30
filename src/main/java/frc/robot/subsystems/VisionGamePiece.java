package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class VisionGamePiece {

    // TODO: READD/REDO COMMENTS, But much better and self explanatory so you can understand the math just by reading comments

    public static Pose2d visionTargetLocation = new Pose2d();
    public static double tagDist;
    public static ArrayList<Integer> tagIds = new ArrayList<>();

    private static final long DETECTION_HOLD = 200; 
    private static long timeDetected = 0;
    private static boolean isStableTv = false;

    private static double stableTx = 0.0;
    private static double stableTy = 0.0;

    static boolean stabilize(String limelightName) {
        long now = System.currentTimeMillis();

        if (LimelightHelpers.getTV(limelightName)) {
            timeDetected = now;
            isStableTv = true;

            stableTx = LimelightHelpers.getTX(limelightName);
            stableTy = LimelightHelpers.getTY(limelightName);
        } else {
            long time = now - timeDetected;
            if (time < DETECTION_HOLD) {
            } else {
                isStableTv = false;
            }
        }
        return isStableTv;
    }

    private static double getStabilizedTx(String limelightName) {
        stabilize(limelightName);
        return stableTx;
    }

    private static double getStabilizedTy(String limelightName) {
        stabilize(limelightName);
        return stableTy;
    }

    public static double limelight_aimX_proportional(String limelightName) {
        if (stabilize(limelightName)) {
            double kP = 0.035;
            double angleErrorDegrees = getStabilizedTx(limelightName);
            double targetingAngularVelocity = angleErrorDegrees * kP * -0.5;
            return targetingAngularVelocity;
        }
        return 0.0;
    }

    public static double limelight_rangeZ_proportional(String limelightName) {
        if (stabilize(limelightName)) {
            double kP = 0.1;
            double dist = straightLineZDistance(limelightName);
            return -dist * kP;
        }
        return 0.0;
    }

    public static double straightLineZDistance(String limelightName) {
        if (stabilize(limelightName)) {
            double limelightMountAngleDegrees = LimelightConstants.mountAngle;
            double angleToGoalDegrees = limelightMountAngleDegrees + getStabilizedTy(limelightName);
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            return verticalYOffsetDistance(limelightName) / Math.tan(angleToGoalRadians);
        }
        return 0.0;
    }

    public static double hypotenuseDistanceXandZ(String limelightName) {
        if (stabilize(limelightName)) {
            double zDistance = straightLineZDistance(limelightName);
            double xDistance = horizontalOffestXDistance(limelightName);
            return Math.sqrt(zDistance * zDistance + xDistance * xDistance);
        }
        return 0.0;
    }

    public static double verticalYOffsetDistance(String limelightName){
        double limelightLensHeightMeters = LimelightConstants.mountHeight;
        double goalHeightMeters = LimelightConstants.targetHeight;
        return goalHeightMeters - limelightLensHeightMeters;
    }

    public static double full3DDistance(String limelightName) {
        if (stabilize(limelightName)) {
            double zDistance = straightLineZDistance(limelightName);
            double xDistance = horizontalOffestXDistance(limelightName);
            double yDistance = LimelightConstants.targetHeight - LimelightConstants.mountHeight;
            return Math.sqrt(zDistance * zDistance + xDistance * xDistance + yDistance * yDistance);
        }
        return 0.0;
    }

    public static double horizontalOffestXDistance(String limelightName) {
        if (stabilize(limelightName)) {
            double legLength = straightLineZDistance(limelightName);
            double txRadians = Math.toRadians(getStabilizedTx(limelightName));
            return legLength * Math.tan(txRadians);
        }
        return 0.0;
    }

    public static boolean isAprilTagPipeline(String limelightName) {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == 0.0;
    }

    public static Pose2d transformTargetLocation(Pose2d pos, String limelightName) {
        if (stabilize(limelightName)) {
            double distance = Math.abs(hypotenuseDistanceXandZ(limelightName));
            Translation2d translation = pos.getTranslation();
            Rotation2d targetRotation = pos.getRotation().plus(Rotation2d.fromDegrees(getStabilizedTx(limelightName)));

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
        
        //is no math fun?
    }

    public static void nextPipeline(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        if (currentIndex < 1) {
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
}
