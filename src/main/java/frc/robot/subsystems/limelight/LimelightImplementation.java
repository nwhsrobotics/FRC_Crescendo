// This class provides implementations for utilizing a Limelight camera for various robot control tasks.
package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class contains implementations for using Limelight camera for aiming, ranging, and transforming target location.
 */
public class LimelightImplementation {

    /**
     * Implements simple proportional turning control with the Limelight.
     * Proportional control adjusts output based on error, here the difference between target's angle and camera's angle.
     * @return The angular velocity proportional to the horizontal angle error (tx) detected by the Limelight.
     */
    public static double limelight_aim_proportional() {
        if(LimelightHelpers.getTV("limelight")){
            // Proportional constant determining the aggressiveness of turning.
            double kP = .035;

            // Calculate targeting angular velocity based on horizontal angle error (tx) from Limelight.
            double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

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
     * @return The forward speed proportional to the vertical angle error (ty) from Limelight.
     */
    public static double limelight_range_proportional() {
        if(LimelightHelpers.getTV("limelight")){
            double targetingForwardSpeed = 0.0;
            if(isAprilTagPipeline()){
                // Calculate targeting forward speed based on target's distance (TZ) in 3D space.
                targetingForwardSpeed = LimelightHelpers.getCameraPose_TargetSpace("limelight")[2];
                //targetingForwardSpeed *= Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
                targetingForwardSpeed *= 0.345;
                //TODO: change the signs
                targetingForwardSpeed *= -1.0;
            } else {
                double kP = .1;
                //double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
                //double targetingForwardSpeed = 2 / (LimelightHelpers.getTY("limelight") * 0.1 * kP);

                //if(Double.isInfinite(targetingForwardSpeed)) return 0;

                //targetingForwardSpeed = -distanceFromLimelight(LimelightHelpers.getTY("limelight") * 0.5 * kP);
                // Calculate targeting forward speed based on vertical angle error (ty) from Limelight.
                targetingForwardSpeed = -hypotenuseLength() * kP;
            }
            return targetingForwardSpeed;
        }
        return 0.0;
    }

    /**
     * Calculates the distance from the Limelight to the target.
     * @return The calculated distance from the Limelight to the target.
     */
    public static double distanceFromLimelight(){
        if(LimelightHelpers.getTV("limelight")){
            // Angle at which the Limelight is mounted (degrees).
            double limelightMountAngleDegrees = -10;
            // Height of Limelight lens above the floor (meters).
            double limelightLensHeightMeters = 1.1;
            // Height of target above the floor (meters).
            double goalHeightMeters = 0.0;

            // Calculate angle to the target.
            double angleToGoalDegrees = limelightMountAngleDegrees + LimelightHelpers.getTY("limelight");
            double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

            // Calculate distance to the target.
            return (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        }
        return 0.0;
    }

    /**
     * Calculates the hypotenuse length between the Limelight and the target.
     * @return The calculated hypotenuse length (aka actual length to object) between the Limelight and the target.
     */
    public static double hypotenuseLength(){
        if(LimelightHelpers.getTV("limelight")){
            double legLength = distanceFromLimelight();
            double cosTheta = Math.cos(LimelightHelpers.getTX("limelight") / 180 * Math.PI);
            return legLength / cosTheta;
        }
        return 0.0;
    }

    /**
     * Calculates the horizontal offset distance from the target.
     * @return The calculated horizontal offset distance from the target.
     */
    public static double horizontalOffestDistance(){
        if(LimelightHelpers.getTV("limelight")){
            double legLength = distanceFromLimelight();
            double tanTheta = Math.tan(LimelightHelpers.getTX("limelight") / 180 * Math.PI);
            return legLength * tanTheta;
        }
        return 0.0;
    }

    /**
     * Checks if the AprilTag pipeline is being used.
     * @return True if the AprilTag pipeline is being used, false otherwise.
     */
    private static boolean isAprilTagPipeline(){
        if(LimelightHelpers.getCurrentPipelineIndex("limelight") == 0.0){
            return true;
        }
        return false;
    }

    /**
     * Transforms the target's location based on Limelight's data.
     * @param pos The current position of the target.
     * @return The transformed position of the target.
     */
    public static Pose2d transformTargetLocation(Pose2d pos) {
        if (LimelightHelpers.getTV("limelight")) {
            double horizontalDistance = horizontalOffestDistance();
            double distance = Math.abs(distanceFromLimelight()); // Ensure distance is always positive

            Translation2d translation = pos.getTranslation().plus(new Translation2d(horizontalDistance, distance));
            
            // Calculate the actual X-coordinate after rotation
            double actualX = translation.getX() + horizontalDistance * Math.cos(Math.toRadians(LimelightHelpers.getTX("limelight")));

            // Calculate the actual Y-coordinate after rotation
            double actualY = translation.getY() + horizontalDistance * Math.sin(Math.toRadians(LimelightHelpers.getTX("limelight")));
            
            Translation2d actualTranslation = new Translation2d(actualX, actualY);

            // Calculate the rotation of the target based on Limelight's horizontal angle (tx).
            // Note: We add the rotation of the Limelight to the target's existing rotation.
            Rotation2d targetRotation = pos.getRotation().plus(Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight")));

            return new Pose2d(actualTranslation, targetRotation);
        }

        return pos;
    }


}
