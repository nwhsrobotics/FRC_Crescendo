// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class LimelightImplementation {
    
    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    public static double limelight_aim_proportional() {
        if(LimelightHelpers.getTV("limelight")){
            // kP (constant of proportionality)
            // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
            // if it is too high, the robot will oscillate around.
            // if it is too low, the robot will never reach its target
            // if the robot never turns in the correct direction, kP should be inverted.
            double kP = .035;

            // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
            // your limelight 3 feed, tx should return roughly 31 degrees.
            double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

            // convert to radians per second for our drive method
            //targetingAngularVelocity *= Constants.DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
            targetingAngularVelocity *= 0.5;
            //invert since tx is positive when the target is to the right of the crosshair
            targetingAngularVelocity *= -1.0;

            return targetingAngularVelocity;
        } 
        return 0.0;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public static double limelight_range_proportional() {
        if(LimelightHelpers.getTV("limelight")){
            double targetingForwardSpeed = 0.0;
            if(isAprilTagPipeline()){
                double kP = .1;
                
                //below is TZ because thats more accurate in 3d
                targetingForwardSpeed = LimelightHelpers.getCameraPose_TargetSpace("limelight")[2] * 10 * kP;
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
                targetingForwardSpeed = Math.min((-hypotenuseLength() * kP), Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            }
            return targetingForwardSpeed;
        }
        return 0.0;
    }

    public static double distanceFromLimelight(){
        if(LimelightHelpers.getTV("limelight")){
            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = -10; 

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightMeters = 1.1; 

            // distance from the target to the floor
            double goalHeightMeters = 0.0; 

            double angleToGoalDegrees = limelightMountAngleDegrees + LimelightHelpers.getTY("limelight");
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            return (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        }
        return 0.0;
    }

    /**
     * Hypotenuse length the the game piece (same height) 2d / actual DISTANCE
     */
    public static double hypotenuseLength(){
        if(LimelightHelpers.getTV("limelight")){
            double legLength = distanceFromLimelight();
            double cosTheta = Math.cos(LimelightHelpers.getTX("limelight") / 180 * 3.14159);
            return legLength / cosTheta;
        }
        return 0.0;
    }

    /**
     * Horizontally how far left or right from the middle of the crosshair
     */
    public static double horizontalOffestDistance(){
        if(LimelightHelpers.getTV("limelight")){
            double legLength = distanceFromLimelight();
            double tanTheta = Math.tan(LimelightHelpers.getTX("limelight") / 180 * 3.14159);
            return legLength * tanTheta;
        }
        return 0.0;
    }

    private static boolean isAprilTagPipeline(){
        if(LimelightHelpers.getCurrentPipelineIndex("limelight") == 0.0){
            return true;
        }
        return false;
    }

public static Pose2d transformTargetLocation(Pose2d pos) {
    if (LimelightHelpers.getTV("limelight")) {
        double horizontalDistance = horizontalOffestDistance();
        double distance = distanceFromLimelight();
        Translation2d translation = pos.getTranslation().plus(new Translation2d(horizontalDistance, distance));

        return new Pose2d(translation, pos.getRotation());
    }

    return pos;
}

}
