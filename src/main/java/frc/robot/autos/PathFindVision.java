// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.LimelightHelpers;

import java.util.List;

public class PathFindVision extends Command {
    private final SwerveSubsystem swerve;
    private final ScoringSubsystem score;
    private Command pathFind;
    private final List<Pose2d> possibleLocations;
    private final Pose2d pathFindLoc;

    /**
     * Creates a new PathFindVision.
     */
    public PathFindVision(SwerveSubsystem swerve, ScoringSubsystem score, List<Pose2d> locations, Pose2d pathFindLoc) {
        this.swerve = swerve;
        this.score = score;
        addRequirements(swerve, score);
        possibleLocations = locations;
        this.pathFindLoc = pathFindLoc;
        pathFind = swerve.pathfindToPosition(Vision.visionTargetLocation);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pathFind.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (pathFind.isFinished()) {
            if (LimelightHelpers.getTV(LimelightConstants.llObjectDetectionName)) {
                //basically if note was pushed while intaking and never came inside then go pathfind again
                pathFind = null;
                pathFind = swerve.pathfindToPosition(Vision.visionTargetLocation);
                pathFind.schedule();
            } else {
                //if note was taken by opponent from center say for example
                possibleLocations.remove(pathFindLoc);
                //then pathfind to other note location until we get one in
                //new PathFindVision(swerve, score, possibleLocations, swerve.getPose().nearest(possibleLocations)).schedule();
                this.cancel();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pathFind.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return score.isNoteInside();
    }
}
