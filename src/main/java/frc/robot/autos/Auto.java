// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.LimelightHelpers;

import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// This class represents an autonomous routine for an FRC robot.
public class Auto extends SequentialCommandGroup {
    private final SwerveSubsystem swerve;
    private final ScoringSubsystem score;
    private final List<Pose2d> possibleLocations;
    private final int noteLimit;
    private final Pose2d initialPos;

    /**
     * Creates a new instance of the Auto class.
     *
     * @param swerve             The SwerveSubsystem object representing the robot's swerve drive.
     * @param blackListLocations A list of Pose2d objects representing locations to blacklist.
     * @param noteLimit          The limit on the number of notes to be obtained during autonomous + 1 preloaded.
     * @param initialPos         The initial position to reset the robot odometry to.
     */
    public Auto(SwerveSubsystem swerve, ScoringSubsystem score, List<Pose2d> blackListLocations, int noteLimit, Pose2d initialPos) {
        this.swerve = swerve;
        this.score = score;
        possibleLocations = Positions.allNotes;
        blackList(blackListLocations);
        this.noteLimit = noteLimit;
        this.initialPos = initialPos;
        addCommands(
                // Reset robot odometry to a initial position.
                new InstantCommand(() -> flipResetOdometry(initialPos)),
                // Retrieve an autonomous initialization command.
                NamedCommands.getCommand("autoInit"),
                // Retrieve a command for shooting game elements.
                NamedCommands.getCommand("shoot"),
                // Execute the command sequence to get notes.
                getNotes()
        );
    }

    /**
     * Generates a SequentialCommandGroup to get notes during autonomous.
     *
     * @return A SequentialCommandGroup containing commands to get notes.
     */
    public SequentialCommandGroup getNotes() {
        SequentialCommandGroup exitReturnCommands = new SequentialCommandGroup();

        for (int i = 0; i < noteLimit; i++) {  //amount of notes to get + 1 preloaded
            exitReturnCommands.addCommands(
                    //currently only using 1 limelight
                    // Set the limelight pipeline index to 1 for vision processing.
                    new InstantCommand(() -> LimelightHelpers.setPipelineIndex(LimelightConstants.llObjectDetectionName, 1)),
                    // Navigate the robot to the closest location without considering vision targeting.
                    swerve.pathfindToPosition(getClosestLocation()).onlyWhile(() -> !LimelightHelpers.getTV(LimelightConstants.llObjectDetectionName)),
                    // Navigate the robot to a specific location based on vision targeting.
                    swerve.pathfindToPosition(Vision.visionTargetLocation),
                    //new PathFindVision(swerve, score, possibleLocations, getClosestLocation()),
                    //or command.repeatedly also works for single command
                    //Commands.repeatingSequence(new PathFindVision(swerve, score, possibleLocations, getClosestLocation()).until(() -> score.isNoteInside())),
                    // Set the limelight pipeline index back to 0 for april tag localization.
                    //note inside logic doesn't work currently but no current spike implementation done
                    new InstantCommand(() -> LimelightHelpers.setPipelineIndex(LimelightConstants.llObjectDetectionName, 0)),
                    // Navigate the robot to the initial position to shoot.
                    swerve.pathfindToPosition(initialPos),
                    //swerve.pathfindToPosition(getClosestLocation()).onlyWhile(() -> !(LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.llObjectDetectionName).rawFiducials[0].id == 7)),
                    // Remove the closest location from the list of possible locations.
                    new InstantCommand(() -> possibleLocations.remove(getClosestLocation())),
                    // Execute the shooting command.
                    NamedCommands.getCommand("shoot")
                    //new AutoScoringCommand(score, ScoringState.FIRE, ScoringState.IDLE)
            );
        }

        return exitReturnCommands;
    }

    /**
     * Blacklists specified locations from the list of possible locations.
     *
     * @param blackListLocations A list of Pose2d objects representing locations to blacklist.
     */
    public void blackList(List<Pose2d> blackListLocations) {
        for (Pose2d pos : blackListLocations) {
            possibleLocations.remove(pos);
        }
    }

    /**
     * Finds the closest location to the current robot pose from the list of possible locations.
     *
     * @return The closest Pose2d location.
     */
    public Pose2d getClosestLocation() {
        return swerve.getPose().nearest(possibleLocations);
    }

    /**
     * Resets robot odometry to a flipped position if the alliance is red.
     *
     * @param loc The position to reset the robot odometry to.
     */
    public void flipResetOdometry(Pose2d loc) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            swerve.resetOdometry(FlippingUtil.flipFieldPose(loc));
        } else {
            swerve.resetOdometry(loc);
        }
    }
}
