// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightImplementation;
import frc.robot.Constants.FavoritePositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// This class represents an autonomous routine for an FRC robot.
public class Auto extends SequentialCommandGroup {
  private SwerveSubsystem swerve;
  private List<Pose2d> possibleLocations;
  private int noteLimit;
  private Pose2d initialPos;

  /** 
   * Creates a new instance of the Auto class.
   * @param swerve The SwerveSubsystem object representing the robot's swerve drive.
   * @param blackListLocations A list of Pose2d objects representing locations to blacklist.
   * @param noteLimit The limit on the number of notes to be obtained during autonomous + 1 preloaded.
   * @param initialPos The initial position to reset the robot odometry to.
   */
  public Auto(SwerveSubsystem swerve, List<Pose2d> blackListLocations, int noteLimit, Pose2d initialPos) {
    this.swerve = swerve;
    possibleLocations = FavoritePositions.allNotes;
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
   * @return A SequentialCommandGroup containing commands to get notes.
   */
  public SequentialCommandGroup getNotes() {
      SequentialCommandGroup exitReturnCommands = new SequentialCommandGroup();
  
      for (int i = 0; i < noteLimit; i++) {  //amount of notes to get + 1 preloaded
          exitReturnCommands.addCommands(
              // Set the limelight pipeline index to 1 for vision processing.
              new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 1)),
              // Navigate the robot to the closest location without considering vision targeting.
              swerve.pathfindToPosition(getClosestLocation()).onlyWhile(() -> !LimelightHelpers.getTV("limelight")),
              // Navigate the robot to a specific location based on vision targeting.
              swerve.pathfindToPosition(LimelightImplementation.visionTargetLocation),
              // Set the limelight pipeline index back to 0 for april tag localization.
              new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0)),
              // Navigate the robot to the initial position to shoot.
              swerve.pathfindToPosition(initialPos),
              // Remove the closest location from the list of possible locations.
              new InstantCommand(() -> possibleLocations.remove(getClosestLocation())),
              // Execute the shooting command.
              NamedCommands.getCommand("shoot")
          );
      }
  
      return exitReturnCommands;
  }

  /**
   * Blacklists specified locations from the list of possible locations.
   * @param blackListLocations A list of Pose2d objects representing locations to blacklist.
   */
  public void blackList(List<Pose2d> blackListLocations){
    for(Pose2d pos: blackListLocations){
      possibleLocations.remove(pos);
    }
  }
  
  /**
   * Finds the closest location to the current robot pose from the list of possible locations.
   * @return The closest Pose2d location.
   */
  public Pose2d getClosestLocation(){
    return swerve.getPose().nearest(possibleLocations);
  }

  /**
   * Resets robot odometry to a flipped position if the alliance is red.
   * @param loc The position to reset the robot odometry to.
   */
  public void flipResetOdometry(Pose2d loc){
    var alliance = DriverStation.getAlliance();
      if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
          swerve.resetOdometry(GeometryUtil.flipFieldPose(loc));
      } else {
          swerve.resetOdometry(loc); 
      }
  }
}
