// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
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
public class Auto extends SequentialCommandGroup {
  private SwerveSubsystem swerve;
  private List<Pose2d> possibleLocations;
  private int noteLimit;
  /** Creates a new Center. */
  public Auto(SwerveSubsystem swerve, List<Pose2d> blackListLocations, int noteLimit) {
    this.swerve = swerve;
    possibleLocations = FavoritePositions.allNotes;
    blackList(blackListLocations);
    this.noteLimit = noteLimit;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                  new InstantCommand(() -> swerve.resetOdometry(FavoritePositions.SPEAKER)),
                  NamedCommands.getCommand("autoInit"),
                  NamedCommands.getCommand("shoot"),
                  getNotes()
    );
  }
    
  public SequentialCommandGroup getNotes() {
      SequentialCommandGroup exitReturnCommands = new SequentialCommandGroup();
  
      for (int i = 0; i < noteLimit; i++) {  //amount of notes to get + 1 preloaded
          exitReturnCommands.addCommands(
              new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 1)),
              swerve.pathfindToPosition(getClosestLocation()).onlyWhile(() -> !LimelightHelpers.getTV("limelight")),
              swerve.pathfindToPosition(LimelightImplementation.visionTargetLocation),
              new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0)),
              swerve.pathfindToPosition(FavoritePositions.SPEAKER),
              new InstantCommand(() -> possibleLocations.remove(getClosestLocation())),
              NamedCommands.getCommand("shoot")
          );
      }
  
      return exitReturnCommands;
  }

  public void blackList(List<Pose2d> blackListLocations){
    for(Pose2d pos: blackListLocations){
      possibleLocations.remove(pos);
    }
  }
  
    
    public Pose2d getClosestLocation(){
      return swerve.getPose().nearest(possibleLocations);
    }
}
