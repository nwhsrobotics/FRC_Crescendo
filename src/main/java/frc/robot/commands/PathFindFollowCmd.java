// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class PathFindFollowCmd extends Command {
  private SwerveSubsystem swerveSubsystem;
  private XboxController controller;
  public static boolean pathControlToggle;
  /** Creates a new PathFindFollowCmd. */
  public PathFindFollowCmd(SwerveSubsystem swerve, XboxController driver) {
    swerveSubsystem = swerve;
    controller = driver;
    addRequirements(swerveSubsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathControlToggle = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getRawButtonPressed(1)){
      if(pathControlToggle){
        swerveSubsystem.pathfindingCommand.cancel();
        swerveSubsystem.pathfindingCommand = null;
      //this will basically take to the starting of the path then follow the path (only recommended if on opposite side of field away from source)
      swerveSubsystem.pathFindThenFollowPath("Source");
      //or if in middle of the field
      //if wanting to path find to exact coord then do this (odometry has to be perfect also the below example is of source)
        //TODO: having more cameras on a robot like limelights will drastically increase field odometry accuracy with pose estimation
        //TODO: Use pos esmitators to combine vision with navx, gyro for very accurate field relative pos (drag and drop in replacement for odometry classes) https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
        //camera pos estimating accuracy for encoder drift 
        //or just use limelight instead of odometry to get distance and rotation to tags and add onto current pos2d for target pos2d
      //swerveSubsystem.pathFindToPos( new Pose2d(15.50, 1.01, Rotation2d.fromDegrees(15)));
      pathControlToggle = false;
      } else {
        swerveSubsystem.pathfindingCommand.cancel();
        swerveSubsystem.pathfindingCommand = null;
        pathControlToggle = true;
      }
    } 
    if(controller.getRawButtonPressed(2)){
      if(pathControlToggle){
      swerveSubsystem.pathfindingCommand.cancel();
      swerveSubsystem.pathfindingCommand = null;
      //or if in middle of the field as it will go to this specific point
      swerveSubsystem.pathFindToPos( new Pose2d(15.50, 1.01, Rotation2d.fromDegrees(15))); //pos in meters
      pathControlToggle = false;
      } else {
        swerveSubsystem.pathfindingCommand.cancel();
        swerveSubsystem.pathfindingCommand = null;
        pathControlToggle = true;
      }
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
