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
  private int number;
  /** Creates a new PathFindFollowCmd. */
  public PathFindFollowCmd(SwerveSubsystem swerve, XboxController driver) {
    swerveSubsystem = swerve;
    controller = driver;
    addRequirements(swerveSubsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    number = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getRawButtonPressed(1)){
      if(number == 1){
      swerveSubsystem.pathFindThenFollowPath("Source");
      //or
      //if wanting to path find to exact coord then do this (odometry has to be perfect also the below example is of source)
      swerveSubsystem.pathFindToPos( new Pose2d(15.50, 1.01, Rotation2d.fromDegrees(15)));
      number = 2;
      } else {
        swerveSubsystem.pathfindingCommand.cancel();
        swerveSubsystem.pathfindingCommand = null;
        number = 1;
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
