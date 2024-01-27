// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveAutoWait extends Command {
  public RobotContainer robotContainer;
  public long time;
  /** Creates a new SwerveAutoWait. */
  public SwerveAutoWait(RobotContainer container) {
    robotContainer = container;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotContainer.getAutonomousCommand().cancel();
    time = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotContainer.swerveSubsystem.stopModules();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotContainer.getAutonomousCommand().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > 1000+time;
  }
}
