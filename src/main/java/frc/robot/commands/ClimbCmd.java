// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCmd extends Command {
  private ClimbSubsystem climb;
  private XboxController xbox;
  /** Creates a new ClimbCmd. */
  public ClimbCmd(ClimbSubsystem climb, XboxController xbox) {
    this.xbox = xbox;
    this.climb = climb;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xbox.getLeftTriggerAxis() > 0.1){
      climb.moveDown();
    } else if (xbox.getRightTriggerAxis() > 0.1) {
      climb.moveUp();
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
