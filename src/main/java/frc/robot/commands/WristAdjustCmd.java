// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristAdjustCmd extends Command {
  public WristSubsystem wristSubsystem;
  public XboxController gunner;
  /** Creates a new WristAdjustCmd. */
  public WristAdjustCmd(WristSubsystem wristSubsystem, XboxController gunner) {
    this.wristSubsystem = wristSubsystem;
    this.gunner = gunner;
    addRequirements(wristSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gunner.getLeftTriggerAxis() > 0.1){
      wristSubsystem.wristMotor.set(-gunner.getLeftTriggerAxis() * 0.1);

    }
    else if (gunner.getRightTriggerAxis() > .1) {
      wristSubsystem.wristMotor.set(gunner.getRightTriggerAxis() * 0.1);
    }
    else {
      wristSubsystem.wristMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.wristMotor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
