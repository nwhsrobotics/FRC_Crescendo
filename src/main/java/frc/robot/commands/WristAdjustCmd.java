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

    /**
     * Creates a new WristAdjustCmd.
     */
    public WristAdjustCmd(WristSubsystem wristSubsystem, XboxController gunner) {
        this.wristSubsystem = wristSubsystem;
        this.gunner = gunner;
        addRequirements(wristSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angleAdjustment = 0;

        if (gunner.getLeftTriggerAxis() > 0.1) {
            angleAdjustment = gunner.getLeftTriggerAxis();
        } else if (gunner.getRightTriggerAxis() > 0.1) {
            angleAdjustment = -gunner.getRightTriggerAxis();
        }

        wristSubsystem.adjustAngle(angleAdjustment);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
