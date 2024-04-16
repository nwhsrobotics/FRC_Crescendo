// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmAdjustCmd extends Command {
    public ArmSubsystem armSubsystem;
    public XboxController gunner;

    /**
     * Creates a new ArmAdjustCmd.
     */
    public ArmAdjustCmd(ArmSubsystem armSubsystem, XboxController gunner) {
        this.armSubsystem = armSubsystem;
        this.gunner = gunner;
        addRequirements(armSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rotationAdjustment = 0;

        if (gunner.getBackButton()) {
            rotationAdjustment = 0.2;
        } else if (gunner.getStartButton()) {
            rotationAdjustment = -0.2;
        }

        armSubsystem.adjustByRotations(rotationAdjustment);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.rightShoulderMotor.stopMotor();
        armSubsystem.leftShoulderMotor.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
