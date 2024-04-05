// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCmd extends Command {
    private final ClimbSubsystem climb;
    public XboxController gunner;

    /**
     * Creates a new ClimbCmd.
     */
    public ClimbCmd(ClimbSubsystem climb, XboxController gunner) {
        this.climb = climb;
        this.gunner = gunner;
        addRequirements(climb);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    public void execute() {
        climb.leftClimbMotor.set(gunner.getLeftY() > .1 ? 1 : gunner.getLeftY() < -.1 ? -1 : 0);
        climb.rightClimbMotor.set(gunner.getRightY() > .1 ? 1 : gunner.getRightY() < -.1 ? -1 : 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
