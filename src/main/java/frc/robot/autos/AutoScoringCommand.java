// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ScoringSubsystem.ScoringState;

public class AutoScoringCommand extends Command {
    ScoringSubsystem score;
    boolean orginal;
    ScoringState state;
    ScoringState endState;

    /**
     * Creates a new ScoringCommand.
     */
    public AutoScoringCommand(ScoringSubsystem score, ScoringState state, ScoringState endState) {
        this.score = score;
        orginal = score.isNoteInside();
        this.state = state;
        this.endState = endState;
        addRequirements(score);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        score.state = state;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        score.state = endState;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return score.isNoteInside() != orginal;
    }
}
