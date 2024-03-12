package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ScoringSubsystem.ScoringState;

/**
 * Set scoring subsystem to a state for a specified duration of time.
 */
public class SetScoringStateCommand extends Command {
    Timer timer;
    ScoringSubsystem subsystem;
    ScoringState state;
    double duration;

    public SetScoringStateCommand(ScoringSubsystem subsystem, ScoringState state, double duration) {
        this.subsystem = subsystem;
        this.state = state;
        this.duration = duration;
        timer = new Timer();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        subsystem.state = state;
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.state = ScoringState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
