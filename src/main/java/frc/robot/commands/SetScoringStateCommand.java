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
    ScoringState endState;

    public SetScoringStateCommand(ScoringSubsystem subsystem, ScoringState state, ScoringState endState, double duration) {
        this.subsystem = subsystem;
        this.state = state;
        this.endState = endState;
        this.duration = duration;
        timer = new Timer();
        addRequirements(subsystem);
    }

    public SetScoringStateCommand(ScoringSubsystem subsystem, ScoringState state, double duration) {
        this.subsystem = subsystem;
        this.state = state;
        this.endState = ScoringState.IDLE;
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
        //crazy logic to potentially fix note getting stuck, basically "in auto" we actually have an end state but not in teleop
        //so everytime we shoot in teleop after shooting ends, it will start unloading to get piece out so you can intake it again


        /*if(state == ScoringState.FIRE && endState == null){
            new SetScoringStateCommand(subsystem, ScoringState.UNLOADING, ScoringState.IDLE, 2).schedule();
        } else {
            subsystem.state = endState;
        }*/
        subsystem.state = endState;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
