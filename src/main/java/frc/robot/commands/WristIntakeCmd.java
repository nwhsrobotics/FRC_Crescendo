package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristIntakeSubsystem;

public class WristIntakeCmd extends Command {
    WristIntakeSubsystem wristIntakeSubsystem;
    private final XboxController gunner;

    public WristIntakeCmd(WristIntakeSubsystem wristIntakeSubsystem, XboxController gunner) {
        this.wristIntakeSubsystem = wristIntakeSubsystem;
        this.gunner = gunner;
        addRequirements(wristIntakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (gunner.getXButton()) {
            wristIntakeSubsystem.forwards();
        } else if (gunner.getYButton()) {
            wristIntakeSubsystem.backwards();
        }
        else {
            
            wristIntakeSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
