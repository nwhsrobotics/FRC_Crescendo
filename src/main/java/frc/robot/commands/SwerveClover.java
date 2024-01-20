package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveClover extends Command {

    private final SwerveSubsystem swerveSubsystem;

    // constructor that initializes SwerveSubsystem, Joystick and adds SwerveSubsystem as a requirement
    public SwerveClover(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.brake();
    }

    @Override
    public void end(boolean interrupted) {
        // stops all the modules in the SwerveSubsystem
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.isClover();
    }
}

