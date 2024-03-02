package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command {
  IntakeSubsystem intakeSubsystem;
  private boolean isForward;

  public IntakeCmd(IntakeSubsystem intakeSubsystem, boolean isForward) {
    this.intakeSubsystem = intakeSubsystem;
    this.isForward = isForward;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (isForward) {
      intakeSubsystem.forwards();
    }
    else {
      intakeSubsystem.backwards();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
