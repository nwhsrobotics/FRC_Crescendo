package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command {
  IntakeSubsystem intakeSubsystem;
  private boolean AButtonState;
  private boolean BButtonState;

  public IntakeCmd(IntakeSubsystem intakeSubsystem, boolean AButtonState, boolean BButtonState) {
    this.intakeSubsystem = intakeSubsystem;
    this.AButtonState = AButtonState;
    this.BButtonState = BButtonState;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (AButtonState) {
      intakeSubsystem.forwards();
    }
    else if (BButtonState) {
      intakeSubsystem.backwards();
    }
    else {
      intakeSubsystem.stop();
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
