package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command {
  IntakeSubsystem intakeSubsystem;
  XboxController gunner;

  public IntakeCmd(IntakeSubsystem intakeSubsystem, XboxController gunnerController) {
    this.intakeSubsystem = intakeSubsystem;
    this.gunner = gunnerController;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (gunner.getAButton()) {
      intakeSubsystem.backwards();
    }
    else if (gunner.getBButton()) {
      intakeSubsystem.forwards();
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
