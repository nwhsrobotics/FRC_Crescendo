package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristIntakeSubsystem;

public class WristIntakeCmd extends Command {
  WristIntakeSubsystem wristIntakeSubsystem;
  private boolean isForward;

  public WristIntakeCmd(WristIntakeSubsystem wristIntakeSubsystem, boolean isForward) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;
    this.isForward = isForward;
    addRequirements(wristIntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (isForward) {
      wristIntakeSubsystem.forwards();
    }
    else {
      wristIntakeSubsystem.backwards();
    }
  }

  @Override
  public void end(boolean interrupted) {
    wristIntakeSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
