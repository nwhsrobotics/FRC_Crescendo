package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristIntakeSubsystem;

public class WristIntakeCmd extends Command {
  WristIntakeSubsystem wristIntakeSubsystem;
  private boolean XButtonState;
  private boolean YButtonState;

  public WristIntakeCmd(WristIntakeSubsystem wristIntakeSubsystem, boolean XButtonState, boolean YButtonState) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;
    this.XButtonState = XButtonState;
    this.YButtonState = YButtonState;
    addRequirements(wristIntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (XButtonState) {
      wristIntakeSubsystem.forwards();
    }
    else if (YButtonState) {
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
