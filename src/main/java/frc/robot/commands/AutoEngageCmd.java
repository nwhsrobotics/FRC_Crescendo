package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoEngageCmd extends Command {

  private SwerveSubsystem m_swerve;
  private double CRITICAL_ANGLE = 9;
  private double ENGAGE_SPEED_MPS = 0.5;

  public AutoEngageCmd(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    m_swerve = swerveSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //this following code autoEngages on the charging station in autonomous
    double pitch = m_swerve.getPitchDeg();

    if (pitch > CRITICAL_ANGLE) { // If the pitch is greater than the tilt angle
      m_swerve.setSpeed(ENGAGE_SPEED_MPS); // Set the speed to the engage speed
    }
    
    else if (pitch < -CRITICAL_ANGLE) { // If the pitch is less than the negative of the tilt angle
      m_swerve.setSpeed(-ENGAGE_SPEED_MPS); // Set the speed to the negative of the engage speed
    }
    
    else {
      m_swerve.setSpeed(0.0); // Set the speed to 0
      // m_swerve.brake();
    }
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;  //command never finishes on its own
  }
}
