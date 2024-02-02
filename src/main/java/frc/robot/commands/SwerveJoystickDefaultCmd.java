package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DriverControls driverControls;

    // constructor that initializes SwerveSubsystem, Joystick and adds SwerveSubsystem as a requirement
    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, DriverControls driverControls) {
        this.swerveSubsystem = swerveSubsystem;
        this.driverControls = driverControls;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // sets the module states for each module based on the ChassisSpeeds
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(driverControls.chassisSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

