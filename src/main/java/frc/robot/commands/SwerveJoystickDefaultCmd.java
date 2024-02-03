package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.DriverXboxControls;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DriverControls driverControls;
    private final DriverXboxControls driverXboxControls;

    // constructor that initializes SwerveSubsystem, Joystick and adds SwerveSubsystem as a requirement
    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, DriverControls driverControls, DriverXboxControls driverXboxControls) {
        this.swerveSubsystem = swerveSubsystem;
        this.driverControls = driverControls;
        this.driverXboxControls = driverXboxControls;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // sets the module states for each module based on the ChassisSpeeds
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(driverControls.chassisSpeeds));
        //swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(driverXboxControls.chassisSpeeds));
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

