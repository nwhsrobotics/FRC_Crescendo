package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Joystick m_driver;

    // constructor that initializes SwerveSubsystem, Joystick and adds SwerveSubsystem as a requirement
    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, Joystick m_driver) {
        this.swerveSubsystem = swerveSubsystem;
        this.m_driver = m_driver;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // calculates the speed coefficient based on the state of the trigger and the joystick axis
        double speedCoefficient = m_driver.getTrigger() ? 1 : (-m_driver.getRawAxis(3)) * 0.3 + 0.5;

        // calculates the xSpeed, ySpeed and rotatingSpeed based on the joystick axis and deadbands
        double xSpeed = Math.abs(-m_driver.getY()) < OIConstants.kXYDeadband ? 0
                : -m_driver.getY() > 0
                        ? (-m_driver.getY() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband))
                        : (-m_driver.getY() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband));
        double ySpeed = Math.abs(-m_driver.getX()) < OIConstants.kXYDeadband ? 0
                : -m_driver.getX() > 0
                        ? (-m_driver.getX() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband))
                        : (-m_driver.getX() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband));
        double rotatingSpeed = Math.abs(-m_driver.getTwist()) < OIConstants.kZDeadband ? 0
                : -m_driver.getTwist() > 0
                        ? (-m_driver.getTwist() - OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient
                        : (-m_driver.getTwist() + OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient;

        // calculates the ChassisSpeeds based on the xSpeed, ySpeed, and rotatingSpeed
        ChassisSpeeds chassisSpeeds = (swerveSubsystem.isFR)
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, Rotation2d.fromDegrees(swerveSubsystem.getHeading()))
                : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, new Rotation2d(0));

        if(xSpeed != 0 && ySpeed != 0 && rotatingSpeed != 0){
            swerveSubsystem.pathfindingCommand.cancel();
            swerveSubsystem.pathfindingCommand = null;
        }
        if(xSpeed == 0 && ySpeed == 0 && rotatingSpeed == 0){
            swerveSubsystem.pathFindThenFollowPath(swerveSubsystem.lastPath);
        }

        // sets the module states for each module based on the ChassisSpeeds
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        // stops all the modules in the SwerveSubsystem
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
