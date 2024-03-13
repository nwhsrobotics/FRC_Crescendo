package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.oi.ControlManager;

public class SwerveJoystickDefaultCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;

    // constructor that initializes SwerveSubsystem, Joystick and adds SwerveSubsystem as a requirement
    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /*swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates((swerveSubsystem.isFieldRelative) && ControlManager.fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(ControlManager.Outputs.xSpeed, ControlManager.Outputs.ySpeed, ControlManager.Outputs.rotatingSpeed, swerveSubsystem.odometer.getEstimatedPosition().getRotation())
                : ChassisSpeeds.fromFieldRelativeSpeeds(ControlManager.Outputs.xSpeed, ControlManager.Outputs.ySpeed, ControlManager.Outputs.rotatingSpeed, new Rotation2d(0))));
        */
        ChassisSpeeds chassisSpeedsFromRelativeSpeeds;

        if (swerveSubsystem.isFieldRelative && ControlManager.fieldRelative) {
            chassisSpeedsFromRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ControlManager.Outputs.xSpeed, ControlManager.Outputs.ySpeed, ControlManager.Outputs.rotatingSpeed, swerveSubsystem.odometer.getEstimatedPosition().getRotation());
        } else {
            chassisSpeedsFromRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ControlManager.Outputs.xSpeed, ControlManager.Outputs.ySpeed, ControlManager.Outputs.rotatingSpeed, new Rotation2d(0));
        }
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeedsFromRelativeSpeeds));

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