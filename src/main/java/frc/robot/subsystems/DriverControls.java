package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;

/**
 * Handles driver inputs.
 * <p>
 * This should be initialized in "RobotContainer" and ran under ".teleopPeriodic" of "Robot."
 */
public class DriverControls {
    private final Joystick controllerDriver;
    private final SwerveSubsystem swerve;

    public double speedCoefficient;
    public double xSpeed;
    public double ySpeed;
    public double rotatingSpeed;
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public DriverControls(SwerveSubsystem swerve, Joystick controllerDriver) {
        this.swerve = swerve;
        this.controllerDriver = controllerDriver;
    }

    /**
     * Process driver inputs.
     */
    public void processCycle() {
        // calculates the speed coefficient based on the state of the trigger and the joystick axis.
        double speedCoefficient = controllerDriver.getTrigger() ? -1 : (controllerDriver.getRawAxis(3));

        // calculates the xSpeed, ySpeed and rotatingSpeed based on the joystick axis and deadbands
        xSpeed = Math.abs(controllerDriver.getY()) < OIConstants.kXYDeadband ? 0
                : controllerDriver.getY() > 0
                ? (controllerDriver.getY() - OIConstants.kXYDeadband) * OIConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband))
                : (controllerDriver.getY() + OIConstants.kXYDeadband) * OIConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband));
        ySpeed = Math.abs(controllerDriver.getX()) < OIConstants.kXYDeadband ? 0
                : controllerDriver.getX() > 0
                ? (controllerDriver.getX() - OIConstants.kXYDeadband) * OIConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband))
                : (controllerDriver.getX() + OIConstants.kXYDeadband) * OIConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband));
        rotatingSpeed = Math.abs(-controllerDriver.getTwist()) < OIConstants.kZDeadband ? 0
                : -controllerDriver.getTwist() > 0
                ? (((OIConstants.isLefty) ? (controllerDriver.getTwist()) : -controllerDriver.getTwist()) - OIConstants.kZDeadband) * OIConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient
                : (controllerDriver.getTwist() + OIConstants.kZDeadband) * OIConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient;

        // calculates the ChassisSpeeds based on the xSpeed, ySpeed, and rotatingSpeed
        chassisSpeeds = (swerve.isFieldRelative)
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, Rotation2d.fromDegrees(swerve.getHeading()))
                : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, new Rotation2d(0));

        if (xSpeed != 0 || ySpeed != 0 || rotatingSpeed != 0) {
            swerve.autonavigator.pauseNavigation();
        } else {
            swerve.autonavigator.resumeNavigation();
        }
    }
}
