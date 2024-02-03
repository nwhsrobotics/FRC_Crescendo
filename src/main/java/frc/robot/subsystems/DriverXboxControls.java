package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

/**
 * Handles driver inputs.
 * <p>
 * This should be initialized in "RobotContainer" and ran under ".teleopPeriodic" of "Robot."
 */
public class DriverXboxControls {
    private final XboxController controllerDriver;
    private final SwerveSubsystem swerve;

    public double speedCoefficient;
    public double xSpeed;
    public double ySpeed;
    public double rotatingSpeed;
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // Constructor to initialize the DriverXboxControls with a SwerveSubsystem and an XboxController
    public DriverXboxControls(SwerveSubsystem swerve, XboxController controllerDriver) {
        this.swerve = swerve;
        this.controllerDriver = controllerDriver;
    }

    /**
     * Process driver inputs.
     */
    public void processCycle() {
        // Set a default speed coefficient (can be adjusted later)
        //speedCoefficient = controllerDriver.getRightBumper() ? 1 : (controllerDriver.getRawAxis(3));
        speedCoefficient = 0.6;

        // Increases the speed if the left joystick button is pressed.
        //speedCoefficient = controllerDriver.getLeftStickButton() ? 1 : (controllerDriver.getRawAxis(3));

        // Calculate x, y, and rotation speeds with deadband applied
        xSpeed = calculateSpeedWithDeadband(-controllerDriver.getLeftY(), OIConstants.kXYDeadband) * speedCoefficient * OIConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = calculateSpeedWithDeadband(-controllerDriver.getLeftX(), OIConstants.kXYDeadband) * speedCoefficient * OIConstants.kTeleDriveMaxSpeedMetersPerSecond;
        rotatingSpeed = calculateSpeedWithDeadband(controllerDriver.getRightY(), OIConstants.kZDeadband) * speedCoefficient * OIConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;


        // Calculate ChassisSpeeds based on field-relative or robot-relative mode
        Rotation2d rotation;
        if (swerve.isFieldRelative) {
          //so this is field relative get current heading rotation
            rotation = Rotation2d.fromDegrees(swerve.getHeading());
        } else {
          // not field relative then rotation is always 0
            rotation = new Rotation2d(0);
        }        
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, rotation);

        // Pause or resume autonavigation based on user input
        if (xSpeed != 0 || ySpeed != 0 || rotatingSpeed != 0) {
            swerve.autonavigator.pauseNavigation();
        } else {
            swerve.autonavigator.resumeNavigation();
        }
    }

    // Function to calculate speed with deadband applied
    private double calculateSpeedWithDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            // If the value is within the deadband, consider it as zero
            return 0;
        } else if (value > 0) {
            // Scale positive values to eliminate deadband
            return (value - deadband) / (1 - deadband);
        } else {
            // Scale negative values to eliminate deadband
            return (value + deadband) / (1 - deadband);
        }
    }
}
