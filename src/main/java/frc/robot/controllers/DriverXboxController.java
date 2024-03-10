package frc.robot.controllers;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.oi.Controller;

public class DriverXboxController implements Controller {
    private static final int port = 0;
    private final XboxController xboxController;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

    private static final SlewRateLimiter speedCoefficientSlewRateLimiter = new SlewRateLimiter(0.9);

    private static double adjustSpeedAfterDeadband(double value, double deadband) {
        double speed = Controller.calculateSpeedWithDeadband(value, deadband) * 1.5;

        if (speed > 0) {
            speed = Math.min(speed, 1);
        } else {
            speed = Math.max(speed, -1);
        }

        return speed;
    }

    public DriverXboxController() {
        xboxController = new XboxController(DriverXboxController.port);
    }

    @Override
    public int getPort() {
        return DriverXboxController.port;
    }

    @Override
    public GenericHID getGenericHID() {
        return xboxController;
    }

    @Override
    public double getX() {
        return DriverXboxController.adjustSpeedAfterDeadband(-xboxController.getLeftY(), kXYDeadband);
    }

    @Override
    public double getY() {
        return DriverXboxController.adjustSpeedAfterDeadband(-xboxController.getLeftX(), kXYDeadband);
    }

    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(-xboxController.getRightX(), kZDeadband);
    }

    @Override
    public double getSpeedCoefficient() {
        return -speedCoefficientSlewRateLimiter.calculate(xboxController.getLeftTriggerAxis()) * 0.65 + 1.0;
    }

    @Override
    public boolean isBoosterPressed() {
        return xboxController.getRightTriggerAxis() > 0.1;
    }

    @Override
    public int getAutonavigationButton() {
        return 10;
    }

    @Override
    public int getAutonavigateToAmpButton() {
        return 2;
    }

    @Override
    public int getAutonavigateToSpeakerButton() {
        return 1;
    }

    @Override
    public int getAutonavigateToSourceButton() {
        return 4;
    }

    @Override
    public int getAutonavigateToVisionTarget() {
        return 3;
    }

    @Override
    public int getNavXResetButton() {
        return 8;
    }

    @Override
    public int getOdometryVisionResetButton() {
        return 180 + Constants.OIConstants.kPOV;
    }

    @Override
    public int getNextPipelineButton() {
        return 0 + Constants.OIConstants.kPOV;
    }

    @Override
    public boolean visionTargetAlignButtonIsPressed() {
        return xboxController.getLeftBumper() || xboxController.getRightBumper();
    }

    @Override
    public int getFieldRelativeButton() {
        return 7;
    }

    @Override
    public String getName() {
        return "4638 Xbox Controller";
    }
}
