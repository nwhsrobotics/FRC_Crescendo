package frc.robot.controllers;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.oi.Controller;

public class DriverXboxController implements Controller {
    private static final int port = 0;
    private final XboxController xboxController;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

    private static final SlewRateLimiter speedCoefficientSlewRateLimiter = new SlewRateLimiter(0.9);

    private static final double adjustSpeedAfterDeadband(double value, double deadband) {
        double speed = Controller.calculateSpeedWithDeadband(value, deadband) * 1.5;

        if (speed > 0) {
            speed = Math.min(speed, 1);
        }
        else {
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
    public int getIntendedUser() {
        return -1;
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
        /*if(xboxController.getLeftTriggerAxis() > xboxController.getRightTriggerAxis()) {
            return -xboxController.getLeftTriggerAxis() * 0.8 + 1.0;
        }
        return -xboxController.getRightTriggerAxis() * 0.8 + 1.0;*/
        //equation range from 0.2 to 1.0
        return -speedCoefficientSlewRateLimiter.calculate(xboxController.getLeftTriggerAxis()) * 0.65 + 1.0;
    }

    @Override
    public boolean isBoosterPressed() {
        //return xboxController.getRightBumper() || xboxController.getLeftBumper();
        return xboxController.getRightTriggerAxis() > 0.1;
    }

    @Override
    public int getAutonavigationButton() {
        return 1;
    }

    @Override
    public int getAutonavigateToAmpButton() {
        return 2;
    }

    @Override
    public int getAutonavigateToSpeakerButton() {
        return 3;
    }

    @Override
    public int getAutonavigateToSourceButton() {
        return 4;
    }

    /* TODO: Stage naviagte buttons
        @Override
    public int getAutonavigateToMidStageButton() {
        return 4;
    }
     */

    @Override
    public int getNavXResetButton() {
        return 8;
    }

    @Override
    public int getFieldRelativeButton() {
        return 9;
    }

    @Override
    public String getName() {
        return "4638 Xbox Controller";
    }
}
