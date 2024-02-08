package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.oi.Controller;

public class DriverLeftJoysticksController implements Controller {
    private static final int port = 1;
    private final Joystick joystickControl;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

    public DriverLeftJoysticksController() {
        joystickControl = new Joystick(DriverLeftJoysticksController.port);
    }

    @Override
    public int getPort() {
        return DriverLeftJoysticksController.port;
    }

    @Override
    public GenericHID getGenericHID() {
        return joystickControl;
    }

    @Override
    public int getIntendedUser() {
        return -1;
    }

    @Override
    public double getX() {
        return Controller.calculateSpeedWithDeadband(-joystickControl.getY(), kXYDeadband);
    }

    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-joystickControl.getX(), kXYDeadband);
    }

    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(joystickControl.getTwist(), kZDeadband);
    }

    @Override
    public double getSpeedCoefficient() {
        return joystickControl.getRawAxis(3);
    }

    @Override
    public boolean isBoosterPressed() {
        return joystickControl.getTrigger();
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
    public int getAutonavigateToSourceButton() {
        return 3;
    }

    @Override
    public int getAutonavigateToSpeakerButton() {
        return 4;
    }

    @Override
    public int getNavXResetButton() {
        return 5;
    }

    @Override
    public int getFieldRelativeButton() {
        return 9;
    }
}
