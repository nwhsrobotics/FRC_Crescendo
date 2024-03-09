package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.oi.Controller;

public class DriverJoysticksController implements Controller {
    private static final int port = 2;
    private final Joystick joystickControl;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

    public DriverJoysticksController() {
        joystickControl = new Joystick(DriverJoysticksController.port);
    }

    @Override
    public int getPort() {
        return DriverJoysticksController.port;
    }

    @Override
    public GenericHID getGenericHID() {
        return joystickControl;
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
        return Controller.calculateSpeedWithDeadband(
                -Math.signum(joystickControl.getTwist()) * Math.pow(joystickControl.getTwist(), 2),
                kZDeadband
        );
    }

    @Override
    public double getSpeedCoefficient() {
        double val = ((-joystickControl.getRawAxis(3) + 1) / 2);
        return Math.max(val, 0.2);
    }

    @Override
    public boolean isBoosterPressed() {
        return joystickControl.getTrigger();
    }

    @Override
    public int getAutonavigationButton() {
        return 6;
    }

    @Override
    public int getAutonavigateToAmpButton() {
        return 4;
    }

    @Override
    public int getAutonavigateToSourceButton() {
        return 2;
    }

    @Override
    public int getAutonavigateToSpeakerButton() {
        return 3;
    }

    @Override
    public int getNavXResetButton() {
        return 8;
    }

    @Override
    public int getFieldRelativeButton() {
        return 7;
    }

    @Override
    public int getAutonavigateToObject() {
        return 12;
    }

    @Override
    public boolean aprilTagAlignButtonIsPressed() {
        return joystickControl.getRawButtonPressed(10);
    }

    @Override
    public String getName() {
        return "4638 Joystick";
    }

    //9 for vision odmetry
}
