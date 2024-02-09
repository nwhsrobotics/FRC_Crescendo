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
        return Controller.calculateSpeedWithDeadband(-joystickControl.getTwist(), kZDeadband);
    }

    @Override
    public double getSpeedCoefficient() {
        //return (-joystickControl.getRawAxis(3)) * 0.5 + 0.5;
        return (((-joystickControl.getRawAxis(3)+1)/2)+.2 //scale from -1 to 1
        );
    }

    @Override
    public boolean isBoosterPressed() {
        return joystickControl.getTrigger();
    }

    @Override
    public int getAutonavigationButton() {
        return 7;
    }

    @Override
    public int getAutonavigateToAmpButton() {
        return 6;
    }

    @Override
    public int getAutonavigateToSourceButton() {
        return 5;
    }

    @Override
    public int getAutonavigateToSpeakerButton() {
        return 8;
    }

    @Override
    public int getNavXResetButton() {
        return 3;
    }

    @Override
    public int getFieldRelativeButton() {
        return 2;
    }

    @Override
    public String getName() {
        return "Dwarakesh's Favorite Stick";
    }
}
