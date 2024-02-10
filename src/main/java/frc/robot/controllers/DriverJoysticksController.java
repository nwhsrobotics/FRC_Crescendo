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
        return Controller.calculateSpeedWithDeadband(-joystickControl.getTwist() * 0.5, kZDeadband);
    }

    @Override
    public double getSpeedCoefficient() {
        //this equation does not work....
        //return (-joystickControl.getRawAxis(3)) * 0.3 + 0.5;
        //the problem with this working equation below is the range is from 0.2 to 1.2 we want max to be 1.0 to avoid confusion
        //return (((-joystickControl.getRawAxis(3)+1)/2)+.2);
        //so this should work
        double val = ((-joystickControl.getRawAxis(3)+1)/2);
        if(val < 0.2){
            return 0.2;
        } else {
            return val;
        }
        //this will also work and do the same thing (assiming 0.2 lower limit)
        //return (-joystickControl.getRawAxis(3)) * 0.5 + 0.5;
    }

    @Override
    public boolean isBoosterPressed() {
        return joystickControl.getTrigger();
    }

    @Override
    public int getAutonavigationButton() {
        return -1;
    }

    @Override
    public int getAutonavigateToAmpButton() {
        return -1;
    }

    @Override
    public int getAutonavigateToSourceButton() {
        return -1;
    }

    @Override
    public int getAutonavigateToSpeakerButton() {
        return -1;
    }

    @Override
    public int getNavXResetButton() {
        return 3;
    }

    @Override
    public int getFieldRelativeButton() {
        return 4;
    }

    @Override
    public String getName() {
        return "4638 Joystick";
    }
}
