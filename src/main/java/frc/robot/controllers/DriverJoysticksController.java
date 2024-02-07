package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.oi.Controller;

public class DriverJoysticksController implements Controller {
    private static final int port = 0;
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
        return Controller.calculateSpeedWithDeadband(joystickControl.getTwist(), kZDeadband);
    }


    @Override
    public double getSpeedCoefficient(){
        return joystickControl.getRawAxis(3);
    }

    @Override
    public boolean isBoosterPressed(){
        return joystickControl.getTrigger();
    }
    @Override
    public int getNavXResetButton() {
        return 5;
    }


}
