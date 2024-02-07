package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.oi.Controller;

public class DriverJoysticksController implements Controller {
    private static final int port = 0;
    private Joystick joystickControl;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

    public DriverJoysticksController() {
        this.joystickControl = new Joystick(DriverJoysticksController.port);
    }

    @Override
    public int getPort() {
        return DriverJoysticksController.port;
    }

    @Override
    public GenericHID getGenericHID() {
        return this.joystickControl;
    }

    @Override
    public int getIntendedUser() {
        return -1;
    }

    @Override
    public double getX() {
        return Controller.calculateSpeedWithDeadband(-this.joystickControl.getY(), kXYDeadband);
        //return xLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftY(), kXYDeadband));
    }

    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-this.joystickControl.getX(), kXYDeadband);
        //return yLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftX(), kXYDeadband));
    }

    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(this.joystickControl.getTwist(), kZDeadband);
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
