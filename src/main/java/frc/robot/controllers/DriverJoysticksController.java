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
        return Controller.calculateSpeedWithDeadband(-this.joystickControl.getY() * getSpeedCoeficient() *getSlowMode(), kXYDeadband);
        //return xLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftY(), kXYDeadband));
    }

    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-this.joystickControl.getX() * getSpeedCoeficient() *getSlowMode(), kXYDeadband);
        //return yLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftX(), kXYDeadband));
    }

    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(this.joystickControl.getTwist() * getSpeedCoeficient() * getSlowModeRotation(), kZDeadband);
    }

    @Override
    public double getBoosterCoefficient(){
        if (this.joystickControl.getTrigger()) {
            return (1.0/ Constants.OIConstants.scaleFactor);
        }
        else {
            return 1.0;
        }
    }

    public double getSpeedCoeficient() {
        if (this.joystickControl.getTrigger()) {
            return (1.0/ Constants.OIConstants.scaleFactor);
            //return 5.0/3.0;
        }
        else {
            return 1.0;
        }
    }

    public double getSlowMode(){
        if(joystickControl.getLeftTriggerAxis() > 0.1 || joystickControl.getRightTriggerAxis() > 0.1){
            return 0.2;
        }
        return 1.0;
    }

    public double getSlowModeRotation(){
        if(joystickControl.getLeftTriggerAxis() > 0.1 || joystickControl.getRightTriggerAxis() > 0.1){
            return 0.5;
        }
        return 1.0;
    }
    @Override
    public int getNavXResetButton() {
        return 8;
    }
}

}
