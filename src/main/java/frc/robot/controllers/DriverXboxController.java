package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.oi.Controller;

public class DriverXboxController implements Controller {
    private static final int port = 0;
    private final XboxController xboxController;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

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
        return Controller.calculateSpeedWithDeadband(-xboxController.getLeftY(), kXYDeadband);
        //return xLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftY(), kXYDeadband));
    }   
    
    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-xboxController.getLeftX(), kXYDeadband);
        //return yLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftX(), kXYDeadband));
    }
    
    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(xboxController.getRightX(), kZDeadband);
    }

    @Override
    public int getNavXResetButton() {
        return 8;
    }

    @Override
    public double getSpeedCoefficient(){
        if(xboxController.getLeftTriggerAxis() > xboxController.getRightTriggerAxis()) {
            return -xboxController.getLeftTriggerAxis() * 0.6 + 0.7;
        }
        return -xboxController.getRightTriggerAxis() * 0.6 + 0.7;
    }

    @Override
    public boolean isBoosterPressed(){
        return xboxController.getRightBumper() || xboxController.getLeftBumper();
    }
}
