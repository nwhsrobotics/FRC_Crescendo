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
    }   
    
    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-xboxController.getLeftX(), kXYDeadband);
    }
    
    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(xboxController.getRightX(), kZDeadband);
    }

    @Override
    public double getSpeedCoefficient(){
        /*if(xboxController.getLeftTriggerAxis() > xboxController.getRightTriggerAxis()) {
            return -xboxController.getLeftTriggerAxis() * 0.8 + 1.0;
        }
        return -xboxController.getRightTriggerAxis() * 0.8 + 1.0;*/
        return -xboxController.getLeftTriggerAxis() * 0.8 + 1.0;
    }

    //TODO: Check people preferences
    @Override
    public boolean isBoosterPressed(){
        //return xboxController.getRightBumper() || xboxController.getLeftBumper();
        return xboxController.getRightTriggerAxis() > 0.1;
    }

    @Override
    public int getAutonavigationButton(){
        return 1;
    }

    @Override
    public int getAutonavigateToAmpButton(){
        return 2;
    }

    @Override
    public int getAutonavigateToSourceButton(){
        return 3;
    }

    @Override
    public int getAutonavigateToStageButton(){
        return 4;
    }
    
    @Override
    public int getNavXResetButton() {
        return 8;
    }

    @Override
    public int getFieldRelativeButton() {
        return 9;
    }
}
