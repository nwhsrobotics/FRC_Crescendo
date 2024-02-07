package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.oi.Controller;

public class DriverXboxController implements Controller {
    private static final int port = 0;
    private XboxController xboxController;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

    public DriverXboxController() {
        this.xboxController = new XboxController(DriverXboxController.port);
    }

    @Override
    public int getPort() {
        return DriverXboxController.port;
    }

    @Override
    public GenericHID getGenericHID() {
        return this.xboxController;
    }

    @Override
    public int getIntendedUser() {
        return -1;
    }

    @Override
    public double getX() {
        return Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftY() * getSpeedCoeficient() *getSlowMode(), kXYDeadband);
        //return xLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftY(), kXYDeadband));
    }   
    
    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftX() * getSpeedCoeficient() *getSlowMode(), kXYDeadband);
        //return yLimiter.calculate(Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftX(), kXYDeadband));
    }
    
    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(this.xboxController.getRightX() * getSpeedCoeficient() * getSlowModeRotation(), kZDeadband);
    }

    public double getSpeedCoeficient() {
        if (this.xboxController.getRightBumper() || this.xboxController.getLeftBumper() ) {
            return (1.0/OIConstants.rateLimiter);
            //return 5.0/3.0;
        }
        else {
            return 1.0;
        }
    }

    public double getSlowMode(){
        if(xboxController.getLeftTriggerAxis() > 0.1 || xboxController.getRightTriggerAxis() > 0.1){
            return 0.2;
        }
        return 1.0;
    }

        public double getSlowModeRotation(){
        if(xboxController.getLeftTriggerAxis() > 0.1 || xboxController.getRightTriggerAxis() > 0.1){
            return 0.5;
        }
        return 1.0;
    }
    @Override
    public int getNavXResetButton() {
        return 8;
    }
}
