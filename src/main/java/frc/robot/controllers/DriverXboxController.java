package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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
        return Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftY(), kXYDeadband);
    }   
    
    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-this.xboxController.getLeftX(), kXYDeadband);
    }
    
    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(this.xboxController.getRightX(), kZDeadband);
    }

    @Override
    public int getNavXResetButton() {
        return 5;
    }
}
