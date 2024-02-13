package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.oi.Controller;

public class GunnerXboxController implements Controller{
    private static final int port = 3;
    private final XboxController xboxController;
    public GunnerXboxController() {
        xboxController = new XboxController(GunnerXboxController.port);
    }

    @Override
    public String getName() {
        return "Gunner Controller";
    }
    @Override
    public int getPort() {
        return GunnerXboxController.port;
    }

    @Override
    public GenericHID getGenericHID() {
        return xboxController;
    }
}
