package frc.robot.subsystems.oi;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

/**
 * Handles multiple control interfaces at the driver station.
 */
public class ControlManager {
    /**
     * Outputs derived from control interfaces.
     */
    public static class Outputs {
        public static double xSpeed = 0;
        public static double ySpeed = 0;
        public static double rotatingSpeed = 0;
    }
    
    private static final Command emptyButtonCommand = new InstantCommand(() -> System.out.println("Achtung! A button command for ControlManager is undefined!"));

    /**
     * Commands that are to be bound to button presses made by the driver.
     * 
     * These should be overwritten during robot initialization,
     * replacing the default placeholders.
     */
    public static class DriverButtonCommands {
        public static Command navXResetCommand = emptyButtonCommand;
        public static Command toggleFieldRelativeCommand = emptyButtonCommand;
        public static Command toggleAutonavigationCommand = emptyButtonCommand;
        public static Command autonavigateToAmp = emptyButtonCommand;
        public static Command autonavigateToSource = emptyButtonCommand;
        public static Command autonavigateToStage = emptyButtonCommand;
    }

    /**
     * Commands that are to be bound to button presses made by the gunner.
     * 
     * These should be overwritten during robot initialization,
     * replacing the default placeholders.
     */
    public static class GunnerButtonCommands {}

    private static HashMap<Integer, Controller> registry = new HashMap<>();
    private static int driverPort = -1;
    private static int gunnerPort = -2;

    /**
     * Generates a "Trigger" for a command to be bound to a button (defined in "DriverButtonCommands" and "GunnerButtonCommands").
     * 
     * The command will not executed by the Trigger,
     * unless the port of the driver/gunner controller it is associated with is currently selected to be active.
     * 
     * Internal use only.
     * 
     * @param hid - "GenericHID" of the controller.
     * @param port - the port of associated controller.
     * @param command - the command to be executed when the button is pressed.
     * @param isCommandDriver - whether the command is for the driver or gunner.
     */
    private static void makeTriggerForButton(GenericHID hid, int port, int button, Command command, boolean isCommandDriver) {
        if (button == -1) {  // -1 means it should not be bound.
            return;
        }

        new Trigger(() -> hid.getRawButton(button)).onTrue(new InstantCommand(() -> {
            if ((isCommandDriver && port != driverPort) || (!isCommandDriver && port != gunnerPort)) {
                return;
            }

           command.schedule(); 
        }));
    }

    /**
     * Register a controller (which should already be associated with a particular port).
     * 
     * You should NEVER register multiple controllers to a single port.
     * If a port is already occupied, a message will be printed in the console,
     * assuming the WPILib framework doesn't immediately crash.
     * 
     * @param controller - controller being registered.
     */
    public static void registerController(Controller controller) {
        if (ControlManager.registry.put(controller.getPort(), controller) != null) {
            System.out.println("Achtung! Handler for OI overwrote controller registered for port " + Integer.toString(controller.getPort()) + ".");
        }

        GenericHID hid = controller.getGenericHID();
        makeTriggerForButton(hid, controller.getPort(), controller.getNavXResetButton(), DriverButtonCommands.navXResetCommand, true);
        makeTriggerForButton(hid, controller.getPort(), controller.getFieldRelativeButton(), DriverButtonCommands.toggleFieldRelativeCommand, true);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigationButton(), DriverButtonCommands.toggleAutonavigationCommand, true);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToAmpButton(), DriverButtonCommands.autonavigateToAmp, true);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToSourceButton(), DriverButtonCommands.autonavigateToSource, true);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToStageButton(), DriverButtonCommands.autonavigateToStage, true);

        Logger.recordOutput("controlmanager.controller." + Integer.toString(controller.getPort()) + ".name", controller.getName());
    }

    /**
     * Get the current port actively being used for driver inputs.
     * 
     * @return - integer ID for port.
     */
    public static int getDriverPort() {
        return ControlManager.driverPort;
    }
    
    /**
     * Set the current port actively being used for driver inputs.
     * 
     * This method will be non-fatally unsuccessful,
     * if no controller is registered with the new port,
     * or if the controller is already being used by the gunner.
     * 
     * @return - boolean for whether change was successful.
     */
    public static boolean setDriverPort(int port) {
        if (ControlManager.registry.get(port) == null || port == ControlManager.gunnerPort) {
            return false;
        }

        ControlManager.driverPort = port;

        return true;
    }

    /**
     * Get the current port actively being used for gunner inputs.
     * 
     * @return - integer ID for port.
     */
    public static int getGunnerPort() {
        return ControlManager.gunnerPort;
    }

    /**
     * Set the current port actively being used for gunner inputs.
     * 
     * This method will be non-fatally unsuccessful,
     * if no controller is registered with the new port,
     * or if the controller is already being used by the driver.
     * 
     * @return - boolean for whether change was successful.
     */
    public static boolean setGunnerPort(int port) {
        if (ControlManager.registry.get(port) == null || port == ControlManager.driverPort) {
            return false;
        }

        ControlManager.gunnerPort = port;

        return true;
    }

    /**
     * Get list of available controllers.
     * 
     * @param areDrivers - whether to filter the list for only driver-enabled or gunner-enabled controllers.
     * @return - list of integer IDs for controller ports.
     */
    public static ArrayList<Integer> getControllers(boolean areDrivers) {
        ArrayList<Integer> controllers = new ArrayList<>();

        for (int port : ControlManager.registry.keySet()) {
            Controller controller = ControlManager.registry.get(port);
            if (areDrivers && controller.getIntendedUser() > 0 || !areDrivers && controller.getIntendedUser() < 0) {
                continue;
            }

            controllers.add(port);
        }

        return controllers;
    }

    /**
     * Get available controller with the lowest integer ID for port.
     * 
     * If no controllers are available, -1 is returned.
     * 
     * @param isDriver - whether the controller is driver-enabled or gunner-enabled.
     * @return - integer ID for controller port.
     */
    public static int getControllerLowest(boolean isDriver) {
        ArrayList<Integer> controllers = getControllers(false);
        
        if (controllers.size() <= 0) {
            return -1;
        }
        
        controllers.sort(Comparator.naturalOrder());
        return controllers.get(0);
    }

    /**
     * Get label for a controller.
     * 
     * If the controller does not exist, the string "DNE" will be returned.
     * 
     * @param port - integer ID for port.
     * @return - label.
     */
    public static String getControllerLabel(int port) {
        Controller controller = ControlManager.registry.get(port);
        if (controller == null) {
            return "DNE";
        }

        return Integer.toString(port) + " - " + controller.getName();
    }

    /**
     * Get list of labels of available controllers.
     * 
     * @param areDrivers - whether to filter the list for only driver-enabled or gunner-enabled controllers.
     * @return - list of labels for controllers.
     */
    public static ArrayList<String> getControllerLabels(boolean areDrivers) {
        return (ArrayList<String>) ControlManager.getControllers(areDrivers)
            .stream()
            .map(port -> ControlManager.getControllerLabel(port))
            .collect(Collectors.toList());
    }

    /**
     * Fetches port number for controller, from its label, as produced by "getControllers()."
     * 
     * @param label - label for controller.
     * @return - integer ID for port.
     */
    public static int getControllerPortFromLabel(String label) {
        return Integer.valueOf(label.split(" - ")[0]);
    }

    /**
     * Process inputs for the driver controls.
     * 
     * Should be called repeatedly in a periodic function.
     */
    public static void processDriver() {
        Logger.recordOutput("controlmanager.driver.port", ControlManager.driverPort);
        
        Controller driverController = ControlManager.registry.get(ControlManager.driverPort);
        if (driverController == null) {
            // kill speed if we've lost the driver controller.
            ControlManager.Outputs.xSpeed = 0;
            ControlManager.Outputs.ySpeed = 0;
            ControlManager.Outputs.rotatingSpeed = 0;
            
            System.out.println("Achtung! Failed to get driver controller!");
            return;
        }
        
        //TODO: Is this right the xLimiter/yLimiter is greater than the kTeleDriveMaxSpeedMetersPerSecond???? Recommend it to be lower
        if(!driverController.isBoosterPressed()) {
            ControlManager.Outputs.xSpeed = OIConstants.xLimiter.calculate(driverController.getX() * OIConstants.kTeleDriveMaxSpeedMetersPerSecond);
            ControlManager.Outputs.ySpeed = OIConstants.yLimiter.calculate(driverController.getY() * OIConstants.kTeleDriveMaxSpeedMetersPerSecond);
            ControlManager.Outputs.rotatingSpeed = OIConstants.zLimiter.calculate(driverController.getZ() * OIConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
        } else {
            //TODO: Determine if slew rate limiters should apply if boosters are toggled
            ControlManager.Outputs.xSpeed = OIConstants.xLimiter.calculate(driverController.getX() * DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            ControlManager.Outputs.ySpeed = OIConstants.yLimiter.calculate(driverController.getY() * DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            ControlManager.Outputs.rotatingSpeed = OIConstants.zLimiter.calculate(driverController.getZ() * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond);
        }
    }

    /**
     * Process inputs for the gunner controls.
     * 
     * Should be called repeatedly in a periodic function.
     */
    public static void processGunner() {
        Logger.recordOutput("controlmanager.gunner.port", ControlManager.gunnerPort);

        Controller gunnerController = ControlManager.registry.get(ControlManager.driverPort);
        if (gunnerController == null) {
            // insert calls here to set outputs that stow the gunner mechanisms safely.

            System.out.println("Achtung! Failed to get gunner controller!");
            return;
        }
    }
}