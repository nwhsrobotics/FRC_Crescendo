package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.limelight.LimelightImplementation;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.stream.Collectors;

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
     * <p>
     * These should be overwritten during robot initialization,
     * replacing the default placeholders.
     */
    public static class DriverButtonCommands {
        public static Command navXResetCommand = emptyButtonCommand;
        public static Command toggleFieldRelativeCommand = emptyButtonCommand;
        public static Command toggleAutonavigationCommand = emptyButtonCommand;
        public static Command autonavigateToAmp = emptyButtonCommand;
        public static Command autonavigateToSource = emptyButtonCommand;
        public static Command autonavigateToSpeaker = emptyButtonCommand;
        public static Command autonavigateToTopStage = emptyButtonCommand;
        public static Command autonavigateToMidStage = emptyButtonCommand;
        public static Command autonavigateToBottomStage = emptyButtonCommand;
        public static Command autonavigateToVisionTarget = emptyButtonCommand;
        public static Command odometryVisionReset = emptyButtonCommand;
        public static Command nextPipeline = emptyButtonCommand;
    }

    private static final HashMap<Integer, Controller> registry = new HashMap<>();
    private static int driverPort = -1;
    private static int reservedPort = -2;
    public static boolean fieldRelative = true;

    /**
     * Generates a "Trigger" for a command to be bound to a button (defined in "DriverButtonCommands").
     * <p>
     * The command will not executed by the Trigger,
     * unless the port of the driver controller it is associated with is currently selected to be active.
     * <p>
     * Internal use only.
     *
     * @param hid     - "GenericHID" of the controller.
     * @param port    - the port of associated controller.
     * @param command - the command to be executed when the button is pressed.
     */
    private static void makeTriggerForButton(GenericHID hid, int port, int button, Command command) {
        if (button == -1) {  // -1 means it should not be bound.
            return;
        }
        if (button < OIConstants.kPOV) {
            new Trigger(() -> hid.getRawButton(button)).onTrue(new InstantCommand(() -> {
                if (port != driverPort) {
                    return;
                }

                command.schedule();
            }));
        } else {
            new Trigger(() -> hid.getPOV() == (button - OIConstants.kPOV)).onTrue(new InstantCommand(() -> {
                if (port != driverPort) {
                    return;
                }
                command.schedule();
            }));
        }
    }

    /**
     * Reserve a controller (for the gunner, which is not managed by ControlManager).
     * <p>
     * If the port is already registered, this method will silently do nothing.
     * Be sure to call this method before controller registration.
     *
     * @param port - port of the reserved controller.
     */
    public static void reserveController(int port) {
        if (registry.get(port) != null) {
            ControlManager.reservedPort = port;
        }
    }

    /**
     * Register a controller (which should already be associated with a particular port).
     * <p>
     * You should NEVER register multiple controllers to a single port.
     * If a port is already occupied, a message will be printed in the console,
     * assuming the WPILib framework doesn't immediately crash.
     *
     * @param controller - controller being registered.
     */
    public static void registerController(Controller controller) {
        if (controller.getPort() == ControlManager.reservedPort) {
            System.out.println("Achtung! Handler for OI dropped controller registered for already reserved port " + controller.getPort() + ".");
            return;
        }

        if (ControlManager.registry.put(controller.getPort(), controller) != null) {
            System.out.println("Achtung! Handler for OI overwrote controller registered for port " + controller.getPort() + ".");
        }

        GenericHID hid = controller.getGenericHID();
        makeTriggerForButton(hid, controller.getPort(), controller.getNavXResetButton(), DriverButtonCommands.navXResetCommand);
        makeTriggerForButton(hid, controller.getPort(), controller.getFieldRelativeButton(), DriverButtonCommands.toggleFieldRelativeCommand);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigationButton(), DriverButtonCommands.toggleAutonavigationCommand);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToAmpButton(), DriverButtonCommands.autonavigateToAmp);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToSourceButton(), DriverButtonCommands.autonavigateToSource);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToSpeakerButton(), DriverButtonCommands.autonavigateToSpeaker);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToTopStageButton(), DriverButtonCommands.autonavigateToTopStage);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToMidStageButton(), DriverButtonCommands.autonavigateToMidStage);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToBottomStageButton(), DriverButtonCommands.autonavigateToBottomStage);
        makeTriggerForButton(hid, controller.getPort(), controller.getAutonavigateToVisionTarget(), DriverButtonCommands.autonavigateToVisionTarget);
        makeTriggerForButton(hid, controller.getPort(), controller.getOdometryVisionResetButton(), DriverButtonCommands.odometryVisionReset);
        makeTriggerForButton(hid, controller.getPort(), controller.getNextPipelineButton(), DriverButtonCommands.nextPipeline);

        Logger.recordOutput("controlmanager.controller." + controller.getPort() + ".name", controller.getName());
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
     * <p>
     * This method will be non-fatally unsuccessful,
     * if no controller is registered with the new port,
     * or if the controller is reserved.
     *
     * @return - boolean for whether change was successful.
     */
    public static boolean setDriverPort(int port) {
        if (ControlManager.registry.get(port) == null || port == ControlManager.reservedPort) {
            return false;
        }

        ControlManager.driverPort = port;

        return true;
    }

    /**
     * Get list of available controllers.
     *
     * @return - list of integer IDs for controller ports.
     */
    public static ArrayList<Integer> getControllers() {
        ArrayList<Integer> controllers = new ArrayList<>();

        for (int port : ControlManager.registry.keySet()) {
            controllers.add(port);
        }

        return controllers;
    }

    /**
     * Get available controller with the lowest integer ID for port.
     * <p>
     * If no controllers are available, -1 is returned.
     *
     * @return - integer ID for controller port.
     */
    public static int getControllerLowest() {
        ArrayList<Integer> controllers = getControllers();

        if (controllers.size() <= 0) {
            return -1;
        }

        controllers.sort(Comparator.naturalOrder());
        return controllers.get(0);
    }

    /**
     * Get label for a controller.
     * <p>
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

        return port + " - " + controller.getName();
    }

    /**
     * Get list of labels of available controllers.
     *
     * @return - list of labels for controllers.
     */
    public static ArrayList<String> getControllerLabels() {
        return (ArrayList<String>) ControlManager.getControllers()
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
        return Integer.parseInt(label.split(" - ")[0]);
    }

    /**
     * Process inputs for the driver controls.
     * <p>
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

            System.out.println("Achtung! Failed to get driver controller! Check SmartDashboard.");
            return;
        }

        Logger.recordOutput("controlmanager.outputs.xspeed", ControlManager.Outputs.xSpeed);
        Logger.recordOutput("controlmanager.outputs.yspeed", ControlManager.Outputs.ySpeed);
        Logger.recordOutput("controlmanager.outputs.rotatingspeed", ControlManager.Outputs.rotatingSpeed);


        if (driverController.visionTargetAlignButtonIsPressed()) {
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;

            ControlManager.Outputs.rotatingSpeed = LimelightImplementation.limelight_aim_proportional();
            ControlManager.Outputs.xSpeed = LimelightImplementation.limelight_range_proportional();

        } else if (!driverController.isBoosterPressed()) {
            fieldRelative = true;
            double val = driverController.getSpeedCoefficient();
            ControlManager.Outputs.xSpeed = OIConstants.xLimiter.calculate(driverController.getX() * val * OIConstants.kTeleDriveMaxSpeedMetersPerSecond);
            ControlManager.Outputs.ySpeed = OIConstants.yLimiter.calculate(driverController.getY() * val * OIConstants.kTeleDriveMaxSpeedMetersPerSecond);
            ControlManager.Outputs.rotatingSpeed = OIConstants.zLimiter.calculate(driverController.getZ() * val * OIConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);
        } else {
            fieldRelative = true;
            ControlManager.Outputs.xSpeed = driverController.getX() * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            ControlManager.Outputs.ySpeed = driverController.getY() * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            ControlManager.Outputs.rotatingSpeed = driverController.getZ() * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        }
    }
}