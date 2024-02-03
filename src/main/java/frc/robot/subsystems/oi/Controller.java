package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Controller usable as an operator interface,
 * that can be registered with the "ControlManager."
 * 
 * See "getIntendedUser()" for guidance on specializing in either driver or gunner bindings.
 */
public interface Controller {
    /**
     * Calculate speed with deadband.
     * 
     * @param value - value scaled from -1 to 1.
     * @param deadband - deadband tolerance.
     * @return - new value with deadband applied.
     */
    public static double calculateSpeedWithDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            // If the value is within the deadband, consider it as zero.
            return 0;
        } else if (value > 0) {
            // Scale positive values to eliminate deadband.
            return (value - deadband) / (1 - deadband);
        } else {
            // Scale negative values to eliminate deadband.
            return (value + deadband) / (1 - deadband);
        }
    }

    /**
     * Get name of controller.
     * 
     * This determines the label that will appear, in the selectors on the driver station,
     * for which port is actively being used for the driver or gunner inputs.
     * 
     * @return - name of controller.
     */
    public default String getName() {
        return "Unnamed Controller";
    }

    /**
     * Get what bindings this controller has,
     * determining which user it is intended for.
     * 
     * The selectors on the driver station will hide controllers that lack appropriate bindings,
     * for driver and gunner inputs respectively.
     * 
     * If a controller is selected to be actively used for the driver's input,
     * methods for gunner-side inputs will be ignored.
     * Vice versa if the controller is selected to be actively used for the gunner's input.
     * 
     * If you are implementing a controller that only has either driver or gunner inputs,
     * then leave extraneous methods to use default implementations.
     * Mandatory methods do not have default implementations.
     * 
     * @return - integer; negative indicates driver, positive indicates gunner, zero indicates both.
     */
    public default int getIntendedUser() {
        return 0;
    }

    /**
     * Get what port the controller should be plugged into.
     * 
     * @return - port number.
     */
    public int getPort();

    /**
     * Get underlying "GenericHID" object.
     * 
     * @return - "GenericHID" object.
     */
    public GenericHID getGenericHID();

    /**
     * Get button binding for resetting the NavX gyroscope. 
     *
     * This is a driver-side input.
     * 
     * @return - button number.
     */
    public default int getNavXResetButton() {
        return -1;
    }

    /**
     * Get button binding for toggling field relative driving. 
     *
     * This is a driver-side input.
     * 
     * @return - button number.
     */
    public default int getFieldRelativeButton() {
        return -1;
    }

    /**
     * Get button binding for toggling autonavigation.
     * 
     * This is a driver-side input.
     * 
     * @return - button number.
     */
    public default int getAutonavigationButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to amp.
     * 
     * This is a driver-side input.
     * 
     * @return - button number.
     */
    public default int getAutonavigateToAmpButton() {
        return -1;
    }  

    /**
     * Get button binding for navigating to source.
     * 
     * This is a driver-side input.
     * 
     * @return - button number.
     */
    public default int getAutonavigateToSourceButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to stage.
     * 
     * This is a driver-side input.
     * 
     * @return - button number.
     */
    public default int getAutonavigateToStageButton() {
        return -1;   
    }

    /**
     * Get X-axis movement.
     * 
     * This is a driver-side input.
     * 
     * @return - input scaled from -1 to 1.
     */
    public default double getX() {
        return 0;
    }

    /**
     * Get Y-axis movement.
     * 
     * This is a driver-site input.
     * 
     * @return - input scaled from -1 to 1.
     */
    public default double getY() {
        return 0;
    }

    /**
     * Get Z-axis movement.
     * 
     * This is a driver-site input.
     * 
     * @return - input scaled from -1 to 1.
     */
    public default double getZ() {
        return 0;
    }

    /**
     * Get throttle.
     * 
     * The output is used as a speed coefficient;
     * it is directly multiplied with the power output of the drive motors.
     * 
     * This is a driver-side input.
     * 
     * @return - input scaled from 0 to 1.
     */
    public default double getThrottle() {
        return 0;
    }

    /**
     * Get whether the operator wishes to dash.
     * 
     * Dashing will raise the power output of the driver motors to their maximum.
     * 
     * This is a driver-side input.
     * 
     * @return - boolean for whether the robot should dash.
     */
    public default boolean isDashing() {
        return false;
    }
}
