package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Controller usable as an operator interface,
 * that can be registered with the "ControlManager."
 */
public interface Controller {
    /**
     * Calculate speed with deadband.
     *
     * @param value    - value scaled from -1 to 1.
     * @param deadband - deadband tolerance.
     * @return - new value with deadband applied.
     */
    static double calculateSpeedWithDeadband(double value, double deadband) {
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
     * <p>
     * This determines the label that will appear, in the selectors on the driver station,
     * for which port is actively being used.
     *
     * @return - name of controller.
     */
    default String getName() {
        return "Unnamed Controller";
    }

    /**
     * Get what port the controller should be plugged into.
     *
     * @return - port number.
     */
    int getPort();

    /**
     * Get underlying "GenericHID" object.
     *
     * @return - "GenericHID" object.
     */
    GenericHID getGenericHID();

    /**
     * Get button binding for resetting the NavX gyroscope.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getNavXResetButton() {
        return -1;
    }

    /**
     * Get button binding for toggling field relative driving.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getFieldRelativeButton() {
        return -1;
    }

    /**
     * Get button binding for toggling autonavigation.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigationButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to amp.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigateToAmpButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to source.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigateToSourceButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to speaker.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigateToSpeakerButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to midstage.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigateToMidStageButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to bottomstage.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigateToBottomStageButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to topstage.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigateToTopStageButton() {
        return -1;
    }

    /**
     * Get button binding for navigating to object detected by limelight.
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default int getAutonavigateToObject() {
        return -1;
    }

    /**
     * Get button binding for auto alligning to april tag
     * <p>
     * This is a driver-side input.
     *
     * @return - button number.
     */
    default boolean aprilTagAllignButtonIsPressed() {
        return false;
    }

    /**
     * Get X-axis movement.
     * <p>
     * This is a driver-side input.
     *
     * @return - input scaled from -1 to 1.
     */
    default double getX() {
        return 0;
    }

    /**
     * Get Y-axis movement.
     * <p>
     * This is a driver-site input.
     *
     * @return - input scaled from -1 to 1.
     */
    default double getY() {
        return 0;
    }

    /**
     * Get Z-axis movement.
     * <p>
     * This is a driver-site input.
     *
     * @return - input scaled from -1 to 1.
     */
    default double getZ() {
        return 0;
    }

    /**
     * Get throttle.
     * <p>
     * The output is used as a speed coefficient;
     * it is directly multiplied with the power output of the drive motors (it will be limited by max telep speeds).
     * Note: Coefficient will not work with booster as of current implementation
     * <p>
     * This is a driver-side input.
     *
     * @return - input scaled from 0 to 1. Ideally should never return less than 0.1
     */
    default double getSpeedCoefficient() {
        return 1.0;
    }

    /**
     * Get whether the operator wishes to boost.
     * <p>
     * Boosting can raise the power output of the driver motors to their maximum bypassing teleop speed to the physicalMaximum.
     * <p>
     * This is a driver-side input.
     *
     * @return - boolean for whether the booster is pressed for the robot to dash.
     */
    default boolean isBoosterPressed() {
        return false;
    }
}
