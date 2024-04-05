package frc.robot.exalted;

/**
 * Conditions to enable burning settings to flash.
 */
public enum REVBurnFlashCondition {
    /**
     * Settings will always be burned to the device's flash memory.
     *
     * <p>
     * <p>
     * Note that this will rapidly degrade the EEPROM of the device,
     * if the robot is frequently turned on and off,
     * as the EEPROM chip itself is only rated for ~10K cycles.
     */
    Always,
    /**
     * Settings will only be burned to the device's flash memory
     * if the driver station is attached to a Field Management System,
     * such as during a competition event.
     *
     * <p>
     * <p>
     * This limits how frequently the EEPROM of the device is reprogrammed.
     */
    OnlyWithFMS,
    /**
     * Settings will only be burned to the device's flash memory,
     * if the robot is enabled under test mode.
     *
     * <p>
     * <p>
     * This limits how frequently the EEPROM of the device is reprogrammed.
     */
    OnlyInTestMode,
    /**
     * Settings will never be burned to the device's flash memory.
     */
    Never
}
