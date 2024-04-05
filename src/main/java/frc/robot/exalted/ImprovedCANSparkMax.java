package frc.robot.exalted;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Like CANSparkMax, but:
 * <p>
 * * Automatically clears sticky faults.
 * * Initially resets settings to default.
 * * Saves new settings to SPARK MAX (call .finalizeSettings() when ready).
 * * Sets current limit depending on whether a NEO or NEO 550 is attached.
 * <p>
 * It assumes that the connected motor is brushless.
 */
public class ImprovedCANSparkMax extends UnstickyCANSparkMax {
    private REVBurnFlashCondition flashCondition = REVBurnFlashCondition.OnlyInTestMode;
    private REVNeoType neoType = REVNeoType.NEO550;  // a neo can be ran as a neo 550, but not the other way around.

    public ImprovedCANSparkMax(int deviceId) {
        super(deviceId, MotorType.kBrushless);
        restoreFactoryDefaults();
    }

    public ImprovedCANSparkMax(int deviceId, REVFaultTolerances tolerances) {
        super(deviceId, MotorType.kBrushless, tolerances);
        restoreFactoryDefaults();
    }

    public ImprovedCANSparkMax(int deviceId, REVFaultTolerances tolerances, REVBurnFlashCondition flashCondition) {
        super(deviceId, MotorType.kBrushless, tolerances);
        this.flashCondition = flashCondition;
        restoreFactoryDefaults();
    }

    public ImprovedCANSparkMax(int deviceId, REVBurnFlashCondition flashCondition) {
        super(deviceId, MotorType.kBrushless);
        this.flashCondition = flashCondition;
        restoreFactoryDefaults();
    }

    public ImprovedCANSparkMax(int deviceId, REVFaultTolerances tolerances, REVBurnFlashCondition flashCondition, REVNeoType neoType) {
        super(deviceId, MotorType.kBrushless, tolerances);
        this.flashCondition = flashCondition;
        this.neoType = neoType;
        restoreFactoryDefaults();
    }

    public ImprovedCANSparkMax(int deviceId, REVFaultTolerances tolerances, REVNeoType neoType) {
        super(deviceId, MotorType.kBrushless, tolerances);
        this.neoType = neoType;
        restoreFactoryDefaults();
    }

    public ImprovedCANSparkMax(int deviceId, REVNeoType neoType) {
        super(deviceId, MotorType.kBrushless);
        this.neoType = neoType;
        restoreFactoryDefaults();
    }

    /**
     * Automatically set current limits based on type of NEO connected.
     */
    public void autoSetSmartCurrentLimits() {
        switch (neoType) {
            case NEO:
                setSmartCurrentLimit(80);
                break;
            case NEO550:
                setSmartCurrentLimit(20);
                break;
            default:
                break;
        }
    }

    /**
     * Burns flash according to the condition set for this SPARK MAX.
     *
     * <p>
     * <p>
     * Call after the SPARK MAX has been configured (with PID and other parameters).
     */
    public void finalizeSettings() {
        switch (flashCondition) {
            case Always:
                burnFlash();
                break;
            case Never:
                break;
            case OnlyInTestMode:
                if (DriverStation.isTest()) {
                    burnFlash();
                }

                break;
            case OnlyWithFMS:
                if (DriverStation.isFMSAttached()) {
                    burnFlash();
                }

                break;
            default:
                break;
        }
    }
}
