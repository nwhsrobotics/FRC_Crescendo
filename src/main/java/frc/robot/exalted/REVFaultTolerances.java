package frc.robot.exalted;

import com.revrobotics.CANSparkBase.FaultID;

import java.util.Arrays;
import java.util.List;

public class REVFaultTolerances {
    private final List<FaultID> toleratedFaults;

    /**
     * Create default fault tolerances.
     */
    public REVFaultTolerances() {
        toleratedFaults = Arrays.asList(FaultID.kBrownout,
                FaultID.kCANRX,
                FaultID.kCANTX,
                FaultID.kHasReset);
    }

    /**
     * Create custom fault tolerances.
     */
    public REVFaultTolerances(FaultID[] tolerances) {
        toleratedFaults = Arrays.asList(tolerances);
    }

    /**
     * Get fault tolerances.
     */
    public List<FaultID> getTolerances() {
        return toleratedFaults;
    }

    /**
     * Check if fault is tolerated.
     */
    public boolean isTolerated(FaultID fault) {
        return toleratedFaults.contains(fault);
    }
}
