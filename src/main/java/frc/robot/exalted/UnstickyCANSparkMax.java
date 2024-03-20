package frc.robot.exalted;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

/**
 * Like CANSparkMax, but automatically clears sticky faults.
 */
public class UnstickyCANSparkMax extends CANSparkMax {
    private REVFaultTolerances tolerances = new REVFaultTolerances();

    public UnstickyCANSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
        
        clearToleratedStickyFaults();
    }

    public UnstickyCANSparkMax(int deviceId, MotorType type, REVFaultTolerances tolerances) {
        super(deviceId, type);

        this.tolerances = tolerances;
        clearToleratedStickyFaults();
    }

    /**
     * Get sticky faults as a list of FaultIDs, instead of a string of bits represented with a short.
     */
    public List<FaultID> getStickyFaultsAsList() {
        List<FaultID> faults = new ArrayList<>();
        
        for (FaultID fault : FaultID.values()) {
            if (getStickyFault(fault)) {
                faults.add(fault);
            }
        }

        return faults;
    }

    /**
     * Check whether all sticky faults are tolerated, and acceptable to clear.
     */
    public boolean areStickyFaultsTolerated() {
        List<FaultID> faults = getStickyFaultsAsList();

        for (FaultID fault : faults) {
            if (!tolerances.isTolerated(fault)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Clear sticky faults if they are all tolerated.
     */
    public void clearToleratedStickyFaults() {
        if (!areStickyFaultsTolerated()) {
            return;
        }

        List<FaultID> cleared = getStickyFaultsAsList();
        REVLibError result = this.clearFaults();

        String clearedHumanReadable = String.join(", ", cleared.stream().map((fault) -> fault.name()).collect(Collectors.toList()));
        clearedHumanReadable = clearedHumanReadable.substring(0, clearedHumanReadable.length() - 2);

        System.out.println("Achtung! For SPARK MAX " + this.getDeviceId() + ", cleared faults: " + clearedHumanReadable + " ...with result: " + result.toString());
    }
}
