package frc.robot.util;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.HashMap;
import java.util.Map;

/**
 * Manages the PDP/PDH of the robot, along with providing logging and retrieval methods for distribution data outputs.
 */
public class ImprovedPowerDistribution extends PowerDistribution {
    private final Map<Integer, Boolean> channelTracker = new HashMap<>();
    private final InstantCommand watchdogCommand = new InstantCommand(() -> watchdog());

    private void watchdog() {
        PowerDistributionFaults activeFaults = getFaults();

        for (int i = 0; i < getNumChannels(); i++) {
            boolean before = channelTracker.get(i);
            boolean now = activeFaults.getBreakerFault(i);
            if (before == now) {
                continue;
            }

            System.out.println("Achtung! Detected new change in PDP/PDH breaker fault for channel " + i + ", whose state went from " + before + " to " + now + ".");
            channelTracker.put(i, now);
        }

        CommandScheduler.getInstance().schedule(watchdogCommand);
    }

    public ImprovedPowerDistribution(int id, ModuleType type) {
        super(id, type);
        clearStickyFaults();

        PowerDistributionFaults activeFaults = getFaults();
        for (int i = 0; i < getNumChannels(); i++) {
            channelTracker.put(i, activeFaults.getBreakerFault(i));
        }

        watchdog();
    }
}
