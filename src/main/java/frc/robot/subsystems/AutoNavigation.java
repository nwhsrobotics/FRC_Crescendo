package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Helper class for autonavigation.
 * <p>
 * Works in conjunction with the swerve subsystem.
 */
public class AutoNavigation {
    private Command navigationCommand;
    private boolean enabled = false;
    private final SwerveSubsystem swerve;

    public AutoNavigation(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    /**
     * Enable autonavigation.
     * <p>
     * If the robot is not operating under tele-op, calls to this method will be ignored.
     */
    public void enable() {
        if (!RobotState.isTeleop()) {
            return;
        }

        enabled = true;

        Logger.recordOutput("autonavigator.enabled", enabled);
    }

    /**
     * Disable autonavigation.
     * <p>
     * The current autonavigation destination will be cancelled.
     */
    public void disable() {
        pauseNavigation();
        navigationCommand = null;
        enabled = false;

        Logger.recordOutput("autonavigator.enabled", enabled);
    }

    /**
     * Returns whether autonavigation is enabled.
     *
     * @return - boolean representing whether autonavigation is enabled.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Toggle autonavigation.
     * <p>
     * Calls ".enable()" if currently disabled, and calls ".disable()" if currently enabled.
     */
    public void toggle() {
        if (enabled) {
            disable();
        } else {
            enable();
        }
    }

    /**
     * Stop current autonavigation, but do not clear current navigation command.
     * <p>
     * This is useful for accomodating manual override and et cetera.
     * Run ".resumeNavigation()" to continue autonavigation.
     * <p>
     * It is safe to execute this command if no autonavigation commmand is currently scheduled.
     */
    public void pauseNavigation() {
        if (navigationCommand == null) {
            return;
        }

        navigationCommand.cancel();
    }

    /**
     * Start current autonavigation, but do not clear current navigation command.
     * <p>
     * This is useful for accomodating manual override and et cetera.
     * Run ".pauseNavigation()" to stop autonavigation.
     * <p>
     * It is safe to execute this command if no autonavigation commmand is currently scheduled,
     * or if the autonavigation command is already scheduled.
     */
    public void resumeNavigation() {
        if (navigationCommand == null || navigationCommand.isScheduled()) {
            return;
        }

        navigationCommand.schedule();
    }

    /**
     * Navigate to position.
     * <p>
     * This will initialize a new navigation command.
     * If an existing navigation command is scheduled, that command will be cancelled,
     * before being overwritten.
     * <p>
     * If autonavigation is disabled, calls to this method will be ignored.
     *
     * @param destination - position to navigate to.
     */
    public void navigateTo(Pose2d destination) {
        if (!enabled || !RobotState.isTeleop()) {
            return;
        }

        if (navigationCommand != null && navigationCommand.isScheduled()) {
            navigationCommand.cancel();
        }

        navigationCommand = swerve.pathfindToPosition(destination);
        Logger.recordOutput("autonavigator.destination", destination);
    }
}
