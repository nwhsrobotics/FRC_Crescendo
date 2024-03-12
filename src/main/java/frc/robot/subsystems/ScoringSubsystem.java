package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Combines the shooter and intake into a singular cooperative subsystem.
 */
public class ScoringSubsystem extends SubsystemBase {
    /*
     * What state the subsystem is in.
     * 
     * <p>
     * 
     * Note that there are no restrictions for transitioning between states.
     * This is to prevent soft-locking of the subsystem,
     * should the game piece get jammed within the assembly.
     * 
     * For example, you may "FIRE" after hitting "UNLOADING." 
     */
    public enum ScoringState {
        /**
         * Drive the intake forwards, and slowly drive the indexer forwards.
         * 
         * <p>
         * 
         * This should press the ring against the flywheel,
         * but not push it past the flywheel and out of the shooter assembly.
         */
        LOADING,
        /**
         * Drive the intake and indexer backwards.
         * 
         * <p>
         * 
         * This should completely eject the ring out of the robot,
         * through the intake assembly.
         */
        UNLOADING,
        /**
         * Drive flywheel forwards, wait for it to reach the RPM target,
         * and then drive the indexer forwards.
         * 
         * <p>
         * 
         * Fire!
         */
        FIRE,
        /*
         * Do nothing.
         */
        IDLE,
    }

    /*
     * State of the subsystem.
     */
    public ScoringState state = ScoringState.IDLE;

    private final CANSparkMax flywheelMotor;
    private final RelativeEncoder flywheelEncoder;
    private final SparkPIDController flywheelPIDController;
    private final CANSparkMax indexMotor;
    private final RelativeEncoder indexEncoder;
    private final SparkPIDController indexPIDController;
    private final CANSparkMax intakeMotor;
    private final SparkPIDController intakePIDController;
    private final RelativeEncoder intakeEncoder;

    private double flywheelRPM = 0;

    public ScoringSubsystem() {
        flywheelMotor = new CANSparkMax(Constants.CANAssignments.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotor.setIdleMode(IdleMode.kCoast);
        flywheelEncoder = flywheelMotor.getEncoder();
        flywheelPIDController = flywheelMotor.getPIDController();
        flywheelPIDController.setP(Constants.ScoringConstants.FLYWHEEL_PID_P);

        indexMotor = new CANSparkMax(Constants.CANAssignments.INDEX_MOTOR_ID, MotorType.kBrushless);
        indexMotor.setIdleMode(IdleMode.kCoast);
        indexEncoder = indexMotor.getEncoder();
        indexPIDController = indexMotor.getPIDController();
        indexPIDController.setP(Constants.ScoringConstants.INDEX_PID_P);

        intakeMotor = new CANSparkMax(Constants.CANAssignments.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeEncoder = intakeMotor.getEncoder();
        intakePIDController = intakeMotor.getPIDController();
        intakePIDController.setP(Constants.ScoringConstants.INTAKE_PID_P);
    }

    /**
     * If the target flywheel RPM is within tolerances, the flywheel is considered ready.
     *
     * @return - boolean representing whether the flywheel is ready.
     */
    public boolean isFlywheelReady() {
        return flywheelEncoder.getVelocity() >= flywheelRPM - Constants.ScoringConstants.FLYWHEEL_TARGET_RPM_TOLERANCE && flywheelEncoder.getVelocity() <= flywheelRPM + Constants.ScoringConstants.FLYWHEEL_TARGET_RPM_TOLERANCE;
    }

    /**
     * Set flywheel to a target RPM.
     * 
     * <p>
     * 
     * When the subsystem is told to fire,
     * this is the target RPM the flywheel will spin up to.
     * 
     * @param rpm - speed in rotations per minute.
     */
    public void setFlywheel(double rpm) {
        flywheelRPM = rpm;
    }

    /**
     * Set flywheel to a target RPM, and set RPM to idle speed if RPM is already at target.
     * 
     * @param rpm - speed in rotations per minute.
     */
    public void toggleFlywheel(double rpm) {
        if (flywheelRPM == rpm) {
            flywheelRPM = Constants.ScoringConstants.FLYWHEEL_IDLE_RPM;
            return;
        }

        setFlywheel(rpm);
    }

    public void toggleAmp() {
        toggleFlywheel(Constants.ScoringConstants.FLYWHEEL_AMP_RPM);
    }

    public void toggleSpeaker() {
        toggleFlywheel(Constants.ScoringConstants.FLYWHEEL_SPEAKER_RPM);
    }

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                flywheelPIDController.setReference(Constants.ScoringConstants.FLYWHEEL_IDLE_RPM, ControlType.kVelocity);
                indexPIDController.setReference(0, ControlType.kDutyCycle);  // don't even bother with velocity control, just turn them off.
                intakePIDController.setReference(0, ControlType.kDutyCycle);
                Logger.recordOutput("scoring.state", "IDLE");
                break;
            case LOADING:
                flywheelPIDController.setReference(Constants.ScoringConstants.FLYWHEEL_IDLE_RPM, ControlType.kVelocity);
                indexPIDController.setReference(Constants.ScoringConstants.INDEX_INTAKE_COOP_RPM, ControlType.kVelocity);
                intakePIDController.setReference(Constants.ScoringConstants.INTAKE_RPM, ControlType.kVelocity);
                Logger.recordOutput("scoring.state", "LOADING");
                break;
            case FIRE:
                flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);

                if (isFlywheelReady() && flywheelEncoder.getVelocity() != Constants.ScoringConstants.FLYWHEEL_IDLE_RPM) {
                    indexPIDController.setReference(Constants.ScoringConstants.INDEX_FLYWHEEL_COOP_RPM, ControlType.kVelocity);
                }

                intakePIDController.setReference(0, ControlType.kDutyCycle);
                Logger.recordOutput("scoring.state", "FIRE");
                break;
            case UNLOADING:
                flywheelPIDController.setReference(Constants.ScoringConstants.FLYWHEEL_IDLE_RPM, ControlType.kVelocity);
                indexPIDController.setReference(-Constants.ScoringConstants.INDEX_INTAKE_UNLOAD_RPM, ControlType.kVelocity);
                intakePIDController.setReference(-Constants.ScoringConstants.INTAKE_RPM, ControlType.kVelocity);
                Logger.recordOutput("scoring.state", "UNLOADING");
                break;     
            default:
                break;
        }

        Logger.recordOutput("scoring.flywheel.isready", isFlywheelReady());
        Logger.recordOutput("scoring.flywheel.rpm", flywheelRPM);
        Logger.recordOutput("scoring.index.rpm", indexEncoder.getVelocity());
        Logger.recordOutput("scoring.intake.rpm", intakeEncoder.getVelocity());
    }
}
