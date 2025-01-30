package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.ElasticNotification;
import frc.robot.util.Elastic.ElasticNotification.NotificationLevel;
import frc.robot.util.ImprovedCanSpark;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
     * should the game piece get jammed within the shooter assembly.
     *
     * For example, you may "FIRE" after hitting "UNLOADING."
     */
    public enum ScoringState {
        /**
         * Intake a game piece into the shooter assembly.
         * but not push it past the flywheel and out of the shooter assembly.
         */
        LOADING,
        /**
         * Eject the game piece currently inside of the robot,
         * through the intake assembly.
         */
        UNLOADING,
        /**
         * Fire!
         */
        FIRE,
        /**
         * Used during autonomous, turns on flywheel and indexer together to optimize time
         */
        FASTFIRE,
        /*
         * Do nothing.
         */
        IDLE,
    }

    /*
     * State of the subsystem.
     */
    public ScoringState state = ScoringState.IDLE;

    private final SparkMax flywheelMotor;
    private final RelativeEncoder flywheelEncoder;
    private final SparkClosedLoopController flywheelPIDController;
    private final SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    private final SparkMax secondaryFlywheelMotor;
    private final RelativeEncoder secondaryFlywheelEncoder;
    private final SparkClosedLoopController secondaryFlywheelPIDController;
    private final SparkMaxConfig secondaryFlywheelConfig = new SparkMaxConfig();
    private final SparkMax indexMotor;
    private final RelativeEncoder indexEncoder;
    private final SparkClosedLoopController indexPIDController;
    private final SparkMaxConfig indexConfig = new SparkMaxConfig();
    private final SparkMax secondaryIndexMotor;
    private final RelativeEncoder secondaryIndexEncoder;
    private final SparkClosedLoopController secondaryIndexPIDController;
    private final SparkMaxConfig secondaryIndexConfig = new SparkMaxConfig();
    private final SparkMax intakeMotor;
    private final SparkClosedLoopController intakePIDController;
    private final RelativeEncoder intakeEncoder;
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    private double flywheelRPM = Constants.ScoringConstants.FLYWHEEL_SPEAKER_RPM;
    private boolean noteInside;

    public ScoringSubsystem() {
        flywheelConfig.closedLoop.pidf(Constants.ScoringConstants.FLYWHEEL_PID_P, 0, 0, Constants.ScoringConstants.FLYWHEEL_PID_FF);
        flywheelMotor = new ImprovedCanSpark(Constants.CANAssignments.FLYWHEEL_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, flywheelConfig, IdleMode.kCoast, 12.25);
        flywheelMotor.setInverted(true);
        flywheelEncoder = flywheelMotor.getEncoder();
        flywheelPIDController = flywheelMotor.getClosedLoopController();

        secondaryFlywheelConfig.closedLoop.pidf(Constants.ScoringConstants.FLYWHEEL_PID_P, 0, 0, Constants.ScoringConstants.FLYWHEEL_PID_FF);
        secondaryFlywheelConfig.follow(flywheelMotor, true);
        secondaryFlywheelMotor = new ImprovedCanSpark(Constants.CANAssignments.SECONDARY_FLYWHEEL_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, secondaryFlywheelConfig, IdleMode.kCoast, 12.25);
        secondaryFlywheelEncoder = secondaryFlywheelMotor.getEncoder();
        secondaryFlywheelPIDController = secondaryFlywheelMotor.getClosedLoopController();

        indexConfig.closedLoop.pidf(0,0, 0, Constants.ScoringConstants.INDEX_PID_FF);
        indexMotor = new ImprovedCanSpark(Constants.CANAssignments.INDEX_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, indexConfig, IdleMode.kCoast, 12.25);
        indexMotor.setInverted(true);
        indexEncoder = indexMotor.getEncoder();
        indexPIDController = indexMotor.getClosedLoopController();

        secondaryIndexConfig.closedLoop.pidf(0, 0, 0, Constants.ScoringConstants.INDEX_PID_FF);
        secondaryFlywheelConfig.follow(indexMotor, true);
        secondaryIndexMotor = new ImprovedCanSpark(Constants.CANAssignments.SECONDARY_INDEX_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, secondaryIndexConfig, IdleMode.kCoast, 12.25);
        secondaryIndexEncoder = secondaryIndexMotor.getEncoder();
        secondaryIndexPIDController = secondaryIndexMotor.getClosedLoopController(); 
        //TODO: set the same P value like the flywheel? Thats why it was losing traction because more load = less speed

        intakeConfig.closedLoop.pidf(0, 0, 0, Constants.ScoringConstants.INTAKE_PID_FF);
        intakeMotor = new ImprovedCanSpark(Constants.CANAssignments.INTAKE_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, intakeConfig, IdleMode.kCoast, 12.25);
        intakeMotor.setInverted(true);
        intakeEncoder = intakeMotor.getEncoder();
        intakePIDController = intakeMotor.getClosedLoopController();
    }

    /**
     * If the target flywheel RPM is within tolerances, the flywheel is considered ready.
     *
     * @return - boolean representing whether the flywheel is ready.
     */
    public boolean isFlywheelReady() {
        return flywheelEncoder.getVelocity() >= flywheelRPM - Constants.ScoringConstants.FLYWHEEL_TARGET_RPM_TOLERANCE &&
                flywheelEncoder.getVelocity() <= flywheelRPM + Constants.ScoringConstants.FLYWHEEL_TARGET_RPM_TOLERANCE;
    }

    /**
     * Set flywheel to a target RPM.
     *
     * <p>
     * <p>
     * When the subsystem is told to fire,
     * this is the target RPM the flywheel will spin up to.
     *
     * @param rpm - speed in rotations per minute.
     */
    public void setFlywheel(double rpm) {
        flywheelRPM = rpm;
    }

    public void increaseRPM() {
        if (flywheelRPM < 5500) {
            flywheelRPM += 100;
        }
    }

    public void decreaseRPM() {
        if (flywheelRPM > 100) {
            flywheelRPM -= 100;
        }
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
                Elastic.sendAlert(new ElasticNotification()
                    .withLevel(NotificationLevel.INFO)
                    .withTitle("Scoring")
                    .withDescription("Your robot is currently loading!")
                    .withDisplaySeconds(3.0)
                );
                /*if (didCurrentSpikeIntake(intakeMotor)) {
                    noteInside = true;
                }*/

                Logger.recordOutput("scoring.state", "LOADING");
                break;
            case FIRE:
                intakePIDController.setReference(Constants.ScoringConstants.INTAKE_RPM, ControlType.kVelocity);
                flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);
                if (isFlywheelReady() && flywheelEncoder.getVelocity() != Constants.ScoringConstants.FLYWHEEL_IDLE_RPM) {
                    indexPIDController.setReference(Constants.ScoringConstants.INDEX_FLYWHEEL_COOP_RPM, ControlType.kVelocity);
                    ElasticNotification notification = new ElasticNotification(NotificationLevel.INFO, "Shot",
                     "Your robot is currently shooting a game piece. Another type of notification");
                    Elastic.sendAlert(notification);
                    /*if (didCurrentSpike(flywheelMotor)) {
                        noteInside = false;
                    }*/
                }
                Logger.recordOutput("scoring.state", "FIRE");
                break;
            case FASTFIRE:
                intakePIDController.setReference(Constants.ScoringConstants.INTAKE_RPM, ControlType.kVelocity);
                flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);
                indexPIDController.setReference(Constants.ScoringConstants.INDEX_FLYWHEEL_COOP_RPM, ControlType.kVelocity);
                /*if (didCurrentSpike(flywheelMotor)) {
                    noteInside = false;
                }*/
                Logger.recordOutput("scoring.state", "FASTFIRE");
                break;
            case UNLOADING:
                flywheelPIDController.setReference(Constants.ScoringConstants.FLYWHEEL_IDLE_RPM, ControlType.kVelocity);
                indexPIDController.setReference(-Constants.ScoringConstants.INDEX_INTAKE_UNLOAD_RPM, ControlType.kVelocity);
                intakePIDController.setReference(-Constants.ScoringConstants.INTAKE_RPM, ControlType.kVelocity);
                /*if (didCurrentSpikeIntake(intakeMotor)) {
                    noteInside = false;
                }*/
                Logger.recordOutput("scoring.state", "UNLOADING");
                break;
            default:
                break;
        }

        Logger.recordOutput("scoring.flywheel.isready", isFlywheelReady());
        Logger.recordOutput("scoring.flywheel.rpm", flywheelEncoder.getVelocity());
        Logger.recordOutput("scoring.flywheel.rpmtarget", flywheelRPM);
        Logger.recordOutput("scoring.index.rpm", indexEncoder.getVelocity());
        Logger.recordOutput("scoring.intake.rpm", intakeEncoder.getVelocity());
    }

    /*
     * TODO: This doesn't work right now
     */
    public boolean didCurrentSpike(SparkMax motor) {
        double current = motor.getOutputCurrent();
        double threshold = 40.0;
        return current > threshold;
    }

    /*
     * TODO: This doesn't work right now
     */
    public boolean didCurrentSpikeIntake(SparkMax motor) {
        double current = motor.getOutputCurrent();
        double threshold = 20.0;
        return current > threshold;
    }

    public boolean isNoteInside() {
        return noteInside;
    }
}
