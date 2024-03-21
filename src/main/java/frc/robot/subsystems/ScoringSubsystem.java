package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

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
    private final CANSparkMax secondaryFlywheelMotor;
    private final RelativeEncoder secondaryFlywheelEncoder;
    private final SparkPIDController secondaryFlywheelPIDController;
    private final CANSparkMax indexMotor;
    private final RelativeEncoder indexEncoder;
    private final SparkPIDController indexPIDController;
    private final CANSparkMax secondaryIndexMotor;
    private final RelativeEncoder secondaryIndexEncoder;
    private final SparkPIDController secondaryIndexPIDController;
    private final CANSparkMax intakeMotor;
    private final SparkPIDController intakePIDController;
    private final RelativeEncoder intakeEncoder;

    private double flywheelRPM = Constants.ScoringConstants.FLYWHEEL_SPEAKER_RPM; //by default shooter is speaker, to avoid toggling

    public ScoringSubsystem() {
        flywheelMotor = new CANSparkMax(Constants.CANAssignments.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotor.restoreFactoryDefaults();
        flywheelMotor.clearFaults();
        flywheelMotor.setIdleMode(IdleMode.kCoast);
        flywheelMotor.setInverted(true);
        flywheelEncoder = flywheelMotor.getEncoder();
        flywheelPIDController = flywheelMotor.getPIDController();
        flywheelPIDController.setP(0);
        flywheelPIDController.setFF(Constants.ScoringConstants.FLYWHEEL_PID_FF);
        flywheelMotor.setSmartCurrentLimit(80); //big neo is good
        // flywheelMotor.setClosedLoopRampRate(0.6); // it takes 0.6second to reach max speed
        flywheelMotor.enableVoltageCompensation(12.0); 
        //flywheelMotor.burnFlash();

        secondaryFlywheelMotor = new CANSparkMax(Constants.CANAssignments.SECONDARY_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        secondaryFlywheelMotor.restoreFactoryDefaults();
        secondaryFlywheelMotor.clearFaults();
        secondaryFlywheelMotor.setIdleMode(IdleMode.kCoast);
        secondaryFlywheelEncoder = secondaryFlywheelMotor.getEncoder();
        secondaryFlywheelPIDController = secondaryFlywheelMotor.getPIDController();
        secondaryFlywheelPIDController.setP(0);
        secondaryFlywheelPIDController.setFF(Constants.ScoringConstants.FLYWHEEL_PID_FF);
        secondaryFlywheelMotor.setSmartCurrentLimit(80); //big neo is good
        // secondaryFlywheelMotor.setClosedLoopRampRate(0.6); // it takes 0.6second to reach max speed
        secondaryFlywheelMotor.enableVoltageCompensation(12.0); 
        secondaryFlywheelMotor.follow(flywheelMotor, true);
        //secondaryFlywheelMotor.burnFlash();

        indexMotor = new CANSparkMax(Constants.CANAssignments.INDEX_MOTOR_ID, MotorType.kBrushless);
        indexMotor.restoreFactoryDefaults();
        indexMotor.clearFaults();
        indexMotor.setIdleMode(IdleMode.kCoast);
        indexMotor.setInverted(true);
        indexEncoder = indexMotor.getEncoder();
        indexPIDController = indexMotor.getPIDController();
        indexPIDController.setP(0);
        indexPIDController.setFF(Constants.ScoringConstants.INDEX_PID_FF);
        indexMotor.setSmartCurrentLimit(80); //big neo is good
        //indexMotor.setClosedLoopRampRate(0.6);
        indexMotor.enableVoltageCompensation(12.0); //TODO: Increase voltage compensation to 12.5+ for it to shoot from right next to speaker
        //indexMotor.burnFlash();

        secondaryIndexMotor = new CANSparkMax(Constants.CANAssignments.SECONDARY_INDEX_MOTOR_ID, MotorType.kBrushless);
        secondaryIndexMotor.restoreFactoryDefaults();
        secondaryIndexMotor.clearFaults();
        secondaryIndexMotor.setIdleMode(IdleMode.kCoast);
        secondaryIndexEncoder = secondaryIndexMotor.getEncoder();
        secondaryIndexPIDController = secondaryIndexMotor.getPIDController();
        secondaryIndexPIDController.setP(0);
        secondaryIndexPIDController.setFF(Constants.ScoringConstants.INDEX_PID_FF);
        secondaryIndexMotor.setSmartCurrentLimit(80); //big neo is good
        // secondaryIndexMotor.setClosedLoopRampRate(0.6);
        secondaryIndexMotor.enableVoltageCompensation(12.0); 
        secondaryIndexMotor.follow(indexMotor, true); //if wrong direction then just add "true" parameter or remove setInverted
        //secondaryIndexMotor.burnFlash();


        intakeMotor = new CANSparkMax(Constants.CANAssignments.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.clearFaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setInverted(true);
        intakeEncoder = intakeMotor.getEncoder();
        intakePIDController = intakeMotor.getPIDController();
        intakePIDController.setP(0);
        intakePIDController.setFF(Constants.ScoringConstants.INTAKE_PID_FF);
        intakeMotor.setSmartCurrentLimit(40); //limit lower cuz of neo 550
        //intakeMotor.setClosedLoopRampRate(0.6); 
        //intakeMotor.enableVoltageCompensation(12.0); 
        //intakeMotor.burnFlash();
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
     * <p>
     * When the subsystem is told to fire,
     * this is the target RPM the flywheel will spin up to.
     *
     * @param rpm - speed in rotations per minute.
     */
    public void setFlywheel(double rpm) {
        flywheelRPM = rpm;
    }

    public void increaseRPM(){
        if(flywheelRPM < 5500){
            flywheelRPM += 100;
        }
    }

    public void decreaseRPM(){
        if(flywheelRPM > 100){
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
                
                Logger.recordOutput("scoring.state", "LOADING");
                break;
            case FIRE:
                //flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);
                //indexPIDController.setReference(Constants.ScoringConstants.INDEX_FLYWHEEL_COOP_RPM, ControlType.kVelocity);
                
                //intakePIDController.setReference(Constants.ScoringConstants.INTAKE_RPM, ControlType.kVelocity);
                flywheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);
                if (isFlywheelReady() && flywheelEncoder.getVelocity() != Constants.ScoringConstants.FLYWHEEL_IDLE_RPM) {
                    indexPIDController.setReference(Constants.ScoringConstants.INDEX_FLYWHEEL_COOP_RPM, ControlType.kVelocity);
                }


                /*
                //flywheelMotor.set(1.0);
                //indexMotor.set(1.0);
                
                flywheelMotor.set(1.0);
                if(flywheelMotor.get() == 1.0){
                    indexMotor.set(1.0);
                }*/
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
        Logger.recordOutput("scoring.flywheel.rpm", flywheelEncoder.getVelocity());
        Logger.recordOutput("scoring.flywheel.rpmtarget", flywheelRPM);
        Logger.recordOutput("scoring.index.rpm", indexEncoder.getVelocity());
        Logger.recordOutput("scoring.intake.rpm", intakeEncoder.getVelocity());
    }

}
