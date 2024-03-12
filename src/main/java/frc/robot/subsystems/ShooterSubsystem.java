package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax flywheelMotor;
    private final RelativeEncoder flywheelEncoder;
    private final SparkPIDController flyWheelPIDController;
    private final CANSparkMax indexMotor;
    private final SparkPIDController indexPIDController;
    private double targetPosition = 0.0;

    /**
     * The target RPM the flywheel motor will spin up to.
     */
    public double flywheelRPM = 0;

    public ShooterSubsystem() {
        flywheelMotor = new CANSparkMax(Constants.CANAssignments.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelMotor.setIdleMode(IdleMode.kCoast);
        flywheelEncoder = flywheelMotor.getEncoder();
        flyWheelPIDController = flywheelMotor.getPIDController();
        flyWheelPIDController.setP(Constants.ShooterConstants.FLYWHEEL_PID_P);

        indexMotor = new CANSparkMax(Constants.CANAssignments.INDEX_MOTOR_ID, MotorType.kBrushless);
        indexMotor.setIdleMode(IdleMode.kCoast);
        indexPIDController = indexMotor.getPIDController();
        indexPIDController.setP(Constants.ShooterConstants.INDEX_PID_P);
    }

    /**
     * If the target flywheel RPM is within tolerances, the flywheel is considered ready.
     *
     * @return - boolean representing whether the flywheel is ready.
     */
    public boolean isFlywheelReady() {
        return flywheelEncoder.getVelocity() >= flywheelRPM - Constants.ShooterConstants.FLYWHEEL_TARGET_RPM_TOLERANCE && flywheelEncoder.getVelocity() <= flywheelRPM + Constants.ShooterConstants.FLYWHEEL_TARGET_RPM_TOLERANCE;
    }

    /**
     * Step the index motor to push a note into the flywheel.
     * <p>
     * The flywheel must be spinning in order for the periodic to process the action.
     */
    public void stepIndex() {
        targetPosition += Constants.ShooterConstants.INDEX_STEP_ROTATIONS;
    }

    /**
     * Set flywheel to a target RPM.
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
            flywheelRPM = Constants.ShooterConstants.FLYWHEEL_IDLE_RPM;
            return;
        }

        setFlywheel(rpm);
    }

    public void toggleAmp() {
        toggleFlywheel(Constants.ShooterConstants.FLYWHEEL_AMP_RPM);
    }

    public void toggleSpeaker() {
        toggleFlywheel(Constants.ShooterConstants.FLYWHEEL_SPEAKER_RPM);
    }

    @Override
    public void periodic() {
        flyWheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);
        
        if (!isFlywheelReady() || flywheelEncoder.getVelocity() == Constants.ShooterConstants.FLYWHEEL_IDLE_RPM) {
            indexMotor.stopMotor();
        } else {
            indexPIDController.setReference(targetPosition, ControlType.kPosition);
        }

        Logger.recordOutput("shooter.isready", isFlywheelReady());
        Logger.recordOutput("shooter.rpm", flywheelRPM);
    }
}
