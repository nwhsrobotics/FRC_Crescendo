package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax flywheelMotor;
    private final RelativeEncoder flywheelEncoder;
    private final SparkPIDController flywheelPidController;
    private final CANSparkMax indexMotor;
    private final SparkPIDController indexPidController;
    private double targetPosition = 0;

    /**
     * The target RPM the flywheel motor will spin up to.
     */
    public double flywheelRPM = 0;

    public ShooterSubsystem() {
        flywheelMotor = new CANSparkMax(Constants.CANAssignments.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelEncoder = flywheelMotor.getEncoder();
        flywheelPidController = flywheelMotor.getPIDController();
        flywheelPidController.setP(Constants.ShooterConstants.FLYWHEEL_PID_P);

        indexMotor = new CANSparkMax(Constants.CANAssignments.INDEX_MOTOR_ID, MotorType.kBrushless);
        indexPidController = indexMotor.getPIDController();
        indexPidController.setP(Constants.ShooterConstants.INDEX_PID_P);
    }

    /**
     * If the target flywheel RPM is within tolerances, the flywheel is considered ready.
     *
     * @return - boolean representing whether the flywheel is ready.
     */
    public boolean isFlywheelReady() {
        double lower = flywheelRPM - Constants.ShooterConstants.FLYWHEEL_TARGET_RPM_TOLERANCE;
        double upper = flywheelRPM + Constants.ShooterConstants.FLYWHEEL_TARGET_RPM_TOLERANCE;
        double x = flywheelEncoder.getVelocity();

        return x >= lower && x <= upper;
    }

    /**
     * Step the index motor to push a note into the flywheel.
     * <p>
     * The flywheel must be spinning in order for the periodic to process the action.
     */
    public void stepIndex() {
        targetPosition += Constants.ShooterConstants.INDEX_STEP_ROTATIONS;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isFlywheelReady", isFlywheelReady());
        flywheelPidController.setReference(flywheelRPM, ControlType.kVelocity);

        if (!isFlywheelReady()) {
            indexMotor.stopMotor();
        } else {
            indexPidController.setReference(targetPosition, ControlType.kPosition);
        }
    }
}
