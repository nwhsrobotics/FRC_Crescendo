package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
        this.flywheelMotor = new CANSparkMax(Constants.CANAssignments.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        this.flywheelEncoder = this.flywheelMotor.getEncoder();
        this.flywheelPidController = this.flywheelMotor.getPIDController();
        this.flywheelPidController.setP(Constants.ShooterConstants.FLYWHEEL_PID_P);

        this.indexMotor = new CANSparkMax(Constants.CANAssignments.INDEX_MOTOR_ID, MotorType.kBrushless);
        this.indexPidController = this.indexMotor.getPIDController();
        this.indexPidController.setP(Constants.ShooterConstants.INDEX_PID_P);
    }

    /**
     * If the target flywheel RPM is within tolerances, the flywheel is considered ready.
     *
     * @return - boolean representing whether the flywheel is ready.
     */
    public boolean isFlywheelReady() {
        double lower = this.flywheelRPM - Constants.ShooterConstants.FLYWHEEL_TARGET_RPM_TOLERANCE;
        double upper = this.flywheelRPM + Constants.ShooterConstants.FLYWHEEL_TARGET_RPM_TOLERANCE;
        double x = this.flywheelEncoder.getVelocity();

        return x >= lower && x <= upper;
    }

    /**
     * Step the index motor to push a note into the flywheel.
     * <p>
     * The flywheel must be spinning in order for the periodic to process the action.
     */
    public void stepIndex() {
        this.targetPosition += Constants.ShooterConstants.INDEX_STEP_ROTATIONS;
    }

    @Override
    public void periodic() {
        this.flywheelPidController.setReference(this.flywheelRPM, ControlType.kVelocity);

        if (!isFlywheelReady()) {
            this.indexMotor.stopMotor();
        } else {
            this.indexPidController.setReference(this.targetPosition, ControlType.kPosition);
        }
    }
}
