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
    private final SparkPIDController flyWheelPIDController;
    private final CANSparkMax indexMotor;
    private final SparkPIDController indexPIDController;
    private double lastFlyWheelRPM = Constants.ShooterConstants.FLYWHEEL_SPEAKER_RPM;
    private double targetPosition = 0.0;

    /**
     * The target RPM the flywheel motor will spin up to.
     */
    public double flywheelRPM = 0;

    private boolean isFlywheelOn = false;

    public ShooterSubsystem() {
        flywheelMotor = new CANSparkMax(Constants.CANAssignments.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        flywheelEncoder = flywheelMotor.getEncoder();

        flyWheelPIDController = flywheelMotor.getPIDController();
        flyWheelPIDController.setP(Constants.ShooterConstants.FLYWHEEL_PID_P);

        indexMotor = new CANSparkMax(Constants.CANAssignments.INDEX_MOTOR_ID, MotorType.kBrushless);
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



    public void toggleFlywheel(){
        if(isFlywheelOn) {
            lastFlyWheelRPM = flywheelRPM;
            flywheelRPM = 0.0;
            isFlywheelOn = false;
        }
        else {
            flywheelRPM = lastFlyWheelRPM;
            isFlywheelOn = true;
        }
    }

    public void toggleAmp() {
        if (isFlywheelOn) {
            flywheelRPM = Constants.ShooterConstants.FLYWHEEL_AMP_RPM;
        }
    }

    public void toggleSpeaker(){
        if (isFlywheelOn) {
            flywheelRPM = Constants.ShooterConstants.FLYWHEEL_SPEAKER_RPM;
        }
    }






    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isFlywheelReady", isFlywheelReady());
        flyWheelPIDController.setReference(flywheelRPM, ControlType.kVelocity);

        if (!isFlywheelReady()) {
            indexMotor.stopMotor();
        } else {
            indexPIDController.setReference(targetPosition, ControlType.kPosition);
        }

        //SmartDashboard.putBoolean("isFlywheelOn",isFlywheelOn);
    }
}
