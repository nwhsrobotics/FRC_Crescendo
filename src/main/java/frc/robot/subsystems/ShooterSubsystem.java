package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax flywheelMotor;
    private RelativeEncoder flywheelEncoder;
    private PIDController flywheelPidController;
    private CANSparkMax indexMotor;
    private RelativeEncoder indexEncoder;
    private PIDController indexPidController;
    private double targetPosition = 0;
    
    /**
     * The target RPM the flywheel motor will spin up to.
     */
    public double flywheelRPM = 0;

    public ShooterSubsystem() {
        this.flywheelMotor = new CANSparkMax(Constants.ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        this.flywheelEncoder = this.flywheelMotor.getEncoder();
        this.indexMotor = new CANSparkMax(Constants.ShooterConstants.INDEX_MOTOR_ID, MotorType.kBrushless);
        this.indexEncoder = this.indexMotor.getEncoder();

        this.flywheelPidController = new PIDController(Constants.ShooterConstants.FLYWHEEL_PID_P, 0, 0);
        this.indexPidController = new PIDController(Constants.ShooterConstants.INDEX_PID_P, 0, 0);
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
     * 
     * The flywheel must be spinning in order for the periodic to process the action.
     */
    public void stepIndex() {
        this.targetPosition += Constants.ShooterConstants.INDEX_STEP_ROTATIONS;
    }

    @Override
    public void periodic() {
        this.flywheelMotor.set(this.flywheelPidController.calculate(this.flywheelEncoder.getVelocity(), this.flywheelRPM));

        if (!isFlywheelReady()) {
            this.indexMotor.stopMotor();
        }
        else {
            this.indexMotor.set(this.indexPidController.calculate(this.indexEncoder.getPosition(), this.targetPosition));
        }
    }
}
