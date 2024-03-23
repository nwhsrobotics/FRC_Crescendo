package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANAssignments;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax rightShoulderMotor;
    public final CANSparkMax leftShoulderMotor;
    private final SparkPIDController rightShoulderPidController;
    private final SparkPIDController leftShoulderPidController;
    private final RelativeEncoder rightShoulderEncoder;
    private final RelativeEncoder leftShoulderEncoder;

    private final DutyCycleEncoder shoulderAbsoluteEncoder;

    private double targetRotations = 0;

    private double getAbsoluteEncoderRotations() {
        return shoulderAbsoluteEncoder.getAbsolutePosition() + ArmConstants.SHOULDER_ABS_ENCODER_ROTATION_OFFSET;
    }

    private double degreesToMotorRotation(double degrees) {
        return ((degrees / 360.0) * ArmConstants.SHOULDER_GEAR_RATIO);
    }

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        rightShoulderMotor = new CANSparkMax(CANAssignments.RIGHT_SHOULDER_MOTOR_ID, MotorType.kBrushless);
        rightShoulderMotor.setIdleMode(IdleMode.kBrake);
        rightShoulderEncoder = rightShoulderMotor.getEncoder();
        rightShoulderPidController = rightShoulderMotor.getPIDController();
        rightShoulderPidController.setP(.25);
        rightShoulderPidController.setOutputRange(-ArmConstants.SHOULDER_OUTPUT_LIMIT, ArmConstants.SHOULDER_OUTPUT_LIMIT);
        
        leftShoulderMotor = new CANSparkMax(CANAssignments.LEFT_SHOULDER_MOTOR_ID, MotorType.kBrushless);
        leftShoulderMotor.setIdleMode(IdleMode.kBrake);
        leftShoulderMotor.setInverted(true);
        leftShoulderEncoder = leftShoulderMotor.getEncoder();
        leftShoulderPidController = leftShoulderMotor.getPIDController();
        leftShoulderPidController.setP(.25);
        leftShoulderPidController.setOutputRange(-ArmConstants.SHOULDER_OUTPUT_LIMIT, ArmConstants.SHOULDER_OUTPUT_LIMIT);
        
        shoulderAbsoluteEncoder = new DutyCycleEncoder(ArmConstants.ABSOLUTE_ENCODER_DIO_CHANNEL);

        rightShoulderEncoder.setPosition(getAbsoluteEncoderRotations());
        leftShoulderEncoder.setPosition(getAbsoluteEncoderRotations());
    }

    /**
     * Move the arm to the preset for the Amp.
     */
    public void ampPreset() {
        targetRotations = degreesToMotorRotation(20);
    }

    /**
     * Move the arm to the preset for the Source.
     */
    public void sourcePreset() {
        targetRotations = degreesToMotorRotation(-33);
    }

    /**
     * Adjust the target position in rotations.
     * 
     * @param changeInPosition - change in rotations. 
     */
    public void adjustByRotations(double changeInPosition) {
        targetRotations += changeInPosition;
    }
    
    /**
     * Move the arm to fit under the stage.
     */
    public void underStage() {
        targetRotations = degreesToMotorRotation(80);
    }

    @Override
    public void periodic() {
        rightShoulderPidController.setReference(0, ControlType.kDutyCycle);
        leftShoulderPidController.setReference(0, ControlType.kDutyCycle);
        // rightShoulderPidController.setReference(targetRotations, ControlType.kPosition);
        // leftShoulderPidController.setReference(targetRotations, ControlType.kPosition);

        Logger.recordOutput("arm.targetposition", targetRotations);
        
        Logger.recordOutput("arm.left.position", leftShoulderEncoder.getPosition());
        Logger.recordOutput("arm.right.position", rightShoulderEncoder.getPosition());

        Logger.recordOutput("arm.absposition", getAbsoluteEncoderRotations());
        Logger.recordOutput("arm.rawposition", shoulderAbsoluteEncoder.getAbsolutePosition());
    }
}
