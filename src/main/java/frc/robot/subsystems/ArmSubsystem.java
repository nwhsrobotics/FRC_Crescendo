package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANAssignments;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax rightShoulderMotor;
    private final SparkPIDController rightShoulderPidController;
    private final SparkPIDController leftShoulderPidController;
    private final RelativeEncoder rightShoulderEncoder;
    private final RelativeEncoder leftShoulderEncoder;

    private final TalonSRX shoulderAbsoluteEncoderController;

    private final CANSparkMax leftShoulderMotor;
    private double targetRotations = 0;
    private double maxRotPerTick = 0.20;

    private double getAbsoluteEncoderRotations() {
        return (shoulderAbsoluteEncoderController.getSelectedSensorPosition() / ArmConstants.SHOULDER_ABS_ENCODER_TICKS_PER_ROTATION) + ArmConstants.SHOULDER_ABS_ENCODER_ROTATION_OFFSET;
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
        
        shoulderAbsoluteEncoderController = new TalonSRX(CANAssignments.ARM_SENSOR_HUB_ID);
        shoulderAbsoluteEncoderController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

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
        targetRotations = degreesToMotorRotation(33);
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
        rightShoulderPidController.setReference(targetRotations, ControlType.kPosition);
        leftShoulderPidController.setReference(targetRotations, ControlType.kPosition);

        Logger.recordOutput("arm.targetposition", targetRotations);
        Logger.recordOutput("arm.left.position", leftShoulderEncoder.getPosition());
        Logger.recordOutput("arm.right.position", rightShoulderEncoder.getPosition());
        Logger.recordOutput("arm.position", getAbsoluteEncoderRotations());
        Logger.recordOutput("arm.rawposition", shoulderAbsoluteEncoderController.getSelectedSensorPosition() / ArmConstants.SHOULDER_ABS_ENCODER_TICKS_PER_ROTATION);
    }
}
