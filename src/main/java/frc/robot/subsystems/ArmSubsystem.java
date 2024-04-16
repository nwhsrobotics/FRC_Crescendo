package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANAssignments;
import frc.robot.util.CanSpark;
import org.littletonrobotics.junction.Logger;

/**
 * This class represents the subsystem responsible for controlling the arm mechanism of the robot.
 */
public class ArmSubsystem extends SubsystemBase {
    // Motors for controlling the arm
    public final CANSparkMax rightShoulderMotor;
    public final CANSparkMax leftShoulderMotor;

    // PID controllers for the shoulder motors
    private final SparkPIDController rightShoulderPidController;
    private final SparkPIDController leftShoulderPidController;

    // Encoders for shoulder position feedback
    private final RelativeEncoder rightShoulderEncoder;
    private final RelativeEncoder leftShoulderEncoder;

    // Absolute encoder for shoulder position
    private final DutyCycleEncoder shoulderAbsoluteEncoder;

    // Target rotations for the arm
    private double targetRotations = 0;

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        // Initialize right shoulder motor
        rightShoulderMotor = new CanSpark(CANAssignments.RIGHT_SHOULDER_MOTOR_ID, CanSpark.MotorKind.NEO, CANSparkBase.IdleMode.kBrake);
        rightShoulderEncoder = rightShoulderMotor.getEncoder();
        rightShoulderPidController = rightShoulderMotor.getPIDController();
        rightShoulderPidController.setP(0.25);
        rightShoulderPidController.setOutputRange(-ArmConstants.SHOULDER_OUTPUT_LIMIT, ArmConstants.SHOULDER_OUTPUT_LIMIT);

        // Initialize left shoulder motor
        leftShoulderMotor = new CanSpark(CANAssignments.LEFT_SHOULDER_MOTOR_ID, CanSpark.MotorKind.NEO, CANSparkBase.IdleMode.kBrake);
        leftShoulderMotor.setInverted(true);
        leftShoulderEncoder = leftShoulderMotor.getEncoder();
        leftShoulderPidController = leftShoulderMotor.getPIDController();
        leftShoulderPidController.setP(0.25); // Set proportional gain
        leftShoulderPidController.setOutputRange(-ArmConstants.SHOULDER_OUTPUT_LIMIT, ArmConstants.SHOULDER_OUTPUT_LIMIT);

        // Initialize absolute encoder
        shoulderAbsoluteEncoder = new DutyCycleEncoder(ArmConstants.ABSOLUTE_ENCODER_DIO_CHANNEL);

        // Initialize shoulder encoder positions
        rightShoulderEncoder.setPosition(getAbsoluteEncoderRotations());
        leftShoulderEncoder.setPosition(getAbsoluteEncoderRotations());
    }

    /**
     * Get the absolute encoder rotations.
     *
     * @return Absolute encoder rotations.
     */
    private double getAbsoluteEncoderRotations() {
        return 0; // TODO: Implement absolute encoder reading
    }

    /**
     * Convert degrees to motor rotations.
     *
     * @param degrees Degrees to convert.
     * @return Motor rotations.
     */
    private double degreesToMotorRotation(double degrees) {
        return (degrees / 360.0) * ArmConstants.SHOULDER_GEAR_RATIO;
    }

    /**
     * Move the arm to the preset position for the Amp.
     */
    public void ampPreset() {
        targetRotations = degreesToMotorRotation(-20);
    }

    /**
     * Reset the arm encoders.
     */
    public void resetArmEncoders() {
        leftShoulderEncoder.setPosition(0);
        rightShoulderEncoder.setPosition(0);
    }

    /**
     * Move the arm to the preset position for the Source.
     */
    public void sourcePreset() {
        targetRotations = degreesToMotorRotation(33);
    }

    /**
     * Adjust the target position by a given amount of rotations.
     *
     * @param changeInPosition Change in rotations.
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
        // Set target position for PID control
        rightShoulderPidController.setReference(targetRotations, ControlType.kPosition, 0, -Math.signum(rightShoulderEncoder.getPosition()) * 0.35, ArbFFUnits.kVoltage);
        leftShoulderPidController.setReference(targetRotations, ControlType.kPosition, 0, -Math.signum(leftShoulderEncoder.getPosition()) * 0.35, ArbFFUnits.kVoltage);

        // Log arm positions
        Logger.recordOutput("arm.targetposition", targetRotations);
        Logger.recordOutput("arm.left.position", leftShoulderEncoder.getPosition());
        Logger.recordOutput("arm.right.position", rightShoulderEncoder.getPosition());
        Logger.recordOutput("arm.absposition", getAbsoluteEncoderRotations());
        Logger.recordOutput("arm.rawposition", shoulderAbsoluteEncoder.getAbsolutePosition());
    }
}
