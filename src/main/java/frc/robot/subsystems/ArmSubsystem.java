package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANAssignments;
import frc.robot.util.ImprovedCanSpark;
import org.littletonrobotics.junction.Logger;

/**
 * This class represents the subsystem responsible for controlling the arm mechanism of the robot.
 */
public class ArmSubsystem extends SubsystemBase {
    // Motors for controlling the arm
    private final SparkMax rightShoulderMotor;
    private final SparkMax leftShoulderMotor;

    // PID controllers for the shoulder motors
    private final SparkClosedLoopController rightShoulderPidController;
    private final SparkClosedLoopController leftShoulderPidController;

    // Encoders for shoulder position feedback
    private final RelativeEncoder rightShoulderEncoder;
    private final RelativeEncoder leftShoulderEncoder;

    // Absolute encoder for shoulder position
    private final DutyCycleEncoder shoulderAbsoluteEncoder;

    private final SparkMaxConfig rightShoulderConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftShoulderConfig = new SparkMaxConfig();

    // Target rotations for the arm
    private double targetRotations = 0;

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        // Initialize right shoulder motor
        rightShoulderConfig.closedLoop.p(0.25);
        rightShoulderConfig.closedLoop.outputRange(-ArmConstants.SHOULDER_OUTPUT_LIMIT, ArmConstants.SHOULDER_OUTPUT_LIMIT);
        rightShoulderMotor = new ImprovedCanSpark(CANAssignments.RIGHT_SHOULDER_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, IdleMode.kBrake);
        rightShoulderEncoder = rightShoulderMotor.getEncoder();
        rightShoulderPidController = rightShoulderMotor.getClosedLoopController();

        // Initialize left shoulder motor
        leftShoulderConfig.closedLoop.p(0.25);
        leftShoulderConfig.closedLoop.outputRange(-ArmConstants.SHOULDER_OUTPUT_LIMIT, ArmConstants.SHOULDER_OUTPUT_LIMIT);
        leftShoulderMotor = new ImprovedCanSpark(CANAssignments.LEFT_SHOULDER_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, IdleMode.kBrake);
        leftShoulderMotor.setInverted(true);
        leftShoulderEncoder = leftShoulderMotor.getEncoder();
        leftShoulderPidController = leftShoulderMotor.getClosedLoopController();

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

    public void stopMotors(){
        rightShoulderMotor.stopMotor();
        leftShoulderMotor.stopMotor();
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
        rightShoulderPidController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, -Math.signum(rightShoulderEncoder.getPosition()) * 0.35, ArbFFUnits.kVoltage);
        leftShoulderPidController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, -Math.signum(leftShoulderEncoder.getPosition()) * 0.35, ArbFFUnits.kVoltage);

        // Log arm positions
        Logger.recordOutput("arm.targetposition", targetRotations);
        Logger.recordOutput("arm.left.position", leftShoulderEncoder.getPosition());
        Logger.recordOutput("arm.right.position", rightShoulderEncoder.getPosition());
        Logger.recordOutput("arm.absposition", getAbsoluteEncoderRotations());
        Logger.recordOutput("arm.rawposition", shoulderAbsoluteEncoder.get());
    }
}
