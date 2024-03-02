package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax shoulderMotor;
    private final SparkPIDController shoulderPidController;
    private final RelativeEncoder shoulderRelativeEncoder;
    private final double ampPosition = 104; // Measured from cad, rounded to the nearest whole number
    private final double sourcePosition = 40; // Measured from cad, rounded to the nearest whole number
    private double desiredPosition = 0; // Set the arms angle at this degree
    private final double currentPosition;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(Constants.CANAssignments.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        shoulderRelativeEncoder = shoulderMotor.getEncoder();
        shoulderPidController = shoulderMotor.getPIDController();
        shoulderPidController.setP(Constants.ArmConstants.SHOULDER_PID_P);
        currentPosition = shoulderRelativeEncoder.getPosition();

    }

    // Converts degrees to units
    public void convertDegreeToRotations() {

    }

    // Sets the desired position to a pre-determined angle for the amp
    public void ampPreset() {
        desiredPosition = (ampPosition / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
    }

    // Sets the desired position to a pre-determined angle for the source
    public void sourcePreset() {


        desiredPosition = (sourcePosition / 360) * ArmConstants.SHOULDER_GEAR_RATIO;


    }

    // Adjusts the current angle by adding a specified amount to the desired position
    public void adjustAngle(double changeInPosition) {

        desiredPosition += changeInPosition;
        //TODO: add logic that will let the operater freely adjust the motor position if autoLockEnabled is not true


    }


    @Override
    public void periodic() {
        shoulderPidController.setReference(desiredPosition, ControlType.kPosition);


        // This method will be called once per scheduler run
    }
}
