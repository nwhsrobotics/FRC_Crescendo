package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANAssignments;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax rightShoulderMotor;
    private final SparkPIDController rightShoulderPidController;
    private final SparkPIDController leftShoulderPidController;
    private final RelativeEncoder rightShoulderRelativeEncoder;
    private final RelativeEncoder leftShoulderRelativeEncoder;

    private final TalonSRX shoulderAbsoluteEncoderController;
    private final SensorCollection shoulderAbsoluteEncoder;

    private final CANSparkMax leftShoulderMotor;
    //private double desiredAngleRotations = 0; // Set the arms angle at this degree
    private double desiredPos_deg = 0;
    private double currentPos_deg;
    private double maxRotPerTick = 0.20;
    //private double absRawRotations;
    //private double absAdjust;
    //private DigitalInput limit;
    //private int cpr = 0;

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        rightShoulderMotor = new CANSparkMax(CANAssignments.RIGHT_SHOULDER_MOTOR_ID, MotorType.kBrushless);
        rightShoulderMotor.setIdleMode(IdleMode.kBrake);
        rightShoulderRelativeEncoder = rightShoulderMotor.getEncoder();
        rightShoulderPidController = rightShoulderMotor.getPIDController();
        rightShoulderPidController.setP(.25);
        
        leftShoulderMotor = new CANSparkMax(CANAssignments.LEFT_SHOULDER_MOTOR_ID, MotorType.kBrushless);
        leftShoulderMotor.setIdleMode(IdleMode.kBrake);
        leftShoulderMotor.setInverted(true);
        leftShoulderRelativeEncoder = leftShoulderMotor.getEncoder();
        leftShoulderPidController = leftShoulderMotor.getPIDController();
        leftShoulderPidController.setP(.25);
        
        shoulderAbsoluteEncoderController = new TalonSRX(CANAssignments.ARM_SENSOR_HUB_ID);
        //shoulderAbsoluteEncoderController
        shoulderAbsoluteEncoder = shoulderAbsoluteEncoderController.getSensorCollection();

        //shoulderPidController.setOutputRange(-0.5, 0.5);

        currentPos_deg = rightShoulderRelativeEncoder.getPosition();

        double absRaw = shoulderAbsoluteEncoder.getPulseWidthPosition();
        double adjustAbs = absRaw - ShoulderConstants.absOffset;

        // Normalize the adjusted absolute position between -0.5 and 0.5 instead of between 0 and 1
        if (adjustAbs > 0.5) {
            adjustAbs -= 1.0;
        }
        if (adjustAbs < -0.5) {
            adjustAbs += 1.0;
        }

        // Convert the normalized absolute position to degrees
        adjustAbs *= 360.0;



        // Log the adjusted absolute position and update the current and desired positions in degrees
        //logger.recordOutput("shoulder.adjustedAbs", -adjustAbs);
        currentPos_deg = -adjustAbs;
        desiredPos_deg = -adjustAbs;

        // Set the relative encoder positions of the shoulder motors
        rightShoulderRelativeEncoder.setPosition(degreesToMotorRotation(currentPos_deg));
        leftShoulderRelativeEncoder.setPosition(degreesToMotorRotation(-currentPos_deg));

        // Set the PID controller references to the current position
        rightShoulderPidController.setReference(degreesToMotorRotation(currentPos_deg), ControlType.kPosition);
        leftShoulderPidController.setReference(-degreesToMotorRotation(currentPos_deg), ControlType.kPosition);

        // Print debugging information
        System.out.printf("===========================Abs raw: %f, adjusted abs: %f\n", absRaw, adjustAbs);
        System.out.printf("===========================Desired pos: %f, Current pos: %f\n\n", desiredPos_deg, currentPos_deg);
    }

    // Converts degrees to units
    public void convertDegreeToRotations() {

    }

    // Sets the desired position to a pre-determined angle for the amp
    public void ampPreset() {
        //angle should be subject to change
        
        //desiredAngleRotations = (20.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
        desiredPos_deg =  20.0;
    }

    // Sets the desired position to a pre-determined angle for the source
    public void sourcePreset() {
        
        //desiredAngleRotations = (33.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
        desiredPos_deg = 33.0;



    }

    // Adjusts the current angle by adding a specified amount to the desired position
    public void adjustAngle(double changeInPosition) {
        desiredPos_deg += (changeInPosition / ArmConstants.SHOULDER_GEAR_RATIO) * 360;
    }

    public double degreesToMotorRotation(double degrees) {
        return ((degrees / 360.0) * ArmConstants.SHOULDER_GEAR_RATIO);
    }

    public double motorRotationTodegrees(double rot) {
        return (rot/ArmConstants.SHOULDER_GEAR_RATIO)*360;
    }
    
    public void underStage(){
        //desiredAngleRotations = (80.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
        desiredPos_deg = 80.0;
    }

    @Override
    public void periodic() {



        if(degreesToMotorRotation(currentPos_deg) + maxRotPerTick < degreesToMotorRotation(desiredPos_deg)){
            currentPos_deg += motorRotationTodegrees(maxRotPerTick);
            System.out.println("Is less");
        } else if (currentPos_deg - motorRotationTodegrees(maxRotPerTick) > desiredPos_deg){
            currentPos_deg -= motorRotationTodegrees(maxRotPerTick);
            System.out.println("Is more");
        } else {
            currentPos_deg = desiredPos_deg;
            System.out.println("Is in range");
        }
/* 
        if(currentPosition > ((33.0/360)*ArmConstants.SHOULDER_GEAR_RATIO)){
            desiredPosition = (33.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
        }
        else if(currentPosition < (-20.0/360)*ArmConstants.SHOULDER_GEAR_RATIO){
            desiredPosition = -(20.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
        }*/


        //System.out.println(currentPosition + "" + desiredPosition);
        //might change the currentPosition parameter back to desiredPosiition
        rightShoulderPidController.setReference(degreesToMotorRotation(currentPos_deg), ControlType.kPosition);
        leftShoulderPidController.setReference(degreesToMotorRotation(currentPos_deg), ControlType.kPosition);
        //shoulderPidController.setReference(currentPosition, ControlType.kPosition, 0, -Math.signum(shoulderRelativeEncoder.getPosition())*0.5, ArbFFUnits.kVoltage); //TODO: THIS
        //System.out.println(desiredPosition + " " +  shoulderRelativeEncoder.getPosition());
        Logger.recordOutput("arm.desiredPosition", desiredPos_deg);


        // Logger.recordOutput("arm.currentPosition", currentPosition);
        //Logger.recordOutput("arm.autoLockEnabledAmp", autoLockEnabledAmp);
        //Logger.recordOutput("arm.autoLockEnabledSource", autoLockEnabledSource);
    }
}
