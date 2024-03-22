package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    public final CANSparkMax wristMotor;
    private final SparkPIDController wristPidController;
    private final RelativeEncoder wristRelativeEncoder;
    private AbsoluteEncoder wristAbsoluteEncoder;
    private double currentPos_deg;
    private double desiredPos_deg;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;
    private double maxRotPerTick = 0.5;
    

    // Constructor for WristSubsystem
    public WristSubsystem() {
        wristMotor = new CANSparkMax(Constants.CANAssignments.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristRelativeEncoder = wristMotor.getEncoder();
        wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();
        wristPidController = wristMotor.getPIDController();
        wristPidController.setP(0.25);
        


        // wristPidController.setP(Constants.WristConstants.WRIST_PID_P);
        currentPos_deg = wristRelativeEncoder.getPosition();

        double absRaw = wristAbsoluteEncoder.getPosition();
        double adjustAbs = absRaw - Constants.WristConstants.absOffset;

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
        wristRelativeEncoder.setPosition(degreesToMotorRotation(currentPos_deg));

        // Set the PID controller references to the current position
        wristPidController.setReference(degreesToMotorRotation(currentPos_deg), ControlType.kPosition);
    }

    // Sets the desired position to a specified angle
    public void adjustAngle(double changeInPosition) {
        desiredPos_deg += motorRotationTodegrees(changeInPosition);
        
    }

    public double degreesToMotorRotation(double degrees) {
        return ((degrees / 360.0) * ArmConstants.SHOULDER_GEAR_RATIO);
    }

    public double motorRotationTodegrees(double rot) {
        return (rot/ArmConstants.SHOULDER_GEAR_RATIO)*360;
    }

    // Sets the desired position to a pre-determined angle for the amp
    public void ampPreset() {

        desiredPos_deg = 0.0;

    }

    // Sets the desired position to a pre-determined angle for the source
    public void sourcePreset() {

        desiredPos_deg = (90.0);

    }

    public void underStage() {

        desiredPos_deg = 0.0;

    }

    // Called once per scheduler run
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
        System.out.println("Wrist current position" + currentPos_deg + "" + desiredPos_deg);
        wristPidController.setReference(currentPos_deg, ControlType.kPosition);
        //System.out.println(String.format("Desired Position: %f Current Position: %f Difference: %f", desiredPosition, wristRelativeEncoder.getPosition(), desiredPosition - wristRelativeEncoder.getPosition()));
        //Logger.recordOutput("wrist.desiredPosition", desiredPosition);
        //Logger.recordOutput("wrist.currentPosition", currentPosition);
        Logger.recordOutput("wrist.autoLockEnabledAmp", autoLockEnabledAmp);
        Logger.recordOutput("wrist.autoLockEnabledSource", autoLockEnabledSource);
    }
}
