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
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    public final CANSparkMax wristMotor;
    private final SparkPIDController wristPidController;
    private final RelativeEncoder wristRelativeEncoder;
    private AbsoluteEncoder wristAbsoluteEncoder;
    private double currentPosition;
    private double desiredPosition;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;
    private double maxRotPerTick = 0.1;
    

    // Constructor for WristSubsystem
    public WristSubsystem() {
        wristMotor = new CANSparkMax(Constants.CANAssignments.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristRelativeEncoder = wristMotor.getEncoder();
        wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();
        wristPidController = wristMotor.getPIDController();
        wristPidController.setP(0.1);
        


        // wristPidController.setP(Constants.WristConstants.WRIST_PID_P);
        currentPosition = wristRelativeEncoder.getPosition();
        desiredPosition = currentPosition;
        // currentPosition = wristAbsoluteEncoder.getPosition();
    }

    // Sets the desired position to a specified angle
    public void adjustAngle(double changeInPosition) {
        desiredPosition += changeInPosition;
        
    }

    

    // Sets the desired position to a pre-determined angle for the amp
    public void ampPreset() {

        desiredPosition = 0.0;

    }

    // Sets the desired position to a pre-determined angle for the source
    public void sourcePreset() {

        desiredPosition = (90.0 / 360) * WristConstants.WRIST_GEAR_RATIO;

    }

    // Called once per scheduler run
    @Override
    public void periodic() {
        if(currentPosition + maxRotPerTick < desiredPosition){
            currentPosition += maxRotPerTick;
            System.out.println("Is less");
        } else if (currentPosition - maxRotPerTick > desiredPosition){
            currentPosition -= maxRotPerTick;
            System.out.println("Is more");
        } else {
            currentPosition = desiredPosition;
            System.out.println("Is in range");
        }
        System.out.println(currentPosition + "" + desiredPosition);
        wristPidController.setReference(currentPosition, ControlType.kPosition);
        //System.out.println(String.format("Desired Position: %f Current Position: %f Difference: %f", desiredPosition, wristRelativeEncoder.getPosition(), desiredPosition - wristRelativeEncoder.getPosition()));
        Logger.recordOutput("wrist.desiredPosition", desiredPosition);
        Logger.recordOutput("wrist.currentPosition", currentPosition);
        Logger.recordOutput("wrist.autoLockEnabledAmp", autoLockEnabledAmp);
        Logger.recordOutput("wrist.autoLockEnabledSource", autoLockEnabledSource);
    }
}
