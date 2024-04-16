package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.CanSpark;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    public final CANSparkMax wristMotor;
    private final SparkPIDController wristPidController;
    private final RelativeEncoder wristRelativeEncoder;
    private final DutyCycleEncoder wristAbsoluteEncoder;
    //private double currentPosition;
    private double desiredPosition;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;
    private final double maxRotPerTick = 0.5;


    // Constructor for WristSubsystem
    public WristSubsystem() {
        wristMotor = new CanSpark(Constants.CANAssignments.WRIST_MOTOR_ID, CanSpark.MotorKind.NEO550, CANSparkBase.IdleMode.kBrake);
        wristRelativeEncoder = wristMotor.getEncoder();
        wristAbsoluteEncoder = new DutyCycleEncoder(WristConstants.ABSOLUTE_ENCODER_DIO_CHANNEL);
        wristPidController = wristMotor.getPIDController();
        wristPidController.setP(0.25);
        //is it the setOutputRange thats caussing it not to move 90 degrees?
        wristPidController.setOutputRange(-maxRotPerTick, maxRotPerTick);

        // wristRelativeEncoder.setPosition(wristAbsoluteEncoder.getAbsolutePosition() + WristConstants.absOffset);
        // wristPidController.setP(Constants.WristConstants.WRIST_PID_P);
        //currentPosition = wristRelativeEncoder.getPosition();
        //desiredPosition = currentPosition;
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

        desiredPosition = (90.0 / 360.0) * WristConstants.WRIST_GEAR_RATIO;

    }

    public void underStage() {

        desiredPosition = 0.0;

    }

    // Called once per scheduler run
    @Override
    public void periodic() {
        wristPidController.setReference(desiredPosition, ControlType.kPosition);
        Logger.recordOutput("wrist.desiredPosition", desiredPosition);
        //Logger.recordOutput("wrist.currentPosition", currentPosition);
        Logger.recordOutput("wrist.autoLockEnabledAmp", autoLockEnabledAmp);
        Logger.recordOutput("wrist.autoLockEnabledSource", autoLockEnabledSource);
    }
}