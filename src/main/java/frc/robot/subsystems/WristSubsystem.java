package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.ImprovedCanSpark;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController wristPidController;
    private final RelativeEncoder wristRelativeEncoder;
    private final DutyCycleEncoder wristAbsoluteEncoder;
    //private double currentPosition;
    private double desiredPosition;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;
    private final double maxRotPerTick = 0.5;
    private final SparkMaxConfig wristConfig = new SparkMaxConfig();

    // Constructor for WristSubsystem
    public WristSubsystem() {
        wristConfig.closedLoop.p(0.25);
        wristConfig.closedLoop.outputRange(-maxRotPerTick, maxRotPerTick);
        wristMotor = new ImprovedCanSpark(Constants.CANAssignments.WRIST_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, IdleMode.kBrake);
        wristRelativeEncoder = wristMotor.getEncoder();
        wristAbsoluteEncoder = new DutyCycleEncoder(WristConstants.ABSOLUTE_ENCODER_DIO_CHANNEL);
        wristPidController = wristMotor.getClosedLoopController();
        //is it the setOutputRange thats caussing it not to move 90 degrees?

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

    public void stopMotor(){
        wristMotor.stopMotor();
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