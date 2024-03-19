package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax shoulderMotor;
    //public final CANSparkMax sensorHub;
    private final SparkPIDController shoulderPidController;
    private final RelativeEncoder shoulderRelativeEncoder;
    private final CANSparkMax forgottenByTim;
    private double desiredPosition = 0; // Set the arms angle at this degree
    private final double currentPosition;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;

    

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(19, MotorType.kBrushless);
        forgottenByTim = new CANSparkMax(17, MotorType.kBrushless);
        forgottenByTim.follow(shoulderMotor, true);
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        forgottenByTim.setIdleMode(IdleMode.kBrake);
        shoulderRelativeEncoder = shoulderMotor.getEncoder();


        shoulderPidController = shoulderMotor.getPIDController();
        shoulderPidController.setP(.1);
        currentPosition = shoulderRelativeEncoder.getPosition();
        desiredPosition = currentPosition;

        //sensorHub = new CANSparkMax(Constants.CANAssignments.ARM_SENSOR_HUB_ID, MotorType.kBrushless);
    }

    // Converts degrees to units
    public void convertDegreeToRotations() {

    }

    // Sets the desired position to a pre-determined angle for the amp
    public void ampPreset() {
        desiredPosition = (140.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
    }

    // Sets the desired position to a pre-determined angle for the source
    public void sourcePreset() {

        desiredPosition = (40.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;

    }

    // Adjusts the current angle by adding a specified amount to the desired position
    public void adjustAngle(double changeInPosition) {

        desiredPosition += changeInPosition;

    }

    public void moveUp(){
        shoulderMotor.set(0.5);
    }

    public void moveDown(){
        shoulderMotor.set(-0.5);
    }

    /*
      public void underStage(){
        desiredPosition = ()
    }

     */
   

    @Override
    public void periodic() {
        shoulderPidController.setReference(desiredPosition, ControlType.kPosition);

        Logger.recordOutput("arm.desiredPosition", desiredPosition);
        // Logger.recordOutput("arm.currentPosition", currentPosition);
        Logger.recordOutput("arm.autoLockEnabledAmp", autoLockEnabledAmp);
        Logger.recordOutput("arm.autoLockEnabledSource", autoLockEnabledSource);
    }
}
