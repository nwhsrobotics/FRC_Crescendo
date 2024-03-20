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
    private final CANSparkMax secondaryShoulderMotor;
    private double desiredPosition = 0; // Set the arms angle at this degree
    private double currentPosition;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;
    private double maxRotPerTick = 0.1;
    

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(19, MotorType.kBrushless);
        secondaryShoulderMotor = new CANSparkMax(17, MotorType.kBrushless);
        secondaryShoulderMotor.follow(shoulderMotor, true);
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        secondaryShoulderMotor.setIdleMode(IdleMode.kBrake);
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
        desiredPosition = -(20.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
    }

    // Sets the desired position to a pre-determined angle for the source
    public void sourcePreset() {

        desiredPosition = (33.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;

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

    
      public void underStage(){
        desiredPosition = (80.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
    }

     
   

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
        shoulderPidController.setReference(currentPosition, ControlType.kPosition);
        //shoulderPidController.setReference(currentPosition, ControlType.kPosition, 0, 2, ArbFFUnits.kVoltage); //TODO: FEEDFORWARD TO STAY IN PLACE/ COUNTERACT GRAVITY LOOK AT 888'S
        //use TrapezoidProfile.State for smoothing out again look at 888's
        // https://github.com/Robotiators-888/2024Season/blob/main/src/main/java/frc/robot/subsystems/SUB_Pivot.java
        System.out.println(desiredPosition + " " +  shoulderRelativeEncoder.getPosition());
        Logger.recordOutput("arm.desiredPosition", desiredPosition);
        // Logger.recordOutput("arm.currentPosition", currentPosition);
        Logger.recordOutput("arm.autoLockEnabledAmp", autoLockEnabledAmp);
        Logger.recordOutput("arm.autoLockEnabledSource", autoLockEnabledSource);
    }
}
