package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax shoulderMotor;
    //public final CANSparkMax sensorHub;
    private final SparkPIDController shoulderPidController;
    private final SparkPIDController shoulderPidController2;
    private final RelativeEncoder shoulderRelativeEncoder;
    private final RelativeEncoder shoulderRelativeEncoder2;
    private final CANSparkMax shoulderMotor2;
    private double desiredPosition = 0; // Set the arms angle at this degree
    private double currentPosition;
    private double maxRotPerTick = 0.20;
    private DigitalInput limit;
    

    // Constructor for ArmSubsystem
    public ArmSubsystem() {
        shoulderMotor = new CANSparkMax(19, MotorType.kBrushless);
        shoulderMotor2 = new CANSparkMax(17, MotorType.kBrushless);

        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderMotor2.setIdleMode(IdleMode.kBrake);

        shoulderMotor2.setInverted(true);

        shoulderRelativeEncoder = shoulderMotor.getEncoder();
        shoulderRelativeEncoder2 = shoulderMotor2.getEncoder();

        shoulderPidController = shoulderMotor.getPIDController();
        shoulderPidController2 = shoulderMotor2.getPIDController();
        shoulderPidController.setP(.25);
        shoulderPidController2.setP(0.1);
        //shoulderPidController.setOutputRange(-0.5, 0.5);

        currentPosition = shoulderRelativeEncoder.getPosition();
        desiredPosition = currentPosition;
        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10));

        limit = new DigitalInput(0);


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
    
    public void home(){
        

        if(currentPosition > 0.0){
            shoulderMotor.set(-0.2);
            shoulderMotor2.set(0.2);
        }

        if(currentPosition < 0.0){
            shoulderMotor.set(0.2);
            shoulderMotor2.set(-0.2);
        }

        if(limit.get()){
            shoulderMotor.set(0);
            shoulderMotor2.set(0);
            
        }

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
/* 
        if(currentPosition > ((33.0/360)*ArmConstants.SHOULDER_GEAR_RATIO)){
            desiredPosition = (33.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
        }
        else if(currentPosition < (-20.0/360)*ArmConstants.SHOULDER_GEAR_RATIO){
            desiredPosition = -(20.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
        }*/


        //System.out.println(currentPosition + "" + desiredPosition);
        //might change the currentPosition parameter back to desiredPosiition
        shoulderPidController.setReference(currentPosition, ControlType.kPosition);
        shoulderPidController2.setReference(currentPosition, ControlType.kPosition);
        //shoulderPidController.setReference(currentPosition, ControlType.kPosition, 0, 0.2, ArbFFUnits.kVoltage); //TODO: THIS
        //System.out.println(desiredPosition + " " +  shoulderRelativeEncoder.getPosition());
        Logger.recordOutput("arm.desiredPosition", desiredPosition);


        // Logger.recordOutput("arm.currentPosition", currentPosition);
        //Logger.recordOutput("arm.autoLockEnabledAmp", autoLockEnabledAmp);
        //Logger.recordOutput("arm.autoLockEnabledSource", autoLockEnabledSource);
    }
}
