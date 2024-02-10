package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax shoulderMotor;
    private SparkPIDController shoulderPidController;
    private RelativeEncoder shoulderRelativeEncoder;
    private double ampPosition = 104; //Measured from cad, rounded to the nearest whole number
    private double sourcePosition = 40; //Measured from cad, rounded to the nearest whole number
    private double desiredPosition = 0; //set the arms angle at this degree
    private double currentPosition = this.shoulderRelativeEncoder.getPosition();
    private boolean autoLockEnabledAmp = false;
    private boolean autoLockEnabledSource = false;
    





    /**
     * Creates a new ArmSubsystem.
     */
    public ArmSubsystem() {
        this.shoulderMotor = new CANSparkMax(Constants.CANAssignments.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        this.shoulderRelativeEncoder = this.shoulderMotor.getEncoder();
        this.shoulderPidController = this.shoulderMotor.getPIDController();
        this.shoulderPidController.setP(Constants.ArmConstants.SHOULDER_PID_P);

    }

    public void convertDegreeToRotations(){

    }



    public void ampPreset(){
        

        desiredPosition = (ampPosition/360) * ArmConstants.SHOULDER_GEAR_RATIO;

    }

    public void sourcePreset(){
        

        desiredPosition = (sourcePosition/360) * ArmConstants.SHOULDER_GEAR_RATIO;
        

    }

    public void adjustAngle(double changeInPosition){
        if(!autoLockEnabledAmp || !autoLockEnabledSource){
            desiredPosition += changeInPosition;
            //TODO: add logic that will let the operater freely adjust the motor position if autoLockEnabled is not tru
        }
        else if(autoLockEnabledAmp){
            ampPreset();
        }
        else if(autoLockEnabledSource){
            sourcePreset();
        }
        

    }



    @Override
    public void periodic() {
        this.shoulderPidController.setReference(this.desiredPosition, ControlType.kPosition);








        // This method will be called once per scheduler run
    }
}
