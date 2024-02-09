package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax shoulderMotor;
    private SparkPIDController shoulderPidController;
    private RelativeEncoder shoulderRelativeEncoder;
    private int ampDegree = 104; //Measured from cad, rounded to the nearest whole number
    private int sourceDegree = 40; //Measured from cad, rounded to the nearest whole number
    private int desiredDegree = 0; //set the arms angle at this degree
    private boolean autoLockEnabled = false;
    





    /**
     * Creates a new ArmSubsystem.
     */
    public ArmSubsystem() {
        this.shoulderMotor = new CANSparkMax(Constants.CANAssignments.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        this.shoulderRelativeEncoder = this.shoulderMotor.getEncoder();
        this.shoulderPidController = this.shoulderMotor.getPIDController();
        this.shoulderPidController.setP(Constants.ArmConstants.SHOULDER_PID_P);

    }

    public void ampPreset(){
        desiredDegree = ampDegree;

    }

    public void sourcePreset(){
        desiredDegree = sourceDegree;

    }

    public void adjustAngle(){
        if(!autoLockEnabled){
            //TODO: add logic that will let the operater freely adjust the motor position if autoLockEnabled is not true
        }

    }



    @Override
    public void periodic() {
        this.shoulderPidController.setReference(this.desiredDegree, ControlType.kPosition);








        // This method will be called once per scheduler run
    }
}
