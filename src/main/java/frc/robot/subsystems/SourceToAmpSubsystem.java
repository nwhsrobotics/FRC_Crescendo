// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
 package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ScoringSubsystem.ScoringState;

public class SourceToAmpSubsystem extends SubsystemBase {

  public enum ControlState {
    
    AMP,
    
    SOURCE,
    
    FIRE,


    LOADING,
    
    IDLE,
}


   public ControlState state = ControlState.IDLE;

    private final CANSparkMax shoulderMotor;
    private final SparkPIDController shoulderPidController;
    private final RelativeEncoder shoulderRelativeEncoder;
    private double desiredPositionArm; // Set the arms angle at this degree
    private final double currentPositionArm;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;

    private final CANSparkMax wristMotor;
    private final SparkPIDController wristPidController;
    private final RelativeEncoder wristRelativeEncoder;
    private final double currentPositionWrist;
    private double desiredPositionWrist;
    private final boolean autoLockEnabledAmp = false;
    private final boolean autoLockEnabledSource = false;

    private final CANSparkMax wristIntakeMotor;


  
  public SourceToAmpSubsystem() {

    shoulderMotor = new CANSparkMax(Constants.CANAssignments.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        shoulderRelativeEncoder = shoulderMotor.getEncoder();
        shoulderPidController = shoulderMotor.getPIDController();
        shoulderPidController.setP(Constants.ArmConstants.SHOULDER_PID_P);
        currentPositionArm = shoulderRelativeEncoder.getPosition();

        wristMotor = new CANSparkMax(Constants.CANAssignments.WRIST_MOTOR_ID, MotorType.kBrushless);
        wristRelativeEncoder = wristMotor.getEncoder();
        wristPidController = wristMotor.getPIDController();
        wristPidController.setP(Constants.WristConstants.WRIST_PID_P);
        currentPositionWrist = wristRelativeEncoder.getPosition();

        wristIntakeMotor = new CANSparkMax(Constants.CANAssignments.WRIST_INTAKE_ID, MotorType.kBrushless);
  }
 

      public void ampPresetArm() {
          desiredPositionArm = (140.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
      }
  
      // Sets the desired position to a pre-determined angle for the source
      public void sourcePresetArm() {
  
          desiredPositionArm = (40.0 / 360) * ArmConstants.SHOULDER_GEAR_RATIO;
  
      }
  
      // Adjusts the current angle by adding a specified amount to the desired position
      public void adjustAngleArm(double changeInPosition) {
  
          desiredPositionArm += changeInPosition;
  
      }


      public void adjustAngleWrist(double changeInPosition) {

        desiredPositionWrist = changeInPosition;

    }

    // Sets the desired position to a pre-determined angle for the amp
    public void ampPresetWrist() {

        desiredPositionWrist = (218.0 / 360) * WristConstants.WRIST_GEAR_RATIO;

    }

    // Sets the desired position to a pre-determined angle for the source
    public void sourcePresetWrist() {

        desiredPositionWrist = (43.1 / 360) * WristConstants.WRIST_GEAR_RATIO;

    }



  

  @Override
  public void periodic() {

    switch (state) {
            case IDLE:
                shoulderPidController.setReference(0, ControlType.kDutyCycle);
                wristPidController.setReference(0, ControlType.kDutyCycle);  // don't even bother with velocity control, just turn them off.
                wristIntakeMotor.set(0);
                //Logger.recordOutput("scoring.state", "IDLE");
                break;
            case AMP:
                shoulderPidController.setReference(desiredPositionArm, ControlType.kPosition);
                wristPidController.setReference(desiredPositionWrist, ControlType.kPosition);
                //Logger.recordOutput("scoring.state", "LOADING");
                break;
            case SOURCE:
                sourcePresetArm();
                sourcePresetWrist();
                shoulderPidController.setReference(desiredPositionArm, ControlType.kPosition);
                wristPidController.setReference(desiredPositionWrist, ControlType.kPosition);
                wristIntakeMotor.set(1.0);
            case FIRE:
                wristIntakeMotor.set(-1.0);
                //Logger.recordOutput("scoring.state", "FIRE");
                break;
            case LOADING:
                wristIntakeMotor.set(1.0);
                
                //Logger.recordOutput("scoring.state", "UNLOADING");
                break;
            default:
                break;













    // This method will be called once per scheduler run
  }
  }


}
*/
