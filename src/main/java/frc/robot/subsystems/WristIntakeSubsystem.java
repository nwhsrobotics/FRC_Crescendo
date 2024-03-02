// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristIntakeSubsystem extends SubsystemBase {
  private final CANSparkMax motor;


  /** Creates a new WristIntakeSubsystem. */
  public WristIntakeSubsystem() {
    motor = new CANSparkMax(Constants.CANAssignments.WRIST_INTAKE_ID, MotorType.kBrushless);


  }

  public void forwards(){
    motor.set(1.0);
  }

  public void backwards(){
    motor.set(-1.0);
  }

  public void stop(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
