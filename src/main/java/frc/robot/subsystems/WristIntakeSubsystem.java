// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ImprovedCanSpark;
import org.littletonrobotics.junction.Logger;

public class WristIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    public WristIntakeSubsystem() {
        motor = new ImprovedCanSpark(Constants.CANAssignments.WRIST_INTAKE_ID, ImprovedCanSpark.MotorKind.NEO550, CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("wrist.intakePower", motor.get());
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }
}
