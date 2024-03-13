// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class WristIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    public WristIntakeSubsystem() {
        motor = new CANSparkMax(Constants.CANAssignments.WRIST_INTAKE_ID, MotorType.kBrushless);
    }

    public void forwards() {
        motor.set(1.0);
    }

    public void backwards() {
        motor.set(-1.0);
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("wrist.intakePower", motor.get());
    }


}
