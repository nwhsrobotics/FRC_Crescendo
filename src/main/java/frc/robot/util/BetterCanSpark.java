// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class BetterCanSpark{
    public static enum MotorKind {
        NEO550,
        NEO
    }

    public static CANSparkMax intializeMax(int id, MotorKind motor, IdleMode mode, double volComp){
        CANSparkMax controller = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
        controller.restoreFactoryDefaults();
        controller.clearFaults();
        controller.setIdleMode(mode);
        if(volComp > 0){
            controller.setVoltage(volComp);
        }
        if(motor.equals(MotorKind.NEO)){
            controller.setSmartCurrentLimit(80);
        } else if(motor.equals(MotorKind.NEO550)){
            controller.setSmartCurrentLimit(20);
        }
        if(DriverStation.isFMSAttached()){
            controller.burnFlash();
        }
        return controller;
    }


}
