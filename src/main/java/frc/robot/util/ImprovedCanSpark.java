package frc.robot.util;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;

public class ImprovedCanSpark extends SparkMax {

    public enum MotorKind {
        NEO550,
        NEO,
        VORTEX
    }

    public ImprovedCanSpark(int id, MotorKind motor, SparkBaseConfig config, IdleMode mode, double volComp) {
        super(id, MotorType.kBrushless);
        clearFaults();
        config.idleMode(mode);
        setVoltage(volComp);
        //config.voltageCompensation(volComp)
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public ImprovedCanSpark(int id, MotorKind motor,SparkBaseConfig config, IdleMode mode) {
        super(id, MotorType.kBrushless);
        clearFaults();
        config.idleMode(mode);
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }


    public ImprovedCanSpark(int id, MotorKind motor, IdleMode mode, double volComp) {
        super(id, MotorType.kBrushless);
        clearFaults();
        SparkMaxConfig config = new SparkMaxConfig();
        setVoltage(volComp);
        //config.voltageCompensation(volComp)
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public ImprovedCanSpark(int id, MotorKind motor, IdleMode mode) {
        super(id, MotorType.kBrushless);
        clearFaults();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
}
