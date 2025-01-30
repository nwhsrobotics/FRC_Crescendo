package frc.robot.util;

import com.revrobotics.sim.SparkFlexExternalEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;

public class ImprovedCanSparkFlex extends SparkFlex {

    public enum MotorKind {
        NEO550,
        NEO,
        VORTEX
    }

    public ImprovedCanSparkFlex(int id, MotorKind motor, SparkBaseConfig config, IdleMode mode, double volComp) {
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

    public ImprovedCanSparkFlex(int id, MotorKind motor, SparkBaseConfig config, IdleMode mode) {
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
}
