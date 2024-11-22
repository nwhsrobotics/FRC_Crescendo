package frc.robot.util;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

public class ImprovedCanSparkFlex extends CANSparkFlex {

    public enum MotorKind {
        NEO550,
        NEO
    }

    public ImprovedCanSparkFlex(int id, MotorKind motor, IdleMode mode, double volComp) {
        super(id, MotorType.kBrushless);
        restoreFactoryDefaults();
        clearFaults();
        setIdleMode(mode);
        setVoltage(volComp);
        switch(motor){
            case NEO -> setSmartCurrentLimit(80);
            case NEO550 -> setSmartCurrentLimit(20);
        }
        if (DriverStation.isFMSAttached()) {
            burnFlash();
        }
    }

    public ImprovedCanSparkFlex(int id, frc.robot.util.ImprovedCanSpark.MotorKind neo, IdleMode mode) {
        super(id, MotorType.kBrushless);
        restoreFactoryDefaults();
        clearFaults();
        setIdleMode(mode);
        switch(neo){
            case NEO -> setSmartCurrentLimit(80);
            case NEO550 -> setSmartCurrentLimit(20);
        }
        if (DriverStation.isFMSAttached()) {
            burnFlash();
        }
    }
}
