package frc.robot.util;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

public class ImprovedCanSpark extends CANSparkMax {

    public enum MotorKind {
        NEO550,
        NEO
    }

    public ImprovedCanSpark(int id, MotorKind motor, IdleMode mode, double volComp) {
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

    public ImprovedCanSpark(int id, MotorKind motor, IdleMode mode) {
        super(id, MotorType.kBrushless);
        restoreFactoryDefaults();
        clearFaults();
        setIdleMode(mode);
        switch(motor){
            case NEO -> setSmartCurrentLimit(80);
            case NEO550 -> setSmartCurrentLimit(20);
        }
        if (DriverStation.isFMSAttached()) {
            burnFlash();
        }
    }
}
