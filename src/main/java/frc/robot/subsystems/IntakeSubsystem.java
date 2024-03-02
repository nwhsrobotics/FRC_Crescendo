package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    public IntakeSubsystem() {
        motor = new CANSparkMax(Constants.CANAssignments.INTAKE_MOTOR_ID, MotorType.kBrushless);
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

    }
}
