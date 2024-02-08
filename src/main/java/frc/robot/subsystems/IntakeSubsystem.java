package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    public IntakeSubsystem() {
        this.motor = new CANSparkMax(Constants.CANAssignments.INTAKE_MOTOR_ID, MotorType.kBrushless);
    }

    /**
     * Run the intake forwards.
     */
    public void forwards() {
        motor.set(1);
    }

    /**
     * Run the intake reverse.
     */
    public void reverse() {
        motor.set(-1);
    }

    /**
     * Stop the intake motor.
     */
    public void deactivate() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
    }
}
