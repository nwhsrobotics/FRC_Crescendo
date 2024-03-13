package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    public final CANSparkMax leftClimbMotor = new CANSparkMax(Constants.CANAssignments.CLIMB_LEFT_MOTOR_ID, MotorType.kBrushless);
    public final CANSparkMax rightClimbMotor = new CANSparkMax(Constants.CANAssignments.CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushless);

    public ClimbSubsystem() {
        leftClimbMotor.setInverted(true);
    }

    @Override
    public void periodic() {
    }
}
