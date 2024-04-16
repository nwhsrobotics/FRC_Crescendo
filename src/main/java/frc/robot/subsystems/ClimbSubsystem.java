package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CanSpark;

public class ClimbSubsystem extends SubsystemBase {
    public final CANSparkMax leftClimbMotor = new CanSpark(Constants.CANAssignments.CLIMB_LEFT_MOTOR_ID, CanSpark.MotorKind.NEO550, CANSparkBase.IdleMode.kBrake);
    public final CANSparkMax rightClimbMotor = new CanSpark(Constants.CANAssignments.CLIMB_RIGHT_MOTOR_ID, CanSpark.MotorKind.NEO550, CANSparkBase.IdleMode.kBrake);

    public ClimbSubsystem() {
        leftClimbMotor.setInverted(true);
    }

    @Override
    public void periodic() {
    }
}
