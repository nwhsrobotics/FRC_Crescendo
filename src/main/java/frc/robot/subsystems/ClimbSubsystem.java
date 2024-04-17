package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ImprovedCanSpark;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax leftClimbMotor = new ImprovedCanSpark(Constants.CANAssignments.CLIMB_LEFT_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, CANSparkBase.IdleMode.kBrake);
    private final CANSparkMax rightClimbMotor = new ImprovedCanSpark(Constants.CANAssignments.CLIMB_RIGHT_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, CANSparkBase.IdleMode.kBrake);

    public ClimbSubsystem() {
        leftClimbMotor.setInverted(true);
    }

    public void setLeftSpeed(double speed){
        leftClimbMotor.set(speed);
    }

    public void setRightSpeed(double speed){
        rightClimbMotor.set(speed);
    }

    @Override
    public void periodic() {
    }
}
