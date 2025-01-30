package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ImprovedCanSpark;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax leftClimbMotor = new ImprovedCanSpark(Constants.CANAssignments.CLIMB_LEFT_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, IdleMode.kBrake);
    private final SparkMax rightClimbMotor = new ImprovedCanSpark(Constants.CANAssignments.CLIMB_RIGHT_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, IdleMode.kBrake);

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
