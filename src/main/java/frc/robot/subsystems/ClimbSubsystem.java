package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax leftClimbMotor = new CANSparkMax(Constants.CANAssignments.CLIMB_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax rightClimbMotor = new CANSparkMax(Constants.CANAssignments.CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder rightClimbEncoder;
    private final RelativeEncoder leftClimbEncoder;

    private final SparkPIDController rightClimbPID;
    private final SparkPIDController leftClimbPID;

    private double desiredHeight = Constants.ClimbConstants.INITIAL_HEIGHT_METERS;

    public ClimbSubsystem() {
        rightClimbEncoder = rightClimbMotor.getEncoder();
        leftClimbEncoder = leftClimbMotor.getEncoder();

        rightClimbPID = rightClimbMotor.getPIDController();
        leftClimbPID = leftClimbMotor.getPIDController();

        rightClimbEncoder.setPosition(0);
        rightClimbMotor.setIdleMode(IdleMode.kBrake);
        rightClimbPID.setP(1.0);
        rightClimbPID.setOutputRange(-1.0, 1.0);
        rightClimbPID.setReference(0.0, ControlType.kPosition);

        leftClimbEncoder.setPosition(0);
        leftClimbMotor.setIdleMode(IdleMode.kBrake);
        leftClimbPID.setP(1.0);
        leftClimbPID.setOutputRange(-1.0, 1.0);
        leftClimbPID.setReference(0.0, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // Covert the meters to the count
        double counts = desiredHeight * Constants.ClimbConstants.COUNTS_PER_METER;

        leftClimbPID.setReference(counts, ControlType.kPosition);
        rightClimbPID.setReference(counts, ControlType.kPosition);
        Logger.recordOutput("Climb Position", desiredHeight * Constants.ClimbConstants.METERS_TO_INCHES);
    }

    public void moveUp() {
        desiredHeight += Constants.ClimbConstants.SPEED_PER_SECOND / Constants.ClimbConstants.TICKS_PER_SECOND;
        if (desiredHeight > Constants.ClimbConstants.MAX_HEIGHT_METERS) {
            desiredHeight = Constants.ClimbConstants.MAX_HEIGHT_METERS;
        }
    }

    public void moveDown() {
        desiredHeight -= Constants.ClimbConstants.SPEED_PER_SECOND / Constants.ClimbConstants.TICKS_PER_SECOND;
        if (desiredHeight < Constants.ClimbConstants.MIN_HEIGHT_METERS) {
            desiredHeight = Constants.ClimbConstants.MIN_HEIGHT_METERS;
        }
    }

    public double getHeight() {
        return desiredHeight;
    }

    public void setHeight(double newHeight) {
        desiredHeight = newHeight;
    }

}
