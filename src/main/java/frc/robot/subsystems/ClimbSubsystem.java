package frc.robot.subsystems;

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

    private static final double MAX_HEIGHT_METERS = 0.0 * 0.0254; // TODO: get actual max
    private static final double MIN_HEIGHT_METERS = 0.0 * 0.0254; // TODO: get actual min
    private static final double INITIAL_HEIGHT_METERS = 0.0;

    public static final double AUTO_CLIMB_RAISE = 0.1524;

    public static final double SPEED_PER_SECOND = 0.22;
    public static final double TICKS_PER_SECOND = 50;
    private static final double GEAR_RATIO = -5.0; // TODO: Get actual gear ratio
    private static final double LEAD_DISTANCE = (0.5 * 0.0254); // (inches * m/in) half an inch to meters

    private static final double COUNTS_PER_METER = (GEAR_RATIO / LEAD_DISTANCE);
    private static final double METERS_TO_INCHES = 39.37;

    private double desiredHeight = INITIAL_HEIGHT_METERS;

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
        double counts = desiredHeight * COUNTS_PER_METER;

        leftClimbPID.setReference(counts, ControlType.kPosition);
        rightClimbPID.setReference(counts, ControlType.kPosition);
        SmartDashboard.putNumber("Climb Position", (desiredHeight * METERS_TO_INCHES));
    }

    public void moveUp() {
        desiredHeight += SPEED_PER_SECOND / TICKS_PER_SECOND;
        if (desiredHeight > MAX_HEIGHT_METERS) {
            desiredHeight = MAX_HEIGHT_METERS;
        }
    }

    public void moveDown() {
        desiredHeight -= SPEED_PER_SECOND / TICKS_PER_SECOND;
        if (desiredHeight < MIN_HEIGHT_METERS) {
            desiredHeight = MIN_HEIGHT_METERS;
        }
    }

    public double getHeight() {
        return desiredHeight;
    }

    public void setHeight(double newHeight) {
        desiredHeight = newHeight;
    }

}
