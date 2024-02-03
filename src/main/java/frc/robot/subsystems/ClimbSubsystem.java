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

    private RelativeEncoder rightClimbEncoder;
    private RelativeEncoder leftClimbEncoder;

    private SparkPIDController rightClimbPID;
    private SparkPIDController leftClimbPID;

    private static final double MAX_UP_DOWN = 25.0 * 0.0254; //21 inches converted to meters
    private static final double MIN_UP_DOWN = 0.0 * 0.0254;
    private static final double INITIAL_UP_DOWN = 0.0;

    public static final double AUTO_CLIMB_RAISE = 0.1524;

    public static final double SPEED_UP_DOWNps = 0.22;
    public static final double TICKS_PER_SECOND = 50;
    private static final double GEAR_RATIO_UP_DOWN = -5.0;
    private static final double LEAD_DISTANCE = (0.5 * 0.0254); // (inches * m/in) half an inch to meters


    // TO DO LIST: FIX REAL SPEED(THE 1.0 VALUES!)
    private static final double UP_DOWN_COUNTS_PER_METER = (GEAR_RATIO_UP_DOWN / LEAD_DISTANCE); //TO DO LIST: FIGURE OUT REAL VALUE
    private static final double METERS_TO_INCHES = 39.37;
    private static final double HEIGHT_1 = 8.0 * 0.0254; //3 inches to meters : less than 3 inches sets to 0
    private static final double HEIGHT_2 = 16.0 * 0.0254;


    private double upDown = INITIAL_UP_DOWN;

    /**
     * Creates a new ClimbSubsystem.
     */
    public ClimbSubsystem() {
        rightClimbEncoder = rightClimbMotor.getEncoder();
        leftClimbEncoder = leftClimbMotor.getEncoder();

        rightClimbPID = rightClimbMotor.getPIDController();
        leftClimbPID = leftClimbMotor.getPIDController();

        rightClimbEncoder.setPosition(0);
        rightClimbMotor.setIdleMode(IdleMode.kBrake);
        rightClimbPID.setP(1.0);
        rightClimbPID.setI(0.0);
        rightClimbPID.setD(0.0);
        rightClimbPID.setIZone(0.0);
        rightClimbPID.setFF(0.0);
        rightClimbPID.setOutputRange(-1.0, 1.0);
        rightClimbPID.setReference(0.0, ControlType.kPosition);


        leftClimbEncoder.setPosition(0);
        leftClimbMotor.setIdleMode(IdleMode.kBrake);
        leftClimbPID.setP(1.0);
        leftClimbPID.setI(0.0);
        leftClimbPID.setD(0.0);
        leftClimbPID.setIZone(0.0);
        leftClimbPID.setFF(0.0);
        leftClimbPID.setOutputRange(-1.0, 1.0);
        leftClimbPID.setReference(0.0, ControlType.kPosition);

    }

    @Override
    public void periodic() {
        // Covert the meters to the count
        double upDown_counts = upDown * UP_DOWN_COUNTS_PER_METER;
        // System.out.printf("upDown_counts = %f\n", upDown_counts);

        leftClimbPID.setReference(upDown_counts, ControlType.kPosition);
        rightClimbPID.setReference(upDown_counts, ControlType.kPosition);
        SmartDashboard.putNumber("Climb Arm UP DOWN Pos", (upDown * METERS_TO_INCHES));
    }

    private void enforceLimits() {
    }

    public void moveUp() {
        upDown += SPEED_UP_DOWNps / TICKS_PER_SECOND;
        if (upDown > MAX_UP_DOWN) {
            upDown = MAX_UP_DOWN;
        }
        enforceLimits();
    }

    public void moveDown() {
        upDown -= SPEED_UP_DOWNps / TICKS_PER_SECOND;
        if (upDown < MIN_UP_DOWN) {
            upDown = MIN_UP_DOWN;
        }
        enforceLimits();
    }

    public double getHeight() {
        return upDown;
    }

    public void setHeight(double currentHeight) {
        upDown = currentHeight;
        enforceLimits();
    }

}
