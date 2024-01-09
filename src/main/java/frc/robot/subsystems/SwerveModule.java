package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    public final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder; // built in NEO encoder (steering)

    public final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /**
     * Initializes a new instance of the SwerveModule class.
     * @param driveMotorId The ID of the CANSparkMax drive motor.
     * @param turningMotorId The ID of the CANSparkMax turning motor.
     * @param driveMotorReversed A boolean indicating whether the drive motor is reversed.
     * @param turningMotorReversed A boolean indicating whether the turning motor is reversed.
     * @param absoluteEncoderId The ID of the CANcoder absolute encoder.
     * @param absoluteEncoderOffset The offset of the absolute encoder in radians.
     * @param absoluteEncoderReversed A boolean indicating whether the absolute encoder is reversed.
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        // Set the absolute encoder offset and reversed value
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // Initialize the absolute encoder using the given id
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        // Initialize the drive and turning motors using the given ids
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        turningMotor.setIdleMode(IdleMode.kBrake);

        // Set the inversion of the drive and turning motors based on the given values
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // Initialize the drive and turning encoders
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // Set the position of the turning encoder based on the absolute encoder value
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        // Set the conversion factors for the drive and turning encoders
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        // Initialize the turning PID controller with the given values
        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, 0);
        // Set the tolerance for the PID controller (commented out in this code)
        // turningPidController.setTolerance(ModuleConstants.kPTolerance);

        // Enable continuous input for the PID controller to handle full rotation of the wheels
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        // Reset the encoders
        resetEncoders();
        // Set the turning motor to the calculated value from the PID controller
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), 0));

    }

    // Get the position of the drive encoder in meters
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    // Get the position of the turning encoder in radians
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    // Get the velocity of the drive encoder in meters per second
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    // Get the velocity of the turning encoder in radians per second
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    // Get the raw absolute encoder reading in radians
    public double getAbsoluteEncoderRadRaw() {
        double angle = absoluteEncoder.getAbsolutePosition() * (Math.PI / 180.0); // convert degrees to radians
        return angle; // shorthand for if the encoder is reversed, multiply by -1, else do nothing (multiply by 1)
    }

    // Get the absolute encoder reading in radians with correction for reversed encoders and offset
    public double getAbsoluteEncoderRad() {
        double angle = this.getAbsoluteEncoderRadRaw();
        angle -= absoluteEncoderOffsetRad;
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);

        // Normalize the angle to be between 0 and 2pi radians
        if (angle > 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    // Reset the turning encoder position to the previous value of the absolute encoder
    public void resetTurnEncoder() {
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    // Reset both drive and turning encoders to 0 and reset the turning encoder position
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        resetTurnEncoder();
    }

    // Get the current state of the swerve module (drive velocity and turning angle)
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition())); // SwerveModuleState takes the velocity and the angle of the module for params
    }

    // Get the current position of the swerve module (drive position and turning angle)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    // Set the desired state of the swerve module (drive velocity and turning angle)
    public void setDesiredState(SwerveModuleState state) {
        // If the desired speed is close to 0, stop the module
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize the desired state based on the current state angle
        state = SwerveModuleState.optimize(state, getState().angle);

        // Set the drive motor speed and the turning motor angle using PID control
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    // Stop the swerve module by setting both motors to 0
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}