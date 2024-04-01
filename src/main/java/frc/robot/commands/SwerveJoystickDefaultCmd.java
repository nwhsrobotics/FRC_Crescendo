package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SwerveJoystickDefaultCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem vision;
    private final XboxController xbox;
    private boolean fieldRelative;

    // constructor that initializes SwerveSubsystem, Joystick and adds SwerveSubsystem as a requirement
    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, XboxController xbox, VisionSubsystem vision) {
        this.vision = vision;
        this.xbox = xbox;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem, vision);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //TODO: If we get 2 limelights, separate the object alligning and april tag alligning button
        if (xbox.getLeftBumper() || xbox.getRightBumper()) {  //vision allign button
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
            swerveSubsystem.drive(
                    vision.limelight_range_proportional(),
                    0,
                    vision.limelight_aim_proportional(),
                    swerveSubsystem.isFieldRelative && fieldRelative, false);

        } else if (!(xbox.getRightTriggerAxis() > 0.1)) {  //if booster not pressed
            fieldRelative = true;
            swerveSubsystem.drive(
                    //TODO: its actually kinda funny we do things manually but dont use MathUtil.clamp, MathUtil.deadband, etc. implement MathUtil. instead of doing that please
                    -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative && fieldRelative, true);
        } else {
            fieldRelative = true;
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative && fieldRelative, false);
        }
        //TODO: Invert if red, multiply values by -1 because pathplanner origin is always at blue

                    /*        swerveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> swerveSubsystem.drive(
                -MathUtil.applyDeadband(Math.copySign(Math.pow(Driver1.getRawAxis(1), 2), Driver1.getRawAxis(1)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.copySign(Math.pow(Driver1.getRawAxis(0), 2), Driver1.getRawAxis(0)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Driver1.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
                swerveSubsystem)); */
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}