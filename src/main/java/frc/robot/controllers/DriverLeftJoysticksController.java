package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.oi.Controller;

public class DriverLeftJoysticksController implements Controller {
    private static final int port = 1;
    private final Joystick joystickControl;

    private static final double kXYDeadband = 0.05;
    private static final double kZDeadband = 0.3;

    public DriverLeftJoysticksController() {
        joystickControl = new Joystick(DriverLeftJoysticksController.port);
    }

    @Override
    public int getPort() {
        return DriverLeftJoysticksController.port;
    }

    @Override
    public GenericHID getGenericHID() {
        return joystickControl;
    }

    @Override
    public double getX() {
        return Controller.calculateSpeedWithDeadband(-joystickControl.getY(), kXYDeadband);
    }

    @Override
    public double getY() {
        return Controller.calculateSpeedWithDeadband(-joystickControl.getX(), kXYDeadband);
    }

    @Override
    public double getZ() {
        return Controller.calculateSpeedWithDeadband(-joystickControl.getTwist(), kZDeadband);
    }

    @Override
    public double getSpeedCoefficient() {
        double val = ((-joystickControl.getRawAxis(3) + 1) / 2);
        return Math.max(val, 0.2);
    }

    @Override
    public boolean isBoosterPressed() {
        return joystickControl.getTrigger();
    }

    @Override
    public int getAutonavigationButton() {
        return 11;
    }

    @Override
    public int getAutonavigateToAmpButton() {
        return 16;
    }

    @Override
    public int getAutonavigateToSourceButton() {
        return 12;
    }

    @Override
    public int getAutonavigateToSpeakerButton() {
        return 15;
    }

    @Override
    public int getNavXResetButton() {
        return 3;
    }

    @Override
    public int getOdometryResetButton() {
        return 5;
    }

    @Override
    public int getNextPipelineButton() {
        return -1;
    }

    @Override
    public int getFieldRelativeButton() {
        return 8;
    }

    @Override
    public boolean visionTargetAlignButtonIsPressed() {
        return joystickControl.getRawButtonPressed(2);
    }

    @Override
    public int getAutonavigateToClosestTarget() {
        return 9;
    }


    @Override
    public String getName() {
        return "Ultimate Method of Controlling the Robot";
    }
}
