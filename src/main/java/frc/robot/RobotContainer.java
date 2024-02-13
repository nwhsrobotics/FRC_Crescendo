package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FavoritePositions;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.controllers.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.oi.ControlManager;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //public final ArmSubsystem armSubsystem = new ArmSubsystem();
    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Auto Square");

    public RobotContainer() {
        configureDriverCommands();
        configureDriverControllers();
        configureGunnerCommands();
        configureGunnerControllers();
        configureControllerChooser(true, "Driver Controllers");
        configureControllerChooser(false, "Gunner Controllers");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem));
    }

    private void configureDriverCommands() {
        ControlManager.DriverButtonCommands.navXResetCommand = new InstantCommand(() -> swerveSubsystem.gyro.zeroYaw());
        ControlManager.DriverButtonCommands.toggleFieldRelativeCommand = new InstantCommand(() -> {swerveSubsystem.isFieldRelative = !swerveSubsystem.isFieldRelative;});
        ControlManager.DriverButtonCommands.toggleAutonavigationCommand = new InstantCommand(() -> swerveSubsystem.autonavigator.toggle());
        ControlManager.DriverButtonCommands.autonavigateToAmp = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.AMP));
        ControlManager.DriverButtonCommands.autonavigateToSource = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SOURCE));
        ControlManager.DriverButtonCommands.autonavigateToSpeaker = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SPEAKER));
        ControlManager.DriverButtonCommands.autonavigateToTopStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.TOPSTAGE));
        ControlManager.DriverButtonCommands.autonavigateToMidStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.MIDSTAGE));
        ControlManager.DriverButtonCommands.autonavigateToBottomStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.BOTTOMSTAGE));
    }

    private void configureGunnerCommands(){
        //ControlManager.GunnerButtonCommands.ampPresetCommand = new InstantCommand(() -> armSubsystem.ampPreset());
    }

    private void configureDriverControllers() {
        ControlManager.registerController(new DriverXboxController());
        ControlManager.registerController(new DriverJoysticksController());
        ControlManager.registerController(new DriverLeftJoysticksController());
    }

    private void configureGunnerControllers() {
        ControlManager.registerGunnerController(new GunnerXboxController());
    }

    private void configureControllerChooser(boolean isDriver, String name) {
        SendableChooser<Integer> controllerChooser = new SendableChooser<>();
        for (String option : ControlManager.getControllerLabels(isDriver)) {
            controllerChooser.addOption(option, ControlManager.getControllerPortFromLabel(option));
        }
        int defaultPort = ControlManager.getControllerLowest(isDriver);
        if (defaultPort != -1) {
            controllerChooser.setDefaultOption(ControlManager.getControllerLabel(defaultPort), defaultPort);
        } else {
            controllerChooser.setDefaultOption("None", -1);
        }
        controllerChooser.onChange((port) -> {
            if (port != null) {
                if (isDriver) ControlManager.setDriverPort(port);
                else ControlManager.setGunnerPort(port);
            }
        });
        SmartDashboard.putData(name, controllerChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
