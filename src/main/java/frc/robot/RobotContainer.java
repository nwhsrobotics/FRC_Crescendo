package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FavoritePositions;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.controllers.DriverJoysticksController;
import frc.robot.controllers.DriverLeftJoysticksController;
import frc.robot.controllers.DriverXboxController;
import frc.robot.controllers.GunnerXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.oi.ControlManager;

public class RobotContainer {
    //intialization of different subsystems and commands
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    //public final ArmSubsystem armSubsystem = new ArmSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        ControlManager.DriverButtonCommands.navXResetCommand = new InstantCommand(() -> swerveSubsystem.gyro.zeroYaw());
        ControlManager.DriverButtonCommands.toggleFieldRelativeCommand = new InstantCommand(() -> {swerveSubsystem.isFieldRelative = !swerveSubsystem.isFieldRelative;});
        ControlManager.DriverButtonCommands.toggleAutonavigationCommand = new InstantCommand(() -> swerveSubsystem.autonavigator.toggle());
        ControlManager.DriverButtonCommands.autonavigateToAmp = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.AMP));
        ControlManager.DriverButtonCommands.autonavigateToSource = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SOURCE));
        ControlManager.DriverButtonCommands.autonavigateToSpeaker = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SPEAKER));
        ControlManager.DriverButtonCommands.autonavigateToTopStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.TOPSTAGE));
        ControlManager.DriverButtonCommands.autonavigateToMidStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.MIDSTAGE));
        ControlManager.DriverButtonCommands.autonavigateToBottomStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.BOTTOMSTAGE));

        DriverXboxController driverXboxController = new DriverXboxController();
        ControlManager.registerController(driverXboxController);
        DriverJoysticksController driverJoysticksController = new DriverJoysticksController();
        ControlManager.registerController(driverJoysticksController);
        DriverLeftJoysticksController driverLeftJoysticksController = new DriverLeftJoysticksController();
        ControlManager.registerController(driverLeftJoysticksController);

        SendableChooser<Integer> driverControllerChooser = new SendableChooser<>();
        SendableChooser<Integer> gunnerControllerChooser = new SendableChooser<>();

        GunnerXboxController gunnerXboxController = new GunnerXboxController();
        ControlManager.registerGunnerController(gunnerXboxController);
        //ControlManager.GunnerButtonCommands.ampPresetCommand = new InstantCommand(() -> armSubsystem.ampPreset());
        for (String option : ControlManager.getControllerLabels(true)) {
            driverControllerChooser.addOption(option, ControlManager.getControllerPortFromLabel(option));
        }
        int defaultDriverControllerPort = ControlManager.getControllerLowest(true);
        if (defaultDriverControllerPort != -1) {
            driverControllerChooser.setDefaultOption(ControlManager.getControllerLabel(defaultDriverControllerPort), defaultDriverControllerPort);
        } else {
            driverControllerChooser.setDefaultOption("None", -1);
        }
        driverControllerChooser.onChange((port) -> {
            if (port != null) {
                ControlManager.setDriverPort(port);
            }
        });

        
        for (String option : ControlManager.getControllerLabels(false)) {
            gunnerControllerChooser.addOption(option, ControlManager.getControllerPortFromLabel(option));
        }
        int defaultGunnerControllerPort = ControlManager.getControllerLowest(false);
        if (defaultGunnerControllerPort != -1) {
            gunnerControllerChooser.setDefaultOption(ControlManager.getControllerLabel(defaultGunnerControllerPort), defaultGunnerControllerPort);
        } else {
            gunnerControllerChooser.setDefaultOption("None", -1);
        }
        gunnerControllerChooser.onChange((port) -> {
            if (port != null) {
                ControlManager.setGunnerPort(port);
            }
        });

        SmartDashboard.putData("Driver Controllers", driverControllerChooser);
        SmartDashboard.putData("Gunner Controllers", gunnerControllerChooser);

        autoChooser = AutoBuilder.buildAutoChooser("Auto Square");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        //CameraServer.startAutomaticCapture();

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        //returns what autonomous path is chosen in shuffleboard currently
        return autoChooser.getSelected();
    }
}
