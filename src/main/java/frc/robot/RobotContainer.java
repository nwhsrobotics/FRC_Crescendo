package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FavoritePositions;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.controllers.DriverXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.oi.ControlManager;

public class RobotContainer {
    //intialization of different subsystems and commands
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        ControlManager.DriverButtonCommands.navXResetCommand = new InstantCommand(() -> swerveSubsystem.gyro.zeroYaw());
        ControlManager.DriverButtonCommands.toggleFieldRelativeCommand = new InstantCommand(() -> {
            swerveSubsystem.isFieldRelative = !swerveSubsystem.isFieldRelative;
        });
        ControlManager.DriverButtonCommands.toggleAutonavigationCommand = new InstantCommand(() -> swerveSubsystem.autonavigator.toggle());
        ControlManager.DriverButtonCommands.autonavigateToAmp = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.AMP));
        ControlManager.DriverButtonCommands.autonavigateToSource = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SOURCE));
        ControlManager.DriverButtonCommands.autonavigateToStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.STAGE));

        DriverXboxController driverXboxController = new DriverXboxController();
        ControlManager.registerController(driverXboxController);

        ControlManager.buildChoosers();
        SmartDashboard.putData("Driver Controllers", ControlManager.driverControllerChooser);
        SmartDashboard.putData("Gunner Controllers", ControlManager.gunnerControllerChooser);

        autoChooser = AutoBuilder.buildAutoChooser("Auto Square");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        //returns what autonomous path is chosen in shuffleboard currently
        return autoChooser.getSelected();
        //return new PathPlannerAuto("Test Auto");
    }
}
