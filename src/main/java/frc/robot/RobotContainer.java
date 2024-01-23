package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveClover;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    //intialization of different subsystems and commands
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public final Joystick m_driver = new Joystick(0);

    public RobotContainer() {
        //choose autonomous paths in shuffleboard
      autoChooser = AutoBuilder.buildAutoChooser("Auto Square");
      SmartDashboard.putData("Auto Chooser", autoChooser);

      // swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));
      
      configureButtonBindings();
    }

    private void configureButtonBindings() {
        //assign the button bindings to different commands
    }

    public Command getAutonomousCommand() {
        //returns what autonomous path is chosen in shuffleboard currently
        return autoChooser.getSelected();
        //return new PathPlannerAuto("Test Auto");

    }
}
