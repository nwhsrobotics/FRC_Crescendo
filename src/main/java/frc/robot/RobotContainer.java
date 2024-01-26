package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.FavoritePositions;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    //intialization of different subsystems and commands
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = new SendableChooser<>();
    //XboxController controller = new XboxController(0);
    public final Joystick m_driver = new Joystick(0);

    public RobotContainer() {
        //TODO: THIS COMMAND is there just for testing (preset pos is disabled for Auto Square path) to test how we can build auto in pathplanner itself
        //NamedCommands.registerCommand("resetOdometry", new InstantCommand(() -> swerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto Square"))));
        NamedCommands.registerCommand("wait5Sec", new WaitCommand(5.0));
        //choose autonomous paths in shuffleboard
        autoChooser = AutoBuilder.buildAutoChooser("Auto Square");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_driver, 1).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.toggle()));
        new JoystickButton(m_driver, 12).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.AMP)));
        new JoystickButton(m_driver, 11).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SOURCE)));
        new JoystickButton(m_driver, 10).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.STAGE)));
    }

    public Command getAutonomousCommand() {
        //returns what autonomous path is chosen in shuffleboard currently
        return autoChooser.getSelected();
        //return new PathPlannerAuto("Test Auto");

    }
}
