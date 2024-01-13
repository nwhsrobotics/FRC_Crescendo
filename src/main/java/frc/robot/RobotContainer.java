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
import frc.robot.commands.AutoBaseCmd;
import frc.robot.commands.AutoEngageCmd;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveClover;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    //intialization of different subsystems and commands
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public final Joystick m_driver = new Joystick(2);

    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem);

    //autonomous paths
    Command blue1_auto = new AutoBaseCmd(swerveSubsystem,
            "paths/Blue1_CubeExit.wpilib.json", "paths/Blue1B.wpilib.json");
    Command blue2_auto = new SequentialCommandGroup(
            new AutoBaseCmd(swerveSubsystem,
                    "paths/Blue2A.wpilib.json", "paths/Blue2B.wpilib.json"),
            new AutoEngageCmd(swerveSubsystem));
    Command blue3_auto = new AutoBaseCmd(swerveSubsystem,
            "paths/Blue3_CubeExit.wpilib.json", "paths/Blue3B.wpilib.json");
    Command red1_auto = new AutoBaseCmd(swerveSubsystem,
            "paths/Red1_CubeExit.wpilib.json", "paths/Red1B.wpilib.json");
    Command red2_auto = new AutoBaseCmd(swerveSubsystem,
            "paths/Red2A.wpilib.json", "paths/Red2B.wpilib.json");
    Command red3_auto = new AutoBaseCmd(swerveSubsystem,
            "paths/Red3_CubeExit.wpilib.json", "paths/Red3B.wpilib.json");
    /*
     * Command bluecharge = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem,
     * m_extendArmSubsystem, m_grabberSubsystem,
     * "paths/BlueChargeStation.wpilib.json", "paths/Red3B.wpilib.json");
     */
    Command bluecharge = new SequentialCommandGroup(
            new AutoBaseCmd(swerveSubsystem,
                    "paths/BlueChargeStation.wpilib.json", "paths/Blue2B.wpilib.json"),
            new AutoEngageCmd(swerveSubsystem));
    /*
     * Command redcharge = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem,
     * m_extendArmSubsystem, m_grabberSubsystem,
     * "paths/RedChargeStation.wpilib.json", "paths/Red3B.wpilib.json");
     */

     //presets for arm
    Command redcharge = new SequentialCommandGroup(
            new AutoBaseCmd(swerveSubsystem,
                    "paths/RedChargeStation.wpilib.json", "paths/Blue2B.wpilib.json"),
            new AutoEngageCmd(swerveSubsystem));

    public RobotContainer() {
        //choose autonomous paths in shuffleboard
      autoChooser = AutoBuilder.buildAutoChooser("Auto Square");
      SmartDashboard.putData("Auto Chooser", autoChooser);
      swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //assign the button bindings to different commands
        new JoystickButton(m_driver, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_driver, 2).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR()));
        new JoystickButton(m_driver, 4).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_driver, 11).onTrue(new SwerveClover(swerveSubsystem));
        new JoystickButton(m_driver, 5).onTrue(new SwerveClover(swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        //returns what autonomous path is chosen in shuffleboard currently
        //return autoChooser.getSelected();
        return new PathPlannerAuto("Auto Square");

    }
}
