package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public final Joystick m_driver = new Joystick(1);
    public final XboxController m_operator = new XboxController(0);

    public final PathFindFollowCmd pathFindFollowCmd = new PathFindFollowCmd(swerveSubsystem, m_operator);

    //controller buttons intialized

    public final JoystickButton m_joyA = new JoystickButton(m_operator, 1); // button A
    public final JoystickButton m_joyB = new JoystickButton(m_operator, 2); // button B
    public final JoystickButton m_joyX = new JoystickButton(m_operator, 3); // button X
    public final JoystickButton m_joyY = new JoystickButton(m_operator, 4); // button Y
    public final JoystickButton m_joyLB = new JoystickButton(m_operator, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(m_operator, 6); // Right bumper
    public final JoystickButton m_joyBK = new JoystickButton(m_operator, 7); // Back Button
    public final JoystickButton m_joyST = new JoystickButton(m_operator, 8); // Start Button

    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem);

    //autonomous paths
    Command blue1_auto = new AutoBaseCmd(swerveSubsystem, "paths/Blue1_CubeExit.wpilib.json", "paths/Blue1B.wpilib.json");
    Command blue2_auto = new AutoBaseCmd(swerveSubsystem, "paths/Blue2A.wpilib.json", "paths/Blue2B.wpilib.json");
    Command blue3_auto = new AutoBaseCmd(swerveSubsystem, "paths/Blue3_CubeExit.wpilib.json", "paths/Blue3B.wpilib.json");
    Command red1_auto = new AutoBaseCmd(swerveSubsystem, "paths/Red1_CubeExit.wpilib.json", "paths/Red1B.wpilib.json");
    Command red2_auto = new AutoBaseCmd(swerveSubsystem, "paths/Red2A.wpilib.json", "paths/Red2B.wpilib.json");
    Command red3_auto = new AutoBaseCmd(swerveSubsystem, "paths/Red3_CubeExit.wpilib.json", "paths/Red3B.wpilib.json");



  public RobotContainer() {
    // Configure the trigger bindings
    //m_autoChooser.setDefaultOption("Blue1", blue1_auto);
    //m_autoChooser.addOption("blue2", blue2_auto);
    //SmartDashboard.putData(m_autoChooser);
      //TODO: Named commands have to be registed be auto is registered intialize swerve subsystem below here
    NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));
    pathFindFollowCmd.schedule();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        //assign the button bindings to different commands
    new JoystickButton(m_driver, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
    new JoystickButton(m_driver, 2).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR()));
    new JoystickButton(m_driver, 4).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
    // new JoystickButton(m_driver, 11).onTrue(new InstantCommand(() -> swerveSubsystem.brake()));
    // new JoystickButton(m_driver, 5).onTrue(new InstantCommand(() -> swerveSubsystem.brake()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        //returns what autonomous path is chosen in shuffleboard currently
        return autoChooser.getSelected();
    }
}
