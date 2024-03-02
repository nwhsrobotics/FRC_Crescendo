package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FavoritePositions;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.controllers.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.oi.ControlManager;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public static XboxController gunner = new XboxController(10);
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Auto Square");
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final WristIntakeSubsystem wristInstakeSubsystem = new WristIntakeSubsystem();

    public RobotContainer() {
        


        
       
       
       
       //command for setting arm to the amp position
        InstantCommand armLockAmp = new InstantCommand(() -> {armSubsystem.ampPreset();});
        armLockAmp.addRequirements(armSubsystem);
        //command for setting arm to the source position
        InstantCommand armLockSource = new InstantCommand(() -> {armSubsystem.sourcePreset();});
        armLockSource.addRequirements(armSubsystem);
        //command for setting wrist to the amp position
        InstantCommand wristLockAmp = new InstantCommand(() -> {wristSubsystem.ampPreset();});
        wristLockAmp.addRequirements(wristSubsystem);
        //command for setting wrist to the source position
        InstantCommand wristLockSource = new InstantCommand(() -> {wristSubsystem.sourcePreset();});
        wristLockSource.addRequirements(wristSubsystem);
        //command for letting the gunner freely adjust the arm position
        InstantCommand armAdjust = new InstantCommand(() -> {armSubsystem.adjustAngle(gunner.getLeftY());});
        armAdjust.addRequirements(armSubsystem);
        //command for letting the gunner freely adjust the wrist position
        InstantCommand wristAdjust = new InstantCommand(() -> {wristSubsystem.adjustAngle(gunner.getRightY());});
        wristAdjust.addRequirements(wristSubsystem);
        //command for letting you adjust the wrist and arm together
        ParallelCommandGroup parallel = new ParallelCommandGroup(armAdjust, wristAdjust);
        parallel.addRequirements(wristSubsystem, armSubsystem);

        //command group that had built in logic 
        SequentialCommandGroup seq = new SequentialCommandGroup();
        seq.addRequirements(wristSubsystem, armSubsystem);
            if(gunner.getAButton()){

                seq.addCommands(
                  armLockAmp,
                  wristLockAmp
              );
              }
          
            if(gunner.getBButton()){
                seq.addCommands(
                  armLockSource,
                  wristLockSource
              );
              }

            if(!gunner.getAButton() && !gunner.getBButton()){
                seq.addCommands(
                  parallel
                );
              }


        armSubsystem.setDefaultCommand(seq);
        wristSubsystem.setDefaultCommand(seq);
        
        




       InstantCommand intakeOn = new InstantCommand(() -> {intakeSubsystem.forwards();});
       InstantCommand intakeOff = new InstantCommand(() -> {intakeSubsystem.deactivate();});

       InstantCommand wristIntakeOn = new InstantCommand(() -> {wristInstakeSubsystem.forwards();});
       InstantCommand wristIntakeOff = new InstantCommand(() -> {wristInstakeSubsystem.stop();});

       NamedCommands.registerCommand("intakeOn", intakeOn);
       NamedCommands.registerCommand("intakeOff", intakeOff);

       NamedCommands.registerCommand("wristIntakeOn", wristIntakeOn);
       NamedCommands.registerCommand("wristIntakeOff", wristIntakeOff);

        
        // initialize driver button commands.
        ControlManager.DriverButtonCommands.navXResetCommand = new InstantCommand(() -> swerveSubsystem.gyro.zeroYaw());
        ControlManager.DriverButtonCommands.toggleFieldRelativeCommand = new InstantCommand(() -> {swerveSubsystem.isFieldRelative = !swerveSubsystem.isFieldRelative;});
        ControlManager.DriverButtonCommands.toggleAutonavigationCommand = new InstantCommand(() -> swerveSubsystem.autonavigator.toggle());
        ControlManager.DriverButtonCommands.autonavigateToAmp = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.AMP));
        ControlManager.DriverButtonCommands.autonavigateToSource = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SOURCE));
        ControlManager.DriverButtonCommands.autonavigateToSpeaker = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SPEAKER));
        ControlManager.DriverButtonCommands.autonavigateToTopStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.TOPSTAGE));
        ControlManager.DriverButtonCommands.autonavigateToMidStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.MIDSTAGE));
        ControlManager.DriverButtonCommands.autonavigateToBottomStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.BOTTOMSTAGE));
        
        // reserve gunner port.
        ControlManager.reserveController(3);  // THE GUNNER CONTROLLER SHOULD BE ON PORT 3.

        // register all driver controllers.
        ControlManager.registerController(new DriverXboxController());
        ControlManager.registerController(new DriverJoysticksController());
        ControlManager.registerController(new DriverLeftJoysticksController());

        // setup selector for controller.
        SendableChooser<Integer> controllerChooser = new SendableChooser<>();
        for (String option : ControlManager.getControllerLabels()) {
            controllerChooser.addOption(option, ControlManager.getControllerPortFromLabel(option));
        }
        int defaultPort = ControlManager.getControllerLowest();
        if (defaultPort != -1) {
            controllerChooser.setDefaultOption(ControlManager.getControllerLabel(defaultPort), defaultPort);
        } else {
            controllerChooser.setDefaultOption("None", -1);
        }
        controllerChooser.onChange((port) -> {
            if (port != null) {
                ControlManager.setDriverPort(port);
            }
        });
        SmartDashboard.putData("Driver Controllers", controllerChooser);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
