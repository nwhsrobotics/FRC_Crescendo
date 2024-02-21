package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
    public static XboxController gunner = new XboxController(0);
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    
    //public final ArmSubsystem armSubsystem = new ArmSubsystem();
    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Auto Square");

    public RobotContainer() {
        

        InstantCommand armLockAmp = new InstantCommand(() -> {armSubsystem.ampPreset();});
        InstantCommand armLockSource = new InstantCommand(() -> {armSubsystem.sourcePreset();});
        InstantCommand wristLockAmp = new InstantCommand(() -> {wristSubsystem.ampPreset();});
        InstantCommand wristLockSource = new InstantCommand(() -> {wristSubsystem.sourcePreset();});
        InstantCommand armAdjust = new InstantCommand(() -> {armSubsystem.adjustAngle(gunner.getLeftY());});
        InstantCommand wristAdjust = new InstantCommand(() -> {wristSubsystem.adjustAngle(gunner.getRightY());});

        ParallelCommandGroup parallel = new ParallelCommandGroup(armAdjust, wristAdjust);

        SequentialCommandGroup seq = new SequentialCommandGroup();
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
