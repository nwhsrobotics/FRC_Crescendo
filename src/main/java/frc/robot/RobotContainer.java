package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.FavoritePositions;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.controllers.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightImplementation;
import frc.robot.subsystems.oi.ControlManager;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    // public final ArmSubsystem armSubsystem = new ArmSubsystem();
    // public final WristSubsystem wristSubsystem = new WristSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Auto Square");
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    // public final WristIntakeSubsystem wristIntakeSubsystem = new WristIntakeSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    public static XboxController gunner = new XboxController(3);

    public final JoystickButton gunner_A = new JoystickButton(gunner, 1); // Button A
    public final JoystickButton gunner_B = new JoystickButton(gunner, 2); // Button B
    public final JoystickButton gunner_X = new JoystickButton(gunner, 3); // Button X
    public final JoystickButton gunner_Y = new JoystickButton(gunner,4); // Button Y
    public final JoystickButton gunner_LB = new JoystickButton(gunner, 5); // Left bumper
    public final JoystickButton gunner_RB = new JoystickButton(gunner, 6); // Right bumper
    public final JoystickButton gunner_V = new JoystickButton(gunner, 7); // View button                    
    public final JoystickButton gunner_M = new JoystickButton(gunner, 8); // Menu button
    public final JoystickButton gunner_LS = new JoystickButton(gunner, 9); // Left Stick Button
    public final JoystickButton gunner_RS = new JoystickButton(gunner, 10); // Right Stick Button

    public final POVButton gunner_pov0 = new POVButton(gunner, 0);
    public final POVButton gunner_pov90 = new POVButton(gunner, 90);
    public final POVButton gunner_pov180 = new POVButton(gunner, 180);
    public final POVButton gunner_pov270 = new POVButton(gunner, 270);
    public Pose2d objectLocation = new Pose2d();

    public RobotContainer() {
        /*
        // Command for setting arm to the amp position
        InstantCommand armLockAmp = new InstantCommand(() -> armSubsystem.ampPreset(), armSubsystem);

        // Command for setting arm to the source position
        InstantCommand armLockSource = new InstantCommand(() -> armSubsystem.sourcePreset(), armSubsystem);

        // Command for setting wrist to the amp position
        InstantCommand wristLockAmp = new InstantCommand(() -> wristSubsystem.ampPreset(), wristSubsystem);

        // Command for setting wrist to the source position
        InstantCommand wristLockSource = new InstantCommand(() -> wristSubsystem.sourcePreset(), wristSubsystem);

        // Command for letting the gunner freely adjust the arm position
        InstantCommand armAdjust = new InstantCommand(() -> armSubsystem.adjustAngle(gunner.getLeftY()), armSubsystem);

        // Command for letting the gunner freely adjust the wrist position
        InstantCommand wristAdjust = new InstantCommand(() -> wristSubsystem.adjustAngle(gunner.getRightY()), wristSubsystem);

        // Command for letting you adjust the wrist and arm together
        ParallelCommandGroup adjust = new ParallelCommandGroup(armAdjust, wristAdjust);
        adjust.addRequirements(wristSubsystem, armSubsystem);
        // Command group that has built-in logic 
        SequentialCommandGroup toAmp = new SequentialCommandGroup(armLockAmp, wristLockAmp);
        toAmp.addRequirements(wristSubsystem, armSubsystem);
        SequentialCommandGroup toSource = new SequentialCommandGroup(armLockSource, wristLockSource);
        toSource.addRequirements(wristSubsystem, armSubsystem);
        SequentialCommandGroup semiWristAdjustAmp = new SequentialCommandGroup(armLockAmp, wristAdjust);
        semiWristAdjustAmp.addRequirements(wristSubsystem, armSubsystem);
        SequentialCommandGroup semiWristAdjustSource = new SequentialCommandGroup(armLockSource, wristAdjust);
        semiWristAdjustSource.addRequirements(wristSubsystem, armSubsystem);
        */

        
        
        
        

        // Creates instant commands for the different robot functionalities
        InstantCommand shoot = new InstantCommand(() -> shooterSubsystem.stepIndex(), shooterSubsystem);
        InstantCommand toggleAmp = new InstantCommand(() -> shooterSubsystem.toggleAmp(), shooterSubsystem);
        InstantCommand toggleSpeaker = new InstantCommand(() -> shooterSubsystem.toggleSpeaker(), shooterSubsystem);
        InstantCommand intakeOn = new InstantCommand(() -> intakeSubsystem.forwards(), intakeSubsystem);
        InstantCommand intakeOf = new InstantCommand(() -> intakeSubsystem.stop(), intakeSubsystem);

        // Registers the instant commands
         NamedCommands.registerCommand("shoot", shoot);
         NamedCommands.registerCommand("intakeOn", intakeOn);
         NamedCommands.registerCommand("intakeOf", intakeOf);
         NamedCommands.registerCommand("toggleSpeaker", toggleSpeaker);
         NamedCommands.registerCommand("toggleAmp", toggleAmp);
        

        // Binds commands to buttons
        gunner_RB.onTrue(shoot);
        gunner_pov90.onTrue(new InstantCommand(() -> LimelightImplementation.nextPipeline()));
        gunner_pov180.onTrue(toggleSpeaker);
        gunner_pov0.onTrue(toggleAmp);
        //gunner_LB.onTrue(toggleFlywheel);
        /*
        gunner_V.whileTrue(toAmp);
        gunner_M.whileTrue(toSource);
        gunner_RS.whileTrue(adjust);
        gunner_LS.whileTrue(semiWristAdjustAmp);
        */
        gunner_pov270.whileTrue(new InstantCommand(() -> LimelightImplementation.nextPipeline()));

        // initialize driver button commands.
        ControlManager.DriverButtonCommands.navXResetCommand = new InstantCommand(() -> swerveSubsystem.gyro.zeroYaw(), swerveSubsystem);
        ControlManager.DriverButtonCommands.toggleFieldRelativeCommand = new InstantCommand(() -> swerveSubsystem.isFieldRelative = !swerveSubsystem.isFieldRelative, swerveSubsystem);
        ControlManager.DriverButtonCommands.toggleAutonavigationCommand = new InstantCommand(() -> swerveSubsystem.autonavigator.toggle(), swerveSubsystem);
        ControlManager.DriverButtonCommands.autonavigateToAmp = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.AMP), swerveSubsystem);
        ControlManager.DriverButtonCommands.autonavigateToSource = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SOURCE), swerveSubsystem);
        ControlManager.DriverButtonCommands.autonavigateToSpeaker = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SPEAKER), swerveSubsystem);
        ControlManager.DriverButtonCommands.autonavigateToTopStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.TOPSTAGE), swerveSubsystem);
        ControlManager.DriverButtonCommands.autonavigateToMidStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.MIDSTAGE), swerveSubsystem);
        ControlManager.DriverButtonCommands.autonavigateToBottomStage = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.BOTTOMSTAGE), swerveSubsystem);
        ControlManager.DriverButtonCommands.autonavigateToVisionTarget = new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(objectLocation), swerveSubsystem);
        ControlManager.DriverButtonCommands.odometryVisionReset = new InstantCommand(() -> swerveSubsystem.resetOdometryWithVision(), swerveSubsystem);
        ControlManager.DriverButtonCommands.nextPipeline = new InstantCommand(() -> LimelightImplementation.nextPipeline());

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
        climbSubsystem.setDefaultCommand(new ClimbCmd(climbSubsystem, gunner));
        intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem, gunner));
        
        // FIXME: should be wrist intake, not intake.
        // intakeSubsystem.setDefaultCommand(new WristIntakeCmd(wristIntakeSubsystem, gunner.getXButton(), gunner.getYButton()));


        
        /*SendableChooser<Integer> pipeline = new SendableChooser<>();
        pipeline.setDefaultOption("AprilTag", Integer.valueOf(0));
        pipeline.addOption("Note", Integer.valueOf(1));
        pipeline.addOption("AprilTagZoom", Integer.valueOf(2));
        pipeline.addOption("NoteZoom", Integer.valueOf(3));
        SmartDashboard.putData(pipeline);
        pipeline.onChange((pipelineNum) -> {
            LimelightHelpers.setPipelineIndex("limelight", pipelineNum);
            Logger.recordOutput("limelight.pipelineIndex", pipelineNum);
            Logger.recordOutput("limelight.pipelineName", LimelightImplementation.getPipelineName());
        });*/

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
