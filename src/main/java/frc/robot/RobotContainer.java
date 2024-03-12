package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.FavoritePositions;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.SetScoringStateCommand;
import frc.robot.controllers.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ScoringSubsystem.ScoringState;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightImplementation;
import frc.robot.subsystems.oi.ControlManager;
import frc.robot.subsystems.oi.XboxControllerButtons;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    // public final ArmSubsystem armSubsystem = new ArmSubsystem();
    // public final WristSubsystem wristSubsystem = new WristSubsystem();
    // public final WristIntakeSubsystem wristIntakeSubsystem = new WristIntakeSubsystem();

    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Auto Square");  // FIXME DEFAULT AUTO SHOULD NOT BE TEST SEQUENCE.
 
    public static XboxController gunner = new XboxController(3);

    public Pose2d objectLocation = new Pose2d();  // FIXME INTEGRATE INTO AUTONAV, SHOULD NOT BE FLOATING INSIDE ROBOTCONTAINER.

    public RobotContainer() {
        SetScoringStateCommand commandShoot = new SetScoringStateCommand(scoringSubsystem, ScoringState.FIRE, 5);  // TODO tune durations.
        SetScoringStateCommand commandLoad = new SetScoringStateCommand(scoringSubsystem, ScoringState.LOADING, 5);
        SetScoringStateCommand commandUnload = new SetScoringStateCommand(scoringSubsystem, ScoringState.UNLOADING, 5);
        InstantCommand toggleAmp = new InstantCommand(() -> scoringSubsystem.toggleAmp(), scoringSubsystem);
        InstantCommand toggleSpeaker = new InstantCommand(() -> scoringSubsystem.toggleSpeaker(), scoringSubsystem);

        // expose scoring-related commands to autonomous routines.
        NamedCommands.registerCommand("shoot", commandShoot);  // TODO check auto command usage.
        NamedCommands.registerCommand("load", commandLoad);
        NamedCommands.registerCommand("speaker", toggleSpeaker);
        NamedCommands.registerCommand("amp", toggleAmp);

        /*
        // Command for setting arm to the amp position
        InstantCommand armLockAmp = new InstantCommand(() -> armSubsystem.ampPreset(), armSubsystem);

        // Command for setting arm to the source position
        InstantCommand armLockSource = new InstantCommand(() -> armSubsystem.sourcePreset(), armSubsystem);

        // Command for setting wrist to the amp position
        InstantCommand wristLockAmp = new InstantCommand(() -> wristSubsystem.ampPreset(), wristSubsystem);

        // Command for setting wrist to the source position
        InstantCommand wristLockSource = new InstantCommand(() -> wristSubsystem.sourcePreset(), wristSubsystem);

        // Command for letting the gunner freely adjust the arm position, tuning for the joystick control will be subject to change
        InstantCommand armAdjust = new InstantCommand(() -> armSubsystem.adjustAngle(gunner.getLeftY() * 0.65), armSubsystem);

        // Command for letting the gunner freely adjust the wrist position, tuning for the joystick control will be subject to change
        InstantCommand wristAdjust = new InstantCommand(() -> wristSubsystem.adjustAngle(gunner.getRightY() * 0.65), wristSubsystem);

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
        
        // bind gunner controls.
        new JoystickButton(gunner, XboxControllerButtons.RIGHT_BUMPER).onTrue(commandShoot);
        new JoystickButton(gunner, XboxControllerButtons.A).onTrue(commandLoad);
        new JoystickButton(gunner, XboxControllerButtons.B).onTrue(commandUnload);
        new POVButton(gunner, 90).onTrue(new InstantCommand(() -> LimelightImplementation.nextPipeline()));  // TODO confirm POVs.
        new POVButton(gunner, 270).onTrue(new InstantCommand(() -> LimelightImplementation.nextPipeline()));
        new POVButton(gunner, 0).onTrue(toggleAmp);
        new POVButton(gunner, 180).onTrue(toggleSpeaker);
        
        /*
        //Arm and Wrist
        gunner_V.whileTrue(toAmp);
        gunner_M.whileTrue(toSource);
        gunner_RS.whileTrue(armAdjust);
        gunner_LS.whileTrue(wristAdjust);
        */

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
        ControlManager.reserveController(3);  // THE GUNNER CONTROLLER SHOULD BE ON PORT 3.
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
            ControlManager.setDriverPort(defaultPort);
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
