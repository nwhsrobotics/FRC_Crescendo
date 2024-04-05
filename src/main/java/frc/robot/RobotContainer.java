package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.FavoritePositions;
import frc.robot.Constants.LimelightConstants;
import frc.robot.autos.Auto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.XboxControllerButtons;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    public final WristIntakeSubsystem wristIntakeSubsystem = new WristIntakeSubsystem();

    SendableChooser<Command> autoChooser;

    public XboxController driver = new XboxController(0);
    public static XboxController gunner = new XboxController(1);

    public RobotContainer() {
        SetScoringStateCommand commandShoot = new SetScoringStateCommand(scoringSubsystem, ScoringSubsystem.ScoringState.FIRE, 3);
        SetScoringStateCommand commandLoad = new SetScoringStateCommand(scoringSubsystem, ScoringSubsystem.ScoringState.LOADING, 2);
        SetScoringStateCommand commandUnload = new SetScoringStateCommand(scoringSubsystem, ScoringSubsystem.ScoringState.UNLOADING, 2);
        SetScoringStateCommand autoShoot = new SetScoringStateCommand(scoringSubsystem, ScoringSubsystem.ScoringState.FIRE, ScoringSubsystem.ScoringState.LOADING, 2);

        ParallelCommandGroup autoInit = new ParallelCommandGroup(); // new ParallelCommandGroup((new InstantCommand(() -> wristSubsystem.ampPreset(), wristSubsystem), (new InstantCommand(() -> armSubsystem.underStage(), armSubsystem));

        NamedCommands.registerCommand("shoot", autoShoot);
        NamedCommands.registerCommand("autoInit", autoInit);

        //INIT after registering named commands
        autoChooser = AutoBuilder.buildAutoChooser("[B]");

        InstantCommand armLockAmp = new InstantCommand(armSubsystem::ampPreset, armSubsystem);
        InstantCommand armLockSource = new InstantCommand(armSubsystem::sourcePreset, armSubsystem);
        InstantCommand armLockUnderStage = new InstantCommand(armSubsystem::underStage, armSubsystem);
        InstantCommand wristLockAmp = new InstantCommand(wristSubsystem::ampPreset, wristSubsystem);
        InstantCommand wristLockSource = new InstantCommand(wristSubsystem::sourcePreset, wristSubsystem);
        InstantCommand wristLockUnderStage = new InstantCommand(wristSubsystem::underStage, wristSubsystem);

        InstantCommand resetArmEncoders = new InstantCommand(armSubsystem::resetArmEncoders, armSubsystem);
        ParallelCommandGroup toAmp = new ParallelCommandGroup(armLockAmp, wristLockAmp);
        ParallelCommandGroup toSource = new ParallelCommandGroup(armLockSource, wristLockSource);
        ParallelCommandGroup toUnderStage = new ParallelCommandGroup(armLockUnderStage, wristLockUnderStage);

        new JoystickButton(gunner, XboxControllerButtons.RIGHT_BUMPER).onTrue(commandShoot);
        new JoystickButton(gunner, XboxControllerButtons.LEFT_BUMPER).onTrue(new Auto(swerveSubsystem, scoringSubsystem, new ArrayList<>(List.of(FavoritePositions.FRONTLEFT, FavoritePositions.FRONTLEFTMOST, FavoritePositions.FRONTRIGHT, FavoritePositions.FRONTRIGHTMOST)), 4, FavoritePositions.SPEAKER));
        new JoystickButton(gunner, XboxControllerButtons.A).onTrue(commandLoad);
        new JoystickButton(gunner, XboxControllerButtons.B).onTrue(commandUnload);
        new JoystickButton(gunner, XboxControllerButtons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(scoringSubsystem::increaseRPM));
        new JoystickButton(gunner, XboxControllerButtons.LEFT_STICK_BUTTON).onTrue(new InstantCommand(scoringSubsystem::decreaseRPM));
        new POVButton(gunner, 0).onTrue(toAmp);
        new POVButton(gunner, 90).onTrue(toSource);
        new POVButton(gunner, 180).onTrue(toUnderStage);
        new POVButton(gunner, 270).onTrue(resetArmEncoders);

        new JoystickButton(driver, XboxControllerButtons.MENU).onTrue(new InstantCommand(swerveSubsystem.gyro::zeroYaw, swerveSubsystem));
        new JoystickButton(driver, XboxControllerButtons.VIEW).onTrue(new InstantCommand(() -> swerveSubsystem.isFieldRelative = !swerveSubsystem.isFieldRelative, swerveSubsystem));
        new JoystickButton(driver, XboxControllerButtons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(swerveSubsystem.autonavigator::toggle, swerveSubsystem));
        new JoystickButton(driver, XboxControllerButtons.X).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.AMP), swerveSubsystem));
        new JoystickButton(driver, XboxControllerButtons.Y).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SOURCE), swerveSubsystem));
        new JoystickButton(driver, XboxControllerButtons.A).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(FavoritePositions.SPEAKER), swerveSubsystem));
        //new JoystickButton(driver, XboxControllerButtons.B).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(swerveSubsystem.odometer.getEstimatedPosition().nearest(Constants.FavoritePositions.allPoses)), swerveSubsystem));
        new JoystickButton(driver, XboxControllerButtons.B).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Vision.visionTargetLocation), swerveSubsystem));
        new POVButton(driver, 0).onTrue(new InstantCommand(() -> new PathPlannerAuto("Starting Point").schedule()));
        new POVButton(driver, 180).onTrue(new InstantCommand(() -> Vision.nextPipeline(LimelightConstants.llObjectDetectionName)));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));
        climbSubsystem.setDefaultCommand(new ClimbCmd(climbSubsystem, gunner));
        wristSubsystem.setDefaultCommand(new WristAdjustCmd(wristSubsystem, gunner));
        armSubsystem.setDefaultCommand(new ArmAdjustCmd(armSubsystem, gunner));
        wristIntakeSubsystem.setDefaultCommand(new WristIntakeCmd(wristIntakeSubsystem, gunner));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
