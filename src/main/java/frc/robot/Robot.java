package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LoggerConstants;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightImplementation;
import frc.robot.subsystems.oi.ControlManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    public RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.recordMetadata("version", LoggerConstants.RUNNING_UNDER);
        Logger.recordMetadata("build_commit", BuildConstants.GIT_SHA);
        Logger.recordMetadata("build_branch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("build_git_uncommitted_changes", String.valueOf(BuildConstants.DIRTY));
        Logger.recordMetadata("build_timestamp", BuildConstants.BUILD_DATE);

        switch (LoggerConstants.MODE) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
                if (!LoggerConstants.SILENT_NT4) {
                    Logger.addDataReceiver(new NT4Publisher());
                }
                new PowerDistribution(1, LoggerConstants.PDU_TYPE);
                break;
            case SIMULATION:
                Logger.addDataReceiver(new WPILOGWriter(""));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        // if you want to stop the robot, use the boolean returned by this method.
        Constants.CANAssignments.checkAssignments();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        robotContainer.swerveSubsystem.autonavigator.disable();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // System.out.println(robotContainer.swerveSubsystem.getPose());
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        //this also works for resetting odometry while flipped 
        //new PathPlannerAuto("Starting Point").schedule();
        //but below this is a better option for restting odometry especially with dynamic positions with vision

        //TODO: Pathplanner always sets blue alliance middle as 0, 0 for odometry and always bases odometry origin at that position therefore we need to flip like this for red alliance
        /*var alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
            robotContainer.swerveSubsystem.resetOdometry(GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile("Starting Point")));
        } else {
            robotContainer.swerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Starting Point")); 
        }*/
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        ControlManager.processDriver();

        if (robotContainer.swerveSubsystem.autonavigator.isEnabled()) {
            if (ControlManager.Outputs.xSpeed != 0 || ControlManager.Outputs.ySpeed != 0 || ControlManager.Outputs.rotatingSpeed != 0) {
                robotContainer.swerveSubsystem.autonavigator.pauseNavigation();
            } else {
                robotContainer.swerveSubsystem.autonavigator.resumeNavigation();
            }
        }
        robotContainer.objectLocation = LimelightImplementation.transformTargetLocation(robotContainer.swerveSubsystem.odometer.getEstimatedPosition());
        Logger.recordOutput("limelight.pipelineIndex", LimelightHelpers.getCurrentPipelineIndex("limelight"));
        Logger.recordOutput("limelight.pipelineName", LimelightImplementation.getPipelineName());
        Logger.recordOutput("limelight.objectDetected", LimelightHelpers.getTV("limelight"));
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
