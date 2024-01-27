package frc.robot;
/*
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.Constants.LoggerConstants;*/

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LoggerConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    public RobotContainer m_robotContainer;

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

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
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
        m_robotContainer.swerveSubsystem.autonavigator.disable();
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
        //TODO: NO need to resetHeadingAndPose because pathplanner automatically does it with auto commands (this might be whats causing robot to jerk when renable in autonoumous)
        //m_robotContainer.swerveSubsystem.resetHeadingAndPose();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            //auto paths can preset pos by default (reset odometry)
            //TODO: Be careful when calling auto paths it resets odometry and assumes you know where you are starting from if not then use pathFindThenFollowPath
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // System.out.println(m_robotContainer.swerveSubsystem.getPose());
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // TODO: formalize auto paths to end at same location.
        m_robotContainer.swerveSubsystem.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Starting Point"));
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        m_robotContainer.driverControls.processCycle();
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
