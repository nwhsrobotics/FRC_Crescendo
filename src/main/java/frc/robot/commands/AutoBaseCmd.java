package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is an implementation of the SequentialCommandGroup class that is
 * used for executing a sequence of commands
 * for autonomous operation of a swerve drive base. The sequence consists of a
 * series of commands including driving along
 * two trajectories, resetting the pose and heading of the swerve drive base,
 * homing the arm of the robot, and performing
 * various commands to manipulate a grabber subsystem for handling game pieces.
 * <p>
 * The constructor of this class takes in several parameters including instances
 * of various subsystems such as the swerve
 * drive, shoulder, arm, and grabber subsystems. It also takes in two paths to
 * two trajectory files that are loaded in as
 * Trajectory objects.
 * <p>
 * This class extends the SequentialCommandGroup class from the WPILib library
 * and is used in combination with the CommandScheduler
 * class to execute the sequence of commands autonomously.
 */
public class AutoBaseCmd extends SequentialCommandGroup {
    private Trajectory trajA = new Trajectory(); // Trajectory A object
    private Trajectory trajB = new Trajectory(); // Trajectory B object

    /**
     * @param s_Swerve     an instance of the swerve subsystem
     * @param m_shoulder   an instance of the shoulder subsystem
     * @param armSubsystem an instance of the arm subsystem
     * @param m_grabber    an instance of the grabber subsystem
     * @param trajAPath    path to Trajectory A file
     * @param trajBPath    path to Trajectory B file
     */
    public AutoBaseCmd(SwerveSubsystem s_Swerve, String trajAPath, String trajBPath) {


        // Get paths to trajectory files
        Path trajectoryPathA = Filesystem.getDeployDirectory().toPath().resolve(trajAPath);
        Path trajectoryPathB = Filesystem.getDeployDirectory().toPath().resolve(trajBPath);

        try {
            // Load Trajectory A and B objects from corresponding files
            trajA = TrajectoryUtil.fromPathweaverJson(trajectoryPathA);
            trajB = TrajectoryUtil.fromPathweaverJson(trajectoryPathB);
        } catch (IOException e) {
            // Print stack trace if error occurs during loading of trajectory files
            e.printStackTrace();
        }

        // Initialize thetaController with appropriate parameters
        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Initialize SwerveControllerCommand objects with appropriate parameters
        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
                trajA,
                s_Swerve::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        // Create a SwerveControllerCommand for trajectory B
        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                trajB, // trajectory to follow
                s_Swerve::getPose, // function to get current pose
                Constants.DriveConstants.kDriveKinematics, // swerve kinematics
                new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X PID controller
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y PID controller
                thetaController, // heading controller
                s_Swerve::setModuleStates, // function to set swerve module states
                s_Swerve); // swerve subsystem

        // Add commands to be run
        addCommands(
                // Reset the swerve heading and pose
                new InstantCommand(() -> s_Swerve.resetHeadingAndPose()),

                // Reset the swerve odometry to the initial pose of trajectory A
                new InstantCommand(() -> s_Swerve.resetOdometry(trajA.getInitialPose())),

                // Print the current pose of the swerve subsystem
                new InstantCommand(() -> System.out.println(s_Swerve.getPose())),

                // Print the current heading of the swerve subsystem
                new InstantCommand(() -> System.out.println(s_Swerve.getHeading())),

                // Print the total time of trajectory A
                new InstantCommand(() -> System.out.println(trajA.getTotalTimeSeconds())),

                /*
                 * // First place preloaded cone
                 * new ParallelCommandGroup(
                 * new InstantCommand(() -> m_shoulder.setPos_deg(55.0)),
                 * new InstantCommand(() -> m_arm.setPos_inch(30.0)),
                 * new InstantCommand(() -> m_grabber.grabberExtend())
                 * ),
                 *
                 * // Wait 0.5 seconds
                 * new InstantCommand(() -> Timer.delay(0.5)),
                 *
                 * // Retract shoulder and arm
                 * new ParallelCommandGroup(
                 * new InstantCommand(() -> m_shoulder.setPos_deg(0.0)),
                 * new InstantCommand(() -> m_arm.setPos_inch(0.0))
                 * ),
                 */

                // Drive to new cone using swerveControllerCommand1
                swerveControllerCommand1,

                /*
                 * // Extend shoulder and arm again
                 * new ParallelCommandGroup(
                 * new InstantCommand(() -> m_shoulder.setPos_deg(55.0)),
                 * new InstantCommand(() -> m_arm.setPos_inch(30.0))
                 * ),
                 *
                 * // Close grabber around cone
                 * new InstantCommand(() -> m_grabber.grabberRetract()),
                 *
                 * // Retract shoulder and arm
                 * new ParallelCommandGroup(
                 * new InstantCommand(() -> m_shoulder.setPos_deg(0.0)),
                 * new InstantCommand(() -> m_arm.setPos_inch(0.0))
                 * ),
                 *
                 * // Drive back to grid using swerveControllerCommand2
                 * swerveControllerCommand2,
                 *
                 * // Extend shoulder and arm
                 * new ParallelCommandGroup(
                 * new InstantCommand(() -> m_shoulder.setPos_deg(55.0)),
                 * new InstantCommand(() -> m_arm.setPos_inch(30.0))
                 * ),
                 *
                 * // Open grabber
                 * new InstantCommand(() -> m_grabber.grabberExtend()),
                 */

                // Stop the swerve subsystem and print the final pose and heading
                new InstantCommand(() -> System.out.println(s_Swerve.getPose())),
                new InstantCommand(() -> System.out.println(s_Swerve.getHeading())));
    }
}