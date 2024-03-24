package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FavoritePositions;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoSelector {
    public AutoSelector(SwerveSubsystem swerve){
        SendableChooser<Command> autoChooser = new SendableChooser<>();
        autoChooser.addOption("B [5 NOTE]", new Auto(swerve, new ArrayList<>(List.of(FavoritePositions.FRONTLEFT, FavoritePositions.FRONTLEFTMOST, FavoritePositions.FRONTRIGHT, FavoritePositions.FRONTRIGHTMOST)), 4, FavoritePositions.SPEAKER));
        SmartDashboard.putData(autoChooser);
    }
}
