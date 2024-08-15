package frc.robot.autons.modes;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class choreotest extends SequentialCommandGroup{

    public choreotest(Swerve swerve){
        addRequirements(swerve);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            swerve.runChoreoTraj(Choreo.getTrajectory("2pOne"), true)
        );
        
    }
    
}
