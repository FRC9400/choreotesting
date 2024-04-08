package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class RACE_CENTERLINE_MID extends SequentialCommandGroup{
    private final PathPlannerPath forward = PathPlannerPath.fromChoreoTrajectory("MidRaceCenterlineForward");
    private final PathPlannerPath backward = PathPlannerPath.fromChoreoTrajectory("MidRaceCenterlineBackward");
    public RACE_CENTERLINE_MID(Swerve swerve, Superstructure superstructure){
        
        addRequirements(swerve, superstructure);
        addCommands(
        AutoBuilder.followPath(forward),
        new WaitCommand(2.5),
        AutoBuilder.followPath(backward)
        );
        
    }
}


