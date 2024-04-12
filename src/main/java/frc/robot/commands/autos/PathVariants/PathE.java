package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class PathE extends SequentialCommandGroup {
    public PathE(SwerveDrivetrain swerve, SuperSystem superSystem, PathPlannerPath path){
        // start of E ****************************************************************
        addCommands(
            Commands.sequence(
                Commands.deadline(
                    AutoBuilder.followPath(path), //path group is anything
                    Commands.sequence(
                        superSystem.backupIndexer(), //TODO: should we move this into path D?
                        superSystem.prepareShooterPodium()
                    ),                
                    Commands.waitSeconds(3.5)
                ),
    
                // Reset odometry
                //driverAssist.resetOdoPoseByVision(swerve, null, 0, 8), //change april tag id ltr
    
                Commands.deadline(
                    Commands.sequence(
                        // Commands.waitSeconds(0.1),
                        Commands.either(
                            //turn to angle
                            swerve.turnToTag(4), //Placeholder turn to tag methods
                            swerve.turnToTag(7),
                            RobotContainer::IsRedSide 
                        )
                    ),
                    Commands.deadline(
                        Commands.waitSeconds(2),
                        // superSystem.shootPodium()
                        superSystem.shootSequenceAdjustableAuto()
                    )
                    // end of E *******************************************************
                )
            )
            );
    }
}

