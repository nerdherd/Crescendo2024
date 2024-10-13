package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class PreloadTaxi extends SequentialCommandGroup {
    public PreloadTaxi(SwerveDrivetrain swerve, List<PathPlannerPath> pathGroup, SuperSystem superSystem) {     
        //List<PathPlannerPath> pathGroup = pathGroupExample;
        Pose2d startingPose = pathGroup.get(0).getPreviewStartingHolonomicPose();

        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // A Start here
            // Preload
            Commands.deadline(
                Commands.waitSeconds(1.5),
                superSystem.shootSubwoofer()
            ),

            // Stop shooter and indexer
            Commands.parallel(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.stopCommand()
            ), 
            
            // Leave
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    superSystem.stow()
                    // ,
                    // Commands.waitSeconds(2),
                    // superSystem.intakeUntilSensed()
                )
            )
            
            // A End here

            // C Starts here

            // C ends here

            //....
        );
    }
}
