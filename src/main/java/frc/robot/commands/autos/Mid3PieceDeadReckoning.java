package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class Mid3PieceDeadReckoning extends SequentialCommandGroup {
    public Mid3PieceDeadReckoning(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            
            // Preload
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                superSystem.shootSubwooferSequence()
            ),

            // Stop shooter and indexer
            Commands.parallel(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.stopCommand()
            ), 
            
            /************************************ NOTE ONE **************************************/

            // Grab First Note
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(2),
                    superSystem.intakeUntilSensed()
                )
            ),

            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(1)), 
                superSystem.stow(),
                Commands.waitSeconds(4)
            ),

            // Shoot
            Commands.deadline(
                Commands.waitSeconds(1.2),
                superSystem.shootPodiumSequence()
            ),
            
            // Stop shooter and indexer
            Commands.parallel(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.stopCommand()
            ), 

            /************************************ NOTE TWO **************************************/

            // Grab note
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(2)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(2),
                    superSystem.intakeUntilSensed()
                )
            ),
            
            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(3)),
                superSystem.stow(),
                Commands.waitSeconds(4)
            ),

            // Shoot
            Commands.deadline(
                Commands.waitSeconds(1.2),
                superSystem.shootPodiumSequence()
            ),

            // Stop shooter and indexer
            Commands.sequence(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.stopCommand()
            ), 

            /************************************ LEAVE **************************************/

            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(4)),
                superSystem.stow()
            )
        );
    }
}
