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
    public PreloadTaxi(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem) {     
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
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
        );
    }
}
