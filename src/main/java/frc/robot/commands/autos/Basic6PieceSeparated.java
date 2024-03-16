package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Basic6PieceSeparated extends SequentialCommandGroup {
    public Basic6PieceSeparated(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(1.5), // possibly change this time based on testing time
                    superSystem.shootSubwooferSequence()
                ),
                Commands.deadline(
                    AutoBuilder.followPath((pathGroup.get(0))),
                    superSystem.intakeDirectShoot(ShooterConstants.k6PieceHandoffPosition.get(), ShooterConstants.kTopOuttakeAuto1.get(), ShooterConstants.kBottomOuttakeAuto1.get())
                ),               
                AutoBuilder.followPath(pathGroup.get(1)),
                Commands.deadline(
                    Commands.sequence(
                        AutoBuilder.followPath(pathGroup.get(2)),
                        Commands.waitSeconds(1)
                    ),
                    superSystem.intakeBasic()
                ),
                AutoBuilder.followPath(pathGroup.get(3)),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(4)),
                    superSystem.shooterRoller.setEnabledCommand(true),
                    superSystem.shooterRoller.shootSpeaker()   
                ),
                superSystem.indexer.setEnabledCommand(true),
                superSystem.indexer.indexCommand(),
                AutoBuilder.followPath(pathGroup.get(5)),
                Commands.deadline(
                    Commands.sequence(
                        AutoBuilder.followPath(pathGroup.get(6)),
                        Commands.waitSeconds(1)
                    ),
                    superSystem.intakeBasic()
                ),
                AutoBuilder.followPath(pathGroup.get(7)),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(8)),
                    superSystem.shooterRoller.setEnabledCommand(true),
                    superSystem.shooterRoller.shootSpeaker()
                ),
                superSystem.indexer.setEnabledCommand(true),
                superSystem.indexer.indexCommand()
                )
            );
    }
}
