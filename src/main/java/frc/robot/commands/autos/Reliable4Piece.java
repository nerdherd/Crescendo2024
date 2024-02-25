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

public class Reliable4Piece extends SequentialCommandGroup {
    public Reliable4Piece(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem) {     
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            Commands.sequence(
                superSystem.intakePivot.setEnabledCommand(true),
                superSystem.intakePivot.moveToIntake(),

                // Preload 
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSequence2()
                ),

                // Piece 1
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    superSystem.intakeBasicHold()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        superSystem.shootSequence2()  
                    )
                ),

                // Piece 2
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeBasicHold()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        superSystem.shootSequence2()  
                    )
                ),

                // Piece 3
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(4)),
                    superSystem.intakeBasicHold()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(5)),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        superSystem.shootSequence2()  
                    )
                ),

                // Leave towards mid
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(6)),
                    superSystem.stow()
                ),

                Commands.none()
            )
            );
    }
}
