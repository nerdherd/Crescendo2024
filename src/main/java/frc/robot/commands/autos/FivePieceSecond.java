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
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class FivePieceSecond extends SequentialCommandGroup {
    public FivePieceSecond(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, ShooterVisionAdjustment sva) {     
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            Commands.sequence(

                // Preload 
                Commands.deadline(
                    // Commands.waitUntil(() -> !superSystem.colorSensor.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                    
                    // Color Sensor wait for preload didn't work at Code Orange: May need more testing
                    Commands.waitSeconds(1.5),
                    superSystem.shootSubwoofer()
                ),
                Commands.sequence(
                    superSystem.indexer.stopCommand(),
                    superSystem.shooterRoller.setVelocityCommand(0, 0)
                ), 

                // Piece 1
                Commands.deadline(
                    Commands.waitSeconds(1.75),
                    AutoBuilder.followPath(pathGroup.get(0)),
                    superSystem.intakeUntilSensed()
                ),

                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    Commands.sequence(
                        Commands.waitSeconds(0.85),
                        Commands.deadline(
                            Commands.waitSeconds(1.2),
                            superSystem.shootSubwooferAuto()  
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(0, 0)
                    )
                ),

                // Piece 2
                Commands.deadline(
                    Commands.waitSeconds(1.75),
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeUntilSensed()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    Commands.sequence(
                        Commands.waitSeconds(0.85),
                        Commands.deadline(
                            Commands.waitSeconds(1.2),
                            superSystem.shootSubwooferAuto()  
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(0, 0)
                    )
                ),

                // Piece 3
                Commands.deadline(
                    Commands.waitSeconds(1.75),
                    AutoBuilder.followPath(pathGroup.get(4)),
                    superSystem.intakeUntilSensed()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(5)),
                    Commands.sequence(
                        Commands.waitSeconds(0.85),
                        Commands.deadline(
                            Commands.waitSeconds(1.2),
                            superSystem.shootSubwooferAuto()  
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(0, 0)
                    )
                ),

                // Piece 4
                Commands.deadline(
                    Commands.waitSeconds(1.75),
                    AutoBuilder.followPath(pathGroup.get(6)),
                    superSystem.intakeUntilSensed()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(7)),
                    Commands.sequence(
                        Commands.waitSeconds(0.85),
                        Commands.deadline(
                            Commands.waitSeconds(1.2),
                            superSystem.shootSequenceAdjustable(sva)  
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(0, 0)
                    )
                ),

                

                Commands.none()
            )
            );
    }
}
