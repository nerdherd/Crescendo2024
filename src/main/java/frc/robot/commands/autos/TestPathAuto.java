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
import frc.robot.subsystems.vision.DriverAssist;

public class TestPathAuto extends SequentialCommandGroup {
    public TestPathAuto(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist) {     
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.parallel(
                superSystem.intakeUntilSensed(),
                AutoBuilder.followPath(pathGroup.get(0)).andThen(Commands.waitSeconds(0.75))
            ),
            // Turn to tag later
            superSystem.prepareShooter().andThen(superSystem.shoot()),
            Commands.parallel(
                superSystem.intakeUntilSensed(),
                AutoBuilder.followPath(pathGroup.get(1)).andThen(Commands.waitSeconds(0.75))
            ),
            // Turn to tag later
            superSystem.prepareShooter().andThen(superSystem.shoot()),
            Commands.parallel(
                superSystem.intakeUntilSensed(),
                AutoBuilder.followPath(pathGroup.get(2)).andThen(Commands.waitSeconds(0.75))
            ),
            superSystem.prepareShooter().andThen(superSystem.shoot())
            );

            // Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            // Commands.sequence(

                // Preload 
                // Commands.deadline(
                //     // Commands.waitUntil(() -> !superSystem.colorSensor.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                    
                //     // Color Sensor wait for preload didn't work at Code Orange: May need more testing
                //     Commands.waitSeconds(1.5),
                //     superSystem.shootSubwoofer()
                // ),
                // Commands.sequence(
                //     superSystem.indexer.stopCommand(),
                //     superSystem.shooterRoller.setVelocityCommand(-10, -10),
                //     superSystem.shooterRoller.setEnabledCommand(true)
                // ), 

                // // Piece 1 intake
                // Commands.race(
                //     Commands.waitSeconds(2),
                //      IT WAS OVER HERRRRREEEEEE
                //     Commands.sequence(
                //         Commands.waitSeconds(0.125),
                //         superSystem.intakeUntilSensedAuto(1.75)
                //     )                
                // ),


                // AutoBuilder.followPath(pathGroup.get(1)),

                // Piece 1 shot
            //     Commands.parallel(
            //         // IT WAS OVER HEREEEEEEEEEEEEE
            //         Commands.sequence(
            //             superSystem.backupIndexerAndShooter(),
            //             Commands.waitSeconds(0.45),
            //             Commands.deadline(
            //                 Commands.waitSeconds(1.2),
            //                 superSystem.shootSubwooferAuto()  
            //             ),
            //             superSystem.indexer.stopCommand(),
            //             superSystem.shooterRoller.setVelocityCommand(-10, -10),
            //             superSystem.shooterRoller.setEnabledCommand(true)
            //         )
            //     ),

            //     // Piece 2 intake
            //     Commands.race(
            //         Commands.waitSeconds(2),
            //         AutoBuilder.followPath(pathGroup.get(2)).andThen(Commands.waitSeconds(0.75)),
            //         Commands.sequence(
            //             Commands.waitSeconds(0.125),
            //             superSystem.intakeUntilSensedAuto(1.75)
            //         )                
            //     ),
            //     // Piece 2 shot
            //     Commands.parallel(
            //         AutoBuilder.followPath(pathGroup.get(3)),
            //         Commands.sequence(
            //             superSystem.backupIndexerAndShooter(),
            //             Commands.waitSeconds(0.45),
            //             Commands.deadline(
            //                 Commands.waitSeconds(1.4),
            //                 superSystem.shootSubwooferAuto()  
            //             ),
            //             superSystem.indexer.stopCommand(),
            //             superSystem.shooterRoller.setVelocityCommand(-10, -10),
            //             superSystem.shooterRoller.setEnabledCommand(true)
            //         )
            //     ),

            //     // Piece 3 intake
            //     Commands.race(
            //         Commands.waitSeconds(1.75),
            //         AutoBuilder.followPath(pathGroup.get(4)).andThen(Commands.waitSeconds(0.5)),
            //         Commands.sequence(
            //             Commands.waitSeconds(0.125),
            //             superSystem.intakeUntilSensedAuto(1.75)
            //         )
            //     ),

            //     // Piece 3 shot
            //     Commands.parallel(
            //         AutoBuilder.followPath(pathGroup.get(5)),
            //         Commands.sequence(
            //             superSystem.backupIndexerAndShooter(),
            //             Commands.waitSeconds(0.45),
            //             Commands.deadline(
            //                 Commands.waitSeconds(1.2),
            //                 superSystem.shootSubwooferAuto()  
            //             ),
            //             superSystem.indexer.stopCommand(),
            //             superSystem.shooterRoller.setVelocityCommand(-10, -10),
            //             superSystem.shooterRoller.setEnabledCommand(true)
            //         )
            //     ),

            //     // Leave towards mid
            //     Commands.parallel(
            //         AutoBuilder.followPath(pathGroup.get(6)),
            //         superSystem.stow()
            //     ),

            //     Commands.none()
            // )
            // );
    }
}
