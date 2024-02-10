package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
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
            Commands.runOnce(()->swerve.setPoseMeters(startingPose)),
            Commands.sequence(
                Commands.parallel(
                    AutoBuilder.followPath((pathGroup.get(0))),
                    superSystem.intakeDirectShoot()
                ),               
                AutoBuilder.followPath(pathGroup.get(1)),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    Commands.run(() -> superSystem.intakePickup())
                )
                AutoBuilder.followPath(pathGroup.get(3)),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(4)),
                    shooterRoller.setEnabledCommand(true),
                    shooterRoller.shootSpeaker()   
                ),
                indexer.setEnabledCommand(true),
                indexer.indexCommand()
                // Commands.parallel(
                //     Commands.sequence(
                //         Commands.waitSeconds(0.5);
                //         indexer.stop();
                //         shooterRoller.stop(); 
                //     )
                //     AutoBuilder.followPath(pathGroup.get(5)),
                //     )
                // Commands.parallel(
                //     AutoBuilder.followPath(pathGroup.get(6)),
                //     s
                // )
                

                



            //     Commands.deadline(
            //         Commands.waitUntil(() -> 
            //         shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
            //     shooterHandoff()
            //     ),
            // shooterRoller.setEnabledCommand(true),
            // indexer.setEnabledCommand(true),
            // indexer.indexCommand(),
            // shooterRoller.shootSpeakerAuto1(),
            // intakeRoller.intakeCommand(),
            // Commands.waitUntil(() -> false)

        //                     // Prepare to shoot
        //     shooterPivot.moveToSpeakerFar(),
        //     shooterRoller.setEnabledCommand(true),
        //     shooterRoller.shootSpeaker(),
        //     Commands.waitSeconds(1),
            
        //     // Shoot
        //     indexer.setEnabledCommand(true),
        //     indexer.indexCommand(),
        //     Commands.waitUntil(() -> false)
        // ).finallyDo(interrupted -> {
        //     indexer.stop();
        //     shooterRoller.stop();
                // superSystem.shootSequence2Far(),
                // Commands.waitSeconds(1),
                // Commands.parallel(
                //     AutoBuilder.followPath(pathGroup.get(2)),
                //     Commands.run(() -> superSystem.intakePickup())
                // )
                // superSystem.shootSequence2Far()
                
            )
            );
    }
}
