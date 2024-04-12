package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class PathA extends SequentialCommandGroup{
    public PathA(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, int index){
        // Pose2d startingPose = path.getPreviewStartingHolonomicPose();
        Pose2d startingPose = new Pose2d(1.33, 5.55, new Rotation2d());//pathGroup.get(0).();
        //Pose2d endingPose = new Pose2d(4.5, 5.55, new Rotation2d());
        addCommands(
            
            //Commands.runOnce(() -> na.setLight(false)),
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),

            Commands.sequence(
                Commands.parallel(
                    // Drive to note 2
                    Commands.race(
                        AutoBuilder.followPath(pathGroup.get(0)),
                        Commands.sequence(
                            // enable after the preload gone
                            Commands.waitSeconds(1.25),
                            Commands.deadline(
                                Commands.waitSeconds(1),
                                Commands.waitUntil(superSystem::noteIntook)
                            )
                        )
                    ),

                    Commands.sequence(

                        // Preload
                        Commands.deadline(
                            Commands.waitUntil(() -> !superSystem.noteIntook()),
                            Commands.parallel(
                                superSystem.shootSubwooferAutoStart(),
                                superSystem.intakeRoller.autoIntakeCommand()
                            )
                        ),
                        
                        superSystem.shooterRoller.setVelocityCommand(-10, -10),
                        superSystem.shooterRoller.setEnabledCommand(true),
                        //superSystem.indexer.stopCommand(),
                        Commands.waitSeconds(0.1),

                        // pick up note 2
                        // Commands.race(
                        //     // Intake
                        //     Commands.sequence(
                        //         Commands.waitSeconds(0.125),
                        //         superSystem.intakeUntilSensedAuto(2.875)
                        //     ),
                        //     Commands.waitUntil(superSystem::noteIntook)
                        // ),

                        Commands.deadline(
                            Commands.waitUntil(superSystem::noteIntook),
                            superSystem.intakeUntilSensedAuto(2.875)
                        ),
                        
                        // Turn to angle and shoot
                        Commands.waitSeconds(0.5),
                        Commands.deadline(
                            Commands.waitUntil(() -> !superSystem.noteIntook()),
                            superSystem.shootSubwooferAutoStart2()
                        )
                    )    
                ),
                
                Commands.parallel(
                    // Drive to note 6
                    //Commands.race(
                        AutoBuilder.followPath(pathGroup.get(1)),
                        // Commands.sequence(
                        //     // enable after the preload gone
                        //     Commands.waitSeconds(1.25),
                        //     Commands.deadline(
                        //         Commands.waitSeconds(1),
                        //         Commands.waitUntil(superSystem::noteIntook)
                        //     )
                        // )
                    //),

                        // note 2
                        Commands.deadline(
                            Commands.waitUntil(() -> !superSystem.noteIntook()),
                            superSystem.shootSubwooferAutoStart2()
                        )
                ),        

                //superSystem.shootSubwooferAutoStart2(),

                // after done with 2 notes 
                //Commands.parallel(
                    // lower it before enter in stage area
                    superSystem.stow(),
                    superSystem.indexer.stopCommand(),
                    Commands.waitSeconds(1)
                //),
            // Commands.sequence(
            //     // Preload
            //     Commands.deadline(
            //         Commands.waitUntil(() -> !superSystem.noteIntook()),
            //         superSystem.shootSubwoofer()
            //     ),

            //     // Stop shooter and indexer
            //     Commands.parallel(
            //         superSystem.indexer.stopCommand(),
            //         superSystem.shooterRoller.stopCommand()
            //     ),

            //     // Drive and intake
            //     Commands.race(
            //         Commands.waitSeconds(3),
            //         // Path
            //         AutoBuilder.followPath(pathGroup.get(index)).andThen(Commands.waitSeconds(1.5)),
            //         // Drive to note (if wanted)
            //         // noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose) // TODO: change target values
            //         // Intake
            //         Commands.sequence(
            //             Commands.waitSeconds(0.125),
            //             superSystem.intakeUntilSensedAuto(2.875)
            //         ),
            //         Commands.waitUntil(superSystem::noteIntook)
            //     ),

            //     // Turn to angle and shoot
            //     Commands.deadline(
            //         Commands.waitUntil(() -> !superSystem.noteIntook()),
            //         Commands.sequence(
            //             Commands.deadline(
            //                 // Turn to angle
            //                 Commands.sequence(
            //                     Commands.deadline(
            //                         Commands.waitSeconds(1),
            //                         Commands.either(
            //                             driverAssist.turnToTag(4, swerve, 2),
            //                             driverAssist.turnToTag(7, swerve, 2),
            //                             RobotContainer::IsRedSide 
            //                         )
            //                     ),
            //                     Commands.runOnce(() -> swerve.towModules()),
            //                     superSystem.shootAuto()
            //                 ),
            //                 // Shoot
            //                 Commands.sequence(
            //                     superSystem.backupIndexerAndShooter(),
            //                     superSystem.prepareShooterVision(sva)
            //                 )
            //             )
            //         )
            //     )
            )
        );
    }

    PathConstraints pathcons = new PathConstraints(
        2, 2, 
        Units.degreesToRadians(180), Units.degreesToRadians(360)
    );

    public Command driveToPose(Pose2d destPoseInBlue) {
        return Commands.either(
            AutoBuilder.pathfindToPose(GeometryUtil.flipFieldPose(destPoseInBlue), pathcons),
            AutoBuilder.pathfindToPose(destPoseInBlue, pathcons,0),
            RobotContainer::IsRedSide  
        );
    }  

    // to be tested. Do not use it before test
    public static Pose2d GetEndPoseInPath(PathPlannerPath path)
    {
        PathPoint tail  = path.getPoint(path.numPoints()-1);
        double rad = tail.rotationTarget.getTarget().getRadians();
        return new Pose2d(tail.position, new Rotation2d(rad));
    } 
    public static Pose2d GetStartPoseInPath(PathPlannerPath path)
    {
        PathPoint tail  = path.getPoint(0);
        double rad = tail.rotationTarget.getTarget().getRadians();
        return new Pose2d(tail.position, new Rotation2d(rad));
    }
}
