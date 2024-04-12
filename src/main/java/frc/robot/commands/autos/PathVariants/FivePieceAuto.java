package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class FivePieceAuto extends SequentialCommandGroup{
    public FivePieceAuto(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, NoteAssistance na) {
        Pose2d startingPose = new Pose2d(1.33, 5.55, new Rotation2d());//pathGroup.get(0).();
        addCommands(
            //Commands.runOnce(() -> na.setLight(false)),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),

            ///A
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitUntil(() -> !superSystem.noteIntook()),
                    superSystem.shootSubwoofer()
                ),

                // Stop shooter and indexer
                Commands.parallel(
                    superSystem.indexer.stopCommand(),
                    superSystem.shooterRoller.stopCommand()
                ),

                // Drive and intake
                Commands.race(
                    Commands.waitSeconds(3),
                    // Path
                    AutoBuilder.followPath(pathGroup.get(0)).andThen(Commands.waitSeconds(1.5)),
                    // Drive to note (if wanted)
                    // noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose) // TODO: change target values
                    // Intake
                    Commands.sequence(
                        Commands.waitSeconds(0.125),
                        superSystem.intakeUntilSensedAuto(2.875)
                    ),
                    Commands.waitUntil(superSystem::noteIntook)
                ),

                // Turn to angle and shoot
                Commands.deadline(
                    Commands.waitUntil(() -> !superSystem.noteIntook()),
                    Commands.sequence(
                        Commands.deadline(
                            // Turn to angle
                            Commands.sequence(
                                Commands.deadline(
                                    Commands.waitSeconds(1),
                                    Commands.either(
                                        swerve.turnToTag(4, 2),
                                        swerve.turnToTag(7, 2),
                                        RobotContainer::IsRedSide 
                                    )
                                ),
                                Commands.runOnce(() -> swerve.towModules()),
                                superSystem.shootAuto()
                            ),
                            // Shoot
                            Commands.sequence(
                                superSystem.backupIndexerAndShooter(),
                                superSystem.prepareShooterVision()
                            )
                        )
                    )
                )
            ),
            // Commands.runOnce(swerve.getImu()::zeroAll),
            //Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            //Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // B (1) Start here *******************************************************************************
            // Intake and Path
            Commands.race(
                Commands.waitSeconds(3),
                AutoBuilder.followPath(pathGroup.get(1)).andThen(Commands.waitSeconds(1.5)),
                Commands.sequence(
                    Commands.waitSeconds(0.125),
                    superSystem.intakeUntilSensedAuto(2.875)
                ),
                Commands.waitUntil(superSystem::noteIntook)         
            ),

            // Turn to angle and shoot
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()),
                Commands.sequence(
                    Commands.deadline(
                        // Turn to angle
                        Commands.sequence(
                            Commands.deadline(
                                Commands.waitSeconds(1),
                                Commands.either(
                                    swerve.turnToTag(4, 2),
                                    swerve.turnToTag(7, 2),
                                    RobotContainer::IsRedSide 
                                )
                            ),
                            Commands.runOnce(() -> swerve.towModules()),
                            superSystem.shootAuto()
                        ),
                        // Shoot
                        Commands.sequence(
                            superSystem.backupIndexerAndShooter(),
                            superSystem.prepareShooterVision()
                        )
                    )
                )
            ),

            // B (2) Start here *******************************************************************************
            // Intake and Path
            Commands.race(
                Commands.waitSeconds(3),
                AutoBuilder.followPath(pathGroup.get(2)).andThen(Commands.waitSeconds(1.5)),
                Commands.sequence(
                    Commands.waitSeconds(0.125),
                    superSystem.intakeUntilSensedAuto(2.875)
                ),
                Commands.waitUntil(superSystem::noteIntook)           
            ),

            // Turn to angle and shoot
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()),
                Commands.sequence(
                    Commands.deadline(
                        // Turn to angle
                        Commands.sequence(
                            Commands.deadline(
                                Commands.waitSeconds(1),
                                Commands.either(
                                    swerve.turnToTag(4, 2),
                                    swerve.turnToTag(7, 2),
                                    RobotContainer::IsRedSide 
                                )
                            ),
                            Commands.runOnce(() -> swerve.towModules()),
                            superSystem.shootAuto()
                        ),
                        // Shoot
                        Commands.sequence(
                            superSystem.backupIndexerAndShooter(),
                            superSystem.prepareShooterVision()
                        )
                    )
                )
            )
            ,
            // C Start here ******************************************************

            Commands.parallel(
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.shooterPivot.moveToHandoff()
                ),
                // Drive in front of mid note
                Commands.deadline(
                    Commands.waitSeconds(2), // TODO: Find a working time
                    swerve.driveToPose(new Pose2d(6, 6.5, new Rotation2d()), 5, 5)
                )
            ),

            Commands.runOnce(() -> swerve.towModules()),

            // This should be Path D


            Commands.deadline(
                Commands.waitUntil(superSystem::noteIntook),
                superSystem.intakeUntilSensedAuto(4),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    na.driveToNoteCommand(swerve, 15, 0, 0, 10, 50, null)
                )
            ),
            
            Commands.parallel(
                superSystem.backupIndexerAndShooter(),
                superSystem.stow(),
                // Drive in front of mid note
                Commands.deadline(
                    Commands.waitSeconds(2), // TODO: Find a working time
                    swerve.driveToPose(new Pose2d(4.5, 5.55, new Rotation2d()), 5, 5)
                )
            ),

            Commands.runOnce(() -> swerve.towModules()),

            // Turn to angle and shoot
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()),
                Commands.sequence(
                    Commands.parallel(
                        // Turn to angle
                        Commands.sequence(
                            Commands.deadline(
                                Commands.waitSeconds(2),
                                Commands.either(
                                    swerve.turnToTag(4, 1),
                                    swerve.turnToTag(7, 1),
                                    RobotContainer::IsRedSide 
                                )
                            ),
                            Commands.runOnce(() -> swerve.towModules()),
                            superSystem.shootAuto()
                        ),
                        // Shoot
                        Commands.sequence(
                            superSystem.backupIndexerAndShooter(),
                            superSystem.prepareShooterVision()
                        )
                    )
                )
            )

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
