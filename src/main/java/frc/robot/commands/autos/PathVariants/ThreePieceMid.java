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

public class ThreePieceMid extends SequentialCommandGroup{
    public ThreePieceMid(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, NoteAssistance na) {
        Pose2d startingPose = new Pose2d(0.70, 4.37, Rotation2d.fromDegrees(-60));//pathGroup.get(0).();
        addCommands(
            //Commands.runOnce(() -> na.setLight(false)),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),

            // Preload
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()),
                superSystem.shootSubwoofer()
            ),

            // Drive to note 8

            Commands.parallel(
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.shooterPivot.moveToHandoff()
                ),
                // Drive in front of mid note
                Commands.deadline(
                    Commands.waitSeconds(2.5),
                    AutoBuilder.followPath(pathGroup.get(0))
                )
            ),

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
                // Drive back
                Commands.deadline(
                    Commands.waitSeconds(2),
                    AutoBuilder.followPath(pathGroup.get(1))
                )
            ),

            // Shoot 8

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
                            superSystem.prepareShooterVision(swerve)
                        )
                    )
                )
            ),

            // Drive to 7

            Commands.parallel(
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.shooterPivot.moveToHandoff()
                ),
                // Drive in front of mid note
                Commands.deadline(
                    Commands.waitSeconds(2.5),
                    AutoBuilder.followPath(pathGroup.get(2))
                )
            ),

            Commands.deadline(
                Commands.waitUntil(superSystem::noteIntook),
                superSystem.intakeUntilSensedAuto(4),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    na.driveToNoteCommand(swerve, 15, 0, 0, 10, 50, null)
                )
            ),
            
            // Shoot 7
            
            Commands.parallel(
                superSystem.backupIndexerAndShooter(),
                superSystem.stow(),
                // Drive back
                Commands.deadline(
                    Commands.waitSeconds(3),
                    AutoBuilder.followPath(pathGroup.get(3))
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
                            superSystem.prepareShooterVision(swerve)
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
