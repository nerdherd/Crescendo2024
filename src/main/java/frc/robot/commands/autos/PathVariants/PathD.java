package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

//TODO: get actual position for default pose
public class PathD extends SequentialCommandGroup{
    // private boolean stop = false;
    // private int i;
    /**
     * Paths for middle notes
     * @param swerve swerve drivetrain instance
     * @param superSystem supersystem instance
     * @param noteAssistance note assistance instance
     * @param targetNoteArea target area for drive to note
     * @param paths paths in order you want to go
     */
    // public PathD(SwerveDrivetrain swerve, SuperSystem superSystem, NoteAssistance noteAssistance, double targetNoteArea, PathPlannerPath... paths) {
    //     for (i = 0; i < paths.length; i++) {
    //         addCommands(
    //             AutoBuilder.followPath(paths[i]),
    //             Commands.either(
    //                 Commands.sequence(
    //                     superSystem.intakeUntilSensedAuto(2),
    //                     noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, minSamples, maxSamples, new Pose2d())
    //                         .onlyWhile(noteAssistance::hasTarget).until(superSystem::noteIntook),
    //                     Commands.runOnce(() -> stop = superSystem.noteIntook() ? true : false)
    //                 ),
    //                 Commands.none(), 
    //                 noteAssistance::hasTarget
    //             )
    //         );
    //         if(stop) break;
    //     }
    // }

    /**
     * max is 2 hops
     * @param swerve
     * @param superSystem
     * @param noteAssistance
     * @param paths max 2 paths
     */
    public PathD(SwerveDrivetrain swerve, SuperSystem superSystem, NoteAssistance na, double targetNoteArea, int minSamples, int maxSamples, PathPlannerPath path0, PathPlannerPath path1, DriverAssist driverAssist, ShooterVisionAdjustment sva) {
        addCommands(
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
                                    driverAssist.turnToTag(4, swerve, 1),
                                    driverAssist.turnToTag(7, swerve, 1),
                                    RobotContainer::IsRedSide 
                                )
                            ),
                            Commands.runOnce(() -> swerve.towModules()),
                            superSystem.shootAuto()
                        ),
                        // Shoot
                        Commands.sequence(
                            superSystem.backupIndexerAndShooter(),
                            superSystem.prepareShooterVision(sva)
                        )
                    )
                )
            )

        );
    }
}
