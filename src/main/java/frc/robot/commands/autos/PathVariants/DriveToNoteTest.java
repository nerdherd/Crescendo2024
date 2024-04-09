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
public class DriveToNoteTest extends SequentialCommandGroup{
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

    /**
     * max is 2 hops
     * @param swerve
     * @param superSystem
     * @param noteAssistance
     * @param paths max 2 paths
     */
    public DriveToNoteTest(SwerveDrivetrain swerve, SuperSystem superSystem, NoteAssistance na, double targetNoteArea, int minSamples, int maxSamples, PathPlannerPath path0, PathPlannerPath path1, DriverAssist driverAssist, ShooterVisionAdjustment sva) {
        Pose2d startingPose = new Pose2d(2.5, 5.58, new Rotation2d());
        
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),

            AutoBuilder.followPath(path0),

            Commands.deadline(
                Commands.waitUntil(superSystem::noteIntook),
                superSystem.intakeUntilSensedAuto(4),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    na.driveToNoteCommand(swerve, 15, 0, 0, 10, 50, null)
                )
            )
            ,
            
            Commands.parallel(
                Commands.sequence(
                    superSystem.backupIndexerAndShooter(),
                    superSystem.stow()
                ),
                // Drive in front of mid note
                Commands.deadline(
                    Commands.waitSeconds(2), // TODO: Find a working time
                    AutoBuilder.followPath(path1)
                )
            )
            // ,

            // Commands.runOnce(() -> swerve.towModules()),

            // // Turn to angle and shoot
            // Commands.deadline(
            //     Commands.waitUntil(() -> !superSystem.noteIntook()),
            //     Commands.sequence(
            //         Commands.parallel(
            //             // Turn to angle
            //             Commands.sequence(
            //                 Commands.deadline(
            //                     Commands.waitSeconds(2),
            //                     Commands.either(
            //                         driverAssist.turnToTag(4, swerve, 1),
            //                         driverAssist.turnToTag(7, swerve, 1),
            //                         RobotContainer::IsRedSide 
            //                     )
            //                 ),
            //                 Commands.runOnce(() -> swerve.towModules()),
            //                 superSystem.shootAuto()
            //             ),
            //             // Shoot
            //             Commands.sequence(
            //                 superSystem.backupIndexerAndShooter(),
            //                 superSystem.prepareShooterVision(sva)
            //             )
            //         )
            //     )
            // )

        );
    }
}
