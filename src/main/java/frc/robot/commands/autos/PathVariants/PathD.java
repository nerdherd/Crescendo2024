package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

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
    public PathD(SwerveDrivetrain swerve, SuperSystem superSystem, NoteAssistance noteAssistance, double targetNoteArea, int minSamples, int maxSamples, List<PathPlannerPath> paths) {
        addCommands(
            Commands.sequence(
                Commands.either(
                    Commands.sequence(
                        superSystem.intakeUntilSensedAuto(2),
                        noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, minSamples, maxSamples, paths.get(0).getPreviewStartingHolonomicPose())
                            .onlyWhile(noteAssistance::hasTarget)
                    ),
                    Commands.none(), 
                    noteAssistance::hasTarget
                ),

                AutoBuilder.followPath(paths.get(0)),
                Commands.either(
                    Commands.sequence(
                        superSystem.intakeUntilSensedAuto(2),
                        noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, minSamples, maxSamples, paths.get(1).getPreviewStartingHolonomicPose())
                            .onlyWhile(noteAssistance::hasTarget)
                    ),
                    Commands.none(), 
                    noteAssistance::hasTarget
                ),

                AutoBuilder.followPath(paths.get(1)),
                Commands.either(
                    Commands.sequence(
                        superSystem.intakeUntilSensedAuto(2),
                        noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, minSamples, maxSamples, swerve.getPose())
                            .onlyWhile(noteAssistance::hasTarget)
                    ),
                    Commands.none(), 
                    noteAssistance::hasTarget
                )

            ).until(superSystem::noteIntook)
        );
    }

}
