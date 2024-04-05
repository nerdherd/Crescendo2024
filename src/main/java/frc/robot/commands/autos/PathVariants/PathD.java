package frc.robot.commands.autos.PathVariants;

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
    private boolean stop = false;
    private int i;
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
    //                     noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, 10, 50, new Pose2d())
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
     */
    public PathD(SwerveDrivetrain swerve, SuperSystem superSystem, NoteAssistance noteAssistance, double targetNoteArea, PathPlannerPath path1, PathPlannerPath path2) {
        addCommands(
            Commands.sequence(
                Commands.either(
                    Commands.sequence(
                        superSystem.intakeUntilSensedAuto(2),
                        noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, 10, 50, new Pose2d())
                            .onlyWhile(noteAssistance::hasTarget)
                    ),
                    Commands.none(), 
                    noteAssistance::hasTarget
                ),

                AutoBuilder.followPath(path1),
                Commands.either(
                    Commands.sequence(
                        superSystem.intakeUntilSensedAuto(2),
                        noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, 10, 50, swerve.getPose())
                            .onlyWhile(noteAssistance::hasTarget)
                    ),
                    Commands.none(), 
                    noteAssistance::hasTarget
                ),

                AutoBuilder.followPath(path2),
                Commands.either(
                    Commands.sequence(
                        superSystem.intakeUntilSensedAuto(2),
                        noteAssistance.driveToNoteCommand(swerve, targetNoteArea, 0, 0, 10, 50, new Pose2d())
                            .onlyWhile(noteAssistance::hasTarget)
                    ),
                    Commands.none(), 
                    noteAssistance::hasTarget
                )

            ).until(superSystem::noteIntook)
        );
    }

    /**
     * @return the path this part of the auto stopped at (ex. 1st path, 2nd path ...)
     */
    // public int getPathStoppedAt() { return i + 1; }
}
