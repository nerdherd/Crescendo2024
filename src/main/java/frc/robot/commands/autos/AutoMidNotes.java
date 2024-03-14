package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class AutoMidNotes extends SequentialCommandGroup {
    public AutoMidNotes(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, NoteAssistance noteAssistance) {
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),

            // Preload shot
            // Turn on intake
            AutoBuilder.followPath(pathGroup.get(0)), // First path to note 1
            new ConditionalCommand(
                    Commands.sequence(
                        // Drive on note with vision
                        AutoBuilder.followPath(pathGroup.get(1)), // Go to speaker
                        // Shoot
                        AutoBuilder.followPath(pathGroup.get(0)) // Drive back to the middle
                    )
                ,
                    // If no target wait for movement to next spot
                    Commands.none()
                , 
                () -> noteAssistance.hasTarget()
            ),
            AutoBuilder.followPath(pathGroup.get(2)), // Go in front of note 2
            new ConditionalCommand(
                    Commands.sequence(
                        noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose), // TODO: change target values
                        AutoBuilder.followPath(pathGroup.get(5)), // Go back to note 1 to be clear of stage
                        AutoBuilder.followPath(pathGroup.get(1)), // Go to speaker
                        // shoot
                        AutoBuilder.followPath(pathGroup.get(0)) // Go to note 1 location to clear the stage
                    )
                , 
                    Commands.none() 
                ,
                () -> noteAssistance.hasTarget()
            ),
            AutoBuilder.followPath(pathGroup.get(3)), // Go in front of note 3
            new ConditionalCommand(
                Commands.sequence(
                    // Intake
                    // Use vision to drive to note
                    AutoBuilder.followPath(pathGroup.get(5)), // Go back to note 1 to be clear of stage
                    AutoBuilder.followPath(pathGroup.get(1)), // Go to speaker
                    // shoot
                    AutoBuilder.followPath(pathGroup.get(0)) // Go to note 1 location to clear the stage
                )
                ,
                    // Wait for next path
                    Commands.none() 
                , 
                () -> noteAssistance.hasTarget()
            ),
            AutoBuilder.followPath(pathGroup.get(4)), // // Go in front of note 4
            new ConditionalCommand(
                Commands.sequence(
                    noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose), //change target values
                    AutoBuilder.followPath(pathGroup.get(5)), // Go back to note 1 to be clear of stage
                    AutoBuilder.followPath(pathGroup.get(1)), // Go to speaker
                    // shoot
                    AutoBuilder.followPath(pathGroup.get(0)) // Go to note 1 location to clear the stage
                )
                , 
                    Commands.none()
                ,
                () -> noteAssistance.hasTarget()
            )
        );
    }
}
