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

public class AutoMid3Notes extends SequentialCommandGroup {
    public AutoMid3Notes(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, NoteAssistance noteAssistance) {
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),

            //first path to note
            AutoBuilder.followPath((pathGroup.get(0))),
            //note 1
            new ConditionalCommand(
                    //if target, go to actual logic
                    Commands.none()
                ,
                    //if no target, turn angle a little bit to check for note
                    Commands.none() //seek note or something
                , 
                () -> noteAssistance.hasTarget()
            ),
            new ConditionalCommand(
                    noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose) //change target values
                    //TODO: path to shoot position
                    //TODO: shoot
                    //TODO: path second position from shoot position
                , 
                    //TODO: go to second position from first position
                    AutoBuilder.followPath(null) //placeholder command so no error
                ,
                () -> noteAssistance.hasTarget()
            ),

            //note 2
            new ConditionalCommand(
                    //if target, go to actual logic
                    Commands.none()
                ,
                    //if no target, turn angle a little bit to check for note
                    Commands.none() //seek note or something
                , 
                () -> noteAssistance.hasTarget()
            ),
            new ConditionalCommand(
                    noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose) //change target values
                    //TODO: go to shoot position
                    //TODO: shoot
                    //TODO: go to third position from shoot position
                , 
                    //TODO: go to third position from secondp position
                    AutoBuilder.followPath(null) //placeholder command so no error
                ,
                () -> noteAssistance.hasTarget()
            ),

            //note 3
            new ConditionalCommand(
                    //if target, go to actual logic
                    Commands.none()
                ,
                    //if no target, turn angle a little bit to check for note
                    Commands.none() //seek note or something
                , 
                () -> noteAssistance.hasTarget()
            ),
            new ConditionalCommand(
                    noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose) //change target values
                    //TODO: go to shoot position
                    //TODO: shoot
                    //TODO: go to fourth position from shoot position
                , 
                    //TODO: go to fourth position right away from third position
                    AutoBuilder.followPath(null) //placeholder command so no error
                ,
                () -> noteAssistance.hasTarget()
            ),

            //note 4
            new ConditionalCommand(
                    //if target, go to actual logic
                    Commands.none()
                ,
                    //if no target, turn angle a little bit to check for note
                    Commands.none() //seek note or something
                , 
                () -> noteAssistance.hasTarget()
            ),
            new ConditionalCommand(
                    noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose) //change target values
                    //TODO: go to shoot position
                    //TODO: shoot
                    //TODO: go to end position from shoot position
                , 
                    //TODO: go to end position right away
                    AutoBuilder.followPath(null) //placeholder command so no error
                ,
                () -> noteAssistance.hasTarget()
            ),


            Commands.none()
        );
    }
}
