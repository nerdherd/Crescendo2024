package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.BannerSensor;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.NoteAssistance;

public class AutoMidNotes extends SequentialCommandGroup {
    public AutoMidNotes(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, NoteAssistance noteAssistance, DriverAssist tagAssist, BannerSensor sensor) {
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),

            superSystem.shootSubwoofer(), // Preload shot
            AutoBuilder.followPath(pathGroup.get(0)), // First path to note 
            new ConditionalCommand(new TurnToAngle(0, swerve), Commands.none(), () -> noteAssistance.hasTarget()), // go to 0 angle
            // TODO: add conditional to scan area if not seen
            new ConditionalCommand(
                    Commands.sequence(
                        superSystem.intakeBasic(), // intake position and start intake
                        noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose), // TODO: change target values
                        superSystem.stopIntaking(), // stop intake
                        new ConditionalCommand(
                                Commands.sequence(
                                    AutoBuilder.followPath(pathGroup.get(1)), // Go to speaker
                                    tagAssist.resetOdoPoseByVision(swerve, swerve.getPose(), 0, false), //TODO: change target tag
                                    superSystem.shootSubwoofer(), // Shoot
                                    AutoBuilder.followPath(pathGroup.get(0)) // Drive back to the middle(note1)
                                )
                            , 
                                Commands.none()
                            , 
                            () -> sensor.noteIntook()
                        )
                    )
                ,
                    // If no target wait for movement to next spot
                    Commands.none()
                , 
                () -> noteAssistance.hasTarget()
            ),
            AutoBuilder.followPath(pathGroup.get(2)), // Go in front of note 2
            tagAssist.resetOdoPoseByVision(swerve, swerve.getPose(), 0, false), //TODO: change target tag
            new ConditionalCommand(new TurnToAngle(0, swerve), Commands.none(), () -> noteAssistance.hasTarget()), // go to 0 angle
            // TODO: add conditional to scan area if not seen
            new ConditionalCommand(
                    Commands.sequence(
                        superSystem.intakeBasic(), // start intake
                        noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose), // TODO: change target values
                        superSystem.stopIntaking(), // stop intake
                        new ConditionalCommand(
                                Commands.sequence(
                                    AutoBuilder.followPath(pathGroup.get(5)), // Go back to note 1 to be clear of stage
                                    AutoBuilder.followPath(pathGroup.get(1)), // Go to speaker
                                    tagAssist.resetOdoPoseByVision(swerve, swerve.getPose(), 0, false), //TODO: change target tag
                                    superSystem.shootSubwoofer(), // Shoot
                                    AutoBuilder.followPath(pathGroup.get(0)) // Drive back to the middle(note1)
                                )
                            , 
                                Commands.none()
                            , 
                            () -> sensor.noteIntook()
                        )
                    )
                , 
                    Commands.none() 
                ,
                () -> noteAssistance.hasTarget()
            ),
            AutoBuilder.followPath(pathGroup.get(3)), // Go in front of note 3
            new ConditionalCommand(new TurnToAngle(0, swerve), Commands.none(), () -> noteAssistance.hasTarget()), // go to 0 angle
            // TODO: add conditional to scan area if not seen
            new ConditionalCommand(
                Commands.sequence(
                    superSystem.intakeBasic(), // Intake
                    noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose), // Use vision to drive to note
                    superSystem.stopIntaking(), // stop intake
                    new ConditionalCommand(
                        Commands.sequence(
                            AutoBuilder.followPath(pathGroup.get(5)), // Go back to note 1 to be clear of stage
                            AutoBuilder.followPath(pathGroup.get(1)), // Go to speaker
                            tagAssist.resetOdoPoseByVision(swerve, swerve.getPose(), 0, false), //TODO: change target tag
                            superSystem.shootSubwoofer(), // Shoot
                            AutoBuilder.followPath(pathGroup.get(0)) // Drive back to the middle(note1)
                        )
                    , 
                        Commands.none()
                    , 
                    () -> sensor.noteIntook()
                )
                )
                ,
                    // Wait for next path
                    Commands.none() 
                , 
                () -> noteAssistance.hasTarget()
            ),
            AutoBuilder.followPath(pathGroup.get(4)) // Go in front of note 4
        );
    }
}
