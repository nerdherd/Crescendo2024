package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;


public class Mid3Piece extends SequentialCommandGroup {
    public Mid3Piece(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist, ShooterVisionAdjustment sva) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        // from auto file
        Pose2d podiumPoseBlue = new Pose2d(2.60, 3.74, Rotation2d.fromDegrees(-59.30));
    

        addCommands(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            driverAssist.resetOdoPoseByVision(swerve, null, 0, 10),
            
            // Preload
            Commands.deadline(
                Commands.race(
                    Commands.waitUntil(() -> !superSystem.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitSeconds(1.5)
                ),
                superSystem.shootSubwoofer()
            ),

            /************************************ NOTE ONE **************************************/

            // Grab First Note
            Commands.race(
                AutoBuilder.followPath(pathGroup.get(0)).andThen(Commands.waitSeconds(0.5)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.intakeUntilSensedAuto(3)
                ),
                Commands.waitUntil(superSystem::noteIntook)
            ),

            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(1)), 
                Commands.sequence(
                    superSystem.stow(),
                    superSystem.backupIndexer(),
                    superSystem.prepareShooterPodium()
                ),                
                Commands.waitSeconds(3.5)
            ),

            // Reset odometry
            driverAssist.resetOdoPoseByVision(swerve, null, 0, 8), //change april tag id ltr

            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    Commands.either(
                        driverAssist.turnToTag(4, swerve),
                        driverAssist.turnToTag(7, swerve),
                        RobotContainer::IsRedSide 
                    )
                ),
                Commands.deadline(
                    Commands.waitSeconds(2),
                    // superSystem.shootPodium()
                    superSystem.shootSequenceAdjustableAuto(sva)
                )
            ),

            /************************************ NOTE TWO **************************************/

            // Grab note
            Commands.race(
                AutoBuilder.followPath(pathGroup.get(2)).andThen(Commands.waitSeconds(0.5)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.intakeUntilSensedAuto(3)
                ),
                Commands.waitUntil(superSystem::noteIntook)
            ),
            
            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(3)),
                Commands.sequence(
                    superSystem.stow(),
                    superSystem.backupIndexer(),
                    superSystem.prepareShooterPodium()
                ),
                Commands.waitSeconds(3.5)
            ),

            // Reset odometry
            driverAssist.resetOdoPoseByVision(swerve, null, 0, 8), //change april tag id ltr

            Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    Commands.either(
                        driverAssist.turnToTag(4, swerve),
                        driverAssist.turnToTag(7, swerve),
                        RobotContainer::IsRedSide 
                    )
                ),
                Commands.deadline(
                    Commands.waitSeconds(2),
                    superSystem.shootSequenceAdjustableAuto(sva)
                )
            ),

            /************************************ LEAVE **************************************/

            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(4)),
                superSystem.stow()
            )
        );
    }
}
