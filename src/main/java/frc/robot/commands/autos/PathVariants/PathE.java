package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class PathE extends SequentialCommandGroup {
    public PathE(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist, ShooterVisionAdjustment sva) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        // start of E ****************************************************************
        addCommands(
            Commands.sequence(
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(1)), //path group is anything
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
                            //turn to angle
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
                    // end of E *******************************************************
                )
            )
            );
    }
}

