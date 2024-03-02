package frc.robot.commands.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;


public class Mid3PieceWithVisionShooting extends SequentialCommandGroup {
    public Mid3PieceWithVisionShooting(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        // Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        Pose2d startingPoseBlue = new Pose2d(0.71, 4.38, new Rotation2d());
        Pose2d shootingPoseBlue = new Pose2d(1.34, 5.56, new Rotation2d());
    

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            // Commands.runOnce(()-> tagCam.resetInitPoseByVision(swerve, startingPose, 4, 4)), // will add it back later

            // Commands.waitSeconds(2), // debug time

            driverAssist.InitPoseByVision(swerve, startingPoseBlue, 0, 50),
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSequence2()
                ),

                // Grab Note
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    Commands.waitSeconds(1),
                    superSystem.intakeBasicHold()
                ),

                // Come back
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(1)), 
                    superSystem.stow()
                    ),

                driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, true), //change april tag id ltr
                    
                Commands.parallel(
                    PathCurrentToDest(shootingPoseBlue),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        superSystem.shootSequence2()
                    )
                ),

                // Grab note
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    Commands.waitSeconds(1),    
                    superSystem.intakeBasicHold()
                ),
                // Come back
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    superSystem.stow()

                    ),

                driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, true), //change april tag id ltr

                Commands.parallel(
                    PathCurrentToDest(shootingPoseBlue),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        superSystem.shootSequence2()
                    )
                ),
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(4))
                )
            )
        );
    }

    
    PathConstraints pathcons = new PathConstraints(
        3, 3, 
        Units.degreesToRadians(180), Units.degreesToRadians(360)
    );

    public Command PathCurrentToDest(Pose2d destPoseInBlue)
    {
        if (RobotContainer.IsRedSide()) {
            destPoseInBlue = (GeometryUtil.flipFieldPose(destPoseInBlue));
        } 

        return AutoBuilder.pathfindToPose(
            destPoseInBlue, pathcons
        );
    }
}
