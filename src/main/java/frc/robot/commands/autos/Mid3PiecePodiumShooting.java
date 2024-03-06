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
import frc.robot.subsystems.vision.ShooterVisionAdjustment;


public class Mid3PiecePodiumShooting extends SequentialCommandGroup {
    public Mid3PiecePodiumShooting(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist, ShooterVisionAdjustment sva) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        Pose2d startingPoseBlue = new Pose2d(0.71, 4.38, Rotation2d.fromDegrees(-39.89));
        Pose2d shootingPoseBlue = new Pose2d(2.6, 3.74, Rotation2d.fromDegrees(86.43));
    

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            // Commands.runOnce(()-> tagCam.resetInitPoseByVision(swerve, startingPose, 4, 4)), // will add it back later

            // Commands.waitSeconds(2), // debug time
            // Commands.either(
            //     driverAssist.InitPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 100), //change april tag id ltr
            //     driverAssist.InitPoseByVision(swerve, startingPoseBlue, 0, 100), //change april tag id ltr
            //     RobotContainer::IsRedSide
            // ),
            
            // Preload
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.colorSensor.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                superSystem.shootSubwoofer()
            ),

            
            Commands.sequence(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.setVelocityCommand(0, 0)
            ), 

            // Grab Note
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(3),
                Commands.sequence(
                    Commands.waitSeconds(2),
                    superSystem.intakeUntilSensed()
                )
            ),

            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(1)), 
                superSystem.stow(),
                Commands.waitSeconds(3)
            ),

            Commands.either(
                driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 100), //change april tag id ltr
                driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, 100), //change april tag id ltr
                RobotContainer::IsRedSide
            ),
                                    
            Commands.parallel(
                driveToPose(shootingPoseBlue),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    driverAssist.resetOdoPoseByVision(swerve, swerve::getPose, 0, 3),
                    Commands.deadline(
                        Commands.waitSeconds(1),
                        superSystem.shootSequenceAdjustable(sva)
                    )
                )
            ),
            Commands.sequence(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.setVelocityCommand(0, 0)
            ), 

            // Grab note
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(2)),
                Commands.waitSeconds(1),
                Commands.sequence(
                    Commands.waitSeconds(2),
                    superSystem.intakeUntilSensed()
                )
            ),
            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(3)),
                superSystem.stow()

            ),

            Commands.either(
                driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 100), //change april tag id ltr
                driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, 100), //change april tag id ltr
                RobotContainer::IsRedSide
            ),
                
            Commands.parallel(
                
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    driverAssist.resetOdoPoseByVision(swerve, swerve::getPose, 0, 3),
                    superSystem.shootSequenceAdjustable(sva)
                )
            ),

            Commands.sequence(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.setVelocityCommand(0, 0)
            ), 

            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(4))
            )
        );
    }

    
    PathConstraints pathcons = new PathConstraints(
        3, 3, 
        Units.degreesToRadians(180), Units.degreesToRadians(360)
    );

    public Command driveToPose(Pose2d destPoseInBlue) {
        return Commands.either(
            AutoBuilder.pathfindToPose(GeometryUtil.flipFieldPose(destPoseInBlue), pathcons),
            AutoBuilder.pathfindToPose(destPoseInBlue, pathcons),
            RobotContainer::IsRedSide  
        );
    }
}
