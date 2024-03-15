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


public class Mid3PieceAlternate extends SequentialCommandGroup {
    public Mid3PieceAlternate(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist, ShooterVisionAdjustment sva) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPoseBlue = new Pose2d(0.71, 4.38, Rotation2d.fromDegrees(-39.89));//PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        Pose2d firstPickBlue = new Pose2d(7.93, 0.77, Rotation2d.fromDegrees(0));
        Pose2d secondPickBlue = new Pose2d(7.95, 2.44, Rotation2d.fromDegrees(0));
        Pose2d thirdPickBlue = new Pose2d(7.94, 4.11, Rotation2d.fromDegrees(0));

        Pose2d shootingPoseSourceBlue = new Pose2d(2.6, 3.74, Rotation2d.fromDegrees(-30)); 
        Pose2d shootingPoseCenterBlue = new Pose2d(5.55, 3.74, Rotation2d.fromDegrees(0)); // wrong pose, to be changed
        Pose2d shootingPoseAmpBlue = new Pose2d(6.25, 3.74, Rotation2d.fromDegrees(30));// wrong pose, to be changed

        // 1. find all pose value above
        // 2. disable all vision features, run code on the cart for red and blue side
        // 3. on the cart, front the speaker, enable and test the vision init function
        // 4. enable next vision function and test
        // 5. tune the shooting pid and samples at the shooting position
        // 6. run all
    

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            //Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPoseBlue)),
            Commands.either(
                driverAssist.InitPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 10), 
                driverAssist.InitPoseByVision(swerve, startingPoseBlue, 0, 10), 
                RobotContainer::IsRedSide
            ),
            
            // Preload
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                superSystem.shootSubwoofer()
            ),
            Commands.sequence(
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.setVelocityCommand(0, 0)
            ), 

            driveToPose(shootingPoseSourceBlue), // not shoot, by pass

            // Grab Note
            Commands.deadline(
                driveToPose(firstPickBlue),
                Commands.waitSeconds(3),
                Commands.sequence(
                    Commands.waitSeconds(2),
                    superSystem.intakeUntilSensed()
                )
            ),
                                    
            Commands.parallel(
                driveToPose(shootingPoseSourceBlue),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    Commands.either(
                        driverAssist.aimToApriltagCommand(swerve, 4, 6, 50), 
                        driverAssist.aimToApriltagCommand(swerve, 7, 6, 50), 
                        RobotContainer::IsRedSide
                    ),
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
/* 
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
            ),
*/
            Commands.none()
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
