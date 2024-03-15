package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
            Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            Commands.either(
                driverAssist.InitPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPose), 0, 10),
                driverAssist.InitPoseByVision(swerve, startingPose, 0, 10),
                RobotContainer::IsRedSide
            ),
            
            // Preload
            Commands.deadline(
                Commands.race(
                    Commands.waitUntil(() -> !superSystem.colorSensor.noteIntook()).andThen(Commands.waitSeconds(0.2)),
                    Commands.waitSeconds(1.5)
                ),
                superSystem.shootSubwoofer()
            ),

            Commands.runOnce(() -> SmartDashboard.putNumber("Mid3Piece", 1)),

            /************************************ NOTE ONE **************************************/

            // Grab First Note
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    // superSystem.stow(),
                    Commands.waitSeconds(2),
                    superSystem.intakeUntilSensed()
                )
            ),

            Commands.runOnce(() -> SmartDashboard.putNumber("Mid3Piece", 2)),

            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(1)), 
                // superSystem.stow(),
                Commands.waitSeconds(4)
            ),

            // Reset odometry
            Commands.either(
                driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(podiumPoseBlue), 0, 8), //change april tag id ltr
                driverAssist.resetOdoPoseByVision(swerve, podiumPoseBlue, 0, 8), //change april tag id ltr
                RobotContainer::IsRedSide
            ),

            // Commands.runOnce(() -> SmartDashboard.putNumber("Mid3Piece", 3)),

            // // Make sure swerve is in the right pos
            // driveToPose(podiumPoseBlue),  

            Commands.runOnce(() -> SmartDashboard.putNumber("Mid3Piece", 3)),

            // Turn towards tag                      
            Commands.deadline(
                Commands.waitSeconds(2),
                Commands.either(
                    // driverAssist.aimToApriltagCommand(swerve, 4, 10, 20),
                    // driverAssist.aimToApriltagCommand(swerve, 7, 10, 20),
                    driverAssist.turnToTag(4, swerve),
                    driverAssist.turnToTag(7, swerve),
                    RobotContainer::IsRedSide 
                )
            ),

            Commands.runOnce(() -> SmartDashboard.putNumber("Mid3Piece", 4)),

            // Shoot
            Commands.deadline(
                Commands.waitSeconds(1.2),
                superSystem.shootPodium()
                // superSystem.shootSequenceAdjustable(sva)
            ),

            Commands.runOnce(() -> SmartDashboard.putNumber("Mid3Piece", 5)),

            /************************************ NOTE TWO **************************************/

            // Grab note
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(2)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    // superSystem.stow(),
                    Commands.waitSeconds(2),
                    superSystem.intakeUntilSensed()
                )
            ),
            
            // Come back
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(3)),
                // superSystem.stow(),
                Commands.waitSeconds(4)
            ),

            // Reset odometry
            Commands.either(
                driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(podiumPoseBlue), 0, 8), //change april tag id ltr
                driverAssist.resetOdoPoseByVision(swerve, podiumPoseBlue, 0, 8), //change april tag id ltr
                RobotContainer::IsRedSide
            ),

            // Commands.runOnce(() -> SmartDashboard.putNumber("Phase", 1)),
            
            // // Make sure swerve is in the right pos
            // driveToPose(podiumPoseBlue),  

            // Turn towards tag                      
            Commands.deadline(
                Commands.waitSeconds(2),
                Commands.either(
                    // driverAssist.aimToApriltagCommand(swerve, 4, 10, 20),
                    // driverAssist.aimToApriltagCommand(swerve, 7, 10, 20),
                    driverAssist.turnToTag(4, swerve),
                    driverAssist.turnToTag(7, swerve),
                    RobotContainer::IsRedSide 
                )
            ),

            // Shoot
            Commands.deadline(
                Commands.waitSeconds(1.2),
                superSystem.shootPodium()
                // superSystem.shootSequenceAdjustable(sva)
            ),

            /************************************ LEAVE **************************************/

            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(4)),
                superSystem.stow()
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
