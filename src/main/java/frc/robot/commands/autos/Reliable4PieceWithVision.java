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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;

public class Reliable4PieceWithVision extends SequentialCommandGroup {
    public Reliable4PieceWithVision(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist) {     
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        //Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        Pose2d startingPoseBlue = new Pose2d(1.33, 5.55, new Rotation2d());
        Pose2d FirstPickPose2d = new Pose2d(2.53, 4.3, new Rotation2d(Units.degreesToRadians(-20)));
        Pose2d SecondPickPoseBlue = new Pose2d(2.95, 5.55, new Rotation2d());
        Pose2d ThirdPickPoseBlue = new Pose2d(2.85, 6.96, new Rotation2d(Units.degreesToRadians(26)));
        
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.either(
                driverAssist.InitPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 10), 
                driverAssist.InitPoseByVision(swerve, startingPoseBlue, 0, 10), 
                RobotContainer::IsRedSide
            ),

            Commands.sequence(
                
                // Preload 
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSubwoofer()
                ),
                Commands.sequence(
                    superSystem.indexer.stopCommand(),
                    superSystem.shooterRoller.setVelocityCommand(0, 0)
                ), 

                // Piece 1
                Commands.deadline(
                    Commands.waitSeconds(1.75),
                    AutoBuilder.followPath(pathGroup.get(0)),
                    superSystem.intakeUntilSensed()
                ),

                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    Commands.sequence(
                        Commands.waitSeconds(0.85),
                        Commands.deadline(
                            Commands.waitSeconds(0.6),
                            superSystem.shootSubwooferAuto()  
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(0, 0)
                    )
                ),

                Commands.either(
                    driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 20), 
                    driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, 20), 
                    RobotContainer::IsRedSide
                ),

/* 
                // Piece 2
                Commands.deadline(
                    Commands.waitSeconds(1.75),
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeUntilSensed()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    Commands.sequence(
                        Commands.waitSeconds(0.85),
                        Commands.deadline(
                            Commands.waitSeconds(0.6),
                            superSystem.shootSequenceAuto()  
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(0, 0)
                    )
                ),

                Commands.either(
                    driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 20),
                    driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, 20), 
                    RobotContainer::IsRedSide
                ),     
                

                // Piece 3
                Commands.deadline(
                    Commands.waitSeconds(1.75),
                    AutoBuilder.followPath(pathGroup.get(4)),
                    superSystem.intakeUntilSensed()
                ),
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(5)),
                    Commands.sequence(
                        Commands.waitSeconds(0.85),
                        Commands.deadline(
                            Commands.waitSeconds(1),
                            superSystem.shootSequenceAuto()  
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(0, 0)
                    )
                ),*/

                Commands.none()
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
