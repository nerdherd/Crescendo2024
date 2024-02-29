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
        Pose2d FirstPickPose2d;
        Pose2d SecondPickPoseBlue = new Pose2d(2.5, 5.55, new Rotation2d());;
        Pose2d ThirdPickPoseBlue = new Pose2d(2.65, 6.96, new Rotation2d(Units.degreesToRadians(26)));
        
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            //Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            //Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            driverAssist.InitPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 50),

            Commands.sequence(
                superSystem.intakePivot.setEnabledCommand(true),
                superSystem.intakePivot.moveToIntake(),
                
                // Preload 
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSequence2()
                ),

                // Piece 1
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    superSystem.intakeBasicHold()
                ),
                Commands.parallel(
                    driveToPose(startingPoseBlue),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        Commands.deadline(
                            Commands.waitSeconds(1),
                            superSystem.shootSequence2()
                        )
                    )
                ),
                Commands.either(
                    driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 100), //change april tag id ltr
                    driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, 100), //change april tag id ltr
                    RobotContainer::IsRedSide
                ),
                
                Commands.waitSeconds(0.5),

                // Piece 2
                // TODO: Change AprilTag ID based on alliance
                Commands.deadline(
                    driveToPose(SecondPickPoseBlue),                    
                    superSystem.intakeBasicHold()
                ),
                Commands.parallel(
                    driveToPose(startingPoseBlue),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        Commands.deadline(
                            Commands.waitSeconds(1),
                            superSystem.shootSequence2()
                        )
                    )
                ),

                Commands.either(
                    driverAssist.resetOdoPoseByVision(swerve, GeometryUtil.flipFieldPose(startingPoseBlue), 0, 100), //change april tag id ltr
                    driverAssist.resetOdoPoseByVision(swerve, startingPoseBlue, 0, 100), //change april tag id ltr
                    RobotContainer::IsRedSide
                ),                
                Commands.waitSeconds(0.5),

                // Piece 3
                Commands.deadline(
                    driveToPose(ThirdPickPoseBlue),
                    superSystem.intakeBasicHold()
                ),
                Commands.parallel(
                    driveToPose(startingPoseBlue),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.backupIndexer(),
                        Commands.waitSeconds(0.5),
                        Commands.deadline(
                            Commands.waitSeconds(1),
                            superSystem.shootSequence2()
                        )
                    )
                ),

                // Leave towards mid
                /*Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(6)),
                    superSystem.stow()
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
