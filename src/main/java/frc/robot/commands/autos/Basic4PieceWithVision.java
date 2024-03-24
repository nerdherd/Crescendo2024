package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;

public class Basic4PieceWithVision extends SequentialCommandGroup {
    public Basic4PieceWithVision(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist driverAssist) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetInitPoseByVision()),
            Commands.sequence(
                // Preload 
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSubwoofer()
                ),

                // Piece 1
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    superSystem.intakeDirectShoot(ShooterConstants.k4PieceHandoffPosition1.get(), 
                                                  ShooterConstants.kTopOuttakeAuto1.get(), 
                                                  ShooterConstants.kBottomOuttakeAuto1.get())
                ),
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    superSystem.intakeDirectShoot(ShooterConstants.k4PieceHandoffPosition1.get(), 
                                                  ShooterConstants.kTopOuttakeAuto1.get(), 
                                                  ShooterConstants.kBottomOuttakeAuto1.get())
                ),

                // Piece 2
                // TODO: Change AprilTag ID based on alliance
                driverAssist.resetOdoPoseByVision(swerve, startingPose, 4, true), //change april tag id ltr
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeDirectShoot(ShooterConstants.k4PieceHandoffPosition2.get(), 
                                                  ShooterConstants.kTopOuttakeAuto2.get(), 
                                                  ShooterConstants.kBottomOuttakeAuto2.get())
                ),
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    superSystem.intakeDirectShoot(ShooterConstants.k4PieceHandoffPosition2.get(), 
                                                  ShooterConstants.kTopOuttakeAuto2.get(), 
                                                  ShooterConstants.kBottomOuttakeAuto2.get())
                ),

                // Piece 3
                // TODO: Change AprilTag ID based on alliance
                driverAssist.resetOdoPoseByVision(swerve, startingPose, 4, true), 
                Commands.deadline(
                    Commands.sequence(
                        AutoBuilder.followPath(pathGroup.get(4)),
                        Commands.waitSeconds(2)
                    ),
                    superSystem.intakeDirectShoot(ShooterConstants.k4PieceHandoffPosition3.get(), 
                                                  ShooterConstants.kTopOuttakeAuto3.get(), 
                                                  ShooterConstants.kBottomOuttakeAuto3.get())
                )
            )
            );
    }
}
