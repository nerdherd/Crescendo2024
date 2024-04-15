package frc.robot.commands.autos.PathVariants;

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

public class PathC extends SequentialCommandGroup{
    public PathC(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, int index) {
        // Pose2d startingPose = pathGroup.get(0).getPreviewStartingHolonomicPose();
        // Pose2d startingPose = new Pose2d(2.45, 5.55, new Rotation2d());//pathGroup.get(0).();
        addCommands(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // C Start here *******************************************************************************
            // PATH CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
            Commands.parallel(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1.5),
                    superSystem.shooterPivot.moveToHandoff(),
                    superSystem.shooterPivot.setEnabledCommand(true)
                )
            )
        );
    }    
}
