package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Test2M extends SequentialCommandGroup {
    public Test2M(SwerveDrivetrain swerve) {        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Test2M");
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Test2M");

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(0)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            AutoBuilder.followPath(pathGroup.get(0)),
            Commands.runOnce(() -> swerve.towModules())
        );
    }
    
}
