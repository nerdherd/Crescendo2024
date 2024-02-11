package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class ExamplePath extends SequentialCommandGroup {
    public ExamplePath(SwerveDrivetrain swerve) {        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example Path");
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Example Path");

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(0)),
            Commands.runOnce(() -> swerve.setPoseMetersWithAlliance(startingPose)),
            AutoBuilder.followPath(pathGroup.get(0)),
            AutoBuilder.followPath(pathGroup.get(1))
        );
    }
    
}
