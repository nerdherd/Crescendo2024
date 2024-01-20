// package frc.robot.commands.autos;

// import java.util.List;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.commands.PathfindHolonomic;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;

// public class SquareTest extends SequentialCommandGroup {
//     public SquareTest(SwerveDrivetrain swerve) {        
//         List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("TestSquare");
//         Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("TestSquare");

//         addCommands(
//             Commands.runOnce(swerve.getImu()::zeroAll),
//             Commands.runOnce(() -> swerve.getImu().setOffset(0)),
//             Commands.runOnce(() -> swerve.setPoseMeters(startingPose)),
//             AutoBuilder.followPath(pathGroup.get(0)),
//             AutoBuilder.followPath(pathGroup.get(1)),
//             AutoBuilder.followPath(pathGroup.get(2)),
//             AutoBuilder.followPath(pathGroup.get(3))
//         );
//     }
    
// }
