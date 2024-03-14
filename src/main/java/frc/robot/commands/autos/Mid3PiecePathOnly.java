package frc.robot.commands.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;

public class Mid3PiecePathOnly extends SequentialCommandGroup {
    public Mid3PiecePathOnly(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem, DriverAssist tagCam) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("intake", superSystem.intakeBasic());

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            // Commands.runOnce(()-> tagCam.resetInitPoseByVision(swerve, startingPose, 4, 4)), // will add it back later
            // Commands.waitSeconds(2), // debug time
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSubwoofer()
                ),
                // Grab Note
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(0))
                ),
                // Come back
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(1))  
                ),
                // Grab note
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(2))
                ),
                // Come back
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(3))
                )
            )
        );
    }
}
