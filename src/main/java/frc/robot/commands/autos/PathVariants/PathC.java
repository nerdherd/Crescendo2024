package frc.robot.commands.autos.PathVariants;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class PathC extends SequentialCommandGroup {
    public PathC(SuperSystem superSystem, SwerveDrivetrain swerve, List<PathPlannerPath> pathGroup, int pathIndex){
        addCommands(
            Commands.sequence(
                // C Start here ******************************************************

                // Drive in front of mid note
                Commands.deadline(
                    AutoBuilder.followPath(pathGroup.get(pathIndex)),
                    Commands.waitSeconds(3) // TODO: Find a working time
                )
            )
        );
    }
}
