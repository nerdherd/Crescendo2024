package frc.robot.commands.autos.PathVariants;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class PathC extends SequentialCommandGroup {
    public PathC(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup){
        addCommands(
            Commands.parallel(
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.shooterPivot.moveToHandoff()
                ),
                // Drive in front of mid note
                Commands.deadline(
                    Commands.waitSeconds(2), // TODO: Find a working time
                    swerve.driveToPose(new Pose2d(6, 6.5, new Rotation2d()), 5, 5)
                )
            )
        );
    }
}
