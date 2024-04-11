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
        Pose2d startingPose = new Pose2d(2.9, 5.5, new Rotation2d());//pathGroup.get(0).();
        addCommands(
            Commands.runOnce(() -> swerve.getImu().zeroAll()),
            Commands.runOnce(() -> swerve.getImu().setOffset((startingPose.getRotation().getDegrees()))),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.parallel(
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.shooterPivot.moveToHandoff()
                ),
                // Drive in front of mid note
                Commands.race(
                    Commands.waitSeconds(5), // TODO: Find a working time
                    // swerve.driveToPose(new Pose2d(6, 6.5, new Rotation2d()), 5, 5)
                    AutoBuilder.followPath(pathGroup.get(0)).andThen(Commands.waitSeconds(1)),
                    Commands.sequence(
                        Commands.waitSeconds(2),
                        superSystem.intakeUntilSensedAuto(2.875)
                    ),
                    Commands.waitUntil(superSystem::noteIntook)  
                ),
                Commands.runOnce(() -> swerve.towModules())
            )
        );
    }
}
