package frc.robot.commands.autos.PathVariants;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;

public class PathF extends SequentialCommandGroup {
    public PathF(SwerveDrivetrain swerve, SuperSystem superSystem, PathPlannerPath path, DriverAssist driverAssist) {
        Pose2d startingPose = path.getPreviewStartingHolonomicPose();
        addCommands(
            Commands.runOnce(() -> swerve.getImu().zeroAll()),
            Commands.runOnce(() -> swerve.getImu().setOffset((startingPose.getRotation().getDegrees()))),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.sequence(
                // F Start here ******************************************************

                //Preload
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSubwoofer()
                ),

                // Stop shooter and indexer
                Commands.parallel(
                    superSystem.indexer.stopCommand(),
                    superSystem.shooterRoller.stopCommand()
                ), 

                // Drive in front of mid note
                Commands.deadline(
                    AutoBuilder.followPath(path),
                    Commands.waitSeconds(4.5) // TODO: Find a working time
                )
            )
        );
    }
}
