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
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class PathF extends SequentialCommandGroup {
    public PathF(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, NoteAssistance na, ShooterVisionAdjustment sva) {
        Pose2d startingPose = new Pose2d(0.70, 4.37, Rotation2d.fromDegrees(-60));//pathGroup.get(0).();
        addCommands(
            Commands.parallel(
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1),
                    superSystem.shooterPivot.moveToHandoff()
                ),
                // Drive in front of mid note
                Commands.deadline(
                    Commands.waitSeconds(2.5),
                    AutoBuilder.followPath(pathGroup.get(0))
                )
            ),

            Commands.deadline(
                Commands.waitUntil(superSystem::noteIntook),
                superSystem.intakeUntilSensedAuto(4),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    na.driveToNoteCommand(swerve, 15, 0, 0, 10, 50, null)
                )
            ),
            
            superSystem.backupIndexerAndShooter(),
            superSystem.stow()
        );
    }
}
