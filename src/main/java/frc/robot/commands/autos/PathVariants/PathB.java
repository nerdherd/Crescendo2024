package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class PathB extends SequentialCommandGroup{
    public PathB(SwerveDrivetrain swerve, List<PathPlannerPath> pathGroup, SuperSystem superSystem, DriverAssist driverAssist, ShooterVisionAdjustment sva) {
        Pose2d startingPose = pathGroup.get(0).getPreviewStartingHolonomicPose();
        addCommands(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // B Start here *******************************************************************************
            // Intake and Path
            Commands.race(
                Commands.waitSeconds(2),
                AutoBuilder.followPath(pathGroup.get(0)).andThen(Commands.waitSeconds(0.75)),
                Commands.sequence(
                    Commands.waitSeconds(0.125),
                    superSystem.intakeUntilSensedAuto(1.75)
                )                
            ),

            // Turn to Angle Shoot
            Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(2),
                    // adjust drive to april tag
                    Commands.either(
                        driverAssist.turnToTag(4, swerve),
                        driverAssist.turnToTag(7, swerve),
                        RobotContainer::IsRedSide 
                    )
                ),
                superSystem.backupIndexerAndShooter(),
                Commands.waitSeconds(0.45),
                Commands.deadline(
                    Commands.waitSeconds(1.4),
                    // adjust shooter angle
                    superSystem.shootSequenceAdjustable(sva)
                ),
                superSystem.indexer.stopCommand(),
                superSystem.shooterRoller.setVelocityCommand(-10, -10),
                superSystem.shooterRoller.setEnabledCommand(true)
            )
        );
    }    
}
