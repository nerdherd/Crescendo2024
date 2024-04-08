package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class PathB extends SequentialCommandGroup{
    public PathB(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, ShooterVisionAdjustment sva, int index) {
        // Pose2d startingPose = pathGroup.get(0).getPreviewStartingHolonomicPose();
        // Pose2d startingPose = new Pose2d(2.45, 5.55, new Rotation2d());//pathGroup.get(0).();
        addCommands(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            // Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // B Start here *******************************************************************************
            // Intake and Path
            Commands.race(
                Commands.waitSeconds(3),
                AutoBuilder.followPath(pathGroup.get(index)).andThen(Commands.waitSeconds(1.5)),
                Commands.sequence(
                    Commands.waitSeconds(0.125),
                    superSystem.intakeUntilSensedAuto(2.875)
                ),
                Commands.waitUntil(superSystem::noteIntook)         
            ),

            // Turn to angle and shoot
            Commands.deadline(
                Commands.waitUntil(() -> !superSystem.noteIntook()),
                Commands.sequence(
                    Commands.deadline(
                        // Turn to angle
                        Commands.sequence(
                            Commands.deadline(
                                Commands.waitSeconds(1),
                                Commands.either(
                                    driverAssist.turnToTag(4, swerve, 2),
                                    driverAssist.turnToTag(7, swerve, 2),
                                    RobotContainer::IsRedSide 
                                )
                            ),
                            Commands.runOnce(() -> swerve.towModules()),
                            superSystem.shootAuto()
                        ),
                        // Shoot
                        Commands.sequence(
                            superSystem.backupIndexerAndShooter(),
                            superSystem.prepareShooterVision(sva)
                        )
                    )
                )
            )
        );
    }    
}
