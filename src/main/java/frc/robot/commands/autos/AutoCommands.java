package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class AutoCommands extends SequentialCommandGroup{

    public void PathA(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, ShooterVisionAdjustment sva, String pathName, int pathIndex){
        // Pose2d startingPose = new Pose2d(1.33, 5.55, new Rotation2d());//pathGroup.get(0).();
        Pose2d startingPose = PathPlannerPath.fromPathFile(pathName).getStartingDifferentialPose();
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            Commands.sequence(
                // Preload
                Commands.deadline(
                    Commands.waitSeconds(1.5),
                    superSystem.shootSubwoofer()
                ),

                // Stop shooter and indexer
                Commands.parallel(
                    superSystem.indexer.stopCommand(),
                    superSystem.shooterRoller.stopCommand()
                ),

                // Drive and intake
                Commands.race(
                    Commands.waitSeconds(2),
                    // Path
                    AutoBuilder.followPath(pathGroup.get(pathIndex)).andThen(Commands.waitSeconds(0.75)),
                    // Drive to note (if wanted)
                    // noteAssistance.driveToNoteCommand(swerve, 0, 0, 0, 0, 0, startingPose) // TODO: change target values
                    // Intake
                    Commands.sequence(
                        Commands.waitSeconds(0.125),
                        superSystem.intakeUntilSensedAuto(1.75)
                    )
                ),

                // Turn to angle and shoot
                Commands.sequence(
                    // Turn to angle
                    Commands.deadline(
                        Commands.waitSeconds(0.4),
                        Commands.either(
                            driverAssist.turnToTag(4, swerve),
                            driverAssist.turnToTag(7, swerve),
                            RobotContainer::IsRedSide 
                          )
                    ),
                    // Shoot
                    Commands.sequence(
                        superSystem.backupIndexerAndShooter(),
                        Commands.waitSeconds(0.45),
                        Commands.deadline(
                            Commands.waitSeconds(1.2),
                            // Adjust shooter Angle
                            superSystem.shootSequenceAdjustable(sva)
                        ),
                        superSystem.indexer.stopCommand(),
                        superSystem.shooterRoller.setVelocityCommand(-10, -10),
                        superSystem.shooterRoller.setEnabledCommand(true)
                    )
                )
            ),
            Commands.none()
        );
    }

    public void PathB(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, ShooterVisionAdjustment sva, String pathName, int pathIndex) {
        // Pose2d startingPose = pathGroup.get(0).getPreviewStartingHolonomicPose();
        Pose2d startingPose = PathPlannerPath.fromPathFile(pathName).getStartingDifferentialPose();
        addCommands(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // B Start here *******************************************************************************
            // Intake and Path
            Commands.race(
                Commands.waitSeconds(2),
                AutoBuilder.followPath(pathGroup.get(pathIndex)).andThen(Commands.waitSeconds(0.75)),
                Commands.sequence(
                    Commands.waitSeconds(0.125),
                    superSystem.intakeUntilSensedAuto(1.75)
                )                
            ),

            // Turn to Angle Shoot
            Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(0.4),
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
    
    public void PathC(SwerveDrivetrain swerve, List<PathPlannerPath> pathGroup, String pathName, int pathIndex){
        Pose2d startingPose = PathPlannerPath.fromPathFile(pathName).getStartingDifferentialPose();
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
