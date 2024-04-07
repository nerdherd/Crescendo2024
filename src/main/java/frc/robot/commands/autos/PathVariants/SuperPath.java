package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class SuperPath extends SequentialCommandGroup{
    public SuperPath(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, ShooterVisionAdjustment sva) {
        Pose2d startingPose = new Pose2d(1.33, 5.55, new Rotation2d());//pathGroup.get(0).();
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),

            ///A
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
                    AutoBuilder.followPath(pathGroup.get(0)).andThen(Commands.waitSeconds(0.75)),
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
            // Commands.runOnce(swerve.getImu()::zeroAll),
            //Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            //Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // B (1) Start here *******************************************************************************
            // Intake and Path
            Commands.race(
                Commands.waitSeconds(2),
                AutoBuilder.followPath(pathGroup.get(1)).andThen(Commands.waitSeconds(0.75)),
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
            ),

            // B (2) Start here *******************************************************************************
            // Intake and Path
            Commands.race(
                Commands.waitSeconds(2),
                AutoBuilder.followPath(pathGroup.get(2)).andThen(Commands.waitSeconds(0.75)),
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

    // to be tested. Do not use it before test
    public static Pose2d GetEndPoseInPath(PathPlannerPath path)
    {
        PathPoint tail  = path.getPoint(path.numPoints()-1);
        double rad = tail.rotationTarget.getTarget().getRadians();
        return new Pose2d(tail.position, new Rotation2d(rad));
    } 
    public static Pose2d GetStartPoseInPath(PathPlannerPath path)
    {
        PathPoint tail  = path.getPoint(0);
        double rad = tail.rotationTarget.getTarget().getRadians();
        return new Pose2d(tail.position, new Rotation2d(rad));
    }
}
