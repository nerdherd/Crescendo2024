package frc.robot.commands.autos.PathVariants;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

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
    
    // to be tested. Do not use it before test
    
    public static Pose2d GetStartPoseInPath(PathPlannerPath path)
    {
        PathPoint tail  = path.getPoint(0);
        RotationTarget rt = tail.rotationTarget;
        double rad;
        if (rt == null) {
            rad  = 0;
        }
        else {
            rad = tail.rotationTarget.getTarget().getRadians();
        }
        return new Pose2d(tail.position, new Rotation2d(rad));
    }
    public static Pose2d GetEndPoseInPath(PathPlannerPath path)
    {
        PathPoint tail  = path.getPoint(path.numPoints()-1);
        double rad = tail.rotationTarget.getTarget().getRadians();
        return new Pose2d(tail.position, new Rotation2d(rad));
    } 

    public PathC(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup){
        Pose2d startingPose = GetStartPoseInPath(pathGroup.get(0));
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            
            Commands.sequence(
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    Commands.sequence(
                        superSystem.stow(),
                        Commands.waitSeconds(1.5),
                        superSystem.shooterPivot.moveToHandoff(),
                        superSystem.shooterPivot.setEnabledCommand(true)
                    )
                ),
                
                // this is for PathD
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    Commands.deadline(
                        Commands.waitUntil(superSystem::noteIntook),
                        superSystem.intakeUntilSensedAuto(2.875)
                    )
                ),

                // PATH EEEEEEEEE
                Commands.sequence(
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    Commands.sequence(
                        superSystem.stow(),
                        Commands.waitSeconds(.9), // regualr path uses 2.4; shorter one uses .9
                        superSystem.shooterPivot.moveToHandoff(),
                        superSystem.shooterPivot.setEnabledCommand(true),

                        Commands.deadline(
                            Commands.waitUntil(() -> !superSystem.noteIntook()),
                            Commands.parallel(
                                superSystem.shootSubwooferAutoStart(),
                                superSystem.intakeRoller.autoIntakeCommand()
                            )
                        ),
                        
                        superSystem.shooterRoller.setVelocityCommand(-10, -10),
                        superSystem.shooterRoller.setEnabledCommand(true)
                    )
                )
            ),

            // PATH LAST AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
            Commands.parallel(
                // Drive to note 123
                AutoBuilder.followPath(pathGroup.get(3)),
                Commands.sequence(
                        Commands.deadline(
                            Commands.waitUntil(superSystem::noteIntook),
                            superSystem.intakeUntilSensedAuto(2.875)
                        ),

                        Commands.deadline(
                            Commands.waitUntil(() -> !superSystem.noteIntook()),
                            Commands.parallel(
                                superSystem.shootSubwooferAutoStart(),
                                superSystem.intakeRoller.autoIntakeCommand()
                            )
                        )
                    )
                )
            ),

            //Commands.runOnce(() -> swerve.towModules()),
            superSystem.stow(),
            superSystem.indexer.stopCommand(),
            Commands.waitSeconds(1)
        );
    }
}
