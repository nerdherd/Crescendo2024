package frc.robot.commands.autos.PathVariants;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class PathE extends SequentialCommandGroup {

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
    
    public PathE(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, ShooterVisionAdjustment sva)
    {
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
                        Commands.waitSeconds(.85), // regualr path uses 2.4; shorter one uses .85
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

            //Commands.runOnce(() -> swerve.towModules()),
            superSystem.stow(),
            superSystem.indexer.stopCommand(),
            Commands.waitSeconds(1)
            
        );
    }
}

