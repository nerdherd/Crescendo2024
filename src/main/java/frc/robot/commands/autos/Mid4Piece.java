package frc.robot.commands.autos;

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
import frc.robot.RobotContainer;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.NoteAssistance;

public class Mid4Piece extends SequentialCommandGroup {
    
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

    public Mid4Piece(SwerveDrivetrain swerve, SuperSystem superSystem, NoteAssistance noteCamera, List<PathPlannerPath> pathGroup){
        Pose2d startingPose = GetStartPoseInPath(pathGroup.get(0));
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            
            Commands.sequence(

            Commands.parallel(
                // Drive to note 2
                Commands.race(
                    AutoBuilder.followPath(pathGroup.get(0)).andThen(Commands.waitSeconds(0.5)),//a02
                    Commands.sequence(
                        // enable after the preload gone
                        Commands.waitSeconds(1.25),
                        Commands.deadline(
                            Commands.waitSeconds(2),
                            Commands.waitUntil(superSystem::noteIntook)
                        )
                    )
                ),

                Commands.sequence(
                    // Preload
                    Commands.deadline(
                        Commands.waitUntil(() -> !superSystem.noteIntook()),
                        Commands.parallel(
                            superSystem.shootSubwooferAutoStart(),
                            superSystem.intakeRoller.autoIntakeCommand()
                        )
                    ),
                    
                    // Stop note from popping out
                    superSystem.shooterRoller.setVelocityCommand(-10, -10),
                    superSystem.shooterRoller.setEnabledCommand(true),
                    Commands.waitSeconds(0.1),

                    Commands.deadline(
                        Commands.waitUntil(superSystem::noteIntook),
                        superSystem.intakeUntilSensedAuto(2.875)
                    ),
                    superSystem.backupIndexerAndShooterLess()
                    
                    // // shoot second piece
                    // Commands.waitSeconds(0.25),
                    // Commands.deadline(
                    //     Commands.waitUntil(() -> !superSystem.noteIntook()),
                    //     superSystem.shootSubwooferAutoStart2()
                    // )
                )    
            ),
            
            // Shoot note 2 while moving back to C start position
            Commands.parallel(
                AutoBuilder.followPath(pathGroup.get(1)),//b2p6

                // note 2
                Commands.deadline(
                    Commands.waitSeconds(0.4).andThen(Commands.waitUntil(() -> !superSystem.noteIntook())),
                    superSystem.shootSubwooferAutoStart2()
                )
            ),

            // PATH CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
            Commands.parallel(
                AutoBuilder.followPath(pathGroup.get(2)),//c26
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(1.5),
                    superSystem.shooterPivot.moveToHandoff(),
                    superSystem.shooterPivot.setEnabledCommand(true)
                )
            ),
                
            // this is for PathD
            Commands.race(
                AutoBuilder.followPath(pathGroup.get(3)).andThen(Commands.waitSeconds(0.5)),//d26
                superSystem.intakeUntilSensedAuto(2.875),
                Commands.waitUntil(() -> superSystem.noteIntook())
            ),

            // Commands.either(
            //     Commands.none(), 
            //     Commands.race(
            //         noteCamera.driveToNoteCommand(swerve, 15, 0, 0, 10, 200, null),   
            //         superSystem.intakeUntilSensedAuto(3)
            //     ),
            //     () -> superSystem.noteIntook()
            // ),
            superSystem.backupIndexerAndShooterLess(),

            // PATH EEEEEEEEE
            Commands.parallel(
                AutoBuilder.followPath(pathGroup.get(4)), //e6Y
                Commands.sequence(
                    superSystem.stow(),
                    Commands.waitSeconds(2.3), // regualr path uses 2.4; shorter one uses .9
                    superSystem.shooterPivot.moveToHandoff(),

                    Commands.deadline(
                        Commands.waitUntil(() -> !superSystem.noteIntook()),
                        superSystem.shootSubwoofer()
                    ),
                    
                    superSystem.shooterRoller.setVelocityCommand(-10, -10),
                    superSystem.shooterRoller.setEnabledCommand(true)
                )
            ),

            // PATH LAST AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
            Commands.parallel(
                // Drive to note 123
                AutoBuilder.followPath(pathGroup.get(5)), //aY3
                Commands.sequence(
                    Commands.deadline(
                        Commands.waitUntil(superSystem::noteIntook),
                        superSystem.intakeUntilSensedAuto(2.875)
                    ),

                    superSystem.backupIndexerAndShooterLess(),

                    Commands.deadline(
                        Commands.waitUntil(() -> !superSystem.noteIntook()),
                        Commands.parallel(
                            Commands.sequence(
                                superSystem.shootSequenceAdjustable(swerve),
                                superSystem.shoot()
                            ),
                            superSystem.intakeRoller.autoIntakeCommand()
                        )
                    )
                    )
                )
            ),

            Commands.race(
                AutoBuilder.followPath(pathGroup.get(6)).andThen(Commands.waitSeconds(0.5)),
                Commands.sequence(
                    superSystem.intakeUntilSensedAuto(3)
                ),
                Commands.waitUntil(superSystem::noteIntook)
            ),
            Commands.parallel(
                Commands.sequence(
                    superSystem.backupIndexerAndShooterLess(),
                    superSystem.shootSequenceAdjustable(swerve),
                    superSystem.shoot()
                ),
                swerve.turnToTag(RobotContainer.IsRedSide() ? 4 : 7)
            )
        );
    }
}
