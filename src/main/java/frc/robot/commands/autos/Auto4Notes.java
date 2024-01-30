package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Auto4Notes extends SequentialCommandGroup {
    public Auto4Notes(SwerveDrivetrain swerve, String autoPath, SuperSystem superSystem) {     
        
        // Use the PathPlannerAuto class to get a path group from an auto
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoPath);

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.setPoseMeters(startingPose)),

            // Commands.runOnce(() -> superSystem.ShootSpeaker()),
            // Commands.runOnce(() -> superSystem.ShooterNeutral()),
            // Commands.runOnce(() -> superSystem.IntakeNeutral()),

            AutoBuilder.followPath((pathGroup.get(0))),
            // Commands.runOnce(() -> superSystem.IntakeSequence()),
            // Commands.runOnce(() -> superSystem.ShootSpeaker()),
            // Commands.runOnce(() -> superSystem.ShooterNeutral()),
            // Commands.runOnce(() -> superSystem.IntakeNeutral()),

            AutoBuilder.followPath((pathGroup.get(1))),
            // Commands.runOnce(() -> superSystem.IntakeSequence()),
            // Commands.runOnce(() -> superSystem.ShootSpeaker()),
            // Commands.runOnce(() -> superSystem.ShooterNeutral()),
            // Commands.runOnce(() -> superSystem.IntakeNeutral()),

            AutoBuilder.followPath((pathGroup.get(2)))
            // Commands.runOnce(() -> superSystem.IntakeSequence()),
            // Commands.runOnce(() -> superSystem.ShootSpeaker()),
            // Commands.runOnce(() -> superSystem.ShooterNeutral()),
            // Commands.runOnce(() -> superSystem.IntakeStow())
            
            );
    }
}