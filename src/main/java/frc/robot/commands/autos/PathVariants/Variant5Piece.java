package frc.robot.commands.autos.PathVariants;

import java.sql.Driver;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.AutoCommands;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class Variant5Piece extends SequentialCommandGroup {
    public Variant5Piece(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, ShooterVisionAdjustment sva, NoteAssistance noteCamera) {
        addCommands(
            Commands.sequence(
                new PathA(swerve, superSystem, pathGroup.get(0), driverAssist, sva),
                new PathB(swerve, superSystem, pathGroup.get(1), driverAssist, sva),
                new PathC(swerve, pathGroup.get(2)),
                new PathD(swerve, superSystem, noteCamera, 15, 10, 50, pathGroup.get(3), pathGroup.get(4)),
                new PathE(swerve, superSystem, pathGroup.get(5), driverAssist, sva)
            )
        );

    }
    
}
