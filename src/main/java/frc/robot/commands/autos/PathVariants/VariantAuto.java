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
import frc.robot.subsystems.vision.ShooterVisionAdjustment;

public class VariantAuto extends SequentialCommandGroup{
    public VariantAuto(SwerveDrivetrain swerve, SuperSystem superSystem, List<PathPlannerPath> pathGroup, DriverAssist driverAssist, ShooterVisionAdjustment sva, String pathName1, int pathIndex1, String pathName2, int pathIndex2, String pathName3, int pathIndex3){
        AutoCommands autoCommand = new AutoCommands();
        if(pathName1!=null){
            autoCommand.PathA(swerve, superSystem, pathGroup, driverAssist, sva, pathName1, pathIndex1);
        }
        if(pathName2!=null){
            autoCommand.PathB(swerve, superSystem, pathGroup, driverAssist, sva, pathName2, pathIndex2);
        }
        if(pathName3!=null){
            autoCommand.PathC(swerve, pathGroup, pathName3, pathIndex3);
        }

        
    }
    
}
