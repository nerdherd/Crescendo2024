// package frc.robot.commands.VisionAutos;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.SwerveDrivetrain;
// import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;

// public class ToNearestGridDebug extends Command {
//     private SwerveDrivetrain swerve;
//     private PrimalSunflower vision;

//     /**
//      * Construct a new ToNearestGrid command
//      * 
//      * Drive the robot to the nearest grid using odometry.
//      * 
//      * @param swerveDrive   Swerve drivetrain to move
//      */
//     public ToNearestGridDebug(SwerveDrivetrain swerve, PrimalSunflower vision) {
//         this.swerve = swerve;

//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {}

//     @Override
//     public void execute() {
//         vision.toNearestGridDebug(swerve);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
