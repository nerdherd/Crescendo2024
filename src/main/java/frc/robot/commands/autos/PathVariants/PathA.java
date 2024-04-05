package frc.robot.commands.autos.PathVariants;

import edu.wpi.first.wpilibj2.command.Command;

public class PathA {
    /*
    public Command PathA() {
        return(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.getImu().setOffset(startingPose.getRotation().getDegrees())),
            Commands.runOnce(()->swerve.resetOdometryWithAlliance(startingPose)),
            
            // A Start here ******************************************************
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
            
            // drive to note
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(4.5),
                Commands.sequence(
                    superSystem.stow()
                    // ,
                    // Commands.waitSeconds(2),
                    // superSystem.intakeUntilSensed()
                )
            )

            // turn angle to tag


            // shoot it
            
            // A End here ******************************************

            // C Starts here

            // C ends here

            //....
        );
    }
    */
    
}
