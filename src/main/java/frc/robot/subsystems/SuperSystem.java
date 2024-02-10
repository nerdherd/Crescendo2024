package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperSystem {
    private IntakePivot intakePivot;
    private IntakeRoller intakeRoller;
    private ShooterPivot shooterPivot;
    private ShooterRoller shooterRoller;
    private Indexer indexer;
    private ColorSensor colorSensor;

    public SuperSystem(IntakePivot intakePivot, IntakeRoller intakeRoller, 
                        ShooterPivot shooterPivot, ShooterRoller shooterRoller,
                        Indexer indexer) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.shooterPivot = shooterPivot;
        this.shooterRoller = shooterRoller;
        this.indexer = indexer;
        this.colorSensor = new ColorSensor();
    }

    public Command intakeStow() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::hasReachedNeutral),
                shooterPivot.moveToNeutral()
            ),
            intakePivot.moveToStow()
        );

        command.addRequirements(intakePivot);

        return command;
    }

    public Command intakeNeutral() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::hasReachedNeutral),            
                shooterPivot.moveToNeutral()
            ),
            intakePivot.moveToNeutral()
        );

        command.addRequirements(intakePivot);

        return command;
    }

    public Command intakePickup() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::hasReachedNeutral), 
                shooterPivot.moveToNeutral()           
            ),
            intakePivot.moveToIntake()
        );

        command.addRequirements(intakePivot);

        return command;
    }

    public Command shooterSpeaker() {
        Command command = Commands.sequence(
            // Commands.deadline(
            //     Commands.waitUntil(intakePivot::hasReachedNeutral),
            //     intakePivot.moveToNeutral()         
            // ),
            shooterPivot.moveToSpeaker()
        );

        command.addRequirements(shooterPivot);

        return command;
    }

    public Command shooterNeutral() {
        Command command = Commands.sequence(
            // Commands.deadline(
            //     Commands.waitUntil(intakePivot::hasReachedNeutral),
            //     intakePivot.moveToNeutral()         
            // ),
            shooterPivot.moveToNeutral()
        );

        command.addRequirements(shooterPivot);

        return command;
    }

    public Command shooterAmp() {
        Command command = Commands.sequence(
            // Commands.deadline(
            //     Commands.waitUntil(intakePivot::hasReachedNeutral),
            //     intakePivot.moveToNeutral()
            // ),
            shooterPivot.moveToAmp()
        );

        command.addRequirements(shooterPivot);

        return command;
    }

    public Command shooterHandoff() {
        Command command = Commands.sequence(
            // Commands.deadline(
            //     Commands.waitUntil(intakePivot::hasReachedNeutral),  
            //     intakePivot.moveToNeutral() 
            // ),
            shooterPivot.moveToHandoff()
        );

        command.addRequirements(shooterPivot);

        return command;
    }

    public Command backupIndexer() {
        return Commands.sequence(
            shooterPivot.moveToSpeaker(),
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.5),
            indexer.stopCommand()
        ).finallyDo(indexer::stop);
    }

    public Command backupIndexerManual() {
        return Commands.sequence(
            shooterPivot.moveToSpeaker(),
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand()
        ).finallyDo(indexer::stop);
    }

    public Command intakeFinal() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                intakePickup(),
                shooterHandoff()
                ),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),
            Commands.waitUntil(colorSensor::noteIntook),
            intakeRoller.stopCommand(),
            indexer.stopCommand()
        );
    }

    public Command intakeBasic() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    // intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                // intakePickup(),
                shooterHandoff()
                ),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );
    }

    public Command intakeDirectShoot() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    // intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                // intakePickup(),
                shooterHandoff()
                ),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            shooterRoller.shootSpeakerAuto1(),
            intakeRoller.intakeCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
                shooterRoller.stop();
            }
        );
    }

    public Command eject() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    // intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                // intakePickup(),
                shooterHandoff()
                ),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Outtaking", true)),
            indexer.reverseIndexCommand(),
            intakeRoller.outtakeCommand(),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Outtaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );
    }

    public Command shootSequence2() {
        return Commands.sequence(
            // Prepare to shoot
            shooterSpeaker(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker(),
            Commands.waitSeconds(1),
            
            // Shoot
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(interrupted -> {
            indexer.stop();
            shooterRoller.stop();
        });
    }

    public Command shootSequence2Far() {
        return Commands.sequence(
            // Prepare to shoot
            shooterPivot.moveToSpeakerFar(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker(),
            Commands.waitSeconds(1),
            
            // Shoot
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(interrupted -> {
            indexer.stop();
            shooterRoller.stop();
        });
    }

    public Command ampSequence() {
        return Commands.sequence( 
            shooterAmp(),
            indexer.setEnabledCommand(false),
            Commands.waitUntil(shooterPivot::atTargetPosition),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootAmp(),
            Commands.waitSeconds(0.5),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                shooterRoller.stop();
                indexer.stop();
            }
        );
    }
}
