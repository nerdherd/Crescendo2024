package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperSystem {
    public IntakePivot intakePivot;
    public IntakeRoller intakeRoller;
    public ShooterPivot shooterPivot;
    public ShooterRoller shooterRoller;
    public IndexerV2 indexer;
    public ColorSensor colorSensor;
    public LinearActuator linearActuator;
    public Climber climber;

    public SuperSystem(IntakePivot intakePivot, IntakeRoller intakeRoller, 
                        ShooterPivot shooterPivot, ShooterRoller shooterRoller,
                        IndexerV2 indexer) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.shooterPivot = shooterPivot;
        this.shooterRoller = shooterRoller;
        this.indexer = indexer;
        this.linearActuator = new LinearActuator();
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
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),
                intakePivot.setEnabledCommand(true),
                intakePivot.moveToNeutral(),
                Commands.waitUntil(() -> false)  
            ),
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
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),
            Commands.waitUntil(colorSensor::noteIntook),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", false)),
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

    public Command intakeBasic1() {
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
        );
    }

    public Command intakeBasic2() {
        return Commands.sequence(
            Commands.runOnce(() -> {
            SmartDashboard.putBoolean("Intaking", false);
            intakeRoller.stop();
            indexer.stop();
            })
        );
    }

    public Command intakeDirectShoot() {
        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Moving Shooter", true)),
            Commands.deadline(
                Commands.waitUntil(() -> 
                    // intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition2.get())),
                // intakePickup(),
                shooterPivot.moveToAutoHandoff()
                ),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Moving Shooter", false)),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.autoIntakeCommand(),
            shooterRoller.shootSpeakerAuto1(),
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
            Commands.waitSeconds(0.8),
            
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
            Commands.waitSeconds(0.8),
            
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

    public Command climbSequence() {
        return Commands.sequence(
            linearActuator.retractCommand(),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> climber.climb())
        
        );
            
    }
}
