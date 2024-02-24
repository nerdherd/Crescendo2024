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
        return intakePivot.moveToIntake();
    }

    public Command shooterSpeaker() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),
                intakePivot.moveToNeutral(),
                Commands.waitUntil(() -> false)  
            ),
            shooterPivot.moveToSpeaker()
        );

        // command.addRequirements(shooterPivot);

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

    public Command amp() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),
                intakePivot.moveToNeutral()
            ),
            shooterPivot.moveToAmp()
        );

        command.addRequirements(shooterPivot);

        return command;
    }

    public Command stow() {
        return Commands.sequence(
            intakePivot.setPositionCommand(IntakeConstants.kNeutralPosition.get()),
            Commands.deadline(
                Commands.waitUntil(shooterPivot::atTargetPosition),
                shooterSpeaker(),
                Commands.waitSeconds(2)
            ),
            intakePivot.setPositionCommand(IntakeConstants.kVerticalPosition.get())
        );
    }

    public Command handoff() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),  
                intakePivot.moveToIntake() 
            ),
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
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(1)
        ).finallyDo(indexer::stop);
    }

    public Command intakeFinal() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
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
                    intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
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
        return intakeDirectShoot(ShooterConstants.kHandoffPosition.get(), 
                                 ShooterConstants.kTopOuttakeAuto1.get(),
                                 ShooterConstants.kBottomOuttakeAuto1.get());
    }

    public Command intakeDirectShoot(double shooterPosition, double topShooterVelocity, double bottomShooterVelocity) {
        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Moving Shooter", true)),
            Commands.deadline(
                Commands.waitUntil(() -> 
                    // intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(shooterPosition)),
                intakePickup(),
                shooterPivot.setPositionCommand(shooterPosition)
                ),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Moving Shooter", false)),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.autoIntakeCommand(),
            shooterRoller.setVelocityCommand(topShooterVelocity, bottomShooterVelocity),
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

    public Command intakeDirectShoot(double shooterPosition, double shooterVelocity) {
        return intakeDirectShoot(shooterPosition, shooterVelocity, shooterVelocity);
    }

    public Command eject() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    // intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                // intakePickup(),
                handoff()
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
            intakePivot.moveToNeutral(),
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
            intakePivot.moveToNeutral(),
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
            amp(),
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
