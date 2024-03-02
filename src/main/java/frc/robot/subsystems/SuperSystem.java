package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperSystem {
    public IntakePivot intakePivot;
    public IntakeRoller intakeRoller;
    public ShooterPivot shooterPivot;
    public ShooterRoller shooterRoller;
    public IndexerV2 indexer;
    public ColorSensor colorSensor;
    public ClimbActuator linearActuator;
    public Climber climber;

    public SuperSystem(IntakePivot intakePivot, IntakeRoller intakeRoller, 
                        ShooterPivot shooterPivot, ShooterRoller shooterRoller,
                        IndexerV2 indexer) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.shooterPivot = shooterPivot;
        this.shooterRoller = shooterRoller;
        this.indexer = indexer;
        this.linearActuator = new ClimbActuator();
        this.colorSensor = new ColorSensor();
    }

    public Command getReadyForAmp() {
        Command command = Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Amp Rotating", false)),
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),
                intakePivot.moveToNeutral()
            ),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Amp Rotating", true)),
            shooterPivot.moveToAmp(),
            Commands.waitUntil(shooterPivot::atTargetPosition),
            intakePivot.setPositionCommand(IntakeConstants.kVerticalPosition.get())
        );

        command.addRequirements(shooterPivot, intakePivot);

        return command;
    }

    public Command shootAmp() {
        Command command = Commands.sequence(
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootAmp(),
            Commands.waitSeconds(0.5),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitSeconds(1)
        ).finallyDo(
            () -> {
                shooterRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(shooterRoller, indexer, intakeRoller);

        return command;
    }

    public Command stow() {
        Command command = Commands.sequence(
            intakeRoller.stopCommand(),
            shooterRoller.stopCommand(),
            indexer.stopCommand(),
            intakePivot.setPositionCommand(IntakeConstants.kNeutralPosition.get()),
            Commands.deadline(
                Commands.waitUntil(shooterPivot::atTargetPosition),
                shooterPivot.setPositionCommand(-0.1),
                Commands.waitSeconds(2)
            ),
            intakePivot.setPositionCommand(IntakeConstants.kVerticalPosition.get())
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);

        return command;
    }

    public Command handoff() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),  
                intakePivot.moveToIntake() 
            ),
            shooterPivot.moveToHandoff()
        );

        return command;
    }

    public Command backupIndexer() {
        Command command = Commands.sequence(
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.5),
            indexer.stopCommand()
        ).finallyDo(indexer::stop);

        command.addRequirements(indexer);
        
        return command;
    }

    public Command backupIndexerManual() {
        Command command = Commands.sequence(
            // shooterPivot.moveToSpeaker(),
            indexer.setEnabledCommand(true),
            shooterRoller.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            shooterRoller.setReverseVelocityCommand(-0.1, -0.1), // TODO: Later
            Commands.waitSeconds(1)
        ).finallyDo(indexer::stop);

        command.addRequirements(shooterRoller, indexer);
        
        return command.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command intakeUntilSensed() {
        Command command = Commands.sequence(
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
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", false)),
            intakeRoller.stopCommand(),
            indexer.stopCommand()
        ).finallyDo(() -> {
            SmartDashboard.putBoolean("Intaking", false);
            intakeRoller.stop();
            indexer.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command intakeBasic() {
        Command command = Commands.sequence(
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

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command intakeBasicHold() {
        Command command = Commands.sequence(
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
            intakeRoller.intakeCommand()
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command stopIntaking() {
        Command command = Commands.sequence(
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
            })
        );

        command.addRequirements(indexer, intakeRoller);
        return command;
    }


    public Command intakeDirectShoot() {
        return intakeDirectShoot(ShooterConstants.kHandoffPosition.get(), 
                                 ShooterConstants.kTopOuttakeAuto1.get(),
                                 ShooterConstants.kBottomOuttakeAuto1.get());
    }

    public Command intakeDirectShoot(double shooterPosition, double topShooterVelocity, double bottomShooterVelocity) {
        Command command = Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Moving Shooter", true)),
            Commands.deadline(
                Commands.waitUntil(() -> 
                    // intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(shooterPosition)),
                intakePivot.moveToIntake(),
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

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command intakeDirectShoot(double shooterPosition, double shooterVelocity) {
        return intakeDirectShoot(shooterPosition, shooterVelocity, shooterVelocity);
    }

    public Command eject() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    intakePivot.hasReachedPosition(IntakeConstants.kPickupPosition.get()) && 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                intakePivot.moveToIntake(),
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

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command prepareShooter() {
        Command command = Commands.sequence(
            intakePivot.moveToNeutral(),
            Commands.waitUntil(intakePivot::hasReachedNeutral),
            shooterPivot.moveToSpeaker(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker()
        );
        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot);
        return command;
    }

    public Command shootShooter() {
        Command command = Commands.sequence(
            intakePivot.moveToNeutral(),
            Commands.waitUntil(intakePivot::hasReachedNeutral),
            shooterPivot.moveToSpeaker(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker(),
            indexer.setEnabledCommand(true),
            indexer.indexCommand()
        );
        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot);
        return command;
    }

    public Command shootSequence2() {
        Command command = Commands.sequence(
            intakePivot.moveToIntake(),
            Commands.waitUntil(intakePivot::hasReachedNeutral),
            // Prepare to shoot
            shooterPivot.moveToSpeaker(),
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

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command shootSequenceAdjustable(ShooterVisionAdjustment sva) {
        Command command = Commands.sequence(
            intakePivot.moveToIntake(),
            Commands.waitUntil(intakePivot::hasReachedNeutral),
            // Prepare to shoot
            shooterPivot.setPositionCommand(sva.getShooterAngle()),
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

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command shootSequence2Far() {
        Command command = Commands.sequence(
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
        command.addRequirements(shooterPivot, shooterRoller, indexer, intakePivot, intakeRoller);
        return command;
    }

    public Command climbSequence() {
        return Commands.sequence(
            linearActuator.retractCommand(),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> climber.climb())
        
        );
            
    }
}
