package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class SuperSystem{
    private IntakePivot intakePivot;
    private IntakeRoller intakeRoller;
    private ShooterPivot shooterPivot;
    private ShooterRoller shooterRoller;
    private Indexer indexer;
    private ColorSensor colorSensor;

    public SuperSystem() {
        intakePivot = new IntakePivot();
        intakeRoller = new IntakeRoller();
        shooterPivot = new ShooterPivot();
        shooterRoller = new ShooterRoller();
        indexer = new Indexer();
        colorSensor = new ColorSensor();

        shooterRoller.stop();
        intakeRoller.setIntakePowerZero();
        indexer.stop();
    }

    public Command IntakeStow() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::reachNeutralPosition),            
                Commands.run(() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition))
            ),
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kStowPosition))
        );
    }

    public Command IntakeNeutral() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::reachNeutralPosition),            
                Commands.run(() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition))
            ),
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition))
        );
    }

    public Command IntakePickup() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::reachNeutralPosition),            
                Commands.run(() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition))
            ),
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kPickupPosition))
        );
    }

    public Command ShooterSpeaker() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::reachNeutralPosition),            
                Commands.run(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition))
            ),
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kSpeakerPosition))
        );
    }

    public Command ShooterNeutral() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::reachNeutralPosition),            
                Commands.run(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition))
            ),
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition))
        );
    }

    public Command ShooterAmp() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::reachNeutralPosition),            
                Commands.run(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition))
            ),
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kAmpPosition))
        );
    }

    public Command ShooterHandoff() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::reachNeutralPosition),            
                Commands.run(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition))
            ),
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kHandoffPosition))
        );
    }

    public void ShootHigh() {
        Commands.runOnce(() -> shooterRoller.setVelocity(ShooterConstants.kOuttakeHigh));
    }

    public void ShootAmp() {
        Commands.runOnce(() -> shooterRoller.setVelocity(ShooterConstants.kOuttakeLow));
    }

    public Command IntakeSequence() {
        return Commands.sequence(
            IntakePickup(),
            ShooterHandoff(),
            startIndexer(),
            startIntakeRollers(),
            Commands.waitUntil(colorSensor::noteIntook),
            Commands.runOnce(() -> intakeRoller.setIntakePowerZero()),
            Commands.runOnce(() -> indexer.stop())
        );
    }

    public Command startIntakeRollers() {
        return Commands.runOnce(() -> intakeRoller.setIntakeSpeed(IntakeConstants.kIntakeVelocity));
    }

    public Command startIndexer() {
        return Commands.runOnce(() -> indexer.setVelocity(IndexerConstants.kIndexerVelociyRPS.get()));
    }

    public void initShooterRollerShuffleboard() {
        shooterRoller.initShuffleboard();
    }
}
