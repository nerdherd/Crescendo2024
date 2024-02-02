package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperSystem{
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

    public Command IntakeStow() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::hasReachedNeutral),
                shooterPivot.moveToNeutral()
            ),
            intakePivot.moveToStow()
        );
    }

    public Command IntakeNeutral() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::hasReachedNeutral),            
                shooterPivot.moveToNeutral()
            ),
            intakePivot.moveToNeutral()
        );
    }

    public Command IntakePickup() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(shooterPivot::hasReachedNeutral), 
                shooterPivot.moveToNeutral()           
            ),
            intakePivot.moveToIntake()
        );
    }

    public Command ShooterSpeaker() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),
                intakePivot.moveToNeutral()         
            ),
            shooterPivot.moveToSpeaker()
        );
    }

    public Command ShooterNeutral() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),
                intakePivot.moveToNeutral()         
            ),
            shooterPivot.moveToNeutral()
        );
    }

    public Command ShooterAmp() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),
                intakePivot.moveToNeutral()
            ),
            shooterPivot.moveToAmp()
        );
    }

    public Command ShooterHandoff() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(intakePivot::hasReachedNeutral),  
                intakePivot.moveToNeutral() 
            ),
            shooterPivot.moveToHandoff()
        );
    }

    public Command IntakeSequence() {
        return Commands.sequence(
            IntakePickup(),
            ShooterHandoff(),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),
            Commands.waitUntil(colorSensor::noteIntook),
            intakeRoller.stopCommand(),
            indexer.stopCommand()
        );
    }
}
