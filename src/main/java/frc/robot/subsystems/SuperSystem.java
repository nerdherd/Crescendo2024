package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class SuperSystem {
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

        shooterRoller.setShooterPowerZero();
        intakeRoller.setIntakePowerZero();
        indexer.setIndexerPowerZero();
    }

    public Command IntakeStow() {

        if (shooterPivot.reachNeutralPosition()) {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kStowPosition));
            SmartDashboard.putBoolean("Within tolerance", true);
        } 
        else {
            SmartDashboard.putBoolean("Within tolerance", false);
            Commands.runOnce (() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
        }
    }

    
    public Command IntakeNeutral() {

        if (shooterPivot.reachNeutralPosition()) {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Within tolerance", true);
        }
        else {
            SmartDashboard.putBoolean("Within tolerance", false);
            Commands.runOnce (() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
        }

    }
    public Command IntakePickup() {

        if (shooterPivot.reachNeutralPosition()) {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kPickupPosition));
            SmartDashboard.putBoolean("Within tolerance", true);
        }
        else {
            SmartDashboard.putBoolean("Within tolerance", false);
            Commands.runOnce (() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
        }


    }

    public Command ShooterSpeaker() {
        
        if (intakePivot.reachNeutralPosition()) {
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kSpeakerPosition));
            SmartDashboard.putBoolean("Within Tolerance", true);
        }
        else {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Within Tolerance", false);
            }

    }
    public Command ShooterNeutral() {
        
        if (intakePivot.reachNeutralPosition()) {
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
SmartDashboard.putBoolean("Within Tolerance", true);
        }
        else {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Within Tolerance", false);
            }
    }

    public Command ShooterAmp() {

        if (intakePivot.reachNeutralPosition()) {
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kAmpPosition));
            SmartDashboard.putBoolean("Within Tolerance", true);
        }
        else {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Within Tolerance", false);
            }
    
    }

    public Command ShooterHandoff() {

        if (intakePivot.reachNeutralPosition()) {
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kHandoffPosition));
            SmartDashboard.putBoolean("Within Tolerance", true);
        }
        else {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Within Tolerance", false);

        }

    }

    public Command ShootSpeaker() {
        Commands.runOnce(() -> shooterRoller.ShootSpeaker(ShooterConstants.kOuttakeHigh));
    }

    public Command ShootAmp() {
        Commands.runOnce(() -> shooterRoller.ShootAmp(ShooterConstants.kOuttakeLow));
    }

    public Command IntakeSequence() {
        Commands.runOnce(() -> {
            IntakePickup();
            ShooterHandoff();
            Indexer();
            IntakeRollers();


            if (colorSensor.noteIntook()) {
                intakeRoller.setIntakePowerZero();
                indexer.setIndexerPowerZero();
                SmartDashboard.putBoolean("Note Intook", true);
            }
            else {
                SmartDashboard.putBoolean("Note Intook", false); 
            }   
            
        });
        
    }

    public Command IntakeRollers() {
        Commands.runOnce(() -> intakeRoller.setIntakeSpeed(IntakeConstants.kIntakeVelocity));
    }

    public Command Indexer() {
        Commands.runOnce(() -> indexer.setIndexerSpeed());
    }

    public Command initShooterRollerShuffleboard() {
        shooterRoller.initShuffleboard();
        shooterRoller.printShooterSpeeds();
    }
}
