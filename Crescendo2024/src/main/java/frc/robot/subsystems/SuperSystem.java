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

    public SuperSystem() {
        intakePivot = new IntakePivot();
        intakeRoller = new IntakeRoller();
        shooterPivot = new ShooterPivot();
        shooterRoller = new ShooterRoller();

        shooterRoller.setShooterPowerZero();
    }

    public void IntakeStow() {

        if (shooterPivot.reachNeutralPosition()) {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kStowPosition));
            SmartDashboard.putBoolean("Within tolerance", true);
        } 
        else {
            SmartDashboard.putBoolean("Within tolerance", false);
            Commands.runOnce (() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
        }
        }

    
    public void IntakeNeutral() {
        Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));

        if (shooterPivot.reachNeutralPosition()) {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Within tolerance", true);
        }
        else {
            SmartDashboard.putBoolean("Within tolerance", false);
            Commands.runOnce (() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
        }

    }
    public void IntakePickup() {
        Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kPickupPosition));

        if (shooterPivot.reachNeutralPosition()) {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kPickupPosition));
            SmartDashboard.putBoolean("Within tolerance", true);
        }
        else {
            SmartDashboard.putBoolean("Within tolerance", false);
            Commands.runOnce (() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
        }


    }

    public void ShooterSpeaker() {
        
        if (intakePivot.reachNeutralPosition()) {
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kSpeakerPosition));
            SmartDashboard.putBoolean("Not within Tolerance", true);
        }
        else {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Not within Tolerance", false);
            }

    }
    public void ShooterNeutral() {
        
        if (intakePivot.reachNeutralPosition()) {
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));
SmartDashboard.putBoolean("Not within Tolerance", true);
        }
        else {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Not within Tolerance", false);
            }
    }

    public void ShooterAmp() {

        if (intakePivot.reachNeutralPosition()) {
            Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kAmpPosition));
            SmartDashboard.putBoolean("Not within Tolerance", true);
        }
        else {
            Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));
            SmartDashboard.putBoolean("Not within Tolerance", false);
            }
    }

    public void ShootHigh() {
        Commands.runOnce(() -> shooterRoller.ShootSpeaker(ShooterConstants.kOuttakeHigh));
    }

    public void ShootLow() {
        Commands.runOnce(() -> shooterRoller.ShootSpeaker(ShooterConstants.kOuttakeLow));
    }

    public void IntakeRollers() {
        Commands.runOnce(() -> shooterRoller.ShootSpeaker(ShooterConstants.kIntake));
    }

    // TODO: Add Manual Arm Control with Joystick
    

    // TODO: Add Manual Intake Control with Joystick


    public void initShooterRollerShuffleboard() {
        shooterRoller.initShuffleboard();
        shooterRoller.printShooterSpeeds();
    }
}
