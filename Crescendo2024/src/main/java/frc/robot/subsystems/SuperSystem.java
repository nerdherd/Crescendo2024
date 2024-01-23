package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
        Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kStowPosition)); // TODO: If structure with Run commands

    }
    public void IntakeNeutral() {
        Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kNeutralPosition));

    }
    public void IntakePickup() {
        Commands.runOnce(() -> intakePivot.setPosition(IntakeConstants.kPickupPosition));

    }

    public void ShooterSpeaker() {
        Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kSpeakerPosition)); // TODO: Run command

    }
    public void ShooterNeutral() {
        Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kNeutralPosition));

    }
    public void ShooterAmp() {
        Commands.runOnce(() -> shooterPivot.setPosition(ShooterConstants.kAmpPosition));

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
