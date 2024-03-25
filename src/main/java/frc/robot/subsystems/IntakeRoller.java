package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeRoller extends SubsystemBase implements Reportable {
    
    private final TalonFX rightIntake;
    private final TalonFXConfigurator rightIntakeConfigurator;
    private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VoltageOut rightVoltageRequest = new VoltageOut(0, true, false, false, false);
    
    private final TalonFX leftIntake;
    private final TalonFXConfigurator leftIntakeConfigurator;
    private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VoltageOut leftVoltageRequest = new VoltageOut(0, true, false, false, false);

    private final NeutralOut brakeRequest = new NeutralOut();
    
    private boolean enabled = true;
    public boolean velocityControl = true;

    private final Follower followRequest = new Follower(IntakeConstants.kLeftIntakeMotorID, false);


    public IntakeRoller() {
        rightIntake = new TalonFX(IntakeConstants.kRightIntakeMotorID, SuperStructureConstants.kCANivoreBusName);
        rightIntake.setInverted(true);
        rightIntakeConfigurator = rightIntake.getConfigurator();

        leftIntake = new TalonFX(IntakeConstants.kLeftIntakeMotorID, SuperStructureConstants.kCANivoreBusName);
        leftIntake.setInverted(false); //check later
        leftIntakeConfigurator = leftIntake.getConfigurator();

        rightIntake.setControl(followRequest);

        CommandScheduler.getInstance().registerSubsystem(this);

        configureMotor();
        configurePID();
    }

    //****************************** SETUP METHODS ******************************/

    public void configureMotor() {
        TalonFXConfiguration leftIntakeConfigs = new TalonFXConfiguration();
        leftIntakeConfigurator.refresh(leftIntakeConfigs);

        leftIntakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftIntakeConfigs.Feedback.RotorToSensorRatio = 1;
        leftIntakeConfigs.Feedback.SensorToMechanismRatio = 1;

        leftIntakeConfigs.Voltage.PeakForwardVoltage = 11.5;
        leftIntakeConfigs.Voltage.PeakReverseVoltage = -11.5;

        leftIntakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftIntakeConfigs.MotorOutput.DutyCycleNeutralDeadband = IntakeConstants.kIntakeNeutralDeadband;
        leftIntakeConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        leftIntakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // intakeConfigs.CurrentLimits.StatorCurrentLimit = 50;
        // intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftIntakeConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        leftIntakeConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        leftIntakeConfigs.Audio.AllowMusicDurDisable = true;

        TalonFXConfiguration rightIntakeConfigs = new TalonFXConfiguration();
        rightIntakeConfigurator.refresh(rightIntakeConfigs);

        rightIntakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightIntakeConfigs.Feedback.RotorToSensorRatio = 1;
        rightIntakeConfigs.Feedback.SensorToMechanismRatio = 1;

        rightIntakeConfigs.Voltage.PeakForwardVoltage = 11.5;
        rightIntakeConfigs.Voltage.PeakReverseVoltage = -11.5;

        rightIntakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightIntakeConfigs.MotorOutput.DutyCycleNeutralDeadband = IntakeConstants.kIntakeNeutralDeadband;
        rightIntakeConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rightIntakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // intakeConfigs.CurrentLimits.StatorCurrentLimit = 50;
        // intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightIntakeConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        rightIntakeConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        rightIntakeConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode rightResult = rightIntakeConfigurator.apply(rightIntakeConfigs);
        if (!rightResult.isOK()) {
            DriverStation.reportError("Could not apply right intake configs, error code: "+ rightResult.toString(), new Error().getStackTrace());
        }

        StatusCode leftResult = leftIntakeConfigurator.apply(leftIntakeConfigs);
        if (!leftResult.isOK()) {
            DriverStation.reportError("Could not apply left intake configs, error code: "+ leftResult.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        IntakeConstants.kIntakeVelocity.loadPreferences();

        TalonFXConfiguration rightIntakeMotorConfigs = new TalonFXConfiguration();
        
        rightIntake.getConfigurator().refresh(rightIntakeMotorConfigs);
        IntakeConstants.kPRightIntakeMotor.loadPreferences();
        IntakeConstants.kIRightIntakeMotor.loadPreferences();
        IntakeConstants.kDRightIntakeMotor.loadPreferences();
        IntakeConstants.kVRightIntakeMotor.loadPreferences();
        rightIntakeMotorConfigs.Slot0.kP = IntakeConstants.kPRightIntakeMotor.get();
        rightIntakeMotorConfigs.Slot0.kI = IntakeConstants.kIRightIntakeMotor.get();
        rightIntakeMotorConfigs.Slot0.kD = IntakeConstants.kDRightIntakeMotor.get();
        rightIntakeMotorConfigs.Slot0.kV = IntakeConstants.kVRightIntakeMotor.get();

        StatusCode rightResult = rightIntake.getConfigurator().apply(rightIntakeMotorConfigs);

        if (!rightResult.isOK()){
            DriverStation.reportError("Could not apply right intake configs, error code:"+ rightResult.toString(), new Error().getStackTrace());
        }

        TalonFXConfiguration leftIntakeMotorConfigs = new TalonFXConfiguration();
        
        leftIntake.getConfigurator().refresh(leftIntakeMotorConfigs);
        IntakeConstants.kPLeftIntakeMotor.loadPreferences();
        IntakeConstants.kILeftIntakeMotor.loadPreferences();
        IntakeConstants.kDLeftIntakeMotor.loadPreferences();
        IntakeConstants.kVLeftIntakeMotor.loadPreferences();
        leftIntakeMotorConfigs.Slot0.kP = IntakeConstants.kPLeftIntakeMotor.get();
        leftIntakeMotorConfigs.Slot0.kI = IntakeConstants.kILeftIntakeMotor.get();
        leftIntakeMotorConfigs.Slot0.kD = IntakeConstants.kDLeftIntakeMotor.get();
        leftIntakeMotorConfigs.Slot0.kV = IntakeConstants.kVLeftIntakeMotor.get();

        StatusCode leftResult = leftIntake.getConfigurator().apply(leftIntakeMotorConfigs);

        if (!leftResult.isOK()){
            DriverStation.reportError("Could not apply left intake configs, error code:"+ leftResult.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            leftIntake.setControl(brakeRequest);
            return;
        }

        if (Math.abs(leftVelocityRequest.Velocity) < 0.5) {
            leftVelocityRequest.Velocity = 0;
            leftIntake.setControl(brakeRequest);
            return;
        }

        if (velocityControl) {
            leftIntake.setControl(leftVelocityRequest);
            return;
        }
        
        leftVoltageRequest.Output = leftVelocityRequest.Velocity * 12 / 100;
        leftIntake.setControl(leftVoltageRequest);
    }

    //****************************** VELOCITY METHODS ******************************//

    public void stop() {
        this.enabled = false;
        leftVelocityRequest.Velocity = 0;
        leftIntake.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setVelocity(double velocity) {
        leftVelocityRequest.Velocity = velocity;
    }

    /**
     * Increment the intake's velocity.
     * @param increment
     */
    public void incrementVelocity(double increment) {
        double newVelocity = leftVelocityRequest.Velocity + increment;
        if ((increment > 0 && newVelocity < IntakeConstants.kIntakeMaxVelocity) ||
            (increment < 0 && newVelocity > IntakeConstants.kIntakeMinVelocity)
            ) {
            leftVelocityRequest.Velocity = newVelocity;
        }
    }

    //****************************** VELOCITY COMMANDS ******************************//

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public Command incrementVelocityCommand(double increment) {
        return Commands.runOnce(() -> incrementVelocity(increment));
    }

    /**
     * Ramp the velocity within a certain timeframe
     * 
     * May take longer if the command scheduler is overrunning
     * @param initialVelocity The initial velocity in RPS
     * @param finalVelocity The final velocity in RPS
     * @param rampTimeSeconds
     * @return
     */
    public Command rampVelocity(double initialVelocity, double finalVelocity, double rampTimeSeconds) {
        final double initialVel = NerdyMath.clamp(initialVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        final double finalVel = NerdyMath.clamp(finalVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        
        // Change in Velocity / Command Scheduler Loops (assumes 20 hz)
        final double increment = (finalVel - initialVel) / (rampTimeSeconds * 20);

        // Check whether the current velocity is over/under final velocity
        BooleanSupplier rampFinished = 
            finalVel > initialVel ? 
                () -> rightVelocityRequest.Velocity >= finalVel : 
                () -> rightVelocityRequest.Velocity <= finalVel;

        if (initialVel == finalVel) {
            return setVelocityCommand(finalVel);
        } 

        return 
            Commands.sequence(
                setVelocityCommand(initialVel),
                Commands.deadline(
                    Commands.waitUntil(rampFinished),
                    incrementVelocityCommand(increment)
                )
            );
    }

    public Command intakeCommand() {
        return setVelocityCommand(IntakeConstants.kIntakeVelocity.get());
    }

    public Command autoIntakeCommand() {
        return setVelocityCommand(IntakeConstants.kAutoIntakeVelocity.get());
    }

    public Command outtakeCommand() {
        return setVelocityCommand(-IntakeConstants.kIntakeVelocity.get());
    }

    //****************************** LOGGING METHODS ******************************//

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addBoolean("Intake Roller Enabled", () -> enabled);
        tab.addNumber("Left Intake Velocity", ()-> leftIntake.getVelocity().getValueAsDouble());
        tab.addNumber("Left Intake Target Velocity", ()-> leftVelocityRequest.Velocity);
        tab.addNumber("Left Intake Supply Current", () -> leftIntake.getSupplyCurrent().getValueAsDouble());
        tab.addNumber("Left Intake Stator Current", () -> leftIntake.getStatorCurrent().getValueAsDouble());
        tab.addNumber("Left Intake Applied Voltage", () -> leftIntake.getMotorVoltage().getValueAsDouble());
        tab.addNumber("Right Intake Velocity", ()-> rightIntake.getVelocity().getValueAsDouble());
        tab.addNumber("Right Intake Supply Current", () -> rightIntake.getSupplyCurrent().getValueAsDouble());
        tab.addNumber("Right Intake Stator Current", () -> rightIntake.getStatorCurrent().getValueAsDouble());
        tab.addNumber("Right Intake Applied Voltage", () -> rightIntake.getMotorVoltage().getValueAsDouble());
    }

}
