package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
    
    private final TalonFX intake;
    private final TalonFXConfigurator intakeConfigurator;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VoltageOut voltageRequest = new VoltageOut(0, true, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();
    
    private boolean enabled = true;
    public boolean velocityControl = true;

    public IntakeRoller() {
        intake = new TalonFX(IntakeConstants.kIntakeMotorID, SuperStructureConstants.kCANivoreBusName);
        intake.setInverted(false);
        intakeConfigurator = intake.getConfigurator();

        CommandScheduler.getInstance().registerSubsystem(this);

        configureMotor();
        configurePID();
    }

    //****************************** SETUP METHODS ******************************/

    public void configureMotor() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        intakeConfigurator.refresh(intakeConfigs);

        intakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeConfigs.Feedback.RotorToSensorRatio = 1;
        intakeConfigs.Feedback.SensorToMechanismRatio = 1;

        intakeConfigs.Voltage.PeakForwardVoltage = 11.5;
        intakeConfigs.Voltage.PeakReverseVoltage = -11.5;

        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = IntakeConstants.kIntakeNeutralDeadband;
        intakeConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        intakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // intakeConfigs.CurrentLimits.StatorCurrentLimit = 50;
        // intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        intakeConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        intakeConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode result = intakeConfigurator.apply(intakeConfigs);
        if (!result.isOK()) {
            DriverStation.reportError("Could not apply intake configs, error code: "+ result.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        IntakeConstants.kIntakeVelocity.loadPreferences();
        TalonFXConfiguration intakeMotorConfigs = new TalonFXConfiguration();
        
        intake.getConfigurator().refresh(intakeMotorConfigs);
        IntakeConstants.kPIntakeMotor.loadPreferences();
        IntakeConstants.kIIntakeMotor.loadPreferences();
        IntakeConstants.kDIntakeMotor.loadPreferences();
        IntakeConstants.kVIntakeMotor.loadPreferences();
        intakeMotorConfigs.Slot0.kP = IntakeConstants.kPIntakeMotor.get();
        intakeMotorConfigs.Slot0.kI = IntakeConstants.kIIntakeMotor.get();
        intakeMotorConfigs.Slot0.kD = IntakeConstants.kDIntakeMotor.get();
        intakeMotorConfigs.Slot0.kV = IntakeConstants.kVIntakeMotor.get();

        StatusCode result = intake.getConfigurator().apply(intakeMotorConfigs);

        if (!result.isOK()){
            DriverStation.reportError("Could not apply intake configs, error code:"+ result.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            intake.setControl(brakeRequest);
            return;
        }

        if (Math.abs(velocityRequest.Velocity) < 0.5) {
            velocityRequest.Velocity = 0;
            intake.setControl(brakeRequest);
            return;
        }

        if (velocityControl) {
            intake.setControl(velocityRequest);
            return;
        }
        
        voltageRequest.Output = velocityRequest.Velocity * 12 / 100;
        intake.setControl(voltageRequest);
    }

    //****************************** VELOCITY METHODS ******************************//

    public void stop() {
        this.enabled = false;
        velocityRequest.Velocity = 0;
        intake.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setVelocity(double velocity) {
        velocityRequest.Velocity = velocity;
    }

    /**
     * Increment the intake's velocity.
     * @param increment
     */
    public void incrementVelocity(double increment) {
        double newVelocity = velocityRequest.Velocity + increment;
        if ((increment > 0 && newVelocity < IntakeConstants.kIntakeMaxVelocity) ||
            (increment < 0 && newVelocity > IntakeConstants.kIntakeMinVelocity)
            ) {
            velocityRequest.Velocity = newVelocity;
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
                () -> velocityRequest.Velocity >= finalVel : 
                () -> velocityRequest.Velocity <= finalVel;

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
        tab.addNumber("Intake Velocity", ()-> intake.getVelocity().getValueAsDouble());
        tab.addNumber("Intake Target Velocity", ()-> velocityRequest.Velocity);
        tab.addNumber("Intake Supply Current", () -> intake.getSupplyCurrent().getValueAsDouble());
        tab.addNumber("Intake Stator Current", () -> intake.getStatorCurrent().getValueAsDouble());
        tab.addNumber("Intake Applied Voltage", () -> intake.getMotorVoltage().getValueAsDouble());
    }

}
