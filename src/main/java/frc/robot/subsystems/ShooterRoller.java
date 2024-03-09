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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;

public class ShooterRoller extends SubsystemBase implements Reportable {
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;
    private final TalonFXConfigurator leftShooterConfigurator;
    private final TalonFXConfigurator rightShooterConfigurator;

    private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0, 0, true, 0,0, false, false, false);
    
    private final VoltageOut leftVoltageRequest = new VoltageOut(0);
    private final VoltageOut rightVoltageRequest = new VoltageOut(0);

    private final NeutralOut brakeRequest = new NeutralOut();
    
    private boolean enabled = true;
    public boolean velocityControl = true;

    public ShooterRoller() {
        leftShooter = new TalonFX(ShooterConstants.kLeftMotorID, SuperStructureConstants.kCANivoreBusName);
        rightShooter = new TalonFX(ShooterConstants.kRightMotorID, SuperStructureConstants.kCANivoreBusName);

        leftVoltageRequest.EnableFOC = true;
        rightVoltageRequest.EnableFOC = true;

        leftShooterConfigurator = leftShooter.getConfigurator();
        rightShooterConfigurator = rightShooter.getConfigurator();

        leftShooter.setInverted(false);
        rightShooter.setInverted(false);

        CommandScheduler.getInstance().registerSubsystem(this);

        configureMotor();
        configurePID();
    }

    public void configureMotor() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        leftShooterConfigurator.refresh(leftMotorConfigs);
        leftMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        leftMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ShooterConstants.kShooterNeutralDeadband;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        leftMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        leftMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = 150;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.Audio.AllowMusicDurDisable = true;

        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
        rightShooterConfigurator.refresh(rightMotorConfigs);
        rightMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        rightMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        rightMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ShooterConstants.kShooterNeutralDeadband;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        rightMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        rightMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        rightMotorConfigs.CurrentLimits.StatorCurrentLimit = 150;
        rightMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotorConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode leftResponse  = leftShooterConfigurator.apply(leftMotorConfigs);
        StatusCode rightResponse = rightShooterConfigurator.apply(rightMotorConfigs);

        if (!leftResponse.isOK()){
            DriverStation.reportError("Could not apply left shooter configs, error code:"+ leftResponse.toString(), new Error().getStackTrace());
        }
        if (!rightResponse.isOK()){
            DriverStation.reportError("Could not apply right shooter configs, error code:"+ rightResponse.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        ShooterConstants.kTopOuttakeHigh.loadPreferences();
        ShooterConstants.kTopOuttakeLow.loadPreferences();
        ShooterConstants.kTopOuttakeAmp.loadPreferences();
        ShooterConstants.kTopOuttakeAuto1.loadPreferences();
        ShooterConstants.kTopOuttakeAuto2.loadPreferences();
        ShooterConstants.kTopIntake.loadPreferences();

        ShooterConstants.kBottomOuttakeHigh.loadPreferences();
        ShooterConstants.kBottomOuttakeLow.loadPreferences();
        ShooterConstants.kBottomOuttakeAmp.loadPreferences();
        ShooterConstants.kBottomOuttakeAuto1.loadPreferences();
        ShooterConstants.kBottomOuttakeAuto2.loadPreferences();
        ShooterConstants.kBottomIntake.loadPreferences();
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        
        leftShooterConfigurator.refresh(leftMotorConfigs);
        ShooterConstants.kPLeftMotor.loadPreferences();
        ShooterConstants.kILeftMotor.loadPreferences();
        ShooterConstants.kDLeftMotor.loadPreferences();
        ShooterConstants.kVLeftMotor.loadPreferences();
        leftMotorConfigs.Slot0.kP = ShooterConstants.kPLeftMotor.get();
        leftMotorConfigs.Slot0.kI = ShooterConstants.kILeftMotor.get();
        leftMotorConfigs.Slot0.kD = ShooterConstants.kDLeftMotor.get();
        leftMotorConfigs.Slot0.kV = ShooterConstants.kVLeftMotor.get();
        
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
        
        rightShooterConfigurator.refresh(rightMotorConfigs);
        ShooterConstants.kPRightMotor.loadPreferences();
        ShooterConstants.kIRightMotor.loadPreferences();
        ShooterConstants.kDRightMotor.loadPreferences();
        ShooterConstants.kVRightMotor.loadPreferences();
        rightMotorConfigs.Slot0.kP = ShooterConstants.kPRightMotor.get();
        rightMotorConfigs.Slot0.kI = ShooterConstants.kIRightMotor.get();
        rightMotorConfigs.Slot0.kD = ShooterConstants.kDRightMotor.get();
        rightMotorConfigs.Slot0.kV = ShooterConstants.kVRightMotor.get();

        StatusCode leftResponse  = leftShooterConfigurator.apply(leftMotorConfigs);
        StatusCode rightResponse = rightShooterConfigurator.apply(rightMotorConfigs);

        if (!leftResponse.isOK()){
            DriverStation.reportError("Could not apply left shooter configs, error code:"+ leftResponse.toString(), new Error().getStackTrace());
        }
        if (!rightResponse.isOK()){
            DriverStation.reportError("Could not apply right shooter configs, error code:"+ rightResponse.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            leftVelocityRequest.Velocity = 0;
            rightVelocityRequest.Velocity = 0;
            leftShooter.setControl(leftVelocityRequest);
            rightShooter.setControl(rightVelocityRequest);
            return;
        }


        // if (velocityControl) {
        leftShooter.setControl(leftVelocityRequest);
        rightShooter.setControl(rightVelocityRequest);
        //     return;
        // } \][]

        // leftVoltageRequest.Output = leftVelocityRequest.Velocity * 12 / 100.0;
        // rightVoltageRequest.Output = rightVelocityRequest.Velocity * 12 / 100.0;
        // leftShooter.setControl(leftVoltageRequest);
        // rightShooter.setControl(rightVoltageRequest);
    }

    //****************************** STATE METHODS ******************************//

    public void stop() {
        this.enabled = false;
        leftVelocityRequest.Velocity = 0;
        rightVelocityRequest.Velocity = 0;
        // leftShooter.setControl(brakeRequest);
        // rightShooter.setControl(brakeRequest);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    //****************************** VELOCITY METHODS ******************************//

    public void setLeftVelocity(double velocity) {
        leftVelocityRequest.Velocity = 
            NerdyMath.clamp(
                velocity, 
                ShooterConstants.kShooterMinVelocityRPS, 
                ShooterConstants.kShooterMaxVelocityRPS);
    }

    public void setRightVelocity(double velocity) {
        rightVelocityRequest.Velocity = 
            NerdyMath.clamp(
                velocity, 
                ShooterConstants.kShooterMinVelocityRPS, 
                ShooterConstants.kShooterMaxVelocityRPS);
    }

    public void setVelocity(double left, double right) {
        setLeftVelocity(left);
        setRightVelocity(right);
    }

    public void incrementLeftVelocity(double increment) {
        setLeftVelocity(leftVelocityRequest.Velocity + increment);
    }

    public void incrementRightVelocity(double increment) {
        setRightVelocity(rightVelocityRequest.Velocity + increment);
    }

    public void incrementVelocity(double leftIncrement, double rightIncrement) {
        incrementLeftVelocity(leftIncrement);
        incrementRightVelocity(rightIncrement);
    }

    //****************************** VELOCITY COMMANDS ******************************//

    public Command setLeftVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setLeftVelocity(velocity));
    }

    public Command setRightVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setRightVelocity(velocity));
    }

    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity, velocity));
    }

    public Command setVelocityCommand(double topVelocity, double bottomVelocity) {
        return Commands.runOnce(() -> setVelocity(topVelocity, bottomVelocity));
    }

    public Command setReverseVelocityCommand(double topVelocity, double bottomVelocity) {
        return Commands.runOnce(() -> setVelocity(topVelocity, bottomVelocity));
    }

    public Command incrementLeftVelocityCommand(double increment) {
        return Commands.runOnce(() -> incrementLeftVelocity(increment));
    }

    public Command incrementRightVelocityCommand(double increment) {
        return Commands.runOnce(() -> incrementRightVelocity(increment));
    }

    public Command incrementVelocityCommand(double leftIncrement, double rightIncrement) {
        return Commands.runOnce(() -> incrementVelocity(leftIncrement, rightIncrement));
    }

    public Command rampLeftVelocity(double initialVelocity, double finalVelocity, double rampTimeSeconds) {
        final double initialVel = NerdyMath.clamp(initialVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        final double finalVel = NerdyMath.clamp(finalVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        
        // Change in Velocity / Command Scheduler Loops (assumes 20 hz)
        final double increment = (finalVel - initialVel) / (rampTimeSeconds * 20);

        // Check whether the current velocity is over/under final velocity
        BooleanSupplier rampFinished = 
            finalVel > initialVel ? 
                () -> leftVelocityRequest.Velocity >= finalVel : 
                () -> leftVelocityRequest.Velocity <= finalVel;

        if (initialVel == finalVel) {
            return setLeftVelocityCommand(finalVel);
        } 

        return 
            Commands.sequence(
                setLeftVelocityCommand(initialVel),
                Commands.deadline(
                    Commands.waitUntil(rampFinished),
                    incrementLeftVelocityCommand(increment)
                )
            );
    }

    public Command rampRightVelocity(double initialVelocity, double finalVelocity, double rampTimeSeconds) {
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
            return setRightVelocityCommand(finalVel);
        } 

        return 
            Commands.sequence(
                setRightVelocityCommand(initialVel),
                Commands.deadline(
                    Commands.waitUntil(rampFinished),
                    incrementRightVelocityCommand(increment)
                )
            );
    }

    public boolean atVelocity(double velocity) {
        return leftShooter.getVelocity().getValue() > velocity;
    }

    public boolean atTargetVelocity() {
        return atVelocity(leftVelocityRequest.Velocity);
    }

    public Command rampVelocity(double initialVelocity, double finalVelocity, double rampTimeSeconds) {
        return Commands.parallel(
            rampLeftVelocity(initialVelocity, finalVelocity, rampTimeSeconds),
            rampRightVelocity(initialVelocity, finalVelocity, rampTimeSeconds)
        );
    }

    public Command shootSpeaker() {
        return setVelocityCommand(ShooterConstants.kTopOuttakeHigh.get());
    }

    public Command shootSpeakerSlow() {
        return setVelocityCommand(ShooterConstants.kTopOuttakeLow.get());
    }

    public Command shootSpeakerAuto1() {
        return setVelocityCommand(ShooterConstants.kTopOuttakeAuto1.get(), ShooterConstants.kBottomOuttakeAuto1.get());
    }

    public Command shootAmp() {
        return setVelocityCommand(ShooterConstants.kTopOuttakeAmp.get());
    }

    //****************************** LOGGING METHODS ******************************//

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addBoolean("Shooter Enabled", () -> this.enabled);
        tab.addNumber("Left Velocity", ()-> leftShooter.getVelocity().getValueAsDouble());
        tab.addNumber("Right Velocity", ()-> rightShooter.getVelocity().getValueAsDouble());
        tab.addNumber("Left Target Velocity", ()-> leftVelocityRequest.Velocity);
        tab.addNumber("Right Target Velocity", ()-> rightVelocityRequest.Velocity);
        tab.addNumber("Left Supply Current", () -> leftShooter.getSupplyCurrent().getValueAsDouble());
        tab.addNumber("Left Stator Current", () -> leftShooter.getStatorCurrent().getValueAsDouble());
        tab.addNumber("Right Supply Current", () -> rightShooter.getSupplyCurrent().getValueAsDouble());
        tab.addNumber("Right Stator Current", () -> rightShooter.getStatorCurrent().getValueAsDouble());
        tab.addDouble("Right Shooter Applied Voltage", () -> rightShooter.getMotorVoltage().getValueAsDouble());
        tab.addDouble("Left Shooter Applied Voltage", () -> leftShooter.getMotorVoltage().getValueAsDouble());
        
    }
}

