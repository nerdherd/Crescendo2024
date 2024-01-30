package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SuperStructureConstants;

public class ShooterRoller extends SubsystemBase{
    
    final TalonFX leftShooter;
    final TalonFX rightShooter;
        
    int velocityLeft = 0; // 11700 is 60% output
    int velocityRight = 0;

    private boolean enabled = true;

    private final NeutralOut brakeRequest = new NeutralOut();

    final VoltageOut m_leftVoltageRequest = new VoltageOut(0);
    final VoltageOut m_rightVoltageRequest = new VoltageOut(0);

    final DutyCycleOut m_leftDutyCycleRequest = new DutyCycleOut(0);
    final DutyCycleOut m_rightDutyCycleRequest = new DutyCycleOut(0);

    final VelocityVoltage m_leftVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    final VelocityVoltage m_rightVelocityRequest = new VelocityVoltage(0, 0, true, 0,0, false, false, false);

    final NeutralOut m_brake = new NeutralOut();

    public ShooterRoller(){
        leftShooter = new TalonFX(ShooterConstants.kLeftMotorID, SuperStructureConstants.kCANivoreBusName);
        rightShooter = new TalonFX(ShooterConstants.kRightMotorID, SuperStructureConstants.kCANivoreBusName);

        leftShooter.setInverted(false);
        // rightShooter.setControl(new Follower(leftShooter.getDeviceID(), false));
        
        configurePID();
    }

    public void configureMotor() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();

        leftShooter.getConfigurator().refresh(leftMotorConfigs);

        leftMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        leftMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        leftMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        leftMotorConfigs.Audio.AllowMusicDurDisable = true;
        leftShooter.getConfigurator().apply(leftMotorConfigs);


        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
        rightShooter.getConfigurator().refresh(rightMotorConfigs);
        rightMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        rightMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        rightMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        rightMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        rightMotorConfigs.Audio.AllowMusicDurDisable = true;
        rightShooter.getConfigurator().apply(rightMotorConfigs);

    }

    public void configurePID() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        
        leftShooter.getConfigurator().refresh(leftMotorConfigs);
        ShooterConstants.kPLeftMotor.loadPreferences();
        ShooterConstants.kILeftMotor.loadPreferences();
        ShooterConstants.kDLeftMotor.loadPreferences();
        ShooterConstants.kVLeftMotor.loadPreferences();
        leftMotorConfigs.Slot0.kP = ShooterConstants.kPLeftMotor.get();
        leftMotorConfigs.Slot0.kI = ShooterConstants.kILeftMotor.get();
        leftMotorConfigs.Slot0.kD = ShooterConstants.kDLeftMotor.get();
        leftMotorConfigs.Slot0.kV = ShooterConstants.kVLeftMotor.get();
        
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
        
        rightShooter.getConfigurator().refresh(rightMotorConfigs);
        ShooterConstants.kPRightMotor.loadPreferences();
        ShooterConstants.kIRightMotor.loadPreferences();
        ShooterConstants.kDRightMotor.loadPreferences();
        ShooterConstants.kVRightMotor.loadPreferences();
        rightMotorConfigs.Slot0.kP = ShooterConstants.kPRightMotor.get();
        rightMotorConfigs.Slot0.kI = ShooterConstants.kIRightMotor.get();
        rightMotorConfigs.Slot0.kD = ShooterConstants.kDRightMotor.get();
        rightMotorConfigs.Slot0.kV = ShooterConstants.kVRightMotor.get();

        StatusCode statusLeft = leftShooter.getConfigurator().apply(leftMotorConfigs);
        StatusCode statusRight = rightShooter.getConfigurator().apply(rightMotorConfigs);

        if (!statusLeft.isOK()){
            DriverStation.reportError("Could not apply left shooter configs, error code:"+ statusLeft.toString(), null);
        }
        if (!statusRight.isOK()){
            DriverStation.reportError("Could not apply right shooter configs, error code:"+ statusRight.toString(), null);
        }
    }

    public void run() {
        if (enabled) {
            leftShooter.setControl(brakeRequest);
            rightShooter.setControl(brakeRequest);
        } else {
            leftShooter.setControl(m_leftVelocityRequest);
            rightShooter.setControl(m_rightVelocityRequest);
        }
    }

    public void stop() {
        this.enabled = false;
        m_leftVelocityRequest.Velocity = 0;
        m_rightVelocityRequest.Velocity = 0;
        leftShooter.setControl(brakeRequest);
        rightShooter.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setVelocity(double velocity) {
        m_leftVelocityRequest.Velocity = velocity;
        m_rightVelocityRequest.Velocity = velocity;
    }

    public void incrementLeftVelocity(double increment) {
        double newLeftVelocity = m_leftVelocityRequest.Velocity + increment;

        if ((increment > 0 && newLeftVelocity < ShooterConstants.kShooterMaxVelocityRPS) ||
            (increment < 0 && newLeftVelocity > -ShooterConstants.kShooterMaxVelocityRPS)
            ) {
            m_leftVelocityRequest.Velocity = newLeftVelocity;
        }
    }

    public void incrementRightVelocity(double increment) {
        double newRightVelocity = m_rightVelocityRequest.Velocity + increment;

        if ((increment > 0 && newRightVelocity < ShooterConstants.kShooterMaxVelocityRPS) ||
            (increment < 0 && newRightVelocity > -ShooterConstants.kShooterMaxVelocityRPS)
            ) {
            m_leftVelocityRequest.Velocity = newRightVelocity;
        }
    }
    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addNumber("Left Velocity", ()-> leftShooter.getVelocity().getValueAsDouble());
        tab.addNumber("Right Velocity", ()-> rightShooter.getVelocity().getValueAsDouble());
    }

}

