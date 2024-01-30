package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;

public class ShooterPivot extends SubsystemBase{
    
    final TalonFX pivot;
    DutyCycleEncoder throughBore;

    final VoltageOut pivotVoltageRequest = new VoltageOut(0);

    final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);

    final NeutralOut brake = new NeutralOut();

    private double TargetPosition = 0;

    public ShooterPivot(){
        pivot = new TalonFX(ShooterConstants.kPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        throughBore = new DutyCycleEncoder(ShooterConstants.kThroughBorePort);

        pivot.setInverted(false);
        throughBore.setDistancePerRotation(1);
        
        init();
    }


    public void configureMotor() {
        TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();
        pivot.getConfigurator().refresh(pivotMotorConfigs);

        pivotMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotMotorConfigs.Feedback.RotorToSensorRatio = ShooterConstants.kGearRatio;
        pivotMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        pivotMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        pivotMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        pivotMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        pivotMotorConfigs.Audio.AllowMusicDurDisable = true;

        MotionMagicConfigs pivotMMConfigs = pivotMotorConfigs.MotionMagic;
        pivotMMConfigs.MotionMagicCruiseVelocity = ShooterConstants.kShooterCruiseVelocity;
        pivotMMConfigs.MotionMagicAcceleration = ShooterConstants.kShooterCruiseAcceleration;
        
        
        pivot.getConfigurator().apply(pivotMotorConfigs);
    }

    public void configurePID() {
        TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();

        ShooterConstants.kPPivotMotor.loadPreferences();
        ShooterConstants.kIPivotMotor.loadPreferences();
        ShooterConstants.kDPivotMotor.loadPreferences();
        ShooterConstants.kVPivotMotor.loadPreferences();

        pivotMotorConfigs.Slot0.kP = ShooterConstants.kPPivotMotor.get();
        pivotMotorConfigs.Slot0.kI = ShooterConstants.kIPivotMotor.get();
        pivotMotorConfigs.Slot0.kD = ShooterConstants.kDPivotMotor.get();
        pivotMotorConfigs.Slot0.kV = ShooterConstants.kVPivotMotor.get();

        StatusCode statusPivot = pivot.getConfigurator().apply(pivotMotorConfigs);

        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), null);
        }
    }

    public void init() {
        configurePID();
        configureMotor();
        resetEncoder();
    }

    public void resetEncoder() {
        pivot.setPosition(throughBore.getAbsolutePosition());
    }

    public void setPosition(double position) {
        // m_pivotMotionMagicRequest.Slot = 0;
        pivot.setControl(m_pivotMotionMagicRequest.withPosition(position));
    }

    public void manualControlPosition(double tickChange) {
        double pos = (pivot.getPosition().getValueAsDouble() * 2048) + tickChange; // Increase by 200 ticks?
        pivot.setPosition(pos);
    }

    public void stowShooter() {
        setPosition(ShooterConstants.kSpeakerPosition);
    }

    public void setAmpPosition() {
        setPosition(ShooterConstants.kAmpPosition);
    }

    public void setSpeakerPosition() {
        setPosition(ShooterConstants.kSpeakerPosition);
    }

    public void setHandoffPosition() {
        setPosition(ShooterConstants.kHandoffPosition);
    }

    public void setShooterPowerZeroCommand() {
        pivot.setControl(m_brake);
        SmartDashboard.putBoolean("Pressed", false);
    }

    public boolean reachNeutralPosition(){
        if (NerdyMath.inRange(pivot.getPosition().getValue(), ShooterConstants.kNeutralPosition - ShooterConstants.kPivotDeadband.get(), ShooterConstants.kNeutralPosition + ShooterConstants.kPivotDeadband.get())){
            return true;   
        } else {
            return false;
        }
    }

}  
