package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.NerdyMath;

public class ShooterPivot extends SubsystemBase{
    
    final TalonFX pivot;
    DutyCycleEncoder throughBore;

    final VoltageOut m_pivotVoltageRequest = new VoltageOut(0);

    final MotionMagicVoltage m_pivotMotionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);

    final NeutralOut m_brake = new NeutralOut();

    private double TargetPosition = 0;

    public ShooterPivot(){
        pivot = new TalonFX(ShooterConstants.kPivotMotorID, ModuleConstants.kCANivoreName);
        throughBore = new DutyCycleEncoder(ShooterConstants.kThroughBorePort);

        // rightShooter.setControl(new Follower(leftShooter.getDeviceID(), false));
        pivot.setInverted(false);
        throughBore.setDistancePerRotation(1);
        
        init();
    }

    public void configurePID() {
        TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();

        pivot.getConfigurator().refresh(pivotMotorConfigs);
        ShooterConstants.kPPivotMotor.loadPreferences();
        ShooterConstants.kIPivotMotor.loadPreferences();
        ShooterConstants.kDPivotMotor.loadPreferences();
        ShooterConstants.kVPivotMotor.loadPreferences();

        pivotMotorConfigs.Slot0.kP = ShooterConstants.kPPivotMotor.get();
        pivotMotorConfigs.Slot0.kI = ShooterConstants.kIPivotMotor.get();
        pivotMotorConfigs.Slot0.kD = ShooterConstants.kDPivotMotor.get();
        pivotMotorConfigs.Slot0.kV = ShooterConstants.kVPivotMotor.get();

        MotionMagicConfigs pivotMMConfigs = pivotMotorConfigs.MotionMagic;
        pivotMMConfigs.MotionMagicCruiseVelocity = ShooterConstants.kShooterCruiseVelocity;
        pivotMMConfigs.MotionMagicAcceleration = ShooterConstants.kShooterCruiseAcceleration;

        pivotMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        StatusCode statusPivot = pivot.getConfigurator().apply(pivotMotorConfigs);

        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), null);
        }
    }

    public void init() {
        configurePID();
        resetEncoder();
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> {
            pivot.setPosition(throughBore.getAbsolutePosition() * 2048 * ShooterConstants.kGearRatio);
        });
    }

    public Command setPosition(double position) {
        return Commands.runOnce(() -> {
           // m_pivotMotionMagicRequest.Slot = 0;
            pivot.setControl(m_pivotMotionMagicRequest.withPosition(position));

        });
    }

    public Command manualControlPosition(double tickChange) {
        double pos = (pivot.getPosition().getValueAsDouble() * 2048) + tickChange; // Increase by 200 ticks?
        return Commands.runOnce(() -> {
            pivot.setPosition(pos);
        });
    }

    public Command stowShooter() {
        return Commands.runOnce(() -> {
            setPosition(ShooterConstants.kSpeakerPosition);
        });
    }

    public Command setAmpPosition() {
        return Commands.runOnce(() -> {
            setPosition(ShooterConstants.kAmpPosition);

        });
    }

    public Command setSpeakerPosition() {
        return Commands.runOnce(() -> {
            setPosition(ShooterConstants.kSpeakerPosition);
        });
    }

    public Command setHandoffPosition() {
        return Commands.runOnce(() -> {
            setPosition(ShooterConstants.kHandoffPosition);
        });
    }

    public Command setShooterPowerZeroCommand() {
        return Commands.runOnce(() -> {
            pivot.setControl(m_brake);
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public boolean reachNeutralPosition(){
        if (NerdyMath.inRange(pivot.getPosition().getValue(), ShooterConstants.kNeutralPosition - ShooterConstants.kPivotDeadband.get(), ShooterConstants.kNeutralPosition + ShooterConstants.kPivotDeadband.get())){
            return true;   
        } else {
            return false;
        }
    }

}  
