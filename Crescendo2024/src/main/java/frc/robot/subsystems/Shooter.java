package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter {
    
    final TalonFX leftShooter;
    final TalonFX rightShooter;
    final TalonFX pivot;

    DutyCycleEncoder throughBore;
        
    int velocityLeft = 0; // 11700 is 60% output
    int velocityRight = 0;

    private boolean tooHigh = false;
    private boolean tooLow = false;

    final VoltageOut m_leftVoltageRequest = new VoltageOut(0);
    final VoltageOut m_rightVoltageRequest = new VoltageOut(0);
    final VoltageOut m_pivotVoltageRequest = new VoltageOut(0);

    final DutyCycleOut m_leftDutyCycleRequest = new DutyCycleOut(0);
    final DutyCycleOut m_rightDutyCycleRequest = new DutyCycleOut(0);

    final VelocityVoltage m_leftVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    final VelocityVoltage m_rightVelocity = new VelocityVoltage(0, 0, true, 0,0, false, false, false);

    final NeutralOut m_brake = new NeutralOut();

    public Shooter(){
        leftShooter = new TalonFX(ShooterConstants.kLeftMotorID, ModuleConstants.kCANivoreName);
        rightShooter = new TalonFX(ShooterConstants.kRightMotorID, ModuleConstants.kCANivoreName);
        pivot = new TalonFX(ShooterConstants.kPivotMotorID, ModuleConstants.kCANivoreName);
        throughBore = new DutyCycleEncoder(ShooterConstants.kThroughBorePort);

        leftShooter.setInverted(false);
        // rightShooter.setControl(new Follower(leftShooter.getDeviceID(), false));
        pivot.setInverted(false);

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

        leftMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        leftMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
        rightMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        rightMotorConfigs.Voltage.PeakReverseVoltage = -11.5;

        StatusCode statusLeft = leftShooter.getConfigurator().apply(rightMotorConfigs);
        StatusCode statusRight = rightShooter.getConfigurator().apply(leftMotorConfigs);

        if (!statusLeft.isOK()){
            DriverStation.reportError("Could not apply left shooter configs, error code:"+ statusLeft.toString(), null);
        }
        if (!statusRight.isOK()){
            DriverStation.reportError("Could not apply right shooter configs, error code:"+ statusRight.toString(), null);
        }
    }

    public Command setSpeed() {
        return Commands.runOnce(() -> {

            // Percent Ouput
            // leftShooter.setControl(m_leftDutyCycleRequest.withOutput(leftSpeeds[index]));
            // leftShooter.setControl(m_leftDutyCycleRequest.withOutput(rightSpeeds[index]));

            // Velocity Control
            m_leftVelocity.Slot = 0;
            m_rightVelocity.Slot = 0;

            leftShooter.setControl(m_leftVelocity.withVelocity(velocityLeft));
            rightShooter.setControl(m_rightVelocity.withVelocity(velocityRight));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }

    public Command setPowerZeroCommand() {
        return Commands.runOnce(() -> {
            leftShooter.setControl(m_brake);
            rightShooter.setControl(m_brake);
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public void setPowerZero() {
        leftShooter.setControl(m_brake);
        rightShooter.setControl(m_brake);
        SmartDashboard.putBoolean("Pressed", false);

    }

    public Command increaseLeft() {
        return Commands.runOnce(() -> {

            if (velocityLeft <= 21000) {
                velocityLeft += 1024;
                tooHigh = false;
            }
            else {
                tooHigh = true;
            }
            SmartDashboard.putBoolean("Too high", tooHigh);
        });
    }
    public Command increaseRight() {
        return Commands.runOnce(() -> {

            if (velocityRight <= 21000) {
                velocityRight += 1024;
                tooHigh = false;
            }
            else {
                tooHigh = true;
            }
            SmartDashboard.putBoolean("Too high", tooHigh);

        });
    }

    public Command decreaseLeft() {
        return Commands.runOnce(() -> {
            // if(percentOutputTop >= 0.051) {
            //     this.speedsLeft[2] -= 0.05; // TODO DEBUG
            //     percentOutputTop -= 0.05;
            //     tooLow = false;

            if(velocityLeft >= 1100) {
                velocityLeft -= 1024;
                tooLow = false;
            }
            else {
                tooLow = true;
            }
            SmartDashboard.putBoolean("Too low", tooLow);
        });
    }
    public Command decreaseRight() {
        return Commands.runOnce(() -> {
            // if(percentOutputBottom >= 0.051) {
            //     this.speedsRight[2] -= 0.05; // TODO DEBUG
            //     percentOutputBottom -= 0.05;
            //     tooLow = false;
            if(velocityRight >= 1100) {
                velocityRight -= 1024;
                tooLow = false;
            }
            else {
                tooLow = true;
            }
            SmartDashboard.putBoolean("Too low", tooLow);
        });
    }

    public void printSpeeds() {
        SmartDashboard.putNumber("Ticks Per Second Top: ", velocityLeft);
        SmartDashboard.putNumber("Ticks Per Second Bottom ", velocityRight);
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Top Velocity", ()-> leftShooter.getVelocity().getValueAsDouble());
        tab.addNumber("Bottom Velocity", ()-> rightShooter.getVelocity().getValueAsDouble());

    }

}
