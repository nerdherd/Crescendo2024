package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot {
    
    // final TalonFX rightIntake;
    final TalonFX pivot;
    final DutyCycleEncoder throughBore;

    int velocityIntake = 0;
    int targetTicks;

    final VoltageOut m_intakeVoltageRequest = new VoltageOut(0);
    // final VoltageOut m_rightVoltageRequest = new VoltageOut(0);
    final VoltageOut m_pivotVoltageRequest = new VoltageOut(0);

    final DutyCycleOut m_intakeDutyCycleRequest = new DutyCycleOut(0);
    // final DutyCycleOut m_rightDutyCycleRequest = new DutyCycleOut(0);

    final VelocityVoltage m_intakeVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // final VelocityVoltage m_rightVelocity = new VelocityVoltage(0, 0, true, 0,0, false, false, false);

    final PositionVoltage m_pivotPositionRequest = new PositionVoltage(0, 0, true, 0,0, false, false, false);
    final MotionMagicVoltage m_pivotMotionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);

    final NeutralOut m_brake = new NeutralOut();

    public BooleanSupplier atTargetPosition;

    public IntakePivot(){
        // rightIntake = new TalonFX(IntakeConstants.kRightMotorID, ModuleConstants.kCANivoreName);
        pivot = new TalonFX(IntakeConstants.kPivotMotorID, ModuleConstants.kCANivoreName);
        throughBore = new DutyCycleEncoder(IntakeConstants.kThroughBorePort);

        // rightIntake.setControl(new Follower(intake.getDeviceID(), false));
        pivot.setInverted(false);
        init();

        atTargetPosition = () -> NerdyMath.inRange(pivot.getPosition().getValueAsDouble() * 2048, targetTicks - 40000, targetTicks + 40000);
    }

    public void configurePID() {

        TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();

        pivot.getConfigurator().refresh(pivotMotorConfigs);
        
        IntakeConstants.kPPivotMotor.loadPreferences();
        IntakeConstants.kIPivotMotor.loadPreferences();
        IntakeConstants.kDPivotMotor.loadPreferences();
        IntakeConstants.kVPivotMotor.loadPreferences();

        pivotMotorConfigs.Slot0.kP = IntakeConstants.kPPivotMotor.get();
        pivotMotorConfigs.Slot0.kI = IntakeConstants.kIPivotMotor.get();
        pivotMotorConfigs.Slot0.kD = IntakeConstants.kDPivotMotor.get();
        pivotMotorConfigs.Slot0.kV = IntakeConstants.kVPivotMotor.get();

        MotionMagicConfigs pivotMMConfigs = pivotMotorConfigs.MotionMagic;
        pivotMMConfigs.MotionMagicCruiseVelocity = IntakeConstants.kIntakeCruiseVelocity;
        pivotMMConfigs.MotionMagicAcceleration = IntakeConstants.kIntakeCruiseAcceleration;


        // rightMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        // rightMotorConfigs.Voltage.PeakReverseVoltage = -11.5;

        pivotMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotMotorConfigs.Voltage.PeakReverseVoltage = -11.5;

        // StatusCode statusRight = rightIntake.getConfigurator().apply(rightMotorConfigs);
        StatusCode statusPivot = pivot.getConfigurator().apply(pivotMotorConfigs);

        // if (!statusRight.isOK()){
        //     DriverStation.reportError("Could not apply right configs, error code:"+ statusRight.toString(), null);
        // }
        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), null);
        }
    }

    public void init() {
        resetEncoder();
        configurePID();
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> {
            pivot.setPosition(throughBore.getAbsolutePosition() * IntakeConstants.kGearRatio);
        });
    }

    public Command setIntakePowerZeroCommand() {
        return Commands.runOnce(() -> {
            pivot.setControl(m_brake);
            // rightIntake.setControl(m_brake);
            resetEncoder();
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public void setIntakePowerZero() {
        pivot.setControl(m_brake);
        resetEncoder();
        // rightIntake.setControl(m_brake);
        SmartDashboard.putBoolean("Pressed", false);

    }
    
    // public Command increaseRight() {
    //     return Commands.runOnce(() -> {

    //         if (velocityRight <= 21000) {
    //             velocityRight += 1024;
    //             tooHigh = false;
    //         }
    //         else {
    //             tooHigh = true;
    //         }
    //         SmartDashboard.putBoolean("Too high", tooHigh);

    //     });
    // }

    public Command setPosition(double position) {
        return Commands.runOnce(() -> {
            //m_pivotMotionMagicRequest.Slot = 0;
            pivot.setControl(m_pivotMotionMagicRequest.withPosition(position));

        });
    }

    public Command stowIntake() {
        return Commands.runOnce(() -> {
            setPosition(IntakeConstants.kStowPosition);
        });
    }

    public Command intakePosition() {
        return Commands.runOnce(() -> {
            setPosition(IntakeConstants.kPickupPosition);
        });
    }

}
