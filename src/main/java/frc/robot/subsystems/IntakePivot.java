package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends SubsystemBase{
    private final TalonFX pivot;
    private final DutyCycleEncoder throughBore;

    private final MotionMagicVoltage m_pivotMotionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
    private final NeutralOut m_brake = new NeutralOut();

    public BooleanSupplier atTargetPosition;

    public IntakePivot(){
        pivot = new TalonFX(IntakeConstants.kPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        throughBore = new DutyCycleEncoder(IntakeConstants.kThroughBorePort);
        
        pivot.setInverted(false);
        init();
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

        pivotMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotMotorConfigs.Voltage.PeakReverseVoltage = -11.5;

        StatusCode statusPivot = pivot.getConfigurator().apply(pivotMotorConfigs);

        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), null);
        }
    }

    public void init() {
        resetEncoder();
        configurePID();
    }

    public boolean atTargetPosition() {
        return true;
// NerdyMath.inRange(pivot.getPosition().getValueAsDouble() * 2048, targetTicks - 40000, targetTicks + 40000);
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> {
            pivot.setPosition(throughBore.getAbsolutePosition() * IntakeConstants.kGearRatio);
        });
    }

    public Command setIntakePowerZeroCommand() {
        return Commands.runOnce(() -> {
            pivot.setControl(m_brake);
            resetEncoder();
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public void setIntakePowerZero() {
        pivot.setControl(m_brake);
        resetEncoder();
        SmartDashboard.putBoolean("Pressed", false);

    }
    
    public Command setPosition(double position) {
        return Commands.runOnce(() -> {
            pivot.setControl(m_pivotMotionMagicRequest.withPosition(position));
        });
    }

    public Command manualControlPosition(double tickChange) {
        double pos = (pivot.getPosition().getValueAsDouble() * 2048) + tickChange; // Increase by 200 ticks?
        return Commands.runOnce(() -> {
            pivot.setPosition(pos);
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

    public boolean reachNeutralPosition() {
        if (NerdyMath.inRange(pivot.getPosition().getValue(), IntakeConstants.kNeutralPosition - IntakeConstants.kPivotDeadband.get(), IntakeConstants.kNeutralPosition + IntakeConstants.kPivotDeadband.get())
            || (NerdyMath.inRange(pivot.getPosition().getValue(), IntakeConstants.kPickupPosition - IntakeConstants.kPivotDeadband.get(), IntakeConstants.kPickupPosition + IntakeConstants.kPivotDeadband.get()))) {
            return true;
        }
        else {
            return false;
        }
    }

}
