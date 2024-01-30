package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;

public class IntakePivot extends SubsystemBase{
    private final TalonFX pivot;
    private final TalonFXConfigurator pivotConfigurator;
    private final DutyCycleEncoder throughBore;

    private boolean enabled = false;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();

    public IntakePivot() {
        pivot = new TalonFX(IntakeConstants.kPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        throughBore = new DutyCycleEncoder(IntakeConstants.kThroughBorePort);
        pivotConfigurator = pivot.getConfigurator();
        
        pivot.setInverted(false);
        configureMotor();
        configurePID();
        resetEncoder();
    }

    public void configureMotor() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        pivotConfigurator.refresh(intakeConfigs);
        intakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeConfigs.Feedback.RotorToSensorRatio = 1;
        intakeConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.kGearRatio;
        intakeConfigs.Voltage.PeakForwardVoltage = 11.5;
        intakeConfigs.Voltage.PeakReverseVoltage = -11.5;
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        intakeConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        intakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        intakeConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        intakeConfigs.Audio.AllowMusicDurDisable = true;
        pivotConfigurator.apply(intakeConfigs);
    }

    public void configurePID() {
        TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();
        pivotConfigurator.refresh(pivotMotorConfigs);
        IntakeConstants.kPPivotMotor.loadPreferences();
        IntakeConstants.kIPivotMotor.loadPreferences();
        IntakeConstants.kDPivotMotor.loadPreferences();
        IntakeConstants.kVPivotMotor.loadPreferences();
        pivotMotorConfigs.Slot0.kP = IntakeConstants.kPPivotMotor.get();
        pivotMotorConfigs.Slot0.kI = IntakeConstants.kIPivotMotor.get();
        pivotMotorConfigs.Slot0.kD = IntakeConstants.kDPivotMotor.get();
        pivotMotorConfigs.Slot0.kV = IntakeConstants.kVPivotMotor.get();
        pivotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kIntakeCruiseVelocity;
        pivotMotorConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.kIntakeCruiseAcceleration;
        StatusCode statusPivot = pivotConfigurator.apply(pivotMotorConfigs);

        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), null);
        }
    }

    @Override
    public void periodic() {
        if (enabled) {
            pivot.setControl(motionMagicRequest);
        } else {
            pivot.setControl(brakeRequest);
        }
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> {
            pivot.setPosition(throughBore.getAbsolutePosition() * IntakeConstants.kGearRatio);
        });
    }

    // Checks if the pivot is within deadband of the target pos
    public boolean atTargetPosition() {
        return NerdyMath.inRange(
            pivot.getPosition().getValueAsDouble(), 
            motionMagicRequest.Position - IntakeConstants.kPivotDeadband.get(), 
            motionMagicRequest.Position + IntakeConstants.kPivotDeadband.get());
    }

    public void stop() {
        enabled = false;
        pivot.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    
    public void setPosition(double position) {
        motionMagicRequest.Position = position;
    }

    public void incrementPosition(double increment) {
        double newPos = motionMagicRequest.Position + increment;
        if (NerdyMath.inRange(newPos, IntakeConstants.kIntakeMinPos, IntakeConstants.kIntakeMaxPos)) {
            motionMagicRequest.Position = newPos;
        }
            
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
