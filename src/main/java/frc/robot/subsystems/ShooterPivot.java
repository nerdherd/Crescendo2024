package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase implements Reportable {
    private final TalonFX leftPivot;
    private final TalonFX rightPivot;
    private final TalonFXConfigurator leftPivotConfigurator;
    private final TalonFXConfigurator rightPivotConfigurator;
    private final DutyCycleEncoder throughBore;

    // Whether the pivot is running
    private boolean enabled = true;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(ShooterConstants.kFullStowPosition.get() / 360, true, 0, 0, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();
    private final Follower followRequest = new Follower(53, true);

    public ShooterPivot() {
        leftPivot = new TalonFX(ShooterConstants.kLeftPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        rightPivot = new TalonFX(ShooterConstants.kRightPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        throughBore = new DutyCycleEncoder(ShooterConstants.kThroughBorePort);
        leftPivotConfigurator = leftPivot.getConfigurator();
        rightPivotConfigurator = rightPivot.getConfigurator();
        rightPivot.setControl(followRequest);

        CommandScheduler.getInstance().registerSubsystem(this);

        configureMotor();
        configurePID();
        syncEncoder();
    }
    
    //****************************** SETUP METHODS ******************************/

    public void configureMotor() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        leftPivotConfigurator.refresh(pivotConfigs);
        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfigs.Feedback.RotorToSensorRatio = 1;
        pivotConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.kPivotGearRatio;
        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        pivotConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotConfigs.Voltage.PeakReverseVoltage = -11.5;
        
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = ShooterConstants.kShooterNeutralDeadband;

        pivotConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        pivotConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode leftStatusPivot = leftPivotConfigurator.apply(pivotConfigs);
        if (!leftStatusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ leftStatusPivot.toString(), new Error().getStackTrace());
        }

        pivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode rightStatusPivot = rightPivotConfigurator.apply(pivotConfigs);
        if (!rightStatusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ rightStatusPivot.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        ShooterConstants.kSpeakerPosition.loadPreferences();
        ShooterConstants.kSpeakerPosition2.loadPreferences();
        ShooterConstants.kAmpPosition.loadPreferences();
        ShooterConstants.kHandoffPosition.loadPreferences();
        ShooterConstants.kNeutralPosition.loadPreferences();

        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        leftPivotConfigurator.refresh(pivotConfigs);
        ShooterConstants.kPPivotMotor.loadPreferences();
        ShooterConstants.kIPivotMotor.loadPreferences();
        ShooterConstants.kDPivotMotor.loadPreferences();
        ShooterConstants.kVPivotMotor.loadPreferences();
        ShooterConstants.kSPivotMotor.loadPreferences();
        ShooterConstants.kAPivotMotor.loadPreferences();
        ShooterConstants.kGPivotMotor.loadPreferences();
        ShooterConstants.kCruiseVelocity.loadPreferences();
        ShooterConstants.kCruiseAcceleration.loadPreferences();
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfigs.Slot0.kP = ShooterConstants.kPPivotMotor.get();
        pivotConfigs.Slot0.kI = ShooterConstants.kIPivotMotor.get();
        pivotConfigs.Slot0.kD = ShooterConstants.kDPivotMotor.get();
        pivotConfigs.Slot0.kV = ShooterConstants.kVPivotMotor.get();
        pivotConfigs.Slot0.kS = ShooterConstants.kSPivotMotor.get();
        pivotConfigs.Slot0.kA = ShooterConstants.kAPivotMotor.get();
        pivotConfigs.Slot0.kG = ShooterConstants.kGPivotMotor.get();
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kCruiseVelocity.get();
        pivotConfigs.MotionMagic.MotionMagicAcceleration   = ShooterConstants.kCruiseAcceleration.get();

        StatusCode leftStatusPivot = leftPivotConfigurator.apply(pivotConfigs);
        if (!leftStatusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ leftStatusPivot.toString(), new Error().getStackTrace());
        }

        rightPivotConfigurator.refresh(pivotConfigs);

        ShooterConstants.kPPivotMotor.loadPreferences();
        ShooterConstants.kIPivotMotor.loadPreferences();
        ShooterConstants.kDPivotMotor.loadPreferences();
        ShooterConstants.kVPivotMotor.loadPreferences();
        ShooterConstants.kSPivotMotor.loadPreferences();
        ShooterConstants.kAPivotMotor.loadPreferences();
        ShooterConstants.kGPivotMotor.loadPreferences();
        ShooterConstants.kCruiseVelocity.loadPreferences();
        ShooterConstants.kCruiseAcceleration.loadPreferences();
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfigs.Slot0.kP = ShooterConstants.kPPivotMotor.get();
        pivotConfigs.Slot0.kI = ShooterConstants.kIPivotMotor.get();
        pivotConfigs.Slot0.kD = ShooterConstants.kDPivotMotor.get();
        pivotConfigs.Slot0.kV = ShooterConstants.kVPivotMotor.get();
        pivotConfigs.Slot0.kS = ShooterConstants.kSPivotMotor.get();
        pivotConfigs.Slot0.kA = ShooterConstants.kAPivotMotor.get();
        pivotConfigs.Slot0.kG = ShooterConstants.kGPivotMotor.get();
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kCruiseVelocity.get();
        pivotConfigs.MotionMagic.MotionMagicAcceleration   = ShooterConstants.kCruiseAcceleration.get();

        StatusCode rightStatusPivot = rightPivotConfigurator.apply(pivotConfigs);
        if (!rightStatusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ rightStatusPivot.toString(), new Error().getStackTrace());
        }
    }

    public void syncEncoder() {
        // Save a consistent position offset
        ShooterConstants.kPivotOffset.loadPreferences();
        // Throughbore uses revolutions, so divide by 360
        throughBore.setPositionOffset(ShooterConstants.kPivotOffset.get() / 360);

        // TalonFX uses revolutions
        double position = getAbsolutePositionRev();
        position = mapRev(position);

        leftPivot.setPosition(position);
        rightPivot.setPosition(position);
    }

    /**
     * Zero the through bore encoder and update the internal encoder
     * ONLY RUN FOR DEBUGGING
     */
    public void zeroAbsoluteEncoder() {
        throughBore.reset();
        ShooterConstants.kPivotOffset.set(throughBore.getPositionOffset() * 360);

        // Save new offset to Preferences
        ShooterConstants.kPivotOffset.uploadPreferences();
        syncEncoder();
    }

    public void zeroAbsoluteEncoderFullStow() {
        throughBore.reset();
        ShooterConstants.kFullStowPosition.loadPreferences();

        // The value is saved as degrees, but the throughbore reads it as revs
        throughBore.setPositionOffset((throughBore.getPositionOffset() + (ShooterConstants.kFullStowPosition.get() / 360)) % 1);

        ShooterConstants.kPivotOffset.set(throughBore.getPositionOffset() * 360);
        ShooterConstants.kPivotOffset.uploadPreferences();

        syncEncoder();
    }

    private int count = 0;
    
    @Override
    public void periodic() {
        count++;
        // SmartDashboard.putNumber("Shooter Count", count);
        if (count > 40) {
            syncEncoder();
            count = 0;
            // SmartDashboard.putBoolean("Shooter Reset", true);
        } else {
            // SmartDashboard.putBoolean("Shooter Reset", false);
        }

        if (ShooterConstants.fullDisableShooter.get()) {
            leftPivot.setControl(brakeRequest);
            rightPivot.setControl(brakeRequest);
            enabled = false;
            return;
        }
        
        // rightPivot.setControl(brakeRequest);
        // leftPivot.setControl(brakeRequest);

        if (enabled) {
            leftPivot.setControl(motionMagicRequest);
            rightPivot.setControl(followRequest);
        } else {
            rightPivot.setControl(brakeRequest);
            leftPivot.setControl(brakeRequest);
        }
    }

    /**
     * Maps the value to the range [-0.25, 0.75]
     */
    public double mapRev(double rev) {
        return NerdyMath.posMod(rev + 0.5, 1) - 0.5;
    }

    /**
     * Maps the value to the range [-180, 180]
     */
    public double mapDegrees(double deg) {
        return NerdyMath.posMod(deg + 180, 360) - 180;
    }

    //****************************** STATE METHODS ******************************/

    public double getTargetPositionRev() {
        return motionMagicRequest.Position;
    }

    public double getTargetPositionDegrees() {
        return getTargetPositionRev() * 360;
    }

    public double getPositionRev() {
        return leftPivot.getPosition().getValueAsDouble();
    }

    public double getPositionDegrees() {
        return getPositionRev() * 360;
    }

    public double getAbsolutePositionRev() {
        if (ShooterConstants.kPivotAbsoluteEncoderInverted) {
            return mapRev(throughBore.getPositionOffset() - throughBore.getAbsolutePosition());
        }
        return mapRev(throughBore.getAbsolutePosition() - throughBore.getPositionOffset());
    }

    public double getAbsolutePositionDegrees() {
        return getAbsolutePositionRev() * 360;
    }

    // Checks whether the pivot is within the deadband for a position
    public boolean hasReachedPosition(double positionDegrees) {
        return NerdyMath.inRange(
            getPositionDegrees(),
            positionDegrees - ShooterConstants.kPivotDeadband.get(), 
            positionDegrees + ShooterConstants.kPivotDeadband.get()
        ) && NerdyMath.inRange(
            getTargetPositionDegrees(),
            positionDegrees - ShooterConstants.kPivotDeadband.get(), 
            positionDegrees + ShooterConstants.kPivotDeadband.get()
        );
    }

    // Check if the shooter is in a safe position for the intake to move
    public boolean hasReachedNeutral() {
        return hasReachedPosition(ShooterConstants.kNeutralPosition.get());
    }

    // Checks if the pivot is within deadband of the target pos
    public boolean atTargetPosition() {
        return hasReachedPosition(getTargetPositionDegrees());
    }

    public void stop() {
        motionMagicRequest.Position = getPositionRev();
        enabled = false;
        leftPivot.setControl(brakeRequest);
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
    
    //****************************** POSITION METHODS ******************************//

    public void setPosition(double positionDegrees) {
        double newPos = NerdyMath.clamp(
                mapDegrees(positionDegrees),
                ShooterConstants.kPivotMinPos,
                ShooterConstants.kPivotMaxPos) / 360;
        motionMagicRequest.Position = newPos;
        SmartDashboard.putNumber("Shooter New Position", newPos);
        SmartDashboard.putNumber("Shooter New Position", newPos * 360);
    }

    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public void incrementPosition(double incrementDegrees) {
        SmartDashboard.putNumber("increment", incrementDegrees);
        if (Math.abs(incrementDegrees) <= 0.001) {
            return;
        } 
        setPosition(getPositionDegrees() + incrementDegrees);   
    }

    public Command incrementPositionCommand(double increment) {
        return Commands.runOnce(() -> incrementPosition(increment));
    }

    //****************************** POSITION COMMANDS *****************************//

    public Command moveToNeutral() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kNeutralPosition.get()));
    }

    public Command moveToAmp() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kAmpPosition.get()));
    }

    public Command moveToSpeaker() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPosition.get()));
    }

    public Command moveToSpeakerAuto() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPositionAuto.get()));
    }

    public Command moveToSpeakerFar() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPosition2.get()));
    }

    public Command moveToHandoff() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kHandoffPosition.get()));
    }

    public Command moveToAutoHandoff() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kHandoffPosition2.get()));
    }

    //****************************** LOGGING METHODS ******************************//

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL level) {
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Shooter");
        }

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addString("Current Command", () -> this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
            case MEDIUM:
            case MINIMAL:
                tab.addDouble("Shooter Desired Position", this::getTargetPositionDegrees);
                tab.addDouble("Left Pivot Position", this::getPositionDegrees);
                tab.addDouble("Left Pivot Velocity (DPS)", () -> leftPivot.getVelocity().getValueAsDouble() * 360);
                tab.addDouble("Left Pivot Applied Voltage", () -> leftPivot.getMotorVoltage().getValueAsDouble());
                tab.addDouble("Left Pivot Stator Current", () -> leftPivot.getStatorCurrent().getValueAsDouble());
                tab.addDouble("Left Pivot Supply Current", () -> leftPivot.getSupplyCurrent().getValueAsDouble());

                tab.addDouble("Right Pivot Position", () -> rightPivot.getPosition().getValueAsDouble() * 360);
                tab.addDouble("Right Pivot Velocity (DPS)", () -> rightPivot.getVelocity().getValueAsDouble() * 360);
                tab.addDouble("Right Pivot Applied Voltage", () -> rightPivot.getMotorVoltage().getValueAsDouble());
                tab.addDouble("Right Pivot Stator Current", () -> rightPivot.getStatorCurrent().getValueAsDouble());
                tab.addDouble("Right Pivot Supply Current", () -> rightPivot.getSupplyCurrent().getValueAsDouble());

                tab.add("Zero Absolute Encoder", Commands.runOnce(this::zeroAbsoluteEncoder));
                tab.add("Full Stow Absolute Encoder", Commands.runOnce(this::zeroAbsoluteEncoderFullStow));
                tab.add("Sync Encoder", Commands.runOnce(this::syncEncoder));

                tab.addDouble("Absolute Encoder Position", this::getAbsolutePositionDegrees);
                tab.addBoolean("Shooter Enabled", () -> enabled);
                break;
        }
    }

}  
