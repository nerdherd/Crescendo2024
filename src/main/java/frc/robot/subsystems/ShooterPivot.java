package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase implements Reportable {
    private final TalonFX leftPivot;
    private final TalonFX rightPivot;
    private final TalonFXConfigurator leftPivotConfigurator;
    private final TalonFXConfigurator rightPivotConfigurator;
    private final DutyCycleEncoder throughBore;

    // Whether the pivot is running
    private boolean enabled = true;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();

    public ShooterPivot() {
        leftPivot = new TalonFX(ShooterConstants.kLeftPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        rightPivot = new TalonFX(ShooterConstants.kRightPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        throughBore = new DutyCycleEncoder(ShooterConstants.kThroughBorePort);
        leftPivotConfigurator = leftPivot.getConfigurator();
        rightPivotConfigurator = rightPivot.getConfigurator();
        leftPivot.setInverted(false);
        rightPivot.setControl(new Follower(ShooterConstants.kLeftPivotMotorID, true));

        CommandScheduler.getInstance().registerSubsystem(this);

        configureMotor();
        configurePID();
        resetEncoder();
    }
    
    //****************************** SETUP METHODS ******************************/

    public void configureMotor() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        leftPivotConfigurator.refresh(pivotConfigs);
        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfigs.Feedback.RotorToSensorRatio = 1;
        pivotConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.kPivotGearRatio;
        
        pivotConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotConfigs.Voltage.PeakReverseVoltage = -11.5;
        
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = ShooterConstants.kShooterNeutralDeadband;

        pivotConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        pivotConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        pivotConfigs.Audio.AllowMusicDurDisable = true;

        leftPivotConfigurator.apply(pivotConfigs);

        StatusCode statusPivot = leftPivotConfigurator.apply(pivotConfigs);
        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), new Error().getStackTrace());
        }

        pivotConfigs = new TalonFXConfiguration();
        rightPivotConfigurator.refresh(pivotConfigs);
        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivotConfigs.Feedback.RotorToSensorRatio = 1;
        pivotConfigs.Feedback.SensorToMechanismRatio = ShooterConstants.kPivotGearRatio;
        
        pivotConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotConfigs.Voltage.PeakReverseVoltage = -11.5;
        
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = ShooterConstants.kShooterNeutralDeadband;

        pivotConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        pivotConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        pivotConfigs.Audio.AllowMusicDurDisable = true;

        rightPivotConfigurator.apply(pivotConfigs);

        statusPivot = rightPivotConfigurator.apply(pivotConfigs);
        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
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
        pivotConfigs.Slot0.kP = ShooterConstants.kPPivotMotor.get();
        pivotConfigs.Slot0.kI = ShooterConstants.kIPivotMotor.get();
        pivotConfigs.Slot0.kD = ShooterConstants.kDPivotMotor.get();
        pivotConfigs.Slot0.kV = ShooterConstants.kVPivotMotor.get();
        pivotConfigs.Slot0.kS = ShooterConstants.kSPivotMotor.get();
        pivotConfigs.Slot0.kA = ShooterConstants.kAPivotMotor.get();
        pivotConfigs.Slot0.kG = ShooterConstants.kGPivotMotor.get();
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kCruiseVelocity.get();
        pivotConfigs.MotionMagic.MotionMagicAcceleration   = ShooterConstants.kCruiseAcceleration.get();

        StatusCode statusPivot = leftPivotConfigurator.apply(pivotConfigs);
        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), new Error().getStackTrace());
        }

        pivotConfigs = new TalonFXConfiguration();
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
        pivotConfigs.Slot0.kP = ShooterConstants.kPPivotMotor.get();
        pivotConfigs.Slot0.kI = ShooterConstants.kIPivotMotor.get();
        pivotConfigs.Slot0.kD = ShooterConstants.kDPivotMotor.get();
        pivotConfigs.Slot0.kV = ShooterConstants.kVPivotMotor.get();
        pivotConfigs.Slot0.kS = ShooterConstants.kSPivotMotor.get();
        pivotConfigs.Slot0.kA = ShooterConstants.kAPivotMotor.get();
        pivotConfigs.Slot0.kG = ShooterConstants.kGPivotMotor.get();
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kCruiseVelocity.get();
        pivotConfigs.MotionMagic.MotionMagicAcceleration   = ShooterConstants.kCruiseAcceleration.get();

        statusPivot = rightPivotConfigurator.apply(pivotConfigs);
        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), new Error().getStackTrace());
        }
    }

    public void resetEncoder() {
        // Save a consistent position offset
        ShooterConstants.kPivotOffset.loadPreferences();
        throughBore.setPositionOffset(ShooterConstants.kPivotOffset.get());

        double position = throughBore.getAbsolutePosition() - throughBore.getPositionOffset();
        position = mapRev(position);

        leftPivot.setPosition(position);
    }

    /**
     * Zero the through bore encoder and update the internal encoder
     * ONLY RUN FOR DEBUGGING
     */
    public void zeroAbsoluteEncoder() {
        throughBore.reset();
        ShooterConstants.kPivotOffset.set(throughBore.getPositionOffset());

        // Save new offset to Preferences
        ShooterConstants.kPivotOffset.uploadPreferences();
        resetEncoder();
    }

    @Override
    public void periodic() {
        if (enabled) {
            leftPivot.setControl(motionMagicRequest);
        } else {
            leftPivot.setControl(brakeRequest);
        }
    }

    /**
     * Maps the value to the range [-0.25, 0.75]
     */
    public double mapRev(double rev) {
        return NerdyMath.posMod(rev + 0.25, 1) - 0.25;
    }

    //****************************** STATE METHODS ******************************/

    public double getTargetPosition() {
        return motionMagicRequest.Position;
    }

    public double getPosition() {
        return leftPivot.getPosition().getValueAsDouble();
    }

    // Checks whether the pivot is within the deadband for a position
    public boolean hasReachedPosition(double position) {
        return NerdyMath.inRange(
            getPosition(),
            position - IntakeConstants.kPivotDeadband.get(), 
            position + IntakeConstants.kPivotDeadband.get()
        ) && NerdyMath.inRange(
            getTargetPosition(),
            position - IntakeConstants.kPivotDeadband.get(), 
            position + IntakeConstants.kPivotDeadband.get()
        );
    }

    // Check if the shooter is in a safe position for the intake to move
    public boolean hasReachedNeutral() {
        return hasReachedPosition(ShooterConstants.kNeutralPosition.get());
    }

    // Checks if the pivot is within deadband of the target pos
    public boolean atTargetPosition() {
        return hasReachedPosition(motionMagicRequest.Position);
    }

    public void stop() {
        motionMagicRequest.Position = getPosition();
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

    public void setPosition(double position) {
        motionMagicRequest.Position = 
            NerdyMath.clamp(
                mapRev(position),
                ShooterConstants.kPivotMinPos,
                ShooterConstants.kPivotMaxPos);  
    }

    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public void incrementPosition(double increment) {
        setPosition(motionMagicRequest.Position + increment);   
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

    public Command moveToHandoff() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kHandoffPosition.get()));
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
                tab.addDouble("Shooter Desired Position", this::getTargetPosition);
                tab.addDouble("Shooter Position", this::getPosition);
                tab.addDouble("Shooter Pivot Velocity", () -> leftPivot.getVelocity().getValueAsDouble());
                break;
        }
    }

}  
