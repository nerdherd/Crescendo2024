package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends SubsystemBase implements Reportable {
    private final TalonFX pivot;
    private final TalonFXConfigurator pivotConfigurator;
    private final DutyCycleEncoder throughBore;

    // Whether the pivot is running
    private boolean enabled = false;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();

    public IntakePivot() {
        pivot = new TalonFX(IntakeConstants.kPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        throughBore = new DutyCycleEncoder(IntakeConstants.kThroughBorePort);
        pivotConfigurator = pivot.getConfigurator();
        pivot.setInverted(IntakeConstants.kPivotInverted);

        CommandScheduler.getInstance().registerSubsystem(this);

        configureMotor();
        configurePID();
        resetEncoder();
    }
    
    //****************************** SETUP METHODS ******************************/

    public void configureMotor() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        pivotConfigurator.refresh(intakeConfigs);
        intakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        intakeConfigs.Feedback.RotorToSensorRatio = 1;
        intakeConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.kPivotGearRatio;
        
        intakeConfigs.Voltage.PeakForwardVoltage = 11.5;
        intakeConfigs.Voltage.PeakReverseVoltage = -11.5;

        intakeConfigs.ClosedLoopGeneral.ContinuousWrap = false;
        
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = IntakeConstants.kIntakeNeutralDeadband;

        intakeConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        intakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        intakeConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        intakeConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode response = pivotConfigurator.apply(intakeConfigs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ response.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        IntakeConstants.kStowPosition.loadPreferences();
        IntakeConstants.kPickupPosition.loadPreferences();
        IntakeConstants.kNeutralPosition.loadPreferences();

        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        pivotConfigurator.refresh(pivotConfigs);
        IntakeConstants.kPPivotMotor.loadPreferences();
        IntakeConstants.kIPivotMotor.loadPreferences();
        IntakeConstants.kDPivotMotor.loadPreferences();
        IntakeConstants.kVPivotMotor.loadPreferences();
        IntakeConstants.kSPivotMotor.loadPreferences();
        IntakeConstants.kAPivotMotor.loadPreferences();
        IntakeConstants.kGPivotMotor.loadPreferences();
        IntakeConstants.kIntakeCruiseVelocity.loadPreferences();
        IntakeConstants.kIntakeCruiseAcceleration.loadPreferences();
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfigs.Slot0.kP = IntakeConstants.kPPivotMotor.get();
        pivotConfigs.Slot0.kI = IntakeConstants.kIPivotMotor.get();
        pivotConfigs.Slot0.kD = IntakeConstants.kDPivotMotor.get();
        pivotConfigs.Slot0.kV = IntakeConstants.kVPivotMotor.get();
        pivotConfigs.Slot0.kS = IntakeConstants.kSPivotMotor.get();
        pivotConfigs.Slot0.kA = IntakeConstants.kAPivotMotor.get();
        pivotConfigs.Slot0.kG = IntakeConstants.kGPivotMotor.get();
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kIntakeCruiseVelocity.get();
        pivotConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.kIntakeCruiseAcceleration.get();

        StatusCode response = pivotConfigurator.apply(pivotConfigs);
        if (!response.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ response.toString(), new Error().getStackTrace());
        }
    }

    public void resetEncoder() {
        // Save a consistent position offset
        IntakeConstants.kPivotOffset.loadPreferences();
        throughBore.setPositionOffset(IntakeConstants.kPivotOffset.get());

        double position = throughBore.getAbsolutePosition() - throughBore.getPositionOffset();
        position = mapRev(position);

        // pivot.setPosition(position);
        pivot.setPosition(IntakeConstants.kPickupPosition.get());
    }

    /**
     * Zero the through bore encoder and update the internal encoder
     * ONLY RUN FOR DEBUGGING
     */
    public void zeroAbsoluteEncoder() {
        throughBore.reset();
        IntakeConstants.kPivotOffset.set(throughBore.getPositionOffset());

        // Save new offset to Preferences
        IntakeConstants.kPivotOffset.uploadPreferences();
        resetEncoder();
    }

    @Override
    public void periodic() {
        if (IntakeConstants.fullDisableIntake.get()) {
            pivot.setControl(brakeRequest);
            return;
        }

        if (enabled) {
            pivot.setControl(motionMagicRequest);
        } else {
            pivot.setControl(brakeRequest);
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
        return pivot.getPosition().getValueAsDouble();
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

    // Check if the intake is in a safe position for the shooter to move
    public boolean hasReachedNeutral() {
        return hasReachedPosition(IntakeConstants.kNeutralPosition.get()) 
            || hasReachedPosition(IntakeConstants.kPickupPosition.get());
    }

    // Checks if the pivot is within deadband of the target pos
    public boolean atTargetPosition() {
        return hasReachedPosition(motionMagicRequest.Position);
    }

    public void stop() {
        motionMagicRequest.Position = getPosition();
        enabled = false;
        pivot.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    
    //****************************** POSITION METHODS ******************************//

    public void setPosition(double position) {
        motionMagicRequest.Position = 
            NerdyMath.clamp(
                mapRev(position),
                IntakeConstants.kPivotMinPos,
                IntakeConstants.kPivotMaxPos);  
    }

    public void incrementPosition(double increment) {
        setPosition(motionMagicRequest.Position + increment);
    }

    //****************************** POSITION COMMANDS *****************************//

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public Command incrementPositionCommand(double increment) {
        return Commands.runOnce(() -> incrementPosition(increment));
    }

    public Command moveToStow() {
        return setPositionCommand(IntakeConstants.kStowPosition.get());
    }

    public Command moveToIntake() {
        return setPositionCommand(IntakeConstants.kPickupPosition.get());
    }

    public Command moveToNeutral() {
        return setPositionCommand(IntakeConstants.kNeutralPosition.get());
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
            tab = Shuffleboard.getTab("Intake");
        }

        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
            case MINIMAL:
                tab.addDouble("Intake Desired Position", this::getTargetPosition);
                tab.addDouble("Intake Position", this::getPosition);
                tab.add("Zero Absolute Encoder", Commands.runOnce(this::zeroAbsoluteEncoder));
                break;
        }
    }
}
