package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.IndexerConstants;


public class Indexer extends SubsystemBase {
    
    private final TalonFX indexer;    
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();

    private boolean enabled = false;

    public Indexer(){
        indexer = new TalonFX(IndexerConstants.kIndexerMotorID, SuperStructureConstants.kCANivoreBusName);
        configureMotor();
        configurePID();
    }

    public void configureMotor() {
        TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
        indexer.getConfigurator().refresh(indexerConfigs);
        indexerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        indexerConfigs.Voltage.PeakForwardVoltage = 11.5;
        indexerConfigs.Voltage.PeakReverseVoltage = -11.5;
        indexerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        indexerConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        indexerConfigs.Audio.AllowMusicDurDisable = true;
        indexer.getConfigurator().apply(indexerConfigs);
    }

    public void configurePID() {
        TalonFXConfiguration indexerMotorConfigs = new TalonFXConfiguration();
        
        indexer.getConfigurator().refresh(indexerMotorConfigs);
        IndexerConstants.kPIndexerMotor.loadPreferences();
        IndexerConstants.kIIndexerMotor.loadPreferences();
        IndexerConstants.kDIndexerMotor.loadPreferences();
        IndexerConstants.kVIndexerMotor.loadPreferences();
        indexerMotorConfigs.Slot0.kP = IndexerConstants.kPIndexerMotor.get();
        indexerMotorConfigs.Slot0.kI = IndexerConstants.kIIndexerMotor.get();
        indexerMotorConfigs.Slot0.kD = IndexerConstants.kDIndexerMotor.get();
        indexerMotorConfigs.Slot0.kV = IndexerConstants.kVIndexerMotor.get();

        indexerMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        indexerMotorConfigs.Voltage.PeakReverseVoltage = -11.5;

        StatusCode statusIndexer = indexer.getConfigurator().apply(indexerMotorConfigs);

        if (!statusIndexer.isOK()){
            DriverStation.reportError("Could not apply indexer configs, error code:"+ statusIndexer.toString(), null);
        }
    }

    public void run() {
        if (enabled) {
            indexer.setControl(brakeRequest);
        } else {
            indexer.setControl(velocityRequest);
        }
    }

    public void stop() {
        this.enabled = false;
        velocityRequest.Velocity = 0;
        indexer.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setVelocity(double velocity) {
        velocityRequest.Velocity = velocity;
    }

    public void incrementVelocity(double increment) {
        double newVelocity = velocityRequest.Velocity + increment;
        if ((increment > 0 && newVelocity < IndexerConstants.kIndexerMaxVelocityRPS) ||
            (increment < 0 && newVelocity > -IndexerConstants.kIndexerMaxVelocityRPS)
            ) {
            velocityRequest.Velocity = newVelocity;
        }
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addNumber("Top Velocity", ()-> indexer.getVelocity().getValueAsDouble());
    }

}

