package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class Indexer {
    
    private final TalonFX indexer;    
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();

    public Indexer(){
        indexer = new TalonFX(IntakeConstants.kIntakeMotorID, ModuleConstants.kCANivoreName);
        configurePID();
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

    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public void setVelocity(double velocity) {
        velocityRequest.Velocity = velocity;
        indexer.setControl(velocityRequest);
    }

    public void stop() {
        indexer.setControl(brakeRequest);
    }

    public Command incrementSpeed(double increment) {
        return Commands.runOnce(() -> {
            double newVelocity = velocityRequest.Velocity + increment;
            if ((increment > 0 && newVelocity < IndexerConstants.kIndexerMaxVelocityRPS) ||
                (increment < 0 && newVelocity > -IndexerConstants.kIndexerMaxVelocityRPS)
                ) {
                velocityRequest.Velocity = newVelocity;
            }
        });
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addNumber("Top Velocity", ()-> indexer.getVelocity().getValueAsDouble());
    }

}

