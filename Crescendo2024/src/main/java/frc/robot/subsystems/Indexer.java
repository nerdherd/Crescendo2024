package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.IndexerConstants;

public class Indexer {
    
    // final TalonFX rightIntake;
    final TalonFX indexer;
    
    int velocity = 0;

    private boolean tooHigh = false;
    private boolean tooLow = false;

    final VoltageOut m_VoltageRequest = new VoltageOut(0);

    final DutyCycleOut m_DutyCycleRequest = new DutyCycleOut(0);

    final VelocityVoltage m_VelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    
    final NeutralOut m_brake = new NeutralOut();

    public Indexer(){
        indexer = new TalonFX(IndexerConstants.kIndexerMotorID, SuperStructureConstants.kCANivoreBusName);
        // rightIntake = new TalonFX(IntakeConstants.kRightMotorID, ModuleConstants.kCANivoreName);
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

   public Command setIndexerSpeed() {
        return Commands.runOnce(() -> {

            // Percent Ouput
            // indexer.setControl(m_leftDutyCycleRequest.withOutput(leftSpeeds[index]));
            // indexer.setControl(m_leftDutyCycleRequest.withOutput(rightSpeeds[index]));

            // Velocity Control
            m_VelocityRequest.Slot = 0;

            indexer.setControl(m_VelocityRequest.withVelocity(velocity));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }

    public Command setIndexerPowerZeroCommand() {
        return Commands.runOnce(() -> {
            indexer.setControl(m_brake);
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public void setIndexerPowerZero() {
        indexer.setControl(m_brake);
        SmartDashboard.putBoolean("Pressed", false);

    }

    public Command increaseIndexer() {
        return Commands.runOnce(() -> {

            if (velocity <= 21000) {
                velocity += 1024;
                tooHigh = false;
            }
            else {
                tooHigh = true;
            }
            SmartDashboard.putBoolean("Too high", tooHigh);
        });
    }

    public Command decreaseIndexer() {
        return Commands.runOnce(() -> {
            // if(percentOutputTop >= 0.051) {
            //     this.speedsLeft[2] -= 0.05; // TODO DEBUG
            //     percentOutputTop -= 0.05;
            //     tooLow = false;

            if(velocity >= 1100) {
                velocity -= 1024;
                tooLow = false;
            }
            else {
                tooLow = true;
            }
            SmartDashboard.putBoolean("Too low", tooLow);
        });
    }

    public void printIndexerSpeeds() {
        SmartDashboard.putNumber("Indexer Ticks Per Second Top: ", velocity);
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addNumber("Top Velocity", ()-> indexer.getVelocity().getValueAsDouble());
    }

}

