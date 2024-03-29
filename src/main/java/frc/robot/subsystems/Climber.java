package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SuperStructureConstants;

public class Climber extends SubsystemBase{
    private final TalonFX climber;
    private final TalonFXConfigurator climberConfigurator;
    private final VoltageOut voltageRequest = new VoltageOut(0, true, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut();

    private boolean enabled = false;

    public Climber() {
        climber = new TalonFX(ClimberConstants.kClimberMotorID, SuperStructureConstants.kCANivoreBusName);
        climberConfigurator = climber.getConfigurator();
    }

    public void configureMotor() {
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberConfigurator.refresh(climberConfigs);
        climberConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        climberConfigs.Voltage.PeakForwardVoltage = 11.5;
        climberConfigs.Voltage.PeakReverseVoltage = -11.5;
        climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfigs.MotorOutput.DutyCycleNeutralDeadband = ClimberConstants.kClimberNeutralDeadband;
        climberConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        climberConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        climberConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        climberConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode result = climberConfigurator.apply(climberConfigs);

        if (!result.isOK()){
            DriverStation.reportError("Could not apply climber configs, error code:"+ result.toString(), true);
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            climber.setControl(brakeRequest);
            return;
        }

        climber.setControl(voltageRequest);
    }

    public void climb() {
        voltageRequest.Output = ClimberConstants.kClimberOutput; 
    }

    public void setOutput(double output) {
        voltageRequest.Output = output;
    }

    public Command setOutputCommand(double output) {
        return Commands.runOnce(() -> setOutput(output));
    }

    public Command climbCommand() {
        return Commands.runOnce(this::climb);
    }

    public void stop() {
        voltageRequest.Output = 0;
        climber.setControl(brakeRequest);
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop);
    }

}
