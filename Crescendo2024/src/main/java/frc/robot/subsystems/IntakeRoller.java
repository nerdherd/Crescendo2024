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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeRoller {
    
    // final TalonFX rightIntake;
    final TalonFX intake;

    private boolean tooHigh = false;
    private boolean tooLow = false;

    int velocityIntake = 0;
    int targetTicks = IntakeConstants.kStowPosition;

    final VoltageOut m_intakeVoltageRequest = new VoltageOut(0);
    // final VoltageOut m_rightVoltageRequest = new VoltageOut(0);

    final DutyCycleOut m_intakeDutyCycleRequest = new DutyCycleOut(0);
    // final DutyCycleOut m_rightDutyCycleRequest = new DutyCycleOut(0);

    final VelocityVoltage m_intakeVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // final VelocityVoltage m_rightVelocity = new VelocityVoltage(0, 0, true, 0,0, false, false, false);

    final NeutralOut m_brake = new NeutralOut();

    public IntakeRoller(){
        intake = new TalonFX(IntakeConstants.kIntakeMotorID, ModuleConstants.kCANivoreName);
        // rightIntake = new TalonFX(IntakeConstants.kRightMotorID, ModuleConstants.kCANivoreName);

        intake.setInverted(false);
        // rightIntake.setControl(new Follower(intake.getDeviceID(), false));

        init();
    }

    public void configurePID() {
        TalonFXConfiguration intakeMotorConfigs = new TalonFXConfiguration();
        
        intake.getConfigurator().refresh(intakeMotorConfigs);
        IntakeConstants.kPIntakeMotor.loadPreferences();
        IntakeConstants.kIIntakeMotor.loadPreferences();
        IntakeConstants.kDIntakeMotor.loadPreferences();
        IntakeConstants.kVIntakeMotor.loadPreferences();
        intakeMotorConfigs.Slot0.kP = IntakeConstants.kPIntakeMotor.get();
        intakeMotorConfigs.Slot0.kI = IntakeConstants.kIIntakeMotor.get();
        intakeMotorConfigs.Slot0.kD = IntakeConstants.kDIntakeMotor.get();
        intakeMotorConfigs.Slot0.kV = IntakeConstants.kVIntakeMotor.get();
        
        // TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
        
        // rightIntake.getConfigurator().refresh(rightMotorConfigs);
        // IntakeConstants.kPRightMotor.loadPreferences();
        // IntakeConstants.kIRightMotor.loadPreferences();
        // IntakeConstants.kDRightMotor.loadPreferences();
        // IntakeConstants.kVRightMotor.loadPreferences();

        // rightMotorConfigs.Slot0.kP = IntakeConstants.kPRightMotor.get();
        // rightMotorConfigs.Slot0.kI = IntakeConstants.kIRightMotor.get();
        // rightMotorConfigs.Slot0.kD = IntakeConstants.kDRightMotor.get();
        // rightMotorConfigs.Slot0.kV = IntakeConstants.kVRightMotor.get();

        intakeMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        intakeMotorConfigs.Voltage.PeakReverseVoltage = -11.5;

        // rightMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
        // rightMotorConfigs.Voltage.PeakReverseVoltage = -11.5;


        StatusCode statusIntake = intake.getConfigurator().apply(intakeMotorConfigs);
        // StatusCode statusRight = rightIntake.getConfigurator().apply(rightMotorConfigs);

        if (!statusIntake.isOK()){
            DriverStation.reportError("Could not apply intake configs, error code:"+ statusIntake.toString(), null);
        }
        // if (!statusRight.isOK()){
        //     DriverStation.reportError("Could not apply right configs, error code:"+ statusRight.toString(), null);
        // }
    }

    public void init() {
        configurePID();
        
    }

    public Command setIntakeSpeed() {
        return Commands.runOnce(() -> {

            // Percent Output
            // intake.setControl(m_intakeDutyCycleRequest.withOutput(intakeSpeeds[index]));
            // intake.setControl(m_intakeDutyCycleRequest.withOutput(rightSpeeds[index]));

            // Velocity Control
            m_intakeVelocityRequest.Slot = 0;
            // m_rightVelocity.Slot = 0;

            intake.setControl(m_intakeVelocityRequest.withVelocity(velocityIntake));
            // rightIntake.setControl(m_rightVelocity.withVelocity(velocityRight));
            SmartDashboard.putBoolean("Pressed", true);
        });
    }

    public Command setIntakePowerZeroCommand() {
        return Commands.runOnce(() -> {
            intake.setControl(m_brake);
            // rightIntake.setControl(m_brake);
            SmartDashboard.putBoolean("Pressed", false);
        });
    }

    public void setIntakePowerZero() {
        intake.setControl(m_brake);
        // rightIntake.setControl(m_brake);
        SmartDashboard.putBoolean("Pressed", false);

    }

    public Command increaseIntake() {
        return Commands.runOnce(() -> {

            if (velocityIntake <= 21000) {
                velocityIntake += 1024;
                tooHigh = false;
            }
            else {
                tooHigh = true;
            }
            SmartDashboard.putBoolean("Too high", tooHigh);
        });
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

    public Command decreaseIntake() {
        return Commands.runOnce(() -> {
            // if (percentOutputTop >= 0.051) {
            //     this.speedsIntake[2] -= 0.05; // TODO DEBUG
            //     percentOutputTop -= 0.05;
            //     tooLow = false;

            if (velocityIntake >= 1100) {
                velocityIntake -= 1024;
                tooLow = false;
            }
            else {
                tooLow = true;
            }
            SmartDashboard.putBoolean("Too low", tooLow);
        });
    }

    // public Command decreaseRight() {
    //     return Commands.runOnce(() -> {
    //         // if (percentOutputBottom >= 0.051) {
    //         //     this.speedsRight[2] -= 0.05; // TODO DEBUG
    //         //     percentOutputBottom -= 0.05;
    //         //     tooLow = false;
    //         if (velocityRight >= 1100) {
    //             velocityRight -= 1024;
    //             tooLow = false;
    //         }
    //         else {
    //             tooLow = true;
    //         }
    //         SmartDashboard.putBoolean("Too low", tooLow);
    //     });
    // }

    public void printIntakeSpeeds() {
        SmartDashboard.putNumber("Intake Ticks Per Second: ", velocityIntake);
        // SmartDashboard.putNumber("Ticks Per Second Bottom ", velocityRight);
    }

    public void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addNumber("Velocity", ()-> intake.getVelocity().getValueAsDouble());
        // tab.addNumber("Bottom Velocity", ()-> rightIntake.getVelocity().getValueAsDouble());

    }

}
