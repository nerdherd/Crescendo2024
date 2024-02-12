package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearActuator extends SubsystemBase {
    Servo linearActuator;
    
    public LinearActuator () {
        CommandScheduler.getInstance().registerSubsystem(this);
        linearActuator = new Servo(0);
        SmartDashboard.putNumber("Extended", 0);
    }

    public void retract() {
        linearActuator.set(0);
    }

    public Command retractCommand() {
        return Commands.runOnce(this::retract);
    }
}
