package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearActuator extends SubsystemBase {
    Servo linearActuator;
    
    public LinearActuator () {
        CommandScheduler.getInstance().registerSubsystem(this);
        linearActuator = new Servo(0);
    }

    public void retract() {
        linearActuator.set(0.2); // if pos is too low, it won't move
    }

    public void extend() {
        linearActuator.set(0.8); // if pos is too high, it won't move
    }

    public Command retractCommand() {
        return Commands.runOnce(this::retract);
    }

    public Command extendCommand() {
        return Commands.runOnce(this::extend);
    }
}
