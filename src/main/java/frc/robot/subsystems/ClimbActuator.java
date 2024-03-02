package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberActuatorConstants;

public class ClimbActuator extends SubsystemBase {
    Servo rightClimbActuator;
    Servo leftClimbActuator;
    
    public ClimbActuator () {
        CommandScheduler.getInstance().registerSubsystem(this);
        rightClimbActuator = new Servo(climberActuatorConstants.rightClimberActuatorChannelID);
        leftClimbActuator = new Servo(climberActuatorConstants.leftClimberActuatorChannelID);
    }

    public void retract() {
        rightClimbActuator.set(0.2); // if pos is too low, it won't move
        leftClimbActuator.set(0.2); // if pos is too low, it won't move
    }

    public void extend() {
        rightClimbActuator.set(0.8); // if pos is too high, it won't move
        leftClimbActuator.set(0.8); // if pos is too high, it won't move
    }

    public Command retractCommand() {
        return Commands.runOnce(this::retract);
    }

    public Command extendCommand() {
        return Commands.runOnce(this::extend);
    }
}
