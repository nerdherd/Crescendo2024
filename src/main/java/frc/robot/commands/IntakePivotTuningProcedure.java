package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.IntakePivot;


public class IntakePivotTuningProcedure extends Command {
    private IntakePivot intakePivot;
    private double voltage;

    public IntakePivotTuningProcedure(IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
        voltage = 0;
        
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        new ParallelDeadlineGroup(

            Commands.waitUntil(() -> voltage <= 0.5),
            Commands.runOnce(() -> voltage = 0),
            Commands.repeatingSequence(
                Commands.waitSeconds(3),
                Commands.runOnce(() -> voltage += 0.05),
                Commands.runOnce(() -> SmartDashboard.putNumber("Voltage", voltage))
            ),
            Commands.run(() -> intakePivot.setVoltage(voltage))
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
