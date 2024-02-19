package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.ShooterPivot;

public class ArmTuningProcedure extends Command {
    private ShooterPivot shooterPivot;
    private double voltage;

    public ArmTuningProcedure(ShooterPivot shooterPivot) {
        this.shooterPivot = shooterPivot;
        voltage = 0;
        addRequirements(shooterPivot);
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
            Commands.run(() -> shooterPivot.setVoltage(voltage))
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
