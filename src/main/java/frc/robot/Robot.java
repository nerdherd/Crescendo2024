// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Robot extends TimedRobot {
  private Command autoCommand;
  private RobotContainer robotContainer;
  
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
	
    DataLogManager.start("/media/sda1/logs");
    DataLogManager.logNetworkTables(true);
    enableLiveWindowInTest(false);
    robotContainer.swerveDrive.refreshModulePID();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    ShooterConstants.fullDisableShooter.loadPreferences();
    IntakeConstants.fullDisableIntake.loadPreferences();
    robotContainer.swerveDrive.refreshModulePID();
    robotContainer.imu.zeroHeading();
    robotContainer.imu.zeroAll();
    // robotContainer.swerveDrive.setVelocityControl(false);
    autoCommand = robotContainer.getAutonomousCommand();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    // robotContainer.swerveDrive.setVelocityControl(true);
    robotContainer.swerveDrive.refreshModulePID();
    ShooterConstants.kPivotDeadband.loadPreferences();
    ShooterConstants.fullDisableShooter.loadPreferences();
    IntakeConstants.fullDisableIntake.loadPreferences();
    
    robotContainer.intakePivot.configurePID();
    robotContainer.shooterPivot.configurePID();
    robotContainer.intakeRoller.configurePID();
    robotContainer.shooterRoller.configurePID();
    robotContainer.indexer.configurePID();
    robotContainer.initDefaultCommands();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
