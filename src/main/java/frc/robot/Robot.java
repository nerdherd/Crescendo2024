// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CANdleSubSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSubSystem.Status;

public class Robot extends TimedRobot {
  private Command autoCommand;
  private RobotContainer robotContainer;

  /*  //One Peak Music 1
  TalonFx talonFX;
  Orchestra orchestra;
   */
  
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
	
    DataLogManager.start("/media/sda1/logs");
    DataLogManager.logNetworkTables(true);
    enableLiveWindowInTest(false);
    robotContainer.swerveDrive.refreshModulePID();
    robotContainer.apriltagCamera.toggleLight(true);
    robotContainer.configureLEDTriggers();
    robotContainer.shooterPivot.syncAbsoluteEncoderToPigeon();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.swerveDrive.setBreak(true);
    robotContainer.CANdle.setStatus(Status.DISABLED);
    robotContainer.apriltagCamera.toggleLight(true);
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.isDSAttached()) {
      robotContainer.CANdle.setStatus(Status.DISABLED);
    } else {
      robotContainer.CANdle.setStatus(Status.DISCONNECTED);
    }
  }

  @Override
  public void autonomousInit() {
    RobotContainer.refreshAlliance();
    robotContainer.swerveDrive.setBreak(false);
    robotContainer.swerveDrive.refreshModulePID();
    robotContainer.imu.zeroHeading();
    robotContainer.imu.zeroAll();
    

    // ShooterConstants.kPivotDeadband.loadPreferences();
    // ShooterConstants.fullDisableShooter.loadPreferences();
    // IntakeConstants.fullDisableIntake.loadPreferences();
    
    // robotContainer.shooterPivot.configurePID();
    // robotContainer.intakeRoller.configurePID();
    // robotContainer.shooterRoller.configurePID();
    // robotContainer.indexer.configurePID();
    // robotContainer.swerveDrive.setVelocityControl(false);
    autoCommand = robotContainer.getAutonomousCommand();
    // robotContainer.apriltagCamera.toggleLight(false);


    if (autoCommand != null) {
      autoCommand.schedule();
    }
    /*    //One Peak Music 3
    orchestra.loadMusic(); //have to get chirp file
    orchestra.play();
     */
    robotContainer.CANdle.setStatus(Status.AUTO);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.refreshAlliance();
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    robotContainer.swerveDrive.setBreak(false);
    robotContainer.swerveDrive.setChassisSpeeds(new ChassisSpeeds());
    robotContainer.apriltagCamera.toggleLight(false);
    // robotContainer.swerveDrive.setVelocityControl(true);
    // robotContainer.swerveDrive.refreshModulePID();
    // ShooterConstants.kPivotDeadband.loadPreferences();
    // ShooterConstants.fullDisableShooter.loadPreferences();
    // IntakeConstants.fullDisableIntake.loadPreferences();
    
    // robotContainer.shooterPivot.configurePID();
    // robotContainer.intakeRoller.configurePID();
    // robotContainer.shooterRoller.configurePID();
    // robotContainer.indexer.configurePID();
    robotContainer.configureBindings_teleop();
    robotContainer.initDefaultCommands_teleop();
    // robotContainer.configureLEDTriggers_teleop();
    robotContainer.CANdle.setStatus(Status.TELEOP);
    // robotContainer.CANdle.changeAnimation(AnimationTypes.Fire);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    RobotContainer.refreshAlliance();
    CommandScheduler.getInstance().cancelAll();
    // ShooterConstants.kPivotDeadband.loadPreferences();
    // ShooterConstants.fullDisableShooter.loadPreferences();
    // IntakeConstants.fullDisableIntake.loadPreferences();
    
    // robotContainer.shooterPivot.configurePID();
    // robotContainer.intakeRoller.configurePID();
    // robotContainer.shooterRoller.configurePID();
    // robotContainer.indexer.configurePID();
    robotContainer.configureBindings_test();
    robotContainer.initDefaultCommands_test();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
