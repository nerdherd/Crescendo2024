// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Basic4PieceWithVision;
import frc.robot.commands.autos.AutoMidNotes;
import frc.robot.commands.autos.Basic2Piece;
import frc.robot.commands.autos.Basic3Piece;
import frc.robot.commands.autos.Basic4Piece;
import frc.robot.commands.autos.Basic4PieceSeparated;
import frc.robot.commands.autos.Basic6PieceSeparated;
import frc.robot.commands.autos.Mid3Piece;
import frc.robot.commands.autos.OneMeterSquareAuto;
import frc.robot.commands.autos.Reliable4Piece;
import frc.robot.commands.autos.Reliable4PieceWithVision;
import frc.robot.commands.autos.RotateSquareAuto;
import frc.robot.commands.autos.Test2M;
import frc.robot.commands.autos.Test2MBack;
import frc.robot.commands.autos.ThreeMeterSquareAuto;
import frc.robot.commands.autos.TwoMeterSquareAuto;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IndexerV2;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.util.NerdyMath;

public class RobotContainer {
  public ShooterRoller shooterRoller = new ShooterRoller();
  public ShooterPivot shooterPivot = new ShooterPivot();
  public IntakeRoller intakeRoller = new IntakeRoller();
  public IntakePivot intakePivot = new IntakePivot();
  public IndexerV2 indexer = new IndexerV2();

  public SuperSystem superSystem = new SuperSystem(intakePivot, intakeRoller, shooterPivot, shooterRoller, indexer);
  
  public Gyro imu = new PigeonV2(2);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

  private final CommandPS4Controller commandDriverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  private final PS4Controller driverController = commandDriverController.getHID();
  private final CommandPS4Controller commandOperatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);
  private final PS4Controller operatorController = commandOperatorController.getHID();

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private NoteAssistance noteCamera; 
  private DriverAssist apriltagCamera;// = new DriverAssist(VisionConstants.kLimelightFrontName, 4);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      noteCamera = new NoteAssistance(VisionConstants.kLimelightFrontName);
      apriltagCamera = new DriverAssist(VisionConstants.kLimelightBackName, 4);
      swerveDrive = new SwerveDrivetrain(imu, apriltagCamera);

    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    apriltagCamera.toggleLight(false);

    initAutoChoosers();
    initShuffleboard();

    // Configure the trigger bindings
    // Moved to teleop init

    DriverStation.reportWarning("Initalization complete", false);
      NamedCommands.registerCommand("intakeBasic1", superSystem.intakeBasicHold());
      NamedCommands.registerCommand("intakeBasic2", superSystem.stopIntaking());
      NamedCommands.registerCommand("shootSequence2Far", superSystem.shootSequence2Far());
      NamedCommands.registerCommand("shootSequence2", superSystem.shootSequence2());

  }

  public static boolean IsRedSide()
  {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
  }

  public void initDefaultCommands_teleop() {

    intakePivot.setDefaultCommand(
      new RunCommand(
        () -> 
          intakePivot.incrementPosition(
            Math.signum(
              NerdyMath.deadband(
              operatorController.getRightY(), 
              -ControllerConstants.kDeadband, 
              ControllerConstants.kDeadband)
            ) / 4000), // (20 / x) revolutions per second
                       // 0.005 rev/seconds @ x = 4000
        intakePivot
      ));
    
    shooterPivot.setDefaultCommand(
      new RunCommand(
        () -> {
          shooterPivot.incrementPosition(
            Math.signum(
              NerdyMath.deadband(
                operatorController.getLeftY(), //0.5 rev/second 
                -ControllerConstants.kDeadband, 
                ControllerConstants.kDeadband)
            ) / 4000); // (20 / x) revolutions per second
                       // 0.005 rev/seconds @ x = 4000
        },
        shooterPivot
      ));

      swerveDrive.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -commandDriverController.getLeftY(), // Horizontal translation
        commandDriverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        commandDriverController::getRightX, // Rotationaq

        // driverController::getSquareButton, // Field oriented
        () -> false, // should be robot oriented now on true

        driverController::getCrossButton, // Towing
        // driverController::getR2Button, // Precision/"Sniper Button"
        () -> driverController.getR2Button(), // Precision mode (disabled)
        () -> driverController.getCircleButton(), // Turn to angle
        // () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          return 0.0;
        }
      ));

      // Point to angle
      // swerveDrive.setDefaultCommand(
      //   new SwerveJoystickCommand(
      //     swerveDrive,
      //     () -> -commandDriverController.getLeftY(), // Horizontal translation
      //     commandDriverController::getLeftX, // Vertical Translation
          
      //     () -> {
      //       if (driverController.getCircleButton()) {
      //         noteCamera.calculateRotationSpeed(0, 0); // Values from SwerveDrive2024/isMeToKitBot
      //         return (noteCamera.getRotationSpeed() * 180 / Math.PI) / 20; // Convert radians to degrees and divide by 20 for how often it's run
      //       }
      //       if(driverController.getR1Button() && driverController.getL2Button()){
      //         return 0.0;
      //       }
      //       if(driverController.getR1Button()){
      //         return -4.5;
      //       }
      //       if(driverController.getL2Button()){
      //         return 4.5;
      //       }
      //       return 0.0;
      //     },
      //     () -> false, // Field oriented
      //     driverController::getCrossButton, // Towing
      //     () -> driverController.getR2Button(), // Precision mode (disabled)
      //     () -> true, // Turn to angle
      //     () -> { // Turn To angle Direction
      //       double xValue = commandDriverController.getRightX();
      //       double yValue = commandDriverController.getRightY();
      //       double magnitude = (xValue*xValue) + (yValue*yValue);
      //       if (magnitude > 0.49) {
      //         double angle = (90 + NerdyMath.radiansToDegrees(Math.atan2(commandDriverController.getRightY(), commandDriverController.getRightX())));
      //         angle = (((-1 * angle) % 360) + 360) % 360;
      //         SmartDashboard.putNumber("desired angle", angle);
      //         return angle;
      //       }
      //       return 1000.0;
      //     }
      //   ));
  }

  public void initDefaultCommands_test() {
    swerveDrive.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -commandDriverController.getLeftY(), // Horizontal translation
        commandDriverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        commandDriverController::getRightX, // Rotationaq

        // driverController::getSquareButton, // Field oriented
        () -> true, // should be robot oriented now on true

        driverController::getCrossButton, // Towing
        // driverController::getR2Button, // Precision/"Sniper Button"
        () -> driverController.getR2Button(), // Precision mode (disabled)
        () -> driverController.getCircleButton(), // Turn to angle
        // () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          return 0.0;
        }
      ));
  }

  public void configureBindings_teleop() {
    // Driver bindings
    commandDriverController.share().onTrue(Commands.runOnce(imu::zeroHeading).andThen(() -> imu.setOffset(0)));
    commandDriverController.triangle()
      .onTrue(Commands.runOnce(() -> swerveDrive.setVelocityControl(false)))
      .onFalse(Commands.runOnce(() -> swerveDrive.setVelocityControl(true)));


    // Operator bindings
    commandOperatorController.triangle().whileTrue(superSystem.eject());
    commandOperatorController.square().whileTrue(superSystem.getReadyForAmp())
                                      .onFalse(superSystem.shootAmp().andThen(superSystem.stow()));

    commandOperatorController.L1().whileTrue(superSystem.backupIndexerManual()) ;
                                  // .onFalse(superSystem.shooterRoller.setReverseVelocityCommand(0, 0)); // TODO: TEST IF THIS WORKS ON FALSE
    
    // TODO: May have to bring manual stow back if we need it faster
    // commandOperatorController.L2().whileTrue(superSystem.intakeBasic());
    
    commandOperatorController.L2().whileTrue(superSystem.intakeBasic())
                                  .onFalse(superSystem.backupIndexer().andThen(superSystem.stow()));

    commandOperatorController.circle().whileTrue(superSystem.intakeDirectShoot());
    commandOperatorController.R2().whileTrue(superSystem.shootSequence2())
                                  .onFalse(superSystem.stow());
    commandOperatorController.R1().whileTrue(superSystem.shootSequence2Far())
                                  .onFalse(superSystem.stow());

    commandOperatorController.cross().onTrue(superSystem.stow());

    commandOperatorController.share().whileTrue(superSystem.linearActuator.retractCommand());
  }

  public void configureBindings_test() {
    // Driver bindings
    commandDriverController.share().onTrue(Commands.runOnce(imu::zeroHeading).andThen(() -> imu.setOffset(0)));
    commandDriverController.triangle()
      .onTrue(Commands.runOnce(() -> swerveDrive.setVelocityControl(false)))
      .onFalse(Commands.runOnce(() -> swerveDrive.setVelocityControl(true)));

    //TODO: Make sure April Tag ID is matching
    commandDriverController.L1().whileTrue(Commands.run(() -> apriltagCamera.TagDriving(swerveDrive, 0.8, 0, 0, 7, 100)))
      .onFalse(Commands.run(() -> apriltagCamera.reset())); //1.8, 0, 0, 7
    commandDriverController.L2().onTrue(apriltagCamera.aimToApriltagCommand(swerveDrive, 7, 5, 100, true));


    // Operator bindings
    commandOperatorController.triangle().whileTrue(superSystem.eject());
    commandOperatorController.square().whileTrue(superSystem.getReadyForAmp())
                                      .onFalse(superSystem.shootAmp().andThen(superSystem.stow()));

    commandOperatorController.L1().whileTrue(superSystem.backupIndexerManual());
    // commandOperatorController.L2().whileTrue(superSystem.intakeBasic());
    
    commandOperatorController.L2().whileTrue(superSystem.intakeBasic())
                                  .onFalse(superSystem.backupIndexer().andThen(superSystem.stow()));

    commandOperatorController.circle().whileTrue(superSystem.intakeDirectShoot());
    commandOperatorController.R2().whileTrue(superSystem.shootSequence2())
                                  .onFalse(superSystem.stow());
    commandOperatorController.R1().whileTrue(superSystem.shootSequence2Far())
                                  .onFalse(superSystem.stow());

    commandOperatorController.cross().onTrue(superSystem.stow());

    commandOperatorController.share().whileTrue(superSystem.linearActuator.retractCommand());
  }

  private void initAutoChoosers() {
  	List<String> paths = AutoBuilder.getAllAutoNames();
    autoChooser.addOption("Do Nothing", Commands.none());
    
    if (paths.contains("Basic2Piece")) {
      autoChooser.addOption("Basic2Piece", new Basic2Piece(swerveDrive, "Basic2Piece", superSystem));
    }

    if (paths.contains("Basic2PiecePos2")) {
      autoChooser.addOption("Basic2PiecePos2", new Basic2Piece(swerveDrive, "Basic2PiecePos2", superSystem));
    }

    if (paths.contains("Basic3Piece")) {
      autoChooser.addOption("Basic3Piece", new Basic3Piece(swerveDrive, "Basic3Piece", superSystem));
    }

    if (paths.contains("Basic4Piece")) {
      autoChooser.addOption("Basic4Piece", new Basic4Piece(swerveDrive, "Basic4Piece", superSystem));
    }

    if (paths.contains("Basic4PieceSeparated")) {
      autoChooser.addOption("Basic4PieceWithVision", new Basic4PieceWithVision(swerveDrive, "Basic4PieceSeparated", superSystem, apriltagCamera));
      autoChooser.setDefaultOption("Basic4PieceSeparated", new Basic4PieceSeparated(swerveDrive, "Basic4PieceSeparated", superSystem));
    }

    // if (paths.contains("Basic5Piece")) {
    //   autoChooser.addOption("Basic5Piece", new Basic5Piece(swerveDrive, "Basic5Piece", superSystem));
    // }

    if (paths.contains("Basic3PiecePos2")) {
      autoChooser.addOption("Basic3PiecePos2", new Basic3Piece(swerveDrive, "Basic3PiecePos2", superSystem));
    }

    if (paths.contains("Basic3PieceV2")) {
      autoChooser.addOption("Basic3PieceV2", new Basic3Piece(swerveDrive, "Basic3PieceV2", superSystem));
    }
    if (paths.contains("Test2M")) {
      autoChooser.addOption("Test2M", new Test2M(swerveDrive));
    }
    if (paths.contains("Test2MBack")) {
      autoChooser.addOption("Test2MBack", new Test2MBack(swerveDrive));
    }
    if (paths.contains("Basic6PieceSeparated")) {
      autoChooser.addOption("Basic6PieceSeparated", new Basic6PieceSeparated(swerveDrive, "Basic6PieceSeparated", superSystem));
    }
    if (paths.contains("OneMeterSquareAuto")) {
      autoChooser.addOption("OneMeterSquareAuto", new OneMeterSquareAuto(swerveDrive, "OneMeterSquareAuto"));
    }
    if (paths.contains("TwoMeterSquareAuto")) {
      autoChooser.addOption("TwoMeterSquareAuto", new TwoMeterSquareAuto(swerveDrive, "TwoMeterSquareAuto"));
    }
    if (paths.contains("ThreeMeterSquareAuto")) {
      autoChooser.addOption("ThreeMeterSquareAuto", new ThreeMeterSquareAuto(swerveDrive, "ThreeMeterSquareAuto"));
    }
    if (paths.contains("RotateSquareAuto")) {
      autoChooser.addOption("RotateSquareAuto", new RotateSquareAuto(swerveDrive, "RotateSquareAuto"));
    }
    
    if (paths.contains("Mid4Notes")) {
      autoChooser.addOption("Mid4Notes with Vision", new AutoMidNotes(swerveDrive, "Mid4Notes", superSystem, noteCamera, apriltagCamera, superSystem.colorSensor));
    }

    if (paths.contains("Mid3Piece")) {
      autoChooser.addOption("Mid3Piece", new Mid3Piece(swerveDrive, "Mid3Piece", superSystem, apriltagCamera));
    }

    if (paths.contains("Reliable4Piece")) {
      autoChooser.addOption("Reliable 4 Piece", new Reliable4Piece(swerveDrive, "Reliable4Piece", superSystem));
      autoChooser.addOption("Reliable 4 Piece with Vision", new Reliable4PieceWithVision(swerveDrive, "Reliable4Piece", superSystem, apriltagCamera));
    }

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    apriltagCamera.initShuffleboard(LOG_LEVEL.MEDIUM);
    noteCamera.initShuffleboard(LOG_LEVEL.MEDIUM);

    shooterRoller.initShuffleboard(loggingLevel);
    shooterPivot.initShuffleboard(loggingLevel);
    intakePivot.initShuffleboard(loggingLevel);
    intakeRoller.initShuffleboard(loggingLevel);
    indexer.initShuffleboard(loggingLevel);
    superSystem.colorSensor.initShuffleboard(loggingLevel);

    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    // tab.addNumber("Total Current Draw", pdp::getTotalCurrent);
    tab.addNumber("Voltage", () -> Math.abs(pdp.getVoltage()));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}
