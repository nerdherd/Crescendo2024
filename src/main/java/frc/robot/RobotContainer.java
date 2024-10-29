// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Mid5PieceMiddle;
// import frc.robot.commands.autos.Preload;
import frc.robot.commands.autos.PreloadTaxi;
import frc.robot.commands.autos.Reliable4Piece;
import frc.robot.commands.autos.ThreePieceMid;
import frc.robot.subsystems.CANdleSubSystem;
import frc.robot.subsystems.CANdleSubSystem.Status;
import frc.robot.subsystems.IndexerV2;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers;
import frc.robot.util.NerdyMath;

public class RobotContainer {
  public ShooterRoller shooterRoller = new ShooterRoller();
  public ShooterPivot shooterPivot = new ShooterPivot();
  public IntakeRoller intakeRoller = new IntakeRoller();
  public IndexerV2 indexer = new IndexerV2();
  // public Climber climb = new Climber();

  public SuperSystem superSystem = new SuperSystem(intakeRoller, shooterPivot, shooterRoller, indexer);
  
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
  
  public CANdleSubSystem CANdle = new CANdleSubSystem();
  private SwerveJoystickCommand swerveJoystickCommand;
  
  /**
   * The container for the robot. Contain
   * s subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      noteCamera = new NoteAssistance(VisionConstants.kLimelightFrontName);
      swerveDrive = new SwerveDrivetrain(imu);

    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }
    
    LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.kLimelightBackName);
    LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.kLimelightFrontName);

    initAutoChoosers();
    initShuffleboard();

    // Configure the trigger bindings
    // Moved to teleop init
    
    DriverStation.reportWarning("Initalization complete", false);
      // NamedCommands.registerCommand("intakeBasic1", superSystem.intakeBasicHold());
      // NamedCommands.registerCommand("intakeBasic2", superSystem.stopIntaking());
      // NamedCommands.registerCommand("shootSequence2Far", superSystem.shootSequence2Far());
      // NamedCommands.registerCommand("shootSequence2", superSystem.shootSequence2());

  }

  static boolean isRedSide = false;

  public static void refreshAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRedSide = (alliance.get() == DriverStation.Alliance.Red);
    }
  }

  public static boolean IsRedSide() {
    return isRedSide;
      // var alliance = DriverStation.getAlliance();
      // if (alliance.isPresent()) {
      //     return alliance.get() == DriverStation.Alliance.Red;
      // }
      // return false;
  }

  public void initDefaultCommands_teleop() {
    
    shooterPivot.setDefaultCommand(
      new RunCommand(
        () -> {
          double increment = Math.signum(
              NerdyMath.deadband(
                -operatorController.getLeftY(), //0.5 rev/second 
                -ControllerConstants.kDeadband, 
                ControllerConstants.kDeadband)
            ) * 0.1;
          SmartDashboard.putNumber("Increment", increment);
          shooterPivot.incrementPosition(increment);
             // (20 * x) degrees per second
            // If x = 0.1, then v = 2 degrees per second
            
        },
        shooterPivot
      ));
      
      swerveJoystickCommand = 
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -commandDriverController.getLeftY(), // Horizontal translation
        commandDriverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        () -> {
          // if (driverController.getL2Button()) {
          //   SmartDashboard.putBoolean("Turn to angle 2", true);
          //   double turnPower = apriltagCamera.getTurnToTagPower(swerveDrive, angleError, IsRedSide() ? 4 : 7, adjustmentCamera); 
          //   SmartDashboard.putNumber("Turn Power", turnPower);
          //   return turnPower;
          // }
          // SmartDashboard.putBoolean("Turn to angle 2", false);
          // return commandDriverController.getRightX(); // Rotation
        
          if (swerveDrive.getTurnToAngleMode()) {
            return 0.0;
          }
          else {
            return commandDriverController.getRightX();
          }
        },


        // driverController::getSquareButton, // Field oriented
        () -> false, // should be robot oriented now on true
        () -> false,
        // driverController::getCrossButton, // Towing
        // driverController::getR2Button, // Precision/"Sniper Button"
        () -> driverController.getR2Button(), // Precision mode (disabled)
        () -> {
          if (swerveDrive.getTurnToAngleMode()) {
            return (
            Math.abs(driverController.getRightX()) > 0.05 
            || Math.abs(driverController.getRightY()) > 0.05
            );
          } 
          else {
            return(false);  
          }
        }, 
        // () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          double xValue = commandDriverController.getRightX();
            double yValue = commandDriverController.getRightY();
            double magnitude = Math.sqrt((xValue*xValue) + (yValue*yValue));
            if (magnitude > 0.49) {
              double angle = (90 + NerdyMath.radiansToDegrees(Math.atan2(commandDriverController.getRightY(), commandDriverController.getRightX())));
              angle = (((-1 * angle) % 360) + 360) % 360;
              SmartDashboard.putNumber("desired angle", angle);
              return angle;
            }
            return 1000.0;
        });

      swerveDrive.setDefaultCommand(swerveJoystickCommand);
  }

  public void initDefaultCommands_test() {}

  public void configureBindings_teleop() {
    // Driver bindings

    commandDriverController.share().whileTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle())
    );
    // commandDriverController.cross().whileTrue(swerveDrive.driveToAmpCommand(3, 3));
    commandDriverController.cross().whileTrue(superSystem.eject());
    // commandDriverController.square().whileTrue(
    //   Commands.parallel(
    //     noteCamera.driveToNoteCommand(swerveDrive, 15, 0, 0, 10, 200, null),
    //     superSystem.intakeUntilSensedNoBackup().andThen(superSystem.stow())
    //   )                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
    // ).whileFalse(superSystem.backupIndexerAndShooter().andThen(superSystem.stow()));    // commandDriverController.square().onTrue(apriltagCamera.TurnToTagCommand4Auto(swerveDrive, 5, 50));

    commandDriverController.square().whileTrue(superSystem.intakeUntilSensedNoBackup().andThen(superSystem.stow()))
                                    .whileFalse(superSystem.backupIndexerAndShooter().andThen(superSystem.stow()));

    //commandDriverController.square().onTrue(apriltagCamera.TurnToAngleByTagCommand4Auto(swerveDrive, 2, 15));
    commandOperatorController.povLeft().whileTrue(
      Commands.repeatingSequence(
        Commands.runOnce(() -> superSystem.getShooterAngle(swerveDrive))
      )
    );

    

    commandOperatorController.povUp().onTrue(
      Commands.runOnce(() -> superSystem.incrementOffset(0.5))
    );
    commandOperatorController.povDown().onTrue(
      Commands.runOnce(() -> superSystem.incrementOffset(-0.5))
    );
    commandOperatorController.povRight().onTrue(
      Commands.runOnce(() -> superSystem.resetOffset())
    );
    commandDriverController.touchpad().whileTrue(superSystem.shoot())
                                      .whileFalse(superSystem.stow());
    commandDriverController.triangle().onTrue(swerveDrive.toggleTurnToAngleMode());
    
    commandDriverController.L2().whileTrue(Commands.sequence(
      Commands.race(
        superSystem.prepareShooterVision(swerveDrive),
        Commands.sequence(
          Commands.race(
            Commands.waitUntil(() -> 
              // aimTrigger.getAsBoolean() 
              // && 
              armTrigger.getAsBoolean()
            ),
            Commands.waitSeconds(1.5)
          ),
          Commands.waitSeconds(0.1),
          superSystem.indexer.setEnabledCommand(true),
          superSystem.indexer.indexCommand(),
          Commands.either(
            Commands.none(),
            Commands.waitSeconds(0.5),
            () -> superSystem.noteIntook()
          ),
          Commands.waitUntil(() -> !superSystem.noteIntook())
        )
      ),
      superSystem.stow()
    )).whileFalse(superSystem.stow());

    // Operator bindings
    commandOperatorController.triangle().whileTrue(superSystem.eject());
    commandOperatorController.square().whileTrue(superSystem.getReadyForAmp())
                                      .whileFalse(superSystem.stow()); // TODO: Can we try getting rid of this whileFalse line here **(field testing)**
    commandOperatorController.cross().whileTrue(superSystem.shootAmp()).whileFalse(superSystem.stow());
    // commandOperatorController.PS().whileTrue(superSystem.climbSequence());

    commandOperatorController.L1().whileTrue(superSystem.backupIndexerManual());
    
    commandOperatorController.L2().whileTrue(superSystem.intakeUntilSensed().andThen(superSystem.stow()))
                                  .whileFalse(superSystem.stow()); // Get rid of both stows

    commandOperatorController.R2().whileTrue(superSystem.prepareShooterSpeaker())
                                  .whileFalse(superSystem.stow());
    commandOperatorController.R1().whileTrue(superSystem.prepareShooterPodium())
                                  .whileFalse(superSystem.stow());

    commandOperatorController.touchpad().whileTrue(superSystem.ejectIntakeOnly());
    commandOperatorController.PS().whileTrue(superSystem.panicEject());
    commandOperatorController.circle().whileTrue(superSystem.stow()); // TODO: Change this binding
    // commandOperatorController.share().whileTrue(superSystem.intakeDirectShoot());
    commandOperatorController.share().whileTrue(superSystem.advanceIndexerManual());
    commandOperatorController.options().whileTrue(superSystem.prepareShooterVision(swerveDrive)) //
                                  .whileFalse(superSystem.stow()); // TODO: Safety *Do nothing if April Tag is not seen*
  }

  public void configureBindings_test() {}

  Trigger armTrigger = new Trigger(
    () -> superSystem.shooterPivot.atTargetPositionAccurate() 
        && superSystem.shooterPivot.getTargetPositionDegrees() > ShooterConstants.kFullStowPosition.get()
        && superSystem.shooterRoller.getVelocity() > (superSystem.shooterRoller.getTargetVelocity() * 0.6) 
        && superSystem.shooterRoller.getTargetVelocity() > 0
    );

  // AprilTag Trigger
  Trigger aimTrigger = new Trigger(() -> {
    double desiredAngle = swerveDrive.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7);// TODO, update?
    SmartDashboard.putNumber("Desired Angle", desiredAngle);
    double angleToleranceScale = swerveDrive.getTurnToAngleToleranceScale(desiredAngle);
    SmartDashboard.putNumber("Angle Tolerance Scale", angleToleranceScale);
    double angleTolerance = angleToleranceScale * swerveDrive.getSpeakerTurnToAngleTolerance();
    SmartDashboard.putNumber("Angle Tolerance", angleTolerance);
    double angleDifference = Math.abs(swerveDrive.getImu().getHeading() - desiredAngle);
    angleDifference = NerdyMath.posMod(angleDifference, 360);
    SmartDashboard.putNumber("Angle Difference", angleDifference);
    if (angleDifference > 360 - angleTolerance && angleDifference < 360 + angleTolerance) {
      return true;
    } else if (angleDifference > -angleTolerance && angleDifference < angleTolerance) {
      return true;
    }
    return false;
    // return apriltagCamera.apriltagInRange(IsRedSide() ? 4 : 7, 0, 0);
  });



  public void configureLEDTriggers() {
    // Note Trigger
    Trigger noteTrigger = new Trigger(superSystem::noteIntook);
    noteTrigger.onTrue(Commands.runOnce(
      () -> {
        CANdle.setStatus(Status.HASNOTE);
        SmartDashboard.putBoolean("Has Note", true);
      }));
    noteTrigger.onFalse(Commands.runOnce(
      () -> {
        SmartDashboard.putBoolean("Has Note", false); 
        CANdle.setStatus(Status.TELEOP);
      }
    ));

    noteTrigger.and(aimTrigger.negate().and(armTrigger.negate())).onTrue(
      Commands.runOnce(
        () -> {
          CANdle.setStatus(Status.HAS_TARGET);
          SmartDashboard.putBoolean("Tag aimed", false); 
        }
      )
    );

    armTrigger.and(aimTrigger.negate()).onTrue(
      Commands.runOnce(
        () -> {
          CANdle.setStatus(Status.SHOOTER_READY);
        }  
      )
    );

    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    tab.addBoolean("Arm aimed", armTrigger);    
    tab.addBoolean("Drivebase aimed", aimTrigger);

    armTrigger.negate().and(aimTrigger.negate()).and(noteTrigger).onTrue(
      Commands.runOnce(
        () -> {
          CANdle.setStatus(Status.HASNOTE);
        }  
      )
    );

    aimTrigger.and(armTrigger).onTrue(
      Commands.runOnce(
        () -> {
          CANdle.setStatus(Status.SHOTREADY);
          SmartDashboard.putBoolean("Tag aimed", true); 
        }
      )
    );

    noteTrigger.negate().onTrue(Commands.runOnce(
      () -> {
        CANdle.setStatus(Status.TELEOP);
        SmartDashboard.putBoolean("Tag aimed", false); 
      }
    ));

    // if (driverController.getCircleButton()) { //turn to amp
    //         if (!IsRedSide()){
    //           return 270.0;
    //         }
    //         return 90.0;
    //       }
    //       else if (driverController.getL1Button()) { //turn to speaker
    //         return 0.0;
    //       }

    // Lined up and ready to shoot Trigger
    // Speaker
    // Trigger L1Trigger = new Trigger(commandDriverController.L1());
    // L1Trigger.whileTrue(Commands.runOnce(
    //   () -> {
    //     heading = NerdyMath.posMod(imu.getHeading(), 360);
    //     angleError = heading - 0; // Heading for speaker
    //     if (Math.abs(angleError) <= 10) {
    //       CANdle.setStatus(Status.SHOTREADY);
    //     } else {
    //       CANdle.setStatus(Status.LASTSTATUS);
    //     }
    //   }
    // ));

    // Trigger shootTrigger = commandDriverController.R1(); // At this point Zach realized that he was about to change the code to the exact same as it was before but with extra variables
    // commandDriverController.R1().whileTrue(Commands.runOnce(
    //   () -> {
    //     heading = NerdyMath.posMod(imu.getHeading(), 360);



    
    //     if (!IsRedSide()) {
    //       angleError = heading - 270;
    //     } else {
    //       angleError = heading - 90;
    //     }
    //     if (Math.abs(angleError) <= 10) {
    //       CANdle.setStatus(Status.SHOTREADY);
    //     } else {
    //       CANdle.setStatus(Status.LASTSTATUS);
    //     }
    //   }
    // ));
    // Trigger shotReadyTriger = new Trigger();
    // tagTrigger.onTrue(Commands.runOnce(
    //   () -> CANdle.setStatus(Status.SHOTREADY)
    // ));
    // tagTrigger.onFalse(Commands.runOnce(
    //   () -> CANdle.setStatus(Status.TELEOP)
    // ));
  }

  //PathPlannerPath a01 = PathPlannerPath.fromPathFile("a01Path");
  PathPlannerPath a02 = PathPlannerPath.fromPathFile("a02Path");
  PathPlannerPath pieceOneGrab = PathPlannerPath.fromPathFile("Piece1Grab");
  PathPlannerPath aSY = PathPlannerPath.fromPathFile("aSYPath");
  PathPlannerPath aY3 = PathPlannerPath.fromPathFile("aY3Path");  
  
  //PathPlannerPath b12 = PathPlannerPath.fromPathFile("b12Path");
  // PathPlannerPath b23 = PathPlannerPath.fromPathFile("b23Path");
  // PathPlannerPath b32 = PathPlannerPath.fromPathFile("b32Path");
  // PathPlannerPath b21 = PathPlannerPath.fromPathFile("b21Path");
  // PathPlannerPath b31 = PathPlannerPath.fromPathFile("b31Path");
  PathPlannerPath b31 = PathPlannerPath.fromPathFile("b31Path");
  PathPlannerPath b2p6 = PathPlannerPath.fromPathFile("b2p6Path");

  // PathPlannerPath c14 = PathPlannerPath.fromPathFile("c14Path");
  // PathPlannerPath c24 = PathPlannerPath.fromPathFile("c24Path");
  // PathPlannerPath c34 = PathPlannerPath.fromPathFile("c34Path");
  // PathPlannerPath c35 = PathPlannerPath.fromPathFile("c35Path");
  // PathPlannerPath c36 = PathPlannerPath.fromPathFile("c36Path");
  // PathPlannerPath c37 = PathPlannerPath.fromPathFile("c37Path");
  // PathPlannerPath c38 = PathPlannerPath.fromPathFile("c38Path");
  // PathPlannerPath c18 = PathPlannerPath.fromPathFile("c18Path");
  // PathPlannerPath c17 = PathPlannerPath.fromPathFile("c17Path");
  // PathPlannerPath c16 = PathPlannerPath.fromPathFile("c16Path");
  // PathPlannerPath c15 = PathPlannerPath.fromPathFile("c15Path");
  // PathPlannerPath c28 = PathPlannerPath.fromPathFile("c28Path");
  // PathPlannerPath c27 = PathPlannerPath.fromPathFile("c27Path");
  PathPlannerPath c26 = PathPlannerPath.fromPathFile("c26Path");
  PathPlannerPath c26Fast = PathPlannerPath.fromPathFile("c26PathFast");
  //PathPlannerPath c25 = PathPlannerPath.fromPathFile("c25Path");
  //PathPlannerPath c25Stage = PathPlannerPath.fromPathFile("c25PathStage");
  //PathPlannerPath c26Short = PathPlannerPath.fromPathFile("c26PathShort");
  
  //PathPlannerPath d26ShortC = PathPlannerPath.fromPathFile("d26PathShortC");
  PathPlannerPath d26 = PathPlannerPath.fromPathFile("d26Path");
  PathPlannerPath d25 = PathPlannerPath.fromPathFile("d25Path");
  PathPlannerPath d27 = PathPlannerPath.fromPathFile("d27Path");
  // PathPlannerPath d45 = PathPlannerPath.fromPathFile("d45Path");
  // PathPlannerPath d56 = PathPlannerPath.fromPathFile("d56Path");
  // PathPlannerPath d87 = PathPlannerPath.fromPathFile("d87Path");
  // PathPlannerPath d76 = PathPlannerPath.fromPathFile("d76Path");
  // PathPlannerPath d65 = PathPlannerPath.fromPathFile("d65Path");
  // PathPlannerPath d67 = PathPlannerPath.fromPathFile("d67Path");
  // PathPlannerPath d78 = PathPlannerPath.fromPathFile("d78Path");
  // PathPlannerPath d54 = PathPlannerPath.fromPathFile("d54Path");

  // PathPlannerPath e4Y = PathPlannerPath.fromPathFile("e4YPath");
  // PathPlannerPath e5Y = PathPlannerPath.fromPathFile("e5YPath");
  // PathPlannerPath e7Y = PathPlannerPath.fromPathFile("e7YPath");
  PathPlannerPath e7Z = PathPlannerPath.fromPathFile("e7ZPath");
  PathPlannerPath e8Z = PathPlannerPath.fromPathFile("e8ZPath");
  PathPlannerPath e5Y = PathPlannerPath.fromPathFile("e5YPath");
  PathPlannerPath e6Y = PathPlannerPath.fromPathFile("e6YPath");
  PathPlannerPath e7Y = PathPlannerPath.fromPathFile("e7YPath");

  // PathPlannerPath e6YShort = PathPlannerPath.fromPathFile("e6YPathShort");

  // PathPlannerPath f04 = PathPlannerPath.fromPathFile("f04Path");
  // PathPlannerPath f05 = PathPlannerPath.fromPathFile("f05Path");
  // PathPlannerPath f06 = PathPlannerPath.fromPathFile("f06Path");
  // PathPlannerPath f07 = PathPlannerPath.fromPathFile("f07Path");
  // PathPlannerPath f08 = PathPlannerPath.fromPathFile("f08Path");
  PathPlannerPath fZ7 = PathPlannerPath.fromPathFile("fZ7Path");
  PathPlannerPath fS8 = PathPlannerPath.fromPathFile("fS8Path");
  // PathPlannerPath fast = PathPlannerPath.fromPathFile("c26TestPath");
  PathPlannerPath fS7 = PathPlannerPath.fromPathFile("fS7");

  // final List<PathPlannerPath> pathGroupExample3 = List.of(
  //   a01, c15, a03
  // );
  
  // final List<PathPlannerPath> pathGroupFivePiece = List.of(
  //    a03, b32, b21, c14
  // );
  // final List<PathPlannerPath> pathGroupThreePiece = List.of(
  //    fS8, e8Z, c37, e7Z
  // );
  // final List<PathPlannerPath> pathGroupTestA = List.of(
  //    a02
  // );
  // final List<PathPlannerPath> pathGroupTestB = List.of(
  //    b23
  // );
  // final List<PathPlannerPath> pathGroupTestC = List.of(
  //    fast
  // );
  // final List<PathPlannerPath> pathGroupTestD = List.of(
  //    d45, d56
  // );
  // final List<PathPlannerPath> pathGroupTestE = List.of(
  //    e5Y
  // );
  // final List<PathPlannerPath> pathGroupTestF = List.of(
  //    f04
  // );
  // final List<PathPlannerPath> variantPathGroup = List.of(
  //    a03, b32, b21, c14, d45, d56, e4Y
  // );

  private void initAutoChoosers() {
  	List<String> paths = AutoBuilder.getAllAutoNames();
    autoChooser.addOption("Do Nothing", Commands.none());

    if (paths.contains("TaxiOnly")) {
      autoChooser.addOption("Taxi Only", AutoBuilder.buildAuto("TaxiOnly"));
      autoChooser.addOption("Preload Taxi Source", new PreloadTaxi(swerveDrive, List.of(aSY), superSystem));
      // autoChooser.addOption("Preload", new Preload(swerveDrive, List.of(a02), superSystem));
    }
    

    if (paths.contains("Reliable4Piece")) {
      autoChooser.setDefaultOption("Reliable 4 Piece", new Reliable4Piece(swerveDrive, "Reliable4Piece", superSystem));
    }

    // if (paths.contains("NEW4Piece")) {
    //   autoChooser.addOption("New 4 Piece", new Reliable4Piece(swerveDrive, "NEW4Piece", superSystem));
    // }



    // autoChooser.addOption("4PieceMiddle",         new Mid4Piece(swerveDrive, superSystem, noteCamera, List.of(a02,b2p6,c26,    d26,e6Y,aY3)));
    autoChooser.addOption("5PieceMiddle",   new Mid5PieceMiddle(swerveDrive, superSystem, noteCamera, List.of(a02,b2p6,c26Fast,d26,e6Y,aY3,b31)));
    // autoChooser.addOption("4PieceAmpSide",    new Mid4PieceSide(swerveDrive, superSystem, noteCamera, List.of(a02,b2p6,c26,    d25,e5Y,aY3)));
    // autoChooser.addOption("4PieceSourceSide", new Mid4PieceSide(swerveDrive, superSystem, noteCamera, List.of(a02,b2p6,c26,    d27,e7Y,aY3)));

    autoChooser.addOption("Three Piece Source",   new ThreePieceMid(swerveDrive, superSystem, List.of(fS8, e8Z, fZ7, e7Z), noteCamera));
    autoChooser.addOption("ThreePieceNote7", new ThreePieceMid(swerveDrive, superSystem, List.of(fS7, e7Z, fS8, e8Z), noteCamera));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(LOG_LEVEL.MINIMAL);    
    noteCamera.initShuffleboard(LOG_LEVEL.MEDIUM);

    shooterRoller.initShuffleboard(loggingLevel);
    shooterPivot.initShuffleboard(loggingLevel);
    intakeRoller.initShuffleboard(loggingLevel);
    indexer.initShuffleboard(loggingLevel);
    // superSystem.colorSensor.initShuffleboard(loggingLevel);
    superSystem.bannerSensor.initShuffleboard(loggingLevel);

    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    // tab.addNumber("Total Current Draw", pdp::getTotalCurrent);
    tab.addNumber("Voltage", () -> Math.abs(pdp.getVoltage()));
    tab.addNumber("apriltag angle", () -> swerveDrive.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7));// TODO, update?
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
