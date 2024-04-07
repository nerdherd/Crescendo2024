// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
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
import frc.robot.commands.autos.AutoCommands;
import frc.robot.commands.autos.FivePieceEnd;
import frc.robot.commands.autos.FivePieceSecond;
import frc.robot.commands.autos.Mid2Piece;
import frc.robot.commands.autos.Mid3Piece;
import frc.robot.commands.autos.Mid3PieceDeadReckoning;
import frc.robot.commands.autos.Mid3PiecePathOnly;
import frc.robot.commands.autos.PoseEstimatorTest;
import frc.robot.commands.autos.PreloadTaxi;
import frc.robot.commands.autos.Reliable4Piece;
import frc.robot.commands.autos.PathVariants.PathA;
import frc.robot.commands.autos.PathVariants.PathB;
import frc.robot.commands.autos.PathVariants.PathC;
import frc.robot.commands.autos.PathVariants.PathD;
import frc.robot.commands.autos.PathVariants.PathE;
import frc.robot.commands.autos.PathVariants.PathF;
import frc.robot.commands.autos.PathVariants.SuperPath;
import frc.robot.commands.autos.PathVariants.Variant5Piece;
import frc.robot.subsystems.CANdleSubSystem;
import frc.robot.subsystems.Climber;
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
import frc.robot.subsystems.vision.DriverAssist;
import frc.robot.subsystems.vision.NoteAssistance;
import frc.robot.subsystems.vision.ShooterVisionAdjustment;
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
  public DriverAssist apriltagCamera;// = new DriverAssist(VisionConstants.kLimelightFrontName, 4);
  public ShooterVisionAdjustment adjustmentCamera;
  
  public CANdleSubSystem CANdle = new CANdleSubSystem();
  private SwerveJoystickCommand swerveJoystickCommand;

  
  /**
   * The container for the robot. Contain
   * s subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      noteCamera = new NoteAssistance(VisionConstants.kLimelightFrontName);
      apriltagCamera = new DriverAssist(VisionConstants.kLimelightBackName, VisionConstants.kAprilTagPipeline);
      swerveDrive = new SwerveDrivetrain(imu, apriltagCamera);
      adjustmentCamera = new ShooterVisionAdjustment(apriltagCamera, swerveDrive.getImu(), superSystem, () -> swerveDrive.getPose());

    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    apriltagCamera.toggleLight(true);

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
          return commandDriverController.getRightX(); // Rotation
        },

        // driverController::getSquareButton, // Field oriented
        () -> false, // should be robot oriented now on true

        driverController::getCrossButton, // Towing
        // driverController::getR2Button, // Precision/"Sniper Button"
        () -> driverController.getR2Button(), // Precision mode (disabled)
        () -> {
          return (
            driverController.getR1Button() 
            || driverController.getL1Button() 
            || driverController.getL2Button() 
            || driverController.getCircleButton()
            || driverController.getTriangleButton()
            // || driverController.getPSButton()
          ); // Turn to angle
        }, 
        // () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          if (driverController.getTriangleButton()) {
            if (!IsRedSide()) {
              return 135.0;
            } else {
              return 225.0;
            }
          }
          if (driverController.getL2Button()) {
            return apriltagCamera.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7);
            // 4 if red side, 7 if blue
          }
          if (driverController.getCircleButton()) { //turn to amp
            if (!IsRedSide()){
              return 270.0;
            }
            return 90.0;
          }
          else 
          if (driverController.getL1Button()) { //turn to speaker
            return 0.0;
          }
          else if (driverController.getR1Button()) {
            return 180.0;
          }
          // if (driverController.getPSButton()) { // Turn to shuffleboard angle
          //   return SmartDashboard.getNumber("Test Desired Angle", 0);
          // }
          return 0.0; 
        }
      );

      swerveDrive.setDefaultCommand(swerveJoystickCommand);

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

  public void initDefaultCommands_test() {}

  public void configureBindings_teleop() {
    // Driver bindings

    commandDriverController.share().whileTrue(Commands.runOnce(imu::zeroHeading).andThen(() -> imu.setOffset(0)));
    commandDriverController.cross().whileTrue(apriltagCamera.driveToAmpCommand(swerveDrive, 3, 3));
    commandDriverController.square().whileTrue(noteCamera.driveToNoteCommand(swerveDrive, 15, 0, 0, 0, 0, null).until(superSystem::noteIntook));
    commandDriverController.square().onTrue(apriltagCamera.TurnToTagCommand4Auto(swerveDrive, 5, 50));
    commandOperatorController.povLeft().whileTrue(
      Commands.repeatingSequence(
        Commands.runOnce(() -> adjustmentCamera.getShooterAngle())
      )
    );

    commandOperatorController.povUp().onTrue(
      Commands.runOnce(() -> adjustmentCamera.incrementOffset(0.5))
    );
    commandOperatorController.povDown().onTrue(
      Commands.runOnce(() -> adjustmentCamera.incrementOffset(-0.5))
    );
    commandOperatorController.povRight().onTrue(
      Commands.runOnce(() -> adjustmentCamera.resetOffset())
    );
    commandDriverController.touchpad().whileTrue(superSystem.shoot())
                                      .whileFalse(superSystem.stow());

    // Operator bindings
    commandOperatorController.triangle().whileTrue(superSystem.eject());
    commandOperatorController.square().whileTrue(superSystem.getReadyForAmp())
                                      .whileFalse(superSystem.stow()); // TODO: Can we try getting rid of this whileFalse line here **(field testing)**
    commandOperatorController.cross().whileTrue(superSystem.shootAmp()).whileFalse(superSystem.stow());
    commandOperatorController.PS().whileTrue(superSystem.climbSequence());

    commandOperatorController.L1().whileTrue(superSystem.backupIndexerManual());
    
    commandOperatorController.L2().whileTrue(superSystem.intakeUntilSensed().andThen(superSystem.stow()))
                                  .whileFalse(superSystem.stow()); // Get rid of both stows

    commandOperatorController.R2().whileTrue(superSystem.prepareShooterSpeaker())
                                  .whileFalse(superSystem.stow());
    commandOperatorController.R1().whileTrue(superSystem.prepareShooterPodium())
                                  .whileFalse(superSystem.stow());

    commandOperatorController.touchpad().whileTrue(superSystem.panicButton())
                                        .whileFalse(superSystem.backupIndexer().andThen(superSystem.stow()));
    commandOperatorController.circle().whileTrue(superSystem.stow()); // TODO: Change this binding
    commandOperatorController.share().whileTrue(superSystem.intakeDirectShoot());
    commandOperatorController.options().whileTrue(superSystem.prepareShooterVision(adjustmentCamera)) //
                                  .whileFalse(superSystem.stow()); // TODO: Safety *Do nothing if April Tag is not seen*
  }

  public void configureBindings_test() {}




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

    Trigger armTrigger = new Trigger(
      () -> superSystem.shooterPivot.atTargetPositionAccurate() 
         && superSystem.shooterPivot.getTargetPositionDegrees() > ShooterConstants.kFullStowPosition.get()
         && superSystem.shooterRoller.getVelocity() > (superSystem.shooterRoller.getTargetVelocity() * 0.6) 
         && superSystem.shooterRoller.getTargetVelocity() > 0
      );

    Trigger tagTrigger = new Trigger(
      () -> {
        return superSystem.noteIntook() 
            && apriltagCamera.hasValidTarget(); 
      }
    );

    // AprilTag Trigger
    Trigger aimTrigger = new Trigger(() -> {
      double desiredAngle = apriltagCamera.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7);
      SmartDashboard.putNumber("Desired Angle", desiredAngle);
      double angleToleranceScale = apriltagCamera.getTurnToAngleToleranceScale(desiredAngle);
      SmartDashboard.putNumber("Angle Tolerance Scale", angleToleranceScale);
      double angleTolerance = angleToleranceScale * apriltagCamera.getTurnToAngleTolerance(adjustmentCamera.getDistanceFromTag(true));
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

    noteTrigger.and(tagTrigger.and(aimTrigger.negate()).and(armTrigger.negate())).onTrue(
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

    armTrigger.negate().and(aimTrigger.negate()).and(tagTrigger.negate()).and(noteTrigger).onTrue(
      Commands.runOnce(
        () -> {
          CANdle.setStatus(Status.HASNOTE);
        }  
      )
    );

    tagTrigger.and(aimTrigger).and(armTrigger).onTrue(
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

  PathPlannerPath a01 = PathPlannerPath.fromPathFile("a01Path");
  PathPlannerPath a02 = PathPlannerPath.fromPathFile("a02Path");
  PathPlannerPath a03 = PathPlannerPath.fromPathFile("a03Path");
  
  PathPlannerPath b12 = PathPlannerPath.fromPathFile("b12Path");
  PathPlannerPath b23 = PathPlannerPath.fromPathFile("b23Path");
  PathPlannerPath b32 = PathPlannerPath.fromPathFile("b32Path");
  PathPlannerPath b21 = PathPlannerPath.fromPathFile("b21Path");
  PathPlannerPath b31 = PathPlannerPath.fromPathFile("b31Path");
  
  PathPlannerPath c14 = PathPlannerPath.fromPathFile("c14Path");
  PathPlannerPath c24 = PathPlannerPath.fromPathFile("c24Path");
  PathPlannerPath c34 = PathPlannerPath.fromPathFile("c34Path");
  PathPlannerPath c35 = PathPlannerPath.fromPathFile("c35Path");
  PathPlannerPath c36 = PathPlannerPath.fromPathFile("c36Path");
  PathPlannerPath c37 = PathPlannerPath.fromPathFile("c37Path");
  PathPlannerPath c38 = PathPlannerPath.fromPathFile("c38Path");
  PathPlannerPath c18 = PathPlannerPath.fromPathFile("c18Path");
  PathPlannerPath c17 = PathPlannerPath.fromPathFile("c17Path");
  PathPlannerPath c16 = PathPlannerPath.fromPathFile("c16Path");
  PathPlannerPath c15 = PathPlannerPath.fromPathFile("c15Path");
  PathPlannerPath c28 = PathPlannerPath.fromPathFile("c28Path");
  PathPlannerPath c27 = PathPlannerPath.fromPathFile("c27Path");
  PathPlannerPath c26 = PathPlannerPath.fromPathFile("c26Path");
  PathPlannerPath c25 = PathPlannerPath.fromPathFile("c25Path");
  
  PathPlannerPath d45 = PathPlannerPath.fromPathFile("d45Path");
  PathPlannerPath d56 = PathPlannerPath.fromPathFile("d56Path");
  PathPlannerPath d87 = PathPlannerPath.fromPathFile("d87Path");
  PathPlannerPath d76 = PathPlannerPath.fromPathFile("d76Path");
  PathPlannerPath d65 = PathPlannerPath.fromPathFile("d65Path");
  PathPlannerPath d67 = PathPlannerPath.fromPathFile("d67Path");
  PathPlannerPath d78 = PathPlannerPath.fromPathFile("d78Path");
  PathPlannerPath d54 = PathPlannerPath.fromPathFile("d54Path");

  PathPlannerPath e4Y = PathPlannerPath.fromPathFile("e4YPath");
  PathPlannerPath e5Y = PathPlannerPath.fromPathFile("e5YPath");
  PathPlannerPath e7Y = PathPlannerPath.fromPathFile("e7YPath");
  PathPlannerPath e7Z = PathPlannerPath.fromPathFile("e7ZPath");
  PathPlannerPath e8Z = PathPlannerPath.fromPathFile("e8ZPath");
  PathPlannerPath e6Y = PathPlannerPath.fromPathFile("e6YPath");

  PathPlannerPath f04 = PathPlannerPath.fromPathFile("f04Path");
  PathPlannerPath f05 = PathPlannerPath.fromPathFile("f05Path");
  PathPlannerPath f06 = PathPlannerPath.fromPathFile("f06Path");
  PathPlannerPath f07 = PathPlannerPath.fromPathFile("f07Path");
  PathPlannerPath f08 = PathPlannerPath.fromPathFile("f08Path");


  // final List<PathPlannerPath> pathGroupExample3 = List.of(
  //   a01, c15, a03
  // );
  final List<PathPlannerPath> pathGroupTestSuper = List.of(
     a02, b23
  );
  final List<PathPlannerPath> pathGroupTestA = List.of(
     a02
  );
  final List<PathPlannerPath> pathGroupTestB = List.of(
     b23
  );
  final List<PathPlannerPath> pathGroupTestC = List.of(
     c24
  );
  final List<PathPlannerPath> pathGroupTestD = List.of(
     d45, d56
  );
  final List<PathPlannerPath> pathGroupTestE = List.of(
     e5Y
  );
  final List<PathPlannerPath> pathGroupTestF = List.of(
     f04
  );
  final List<PathPlannerPath> variantPathGroup = List.of(
     a03, b32, b21, c14, d45, d56, e4Y
  );

  private void initAutoChoosers() {
  	List<String> paths = AutoBuilder.getAllAutoNames();
    autoChooser.addOption("Do Nothing", Commands.none());
    if (paths.contains("PoseEstimatorTest")) 
    autoChooser.addOption("Pose Estimator Test Auto", new PoseEstimatorTest(swerveDrive,"PoseEstimatorTest", superSystem ));
    
    // Testing/characterization autos
    // if (paths.contains("Test2M")) {
    //   // autoChooser.addOption("Test2M", new Test2M(swerveDrive));
    //   autoChooser.addOption("Preload Taxi Straight", new PreloadTaxi(swerveDrive, pathGroupExample3, superSystem));
    // }

    if (paths.contains("PoseEstimatorTest")) {
      autoChooser.addOption("Pose Estimator Test Auto", new PoseEstimatorTest(swerveDrive,"PoseEstimatorTest", superSystem));
    }


    // Note to self: IMU may not be facing the right way at the end of the auto
    if (paths.contains("Mid3Piece")) {
      autoChooser.addOption("Mid3Piece", new Mid3Piece(swerveDrive, "Mid3Piece", superSystem, apriltagCamera, adjustmentCamera));
      autoChooser.addOption("Mid3Piece Path Only", new Mid3PiecePathOnly(swerveDrive, "Mid3Piece", superSystem, apriltagCamera));
      autoChooser.addOption("Mid3Piece Dead Reckoning", new Mid3PieceDeadReckoning(swerveDrive, "Mid3Piece", superSystem));
      autoChooser.addOption("Mid2Piece", new Mid2Piece(swerveDrive, "Mid3Piece", superSystem, apriltagCamera, adjustmentCamera));
    }

    if (paths.contains("PreloadTaxiSourceSide")) {
      //autoChooser.addOption("Preload Taxi Source Side", new PreloadTaxi(swerveDrive, "PreloadTaxiSourceSide", superSystem));
    }

    if (paths.contains("PreloadTaxiPodiumSide")) {
      //autoChooser.addOption("Preload Taxi Podium Side", new PreloadTaxi(swerveDrive, "PreloadTaxiPodiumSide", superSystem));
    }

    if (paths.contains("TaxiOnly")) {
      autoChooser.addOption("Taxi Only", AutoBuilder.buildAuto("TaxiOnly"));
    }
    

    // if (paths.contains("Reliable4Piece")) {
    //   autoChooser.setDefaultOption("Reliable 4 Piece", new Reliable4Piece(swerveDrive, "Reliable4Piece", superSystem));
    //   // autoChooser.addOption("Reliable 4 Piece with Vision", new Reliable4PieceWithVision(swerveDrive, "Reliable4Piece", superSystem, apriltagCamera));
    // }

    if (paths.contains("NEW4Piece")) {
      autoChooser.addOption("New 4 Piece", new Reliable4Piece(swerveDrive, "NEW4Piece", superSystem));
    }

    if (paths.contains("5PieceMid")){
      autoChooser.addOption("5 Piece Mid", new FivePieceEnd(swerveDrive, "5PieceMid", superSystem, adjustmentCamera));
    }

    if (paths.contains("5PieceMidSecond")){
      autoChooser.addOption("5 Piece Mid Second", new FivePieceSecond(swerveDrive, "5PieceMid", superSystem, adjustmentCamera));
    }

    autoChooser.setDefaultOption("SuperPath", new SuperPath(swerveDrive, superSystem, pathGroupTestSuper, apriltagCamera, adjustmentCamera));
    autoChooser.addOption("PathA", new PathA(swerveDrive, superSystem, a02, apriltagCamera, adjustmentCamera));
    autoChooser.addOption("PathB", new PathB(swerveDrive, superSystem, b23, apriltagCamera, adjustmentCamera));
    autoChooser.addOption("PathC", new PathC(swerveDrive, c24));
    autoChooser.addOption("PathD", new PathD(swerveDrive, superSystem, noteCamera, 1, 10, 50, pathGroupTestD.get(0), pathGroupTestD.get(1)));
    autoChooser.addOption("PathE", new PathE(swerveDrive, superSystem, e5Y, apriltagCamera, adjustmentCamera));
    autoChooser.addOption("PathF", new PathF(swerveDrive, superSystem, f04, apriltagCamera));
    autoChooser.addOption("TestPath", new Variant5Piece(swerveDrive, superSystem, variantPathGroup, apriltagCamera, adjustmentCamera, noteCamera));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    apriltagCamera.initShuffleboard(LOG_LEVEL.MEDIUM);
    noteCamera.initShuffleboard(LOG_LEVEL.MEDIUM);
    adjustmentCamera.initShuffleboard(LOG_LEVEL.ALL);

    shooterRoller.initShuffleboard(loggingLevel);
    shooterPivot.initShuffleboard(loggingLevel);
    intakeRoller.initShuffleboard(loggingLevel);
    indexer.initShuffleboard(loggingLevel);
    // superSystem.colorSensor.initShuffleboard(loggingLevel);
    superSystem.bannerSensor.initShuffleboard(loggingLevel);

    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    // tab.addNumber("Total Current Draw", pdp::getTotalCurrent);
    tab.addNumber("Voltage", () -> Math.abs(pdp.getVoltage()));
    tab.addNumber("apriltag angle", () -> apriltagCamera.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7));
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
