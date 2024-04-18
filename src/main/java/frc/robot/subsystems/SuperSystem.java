package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
//import frc.robot.subsystems.vision.DriverAssist;
//import frc.robot.subsystems.vision.ShooterVisionAdjustment;
import frc.robot.util.NerdyLine;
import frc.robot.util.NerdyMath;

import frc.robot.subsystems.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SuperSystem {
    public IntakeRoller intakeRoller;
    public ShooterPivot shooterPivot;
    public ShooterRoller shooterRoller;
    public IndexerV2 indexer;
    // public ColorSensor colorSensor;
    public BannerSensor bannerSensor;
    // add line below once new banner sensor is added
    // public BannerSensor bannerSensorTwo;
    // public BeamBreak noteSensor;
    //public ClimbActuator linearActuator;
    // public SwerveDrivetrain swerve;
    // public Climber climber;

    //vision
    // Warbots field
    private double[] distances = {1.2,   2.483,   3.015,    3.573,   4.267,   4.697}; // distances from 4/6
    private double[] angles = {-52.470, -32.861, -29.114, -25.663, -21.413, -20.8}; // angles from 4/6
    private double lastAngle = -0.2;
    private double angleOffset = 0;
    private NerdyLine angleLine;

    public SuperSystem(IntakeRoller intakeRoller, 
                        ShooterPivot shooterPivot, ShooterRoller shooterRoller,
                        IndexerV2 indexer) {
        this.intakeRoller = intakeRoller;
        this.shooterPivot = shooterPivot;
        this.shooterRoller = shooterRoller;
        this.indexer = indexer;
        // this.climber = climber;
        //this.linearActuator = new ClimbActuator();
        // this.colorSensor = new ColorSensor();
        this.bannerSensor = new BannerSensor();
        // add line below once we add new banner sensor
        // this.bannerSensorTwo = new BannerSensor();
        // this.noteSensor = new BeamBreak();
        angleLine = new NerdyLine(distances, angles);
    }

    public boolean noteIntook() {
        // return colorSensor.noteIntook() || bannerSensor.noteIntook();
        return bannerSensor.noteIntook();
    }

    public double getShooterAngle(SwerveDrivetrain swerve)
    {
        return getShooterAngle(swerve, false);
    }
    public void incrementOffset(double increment)
    {
        angleOffset += increment;
        angleOffset = NerdyMath.clamp(angleOffset, -10, 10);
    }
    public void resetOffset()
    {
        angleOffset = 0;
    }

    public double getShooterAngle(SwerveDrivetrain swerve, boolean preserveOldValue)
    {
        double distance = swerve.getDistanceFromTag(preserveOldValue, RobotContainer.IsRedSide() ? 4 : 7);
        if(distance < distances[0]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return (preserveOldValue ? lastAngle : -0.2);
        }
        if (distance > distances[distances.length - 1]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return (preserveOldValue ? lastAngle : -0.3);
        }
        
        double output = NerdyMath.clamp(angleLine.getOutput(distance), ShooterConstants.kFullStowPosition.get(), 20);
        output += angleOffset;
        lastAngle = output;
        return output + 1.5;
    }

    public Command getReadyForAmp() {
        Command command = Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Amp Rotating", true)),
            shooterPivot.moveToAmp(),
            Commands.waitUntil(shooterPivot::atTargetPosition)
        );

        command.addRequirements(shooterPivot);

        return command;
    }

    public Command shootAmp() {
        Command command = Commands.sequence(
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootAmp(),
            Commands.waitSeconds(0.3), // Was 0.5    3/3/24    Code Orange
            indexer.setEnabledCommand(true),
            indexer.setVelocityCommand(30),
            Commands.waitSeconds(1)
        ).finallyDo(
            () -> {
                shooterRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(shooterRoller, indexer, intakeRoller);

        return command;
    }

    public Command stow() {
        Command command = Commands.sequence(
            intakeRoller.stopCommand(),
            shooterRoller.stopCommand(),
            indexer.stopCommand(),
            shooterPivot.setPositionCommand(ShooterConstants.kFullStowPosition.get())
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);

        return command;
    }

    public Command handoff() {
        Command command = shooterPivot.moveToHandoff();
        return command;
    }

    public Command panicButton() {
        Command command = Commands.sequence(
            shooterPivot.setPositionCommand(6),
            shooterRoller.setVelocityCommand(-20),
            shooterRoller.setEnabledCommand(true),
            indexer.indexCommand(),
            indexer.setEnabledCommand(true),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                shooterRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(shooterPivot, indexer, shooterRoller);

        return command;
    }

    public Command backupIndexer() {
        Command command = Commands.sequence(
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.2),
            indexer.stopCommand()
        ).finallyDo(indexer::stop);

        command.addRequirements(indexer);
        
        return command;
    }

    public Command backupIndexerAndShooter() {
        Command command = Commands.sequence(
            shooterRoller.setVelocityCommand(-20, -20),
            shooterRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.2),
            indexer.stopCommand(),
            shooterRoller.stopCommand()
        ).finallyDo(() -> {
            indexer.stop();
            shooterRoller.stop();    
        });

        command.addRequirements(indexer, shooterRoller);
        
        return command;
    }

    public Command backupIndexerAndShooterLess() {
        Command command = Commands.sequence(
            shooterRoller.setVelocityCommand(-20, -20),
            shooterRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.15),
            indexer.stopCommand(),
            shooterRoller.stopCommand()
        ).finallyDo(() -> {
            indexer.stop();
            shooterRoller.stop();    
        });

        command.addRequirements(indexer, shooterRoller);
        
        return command;
    }

    public Command backupIndexerManual() {
        Command command = Commands.sequence(
            // shooterPivot.moveToSpeaker(),
            indexer.setEnabledCommand(true),
            shooterRoller.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            shooterRoller.setReverseVelocityCommand(-10, -10), // TODO: Later
            Commands.waitSeconds(0.2)
        ).finallyDo(() -> {
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterRoller, indexer);
        
        return command.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command advanceIndexerManual() {
        Command command = Commands.sequence(
            // shooterPivot.moveToSpeaker(),
            indexer.setEnabledCommand(true),
            shooterRoller.setEnabledCommand(true),
            indexer.reverseIndexCommand(),
            shooterRoller.setReverseVelocityCommand(10, 10), // TODO: Later
            Commands.waitSeconds(0.2)
        ).finallyDo(() -> {
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterRoller, indexer);
        
        return command.withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command intakeUntilSensed() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
            ),
            shooterRoller.setVelocityCommand(0, 0),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

            // Commands.deadline(
                // Commands.waitSeconds(1), // testing - check wait time             
            Commands.waitUntil(this::noteIntook),
            // ),
            
            // Move note back
            intakeRoller.stopCommand(),
            indexer.reverseIndexCommand(),
            shooterRoller.setVelocityCommand(0, 0),
            Commands.waitSeconds(0.2), // Was 0.6   3/3/24   Code Orange

            indexer.stopCommand(),
            shooterRoller.stopCommand()
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command intakeUntilSensed(double timeout) {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
            ),
            shooterRoller.setVelocityCommand(-10, -10),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

            Commands.deadline(
                Commands.waitSeconds(timeout), // testing - check wait time             
                Commands.waitUntil(this::noteIntook)
            ),
            
            // Move note back
            indexer.reverseIndexCommand(),
            shooterRoller.setVelocityCommand(-10, -10),
            Commands.waitSeconds(0.2), // Was 0.6   3/3/24   Code Orange

            intakeRoller.stopCommand(),
            indexer.stopCommand(),
            shooterRoller.stopCommand()
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command intakeUntilSensedNoBackup() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
            ),
            shooterRoller.setVelocityCommand(-10, -10),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

            Commands.waitUntil(this::noteIntook),
            intakeRoller.stopCommand(),
            indexer.stopCommand(),
            shooterRoller.stopCommand()
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command intakeUntilSensedAuto(double timeout) {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
            ),
            shooterRoller.setVelocityCommand(-10, -10),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

            Commands.deadline(
                Commands.waitSeconds(timeout), // testing - check wait time             
                Commands.waitUntil(this::noteIntook)
            )
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command intakeUntilSensedAutoShoot(double noNoteTimeout) {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
            ),

            shooterRoller.setVelocityCommand(-10, -10),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),

             Commands.deadline(
                 Commands.waitSeconds(noNoteTimeout), // testing - check wait time             
                 Commands.waitUntil(this::noteIntook)
             )
        ).finallyDo(() -> {
            intakeRoller.stop();
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }
    
    // public Command climbSequence(){
    //     return shooterPivot.climbSequence();
    // }

    public Command intakeBasic() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
                ),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.intakeCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command intakeBasicHold() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(ShooterConstants.kHandoffPosition.get())),
                handoff(),
                Commands.waitSeconds(1)
                ),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.intakeCommand()
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command stopIntaking() {
        Command command = Commands.sequence(
            indexer.reverseIndexCommand(),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
            })
        );

        command.addRequirements(indexer, intakeRoller);
        return command;
    }


    public Command intakeDirectShoot() {
        return intakeDirectShoot(ShooterConstants.kHandoffPosition.get(), 
                                 ShooterConstants.kTopOuttakeAuto1.get(),
                                 ShooterConstants.kBottomOuttakeAuto1.get());
    }

    public Command intakeDirectShootAutoStart() {
        return intakeDirectShoot(ShooterConstants.kHandoffPosition.get(), 
                                 30,
                                 30);
    }

    public Command intakeDirectShoot(double shooterPosition, double topShooterVelocity, double bottomShooterVelocity) {
        Command command = Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Moving Shooter", true)),
            Commands.deadline(
                Commands.waitUntil(() -> 
                    shooterPivot.hasReachedPosition(shooterPosition)),
                shooterPivot.setPositionCommand(shooterPosition)
                // prepareShooterVision()
                ),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Moving Shooter", false)),
            shooterRoller.setEnabledCommand(true),
            intakeRoller.setEnabledCommand(true),
            indexer.setEnabledCommand(true),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intaking", true)),
            indexer.indexCommand(),
            intakeRoller.autoIntakeCommand(),
            shooterRoller.setVelocityCommand(topShooterVelocity, bottomShooterVelocity),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intaking", false);
                intakeRoller.stop();
                indexer.stop();
                shooterRoller.setVelocity(0, 0);
            }
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command intakeDirectShoot(double shooterPosition, double shooterVelocity) {
        return intakeDirectShoot(shooterPosition, shooterVelocity, shooterVelocity);
    }

    public Command eject() {
        Command command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> 
                shooterPivot.hasReachedPosition(ShooterConstants.kEjectPosition.get())),
                shooterPivot.setPositionCommand(ShooterConstants.kEjectPosition.get()),
                Commands.waitSeconds(0.5)
            ),
            intakeRoller.setEnabledCommand(true),
            intakeRoller.setVelocityCommand(-100),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Outtaking", true)),
            Commands.waitSeconds(0.25),
            indexer.setEnabledCommand(true),
            indexer.setVelocityCommand(-50),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intake roller", true)),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intake roller", false);
                SmartDashboard.putBoolean("Outtaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command panicEject() {
        Command command = Commands.sequence(
            shooterPivot.setPositionCommand(ShooterConstants.kFullStowPosition.get() + 2),
            intakeRoller.setEnabledCommand(true),
            intakeRoller.setVelocityCommand(-50),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Outtaking", true)),
            indexer.setEnabledCommand(true),
            indexer.setVelocityCommand(-50, 50),
            Commands.runOnce(() -> SmartDashboard.putBoolean("Intake roller", true)),
            Commands.waitUntil(() -> false)
        ).finallyDo(
            () -> {
                SmartDashboard.putBoolean("Intake roller", false);
                SmartDashboard.putBoolean("Outtaking", false);
                intakeRoller.stop();
                indexer.stop();
            }
        );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command ejectIntakeOnly() {
        Command command = Commands.sequence(
            intakeRoller.setEnabledCommand(true),
            intakeRoller.setVelocityCommand(-100)
        ).finallyDo(
            () -> {
                intakeRoller.stop();
            }
        );

        command.addRequirements(intakeRoller);
        return command;
    }

    public Command prepareShooter() {
        Command command = Commands.sequence(
            shooterPivot.moveToSpeaker(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker()
        );
        command.addRequirements(shooterPivot, shooterRoller, indexer);
        return command;
    }

    public Command prepareShooterVision(SwerveDrivetrain swerve) {
        Command command = Commands.sequence(
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker(),
            Commands.repeatingSequence(
                Commands.runOnce(() -> shooterPivot.setPosition(getShooterAngle(swerve, true))),
                Commands.waitSeconds(0.02)
            )
        );

        command.addRequirements(shooterPivot, shooterRoller);

        return command;
    }

    public Command prepareShooterRollersOnly() {
        Command command = Commands.sequence(
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker()        
        );

        command.addRequirements(shooterRoller);
        return command;
    }

    public Command prepareShooterSpeaker() {
        Command command = Commands.parallel(
            shooterPivot.moveToSpeaker(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker()
        );
        command.addRequirements(shooterPivot, shooterRoller, indexer);
        return command;
    }

    public Command prepareShooterPodium() {
        Command command = Commands.parallel(
            shooterPivot.moveToSpeakerFar(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker()
        );
        command.addRequirements(shooterPivot, shooterRoller, indexer);
        return command;
    }
    

    private boolean isPassing = false;

    public boolean getIsPassing() {
        return isPassing;
    }

    public Command shoot() {
        Command command = Commands.either(
            // Pass
            Commands.sequence(
                Commands.runOnce(() -> isPassing = true),
                shooterPivot.setPositionCommand(ShooterConstants.kSpeakerPosition.get()),
                shooterRoller.setEnabledCommand(true),
                shooterRoller.setVelocityCommand(37),
                Commands.waitSeconds(0.3),
                indexer.setEnabledCommand(true),
                indexer.indexCommand()
            ),
            // Shoot
            Commands.sequence(
                Commands.runOnce(() -> isPassing = false),
                indexer.setEnabledCommand(true),
                indexer.indexCommand()
            ),
            () -> shooterRoller.getTargetVelocity() == 0
        );

        command.addRequirements(indexer);

        return command;
    }

    public Command shootAuto() {
        Command command =
            Commands.sequence(
                indexer.setEnabledCommand(true),
                indexer.indexCommand()
            );

        command.addRequirements(indexer);

        return command;
    }

    

    public Command shootSubwoofer() {
        Command command = Commands.sequence(
            // Prepare to shoot
            shooterPivot.moveToSpeaker(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker(),
            Commands.waitSeconds(0.45), // Was 0.2     3/3/24     But 0.8   @Code Orange
            
            // Shoot
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(interrupted -> {
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command shootSubwooferAutoStart() {
        Command command = Commands.sequence(
            // Prepare to shoot
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeakerAutoStart(),
            Commands.deadline(
                Commands.waitSeconds(0.45), // Was 0.2     3/3/24     But 0.8   @Code Orange
                
                Commands.deadline(
                    Commands.waitUntil(() -> 
                        shooterPivot.hasReachedPosition(ShooterConstants.kSpeakerPositionAutoStart.get())),
                    shooterPivot.moveToSpeakerAutoStart()
                )
            ),
            // Shoot
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(interrupted -> {
            indexer.stop(); // keep it running for next pickup
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    } 

    public Command shootSubwooferAutoStart2() {
        Command command = Commands.sequence(
            // Prepare to shoot
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeakerAutoStart(),
            Commands.deadline(
                Commands.waitSeconds(0.32), // Was 0.2     3/3/24     But 0.8   @Code Orange
                
                Commands.deadline(
                    Commands.waitUntil(() -> 
                        shooterPivot.hasReachedPosition(ShooterConstants.kSpeakerPositionAutoStart2.get())),
                    shooterPivot.moveToSpeakerAutoStart2()
                )
            ),
            // Shoot
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(interrupted -> {
            indexer.stop(); // keep it running for next pickup
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    } 

    public Command shootSubwooferAuto() {
        Command command = Commands.sequence(
            // Prepare to shoot
            shooterPivot.moveToSpeakerAuto(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker(),
            Commands.deadline(
                Commands.waitSeconds(0.6),
                Commands.waitUntil(shooterRoller::atTargetVelocity)
            ),
            
            // Shoot
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(interrupted -> {
            indexer.stop();
            shooterRoller.stop();
        });

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command shootSequenceAdjustable(SwerveDrivetrain swerve) {
        Command command = 
            // Commands.either(
            Commands.sequence(
                // Prepare to shoot
                Commands.runOnce(() -> shooterPivot.setPosition(getShooterAngle(swerve))),
                shooterRoller.setEnabledCommand(true),
                shooterRoller.shootSpeaker(),
                Commands.race(
                    Commands.waitUntil(() -> shooterPivot.atTargetPositionAccurate()),
                    Commands.waitSeconds(0.8)
                )
                // ,
                
                // // Shoot
                // indexer.setEnabledCommand(true),
                // indexer.indexCommand(),
                // Commands.waitUntil(() -> false)
                // ).finallyDo(interrupted -> {
                //     indexer.stop();
                //     shooterRoller.stop();
                // }),
                // Commands.none(),
                // sva::hasValidTarget
            );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command shootSequenceAdjustableAuto(SwerveDrivetrain swerve) {
        Command command = 
            //Commands.either(
            Commands.sequence(
                    // Prepare to shoot
                    Commands.runOnce(() -> shooterPivot.setPosition(getShooterAngle(swerve))),
                    shooterRoller.setEnabledCommand(true),
                    shooterRoller.shootSpeaker(),
                    Commands.race(
                        Commands.sequence(
                            Commands.waitUntil(() -> shooterPivot.atTargetPosition()),
                            Commands.waitSeconds(0.2)
                        ),
                        Commands.waitSeconds(0.4)
                    )
                //     ,
                    
                //     // Shoot
                //     indexer.setEnabledCommand(true),
                //     indexer.indexCommand(),
                //     Commands.waitUntil(() -> false)
                // ).finallyDo(interrupted -> {
                //     indexer.stop();
                //     shooterRoller.stop();
                // }),
                // Commands.none(),
                // sva::hasValidTarget
            );

        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command turnAndShootWithVision(frc.robot.subsystems.swerve.SwerveDrivetrain swerve) {
        return Commands.sequence(
            Commands.race(
                swerve.turnToTag(RobotContainer.IsRedSide() ? 4 : 7),
                Commands.waitSeconds(2)
            ),
            Commands.runOnce(swerve::towModules),
            shootSequenceAdjustable(swerve)
        );
    }

    public Command shootPodium() {
        Command command = Commands.sequence(
            // Prepare to shoot
            shooterPivot.moveToSpeakerFar(),
            shooterRoller.setEnabledCommand(true),
            shooterRoller.shootSpeaker(),
            Commands.waitSeconds(0.8),
            
            // Shoot
            indexer.setEnabledCommand(true),
            indexer.indexCommand(),
            Commands.waitUntil(() -> false)
        ).finallyDo(interrupted -> {
            indexer.stop();
            shooterRoller.stop();
        });
        command.addRequirements(shooterPivot, shooterRoller, indexer, intakeRoller);
        return command;
    }

    public Command climbSequence(){
        Command command = Commands.sequence(
            Commands.runOnce(() -> shooterPivot.configureClimb()),
            shooterPivot.setPositionCommand(ShooterConstants.kPrepClimbPosition.get()),
            Commands.waitSeconds(1),
            shooterPivot.setPositionCommand(ShooterConstants.kFullStowPosition.get()),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> shooterPivot.setBreakMode(true)),
            Commands.waitUntil(() -> false)
        ).finallyDo(() -> {
            shooterPivot.stop();
        });

        command.addRequirements(shooterPivot);

        return command;
    }

}
