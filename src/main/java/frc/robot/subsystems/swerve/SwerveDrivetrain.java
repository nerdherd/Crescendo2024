package frc.robot.subsystems.swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.commands.TurnToAngleLive;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers.PoseEstimate;
import frc.robot.util.NerdyLine;
import frc.robot.util.NerdyMath;
import frc.robot.subsystems.Reportable;

import static frc.robot.Constants.PathPlannerConstants.kPPMaxVelocity;
import static frc.robot.Constants.PathPlannerConstants.kPPRotationPIDConstants;
import static frc.robot.Constants.PathPlannerConstants.kPPTranslationPIDConstants;
import static frc.robot.Constants.SwerveDriveConstants.*;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveDrivetrain extends SubsystemBase implements Reportable {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Gyro gyro;
    // private final SwerveDriveOdometry odometer;
    private boolean isTest = false;
    private final SwerveDrivePoseEstimator poseEstimator;
    private DRIVE_MODE driveMode = DRIVE_MODE.FIELD_ORIENTED;

    //Vision
    private int counter = 0;
    private int visionFrequency = 1;
    private AprilTagFieldLayout layout;
    private double lastDistance;
    private double[] distances = new double[] {0, 1, 1.72, 2, 3, 3.5, 4, 5};
    private double[] tolerances = new double[] {12, 11, 10, 9, 5, 4, 2, 2};
    private NerdyLine toleranceSpline = new NerdyLine(distances, tolerances);
    private double[] angles          = new double[] {0, 10, 30, 45, 90};
    private double[] toleranceScales = new double[] {1, 0.95, 0.75, 0.4, 0};
    private NerdyLine angleToleranceSpline = new NerdyLine(angles, toleranceScales);
    
    private Field2d field;

    public enum DRIVE_MODE {
        FIELD_ORIENTED,
        ROBOT_ORIENTED,
        AUTONOMOUS
    }

    /**
     * Construct a new {@link SwerveDrivetrain}
     */
    public SwerveDrivetrain(Gyro gyro) throws IllegalArgumentException {
        frontLeft = new SwerveModule(
            kFLDriveID,
            kFLTurningID,
            kFLDriveReversed,
            kFLTurningReversed,
            CANCoderConstants.kFLCANCoderID,
            CANCoderConstants.kFLCANCoderReversed);
        frontRight = new SwerveModule(
            kFRDriveID,
            kFRTurningID,
            kFRDriveReversed,
            kFRTurningReversed,
            CANCoderConstants.kFRCANCoderID,
            CANCoderConstants.kFRCANCoderReversed);
        backLeft = new SwerveModule(
            kBLDriveID,
            kBLTurningID,
            kBLDriveReversed,
            kBLTurningReversed,
            CANCoderConstants.kBLCANCoderID,
            CANCoderConstants.kBLCANCoderReversed);
        backRight = new SwerveModule(
            kBRDriveID,
            kBRTurningID,
            kBRDriveReversed,
            kBRTurningReversed,
            CANCoderConstants.kBRCANCoderID,
            CANCoderConstants.kBRCANCoderReversed);

        this.gyro = gyro;

        /** @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
         *     in meters, and heading in radians). Increase these numbers to trust your state estimate
         *     less.
         * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
         *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
         *     the vision pose measurement less.
        */
        this.poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // TODO: Set pose estimator weights
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); 
        
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();  

        field = new Field2d();
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry, 
            this::getChassisSpeeds, 
            this::setChassisSpeeds, 
            new HolonomicPathFollowerConfig(
                kPPTranslationPIDConstants, 
                kPPRotationPIDConstants, 
                kPPMaxVelocity,
                kTrackWidth,
                new ReplanningConfig()), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);

            enableVisionPE = true;
    }

    private void visionupdateOdometry(String limelightName) {
        boolean doRejectUpdate = false;

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        double xyStds = 0.5;
        double degStds = 999999;

        boolean receivedValidData = LimelightHelpers.getTV(limelightName);
        Pose3d botPose1 = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
        if(!receivedValidData)
            doRejectUpdate = true;
        else if(botPose1.getZ() > 0.3 || botPose1.getZ() < -0.3)
            doRejectUpdate = true;
        else if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
                doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
                doRejectUpdate = true;
            }

            
            // 1 target with large area and close to estimated pose
            if (mt1.avgTagArea > 0.8 && mt1.rawFiducials[0].distToCamera < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (mt1.avgTagArea > 0.1 && mt1.rawFiducials[0].distToCamera < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
        }
        else if (mt1.tagCount >= 2) {
            xyStds = 0.5;
            degStds = 6;
        }

        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(
              VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

            //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            poseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }
    }

    private boolean enableVisionPE;
    public void enableVisionPoseEstm(Boolean enable)
    {
        enableVisionPE = enable;
    }


    boolean initPoseByVisionDone = false;

    /**
     * Have modules move towards states and update odometry
     */
    @Override
    public void periodic() {
        if (!isTest) {
            runModules();
        }
        
        poseEstimator.update(gyro.getRotation2d(), getModulePositions());

        if (enableVisionPE)
        {
            visionupdateOdometry(VisionConstants.kLimelightBackName);
            //visionupdateOdometry(VisionConstants.kLimelighLeftName);
            //visionupdateOdometry(VisionConstants.kLimelightRightName);
        }

        // counter = (counter + 1) % visionFrequency;

        // if(vision != null && vision.getAprilTagID() != -1)
        // {
        //     if(vision.getTA() > 0.5) {
        //         PoseEstimate visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightBackName);
        //         poseEstimator.addVisionMeasurement(visionPoseEstimate.pose, visionPoseEstimate.timestampSeconds);
        //     }
            
        // }
        // else
        // {
        //     SmartDashboard.putBoolean("Vision Used", false);
        // }
        
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
    
    //****************************** RESETTERS ******************************/

    public void updatePoseEstimatorWithVisionBotPose() {
        PoseEstimate visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightBackName);
        // invalid LL data
        if (visionPoseEstimate.pose.getX() == 0.0) {
          return;
        }
    
        // distance from current pose to vision estimated pose
        double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
            .getDistance(visionPoseEstimate.pose.getTranslation());
    
        if (visionPoseEstimate.tagCount > 0) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (visionPoseEstimate.tagCount >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (visionPoseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (visionPoseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }
        
            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            poseEstimator.addVisionMeasurement(visionPoseEstimate.pose,
                Timer.getFPGATimestamp() - visionPoseEstimate.latency);
        }
    }

    /**
     * Resets the odometry to given pose 
     * @param pose  A Pose2D representing the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometryWithAlliance(Pose2d pose){
        if (RobotContainer.IsRedSide()) {
            resetOdometry(GeometryUtil.flipFieldPose(pose));
        } else {
            resetOdometry(pose);
        }
    }

    public void zeroGyroAndPoseAngle() {
        gyro.zeroHeading();
        gyro.setOffset(0);
        Pose2d pose = getPose();
        Pose2d newPose = new Pose2d(pose.getX(), pose.getY(), RobotContainer.IsRedSide() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), newPose);
    }

    public void resetGyroFromPoseWithAlliance(Pose2d pose) {
        if (RobotContainer.IsRedSide()) {
            double angle = GeometryUtil.flipFieldPose(pose).getRotation().getDegrees() - 180;
            angle = NerdyMath.posMod(angle, 360);
            gyro.resetHeading(angle);
        } else {
            gyro.resetHeading(NerdyMath.posMod(pose.getRotation().getDegrees(), 360));
        }
    }

    public void refreshModulePID() {
        frontLeft.refreshPID();
        backLeft.refreshPID();
        frontRight.refreshPID();  
        backRight.refreshPID();
    }

    /**
     * Stops all modules. See {@link SwerveModule#stop()} for more info.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Have modules move to their desired states. See {@link SwerveModule#run()} for more info.
     */
    public void runModules() {
        frontLeft.run();
        frontRight.run();
        backLeft.run();
        backRight.run();
    }

    //****************************** GETTERS ******************************/

    public Gyro getImu() {
        return this.gyro;
    }

    /**
     * Gets a pose2d representing the position of the drivetrain
     * @return A pose2d representing the position of the drivetrain
     */
    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getTagPose2D(int tagID)
    {
        return getTagPose3D(tagID).toPose2d();
    }

    public Pose3d getTagPose3D(int tagID)
    {
        Optional<Pose3d> tagPose = layout.getTagPose(tagID);
        if(tagPose.isEmpty()) return null;
        return tagPose.get();
    }

    public double getDistanceFromTag(boolean preserveOldValue, int tagID)
    {
        Pose2d tagPose = getTagPose2D(tagID);
        if(tagPose == null) return (preserveOldValue ? lastDistance : 0.01);

        Pose2d robotPose = getPose();
        lastDistance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
        // lastDistance = Math.sqrt(Math.pow(robotPose.getX()-tagPose.getX(), 2) + Math.pow(robotPose.getY()-tagPose.getY(), 2));

        return lastDistance;
    }
    public double getSpeakerTurnToAngleTolerance ()
    {
        double distance = getDistanceFromTag(true, RobotContainer.IsRedSide() ? 4 : 7);
        if(distance > 5) {
            return 0;
        }
        if(distance > 4) {
            return 1;
        }
        double tolerance = toleranceSpline.getOutput(distance);
        if(tolerance < 0) {
            return 0;
        }
        return tolerance;
    }

    public double getTurnToSpecificTagAngle(int tagID)
    {
        Pose2d tagPose = getTagPose2D(tagID);
        Pose2d robotPose = getPose();
        double xOffset = tagPose.getX() - robotPose.getX();
        double yOffset = tagPose.getY() - robotPose.getY();

        double allianceOffset = 90;
        double angle = NerdyMath.posMod(-Math.toDegrees(Math.atan2(xOffset, yOffset)) + allianceOffset, 360);
        if(RobotContainer.IsRedSide()) {
            return angle; //TODO: test if works since this is a bit different than original code
        }
        return (180 + angle) % 360;
    }

    public double getTurnToAngleToleranceScale(double targetAngle)
    {
        double angleToSpeaker = 10000;
        targetAngle = NerdyMath.posMod(targetAngle, 360);
        if (targetAngle > 180) {
            angleToSpeaker = Math.abs(360 - targetAngle);
        }
        else if (targetAngle < 180) {
            angleToSpeaker = targetAngle;
        }
        return angleToleranceSpline.getOutput(angleToSpeaker);
    }

    public Command driveToAmpCommand( double maxVelocityMps, double maxAccelerationMpsSq)
    {
        Pose2d targetPose = VisionConstants.kBlueAmpPose;
        return Commands.sequence(
            driveToPose(targetPose, maxVelocityMps, maxAccelerationMpsSq) //TODO: verify if works on red side
        );
    }

    public Command turnToTag(int tagID)
    {
        return Commands.sequence(
            new TurnToAngleLive(() -> getTurnToSpecificTagAngle(tagID), this, 1)
        );
    }
    public Command turnToTag(int tagID, double angleTolerance)
    {
        return Commands.sequence(
            new TurnToAngleLive(() -> getTurnToSpecificTagAngle(tagID), this, angleTolerance)
        );
    }
    /**
     * Get the position of each swerve module
     * @return An array of swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(), 
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    private ChassisSpeeds getChassisSpeeds() {
        return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, turnSpeed)
            )
        );
    }

    public void drive(double xSpeed, double ySpeed) {
        drive(xSpeed, ySpeed, 0);
    }

    public void driveFieldOriented(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, gyro.getRotation2d())
            )
        );
    }

    public void driveFieldOriented(double xSpeed, double ySpeed) {
        driveFieldOriented(xSpeed, ySpeed, 0);
    }

    public Command driveToPose(Pose2d destPoseInBlue, double maxVelocityMps, double maxAccelerationMpsSq) {
        PathConstraints pathcons = new PathConstraints(
            maxVelocityMps, maxAccelerationMpsSq, 
            Units.degreesToRadians(180), Units.degreesToRadians(360)
        );
        return Commands.either(
            AutoBuilder.pathfindToPose(GeometryUtil.flipFieldPose(destPoseInBlue), pathcons),
            AutoBuilder.pathfindToPose(destPoseInBlue, pathcons),
            RobotContainer::IsRedSide  
        );
    }
    
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(targetStates);
    }


    //****************************** SETTERS ******************************/

    /**
     * Set the drive mode (only for telemetry purposes)
     * @param driveMode
     */
    public void setDriveMode(DRIVE_MODE driveMode) {
        this.driveMode = driveMode;
    }

    public void setVelocityControl(boolean withVelocityControl) {
        frontLeft.toggleVelocityControl(withVelocityControl);
        frontRight.toggleVelocityControl(withVelocityControl);
        backLeft.toggleVelocityControl(withVelocityControl);
        backRight.toggleVelocityControl(withVelocityControl);
    }

    /**
     * Set the neutral modes of all modules.
     * <p>
     * true sets break mode, false sets coast mode
     * 
     * @param breaking  Whether or not the modules should be in break
     */
    public void setBreak(boolean breaking) {
        frontLeft.setBreak(breaking);
        frontRight.setBreak(breaking);
        backLeft.setBreak(breaking);
        backRight.setBreak(breaking);
    }

    /**
     * Sets module desired states
     * @param desiredStates desired states of the four modules (FL, FR, BL, BR)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void towModules() {
        frontLeft.setDesiredState(towModuleStates[0], false);
        frontRight.setDesiredState(towModuleStates[1], false);
        backLeft.setDesiredState(towModuleStates[2], false);
        backRight.setDesiredState(towModuleStates[3], false);
    }

    public Command towCommand() {
        return Commands.runOnce(this::towModules, this);
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Swerve");
        }

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.add("Field Position", field).withSize(6, 3);
                tab.addString(("Current Command"), () -> {
                    Command currCommand = this.getCurrentCommand();
                    if (currCommand == null) {
                        return "null";
                    } else {
                        return currCommand.getName();
                    }
                }
                );
                tab.add("Toggle Test", Commands.runOnce(() -> isTest = !isTest));
                tab.addBoolean("Test Mode", () -> isTest);
                // Might be negative because our swerveDriveKinematics is flipped across the Y axis
            case MEDIUM:
            case MINIMAL:
                tab.addNumber("X Position (m)", () -> poseEstimator.getEstimatedPosition().getX());
                tab.addNumber("Y Position (m)", () -> poseEstimator.getEstimatedPosition().getY());
                tab.addNumber("Odometry Angle", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
                tab.addString("Drive Mode", () -> this.driveMode.toString());
                break;
        }
    }

    public void initModuleShuffleboard(LOG_LEVEL level) {
        frontRight.initShuffleboard(level);
        frontLeft.initShuffleboard(level);
        backLeft.initShuffleboard(level);
        backRight.initShuffleboard(level);
    }

    /**
     * Report values to smartdashboard.
     */
     public void reportToSmartDashboard(LOG_LEVEL level) {
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //         case MEDIUM:
    //             SmartDashboard.putData("Zero Modules", Commands.runOnce(this::zeroModules));
    //         case MINIMAL:
    //             SmartDashboard.putNumber("Odometer X Meters", poseEstimator.getEstimatedPosition().getX());
    //             SmartDashboard.putNumber("Odometer Y Meters", poseEstimator.getEstimatedPosition().getY());
    //             SmartDashboard.putString("Drive Mode", this.driveMode.toString());
    //             break;
    //     }
     }

    // public void reportModulesToSmartDashboard(LOG_LEVEL level) {
    //     frontRight.reportToSmartDashboard(level);
    //     frontLeft.reportToSmartDashboard(level);
    //     backLeft.reportToSmartDashboard(level);
    //     backRight.reportToSmartDashboard(level);
    // }
}