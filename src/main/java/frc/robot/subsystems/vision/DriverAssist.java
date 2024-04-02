package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleLive;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers.Results;
import frc.robot.util.NerdyMath;
import frc.robot.util.NerdySpline;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Subsystem that uses Limelight for vision
 */
public class DriverAssist implements Reportable{
    private Limelight limelight;
    private String limelightName;

    private AprilTagFieldLayout layout;
    private int pipeline;
    private boolean lightsON;

    private GenericEntry targetId;
    private GenericEntry currentTaOffset;
    private GenericEntry currentTxOffset;
    private GenericEntry currentAngleOffset;
    private GenericEntry forwardSpeed;
    private GenericEntry sidewaysSpeed;
    private GenericEntry angularSpeed;

    private GenericEntry botPoseByVisionX;
    private GenericEntry botPoseByVisionY;
    private GenericEntry botPoseByVisionR;
    
    /**
     * Makes a new EMPeach to utilize vision
     * @param name name of the limelight
     */
    public DriverAssist(String name, int pipeline) {
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        limelightName = name;
        ShuffleboardTab tab = Shuffleboard.getTab(limelightName);
        toleranceSpline.create();
        angleToleranceSpline.create();
        try {
            limelight = new Limelight(limelightName);
            toggleLight(false);
            changePipeline(pipeline);

            tab.add(limelightName + " inited ", true);
        } catch (Exception e) {
            tab.add(limelightName + " inited ", false);
        }
        
    }

    public Limelight getLimelight() { return limelight; }

    public void reset()
    {
        limelight.reinitBuffer();
        dataSampleCount = 0;
    }

    private double[] distances = new double[] {0, 1, 1.72, 2, 3, 3.5, 4, 5};
    private double[] tolerances = new double[] {10, 9, 8, 7, 4, 3, 1, 1};
    private NerdySpline toleranceSpline = new NerdySpline(distances, tolerances);

    public double getTurnToAngleTolerance(double distance) {
        if (distance > 5) {
            return 0;
        }
        if (distance > 4) {
            return 1;
        }
        double tolerance = toleranceSpline.getOutput(distance);
        if (tolerance < 0) {
            return 0;
        }
        return tolerance;
    }

    private double[] angles          = new double[] {0, 10, 30, 45, 90};
    private double[] toleranceScales = new double[] {1, 0.95, 0.75, 0.4, 0};
    private NerdySpline angleToleranceSpline = new NerdySpline(angles, toleranceScales);

    public double getTurnToAngleToleranceScale(double targetAngle) {
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

    /**
     * 
     * @param ID april tag ID to turn to
     * @return the angle in degrees to get to the april tag
     */
    public double getTurnToSpecificTagAngle(int ID) {
        if(ID < 1 || ID > 16) return 1000;
        if(!limelight.hasValidTarget()) return 1000;

        Optional<Pose3d> tagPoseOptional = layout.getTagPose(ID);
        if(tagPoseOptional.isEmpty()) return 1000;
        Pose3d tagPose = tagPoseOptional.get();

        Pose3d robotPose = getCurrentPose3DVision();

        double xOffset = tagPose.getX() - robotPose.getX();
        double yOffset = tagPose.getY() - robotPose.getY();

        double allianceOffset = 90;
        double angle = NerdyMath.posMod(-Math.toDegrees(Math.atan2(xOffset, yOffset)) + allianceOffset, 360);
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            return Rotation2d.fromDegrees(angle).getDegrees();
        }
        return (180 + angle) % 360;
    }

    public Command turnToTag(int ID, SwerveDrivetrain swerve) {
        return Commands.sequence(
            new TurnToAngleLive(() -> getTurnToSpecificTagAngle(ID), swerve)
        );
    }

    public void TagDriving(SwerveDrivetrain swerveDrive, double targetTA, double targetTX, double targetSkew, int tagID, int maxSamples) {
        calculateTagDriving(targetTA, targetTX, targetSkew, tagID, maxSamples);

        swerveDrive.drive(getForwardPower(), getSidewaysPower(), getAngledPower()); //TODO: //getSidewaysPower(), getAngledPower());
    }
    public Command driveToApriltagCommand(SwerveDrivetrain drivetrain, double targetTA, double targetTX, double targetSkew, int tagID, int minSamples, int maxSamples) {
        return Commands.sequence(      
        Commands.runOnce(() -> reset()),
            Commands.run(
                () -> TagDriving(drivetrain, targetTA, targetTX, targetSkew, tagID, maxSamples)
            ).until(() -> (dataSampleCount >= minSamples && Math.abs(getForwardPower()) <= 0.1 && Math.abs(getSidewaysPower()) <= 0.1 && Math.abs(getAngledPower()) <= 0.1))
        );
    }

    Pose2d blueAmpPose = new Pose2d(new Translation2d(1.85, 7.5), Rotation2d.fromDegrees(270));
    public Command driveToAmpCommand(SwerveDrivetrain drivetrain, double maxVelocityMps, double maxAccelerationMpsSq) {
        return Commands.sequence(
            Commands.runOnce(() -> resetOdoPoseByVision(drivetrain, 0, 10)),
            drivetrain.driveToPose(blueAmpPose, maxVelocityMps, maxAccelerationMpsSq)
        );
    }


    private final double turnToAngleScale = 1;

    public double getTurnToTagPower(SwerveDrivetrain swerveDrive, double targetTX, int tagID, ShooterVisionAdjustment sva) {
        calculateTagTurning(targetTX, tagID);

        double distance = sva.getDistanceFromTag(true);
        if (distance < 0.01) {
            return 0;
        }

        // TODO: Tune getAngledPower to allow for higher range of speeds *TESTING NEEDED*
        double power = getAngledPower();
        power = turnToAngleScale * power / distance;

        return power;
    }

    double lastTX = 0;

    public boolean apriltagInRange(int tagID, double targetTX, double tolerance) {
        int foundId = -1;

        foundId = getAprilTagID();
        if(tagID == foundId) {
            lastTX = targetTX - limelight.getXAngle_avg();
        }

        return Math.abs(lastTX) < tolerance;
    }

    public void TurnToTag(SwerveDrivetrain swerveDrive, double targetTX, int tagID, ShooterVisionAdjustment sva) {
        swerveDrive.drive(0, 0, getTurnToTagPower(swerveDrive, targetTX, tagID, sva)); //TODO: What should Sideways and Angled Speed be based on? Pose? TX?
    }

    public Command TurnToTagCommand(SwerveDrivetrain drivetrain, double targetSkew, int tagID, ShooterVisionAdjustment sva) {
        return Commands.sequence(
            Commands.run(
                () -> TurnToTag(drivetrain, targetSkew, tagID, sva)
            ).until(() -> Math.abs(Math.abs(getAngledPower())) <= 0.05)
        );
    }

    public int convertTagToAlliance(int tagID) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
            switch (tagID) {
                case 1: case 2: case 6: case 7: case 8:
                    return 11 - tagID;
                // 14 -> 13, 15 -> 12, 16 -> 11
                case 14: case 15: case 16:
                    return 27 - tagID;
                default:
                    return tagID;
            }
        } else {
            switch (tagID) {
                case 3: case 4: case 5: case 9: case 10:
                    return 11 - tagID;
                // 14 <- 13, 15 <- 12, 16 <- 11
                case 11: case 12: case 13:
                    return 27 - tagID;
                default:
                    return tagID;
            }
        }
    }

    private void setRobotPoseByApriltag(SwerveDrivetrain drivetrain, int tagID, boolean resetToCurrentPose)
    {
        if(resetToCurrentPose)
        {
            dataSampleCount++;
            if(limelight.getAprilTagID() == tagID)
            {
                Pose3d p = getCurrentPose3DVision();
                drivetrain.resetOdometry(p.toPose2d());
                drivetrain.getImu().setOffset(p.getRotation().getZ());
                return;
            }
        }
        else
        {
            dataSampleCount = 10000; // make it very big to exit
        }
    }

    int dataSampleCount = 0;
    public Command aimToApriltagCommand(SwerveDrivetrain drivetrain, int tagID, int minSamples, int maxSamples) {
        Command command = Commands.sequence(
            Commands.runOnce(() -> reset()),
            Commands.run(
                () -> TagAimingRotation(drivetrain, tagID, maxSamples)
            ).until(() -> dataSampleCount >= minSamples && Math.abs(calculatedAngledPower) <= 0.1)

            //Commands.runOnce(() -> reset())
            // ,
            // resetOdoPoseByVision(drivetrain, tagID, maxSamples)
        );
        command.addRequirements(drivetrain);

        return command;
    }



    final double MaxTA = 5;
    final double MinTA = 1;
    final double MaxTxIn = 10;
    final double MinTxIn = 2;
    private double TxTargetOffsetForCurrentTa(double currentTa )
    {
        // make sure to return positive value
        return ((MaxTxIn-MinTxIn)/(MaxTA-MinTA))*(currentTa-MinTA) + MinTxIn;
    }
    PIDController pidTxRotation = new PIDController(0.1, 0, 0); // todo, tuning pls.

    private double lastAnglePower = 0;

    public void TagAimingRotation(SwerveDrivetrain swerveDrive, int tagID, int maxSamples) {
        // make sure to reset before or after calling this function
        // make sure the cross is at the center!!! for tx
        // tagID == 0 means: don't care of tag's id, be careful for multi-tags location
        double taOffset;
        double txOffset;

        dataSampleCount++;
        SmartDashboard.putNumber("dataSampleCount", dataSampleCount);
        if(maxSamples > 0 && dataSampleCount >= maxSamples)
        {
            calculatedAngledPower = 0;
            lastAnglePower = 0;
        }
        else
        {
            int foundId = getAprilTagID();
            if(targetId != null)
                targetId.setInteger(foundId);

            if(tagID == foundId || (tagID == 0 && foundId != -1)) {

                lastTagSeenId = foundId;
                
                SmartDashboard.putNumber("lastTagSeenId", lastTagSeenId);
                LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                lastTagSeenDist = limelightMeasurement.avgTagDist;

                lastTagSeenPose2d = limelightMeasurement.pose;
                //double currentAngle = lastTagSeenPose2d.getRotation().getDegrees();
                //SmartDashboard.putNumber("currentAngle", currentAngle);
                
                //Results a = LimelightHelpers.getLatestResults(limelightName).targetingResults;
                double currentAngle;// = a.getBotPose2d().getRotation().getDegrees();
                currentAngle = LimelightHelpers.getBotPose3d_TargetSpace(limelightName).toPose2d().getRotation().getDegrees();
                SmartDashboard.putNumber("currentAngle", currentAngle);

                taOffset = limelight.getArea_avg();
                txOffset = limelight.getXAngle_avg();
                    
                if(currentTaOffset != null)
                    currentTaOffset.setDouble(taOffset);
                if(currentTxOffset != null)
                    currentTxOffset.setDouble(txOffset);
                if(currentAngleOffset != null)
                    currentAngleOffset.setDouble(0);

                // double txInRangeValue = Math.abs(TxTargetOffsetForCurrentTa(taOffset));// todo, tuning pls!!!
                // if( txOffset < txInRangeValue && txOffset > -1*txInRangeValue ) // todo, tuning pls!!!
                // {
                //     calculatedAngledPower = 0; // in good tx ranges. faster than the pid
                // } 
                // // else if(){} // out of range cases. todo 
                // else
                {
                    // pid based on tx, and adding ta/distance as the factor
                    //calculatedAngledPower = pidTxRotation.calculate(txOffset, 0)  * Math.sqrt(taOffset);
                    
                    calculatedAngledPower = 0;//pidTxRotation.calculate(currentAngle * -1, 0);//  * Math.sqrt(taOffset);
                    calculatedAngledPower = NerdyMath.deadband(calculatedAngledPower, -0.2, 0.2); // todo, tuning pls. Have to consider the Ta for all coeffs!!! Todo
                }

                lastAnglePower = calculatedAngledPower;

                if(forwardSpeed != null)
                    forwardSpeed.setDouble(0);
                if(sidewaysSpeed != null)
                    sidewaysSpeed.setDouble(0);
                if(angularSpeed != null)
                    angularSpeed.setDouble(calculatedAngledPower);
            }
            else {
                calculatedAngledPower = lastAnglePower;
            }
        }

        swerveDrive.drive(0, 0, calculatedAngledPower);
    }

    Pose2d lastTagSeenPose2d;
    int lastTagSeenId;
    double lastTagSeenDist;
    public int getLastSeenAprilTagID() {
        return lastTagSeenId;
    }
    public Pose2d getLastSeenPose2d() {
        return lastTagSeenPose2d;
    }
    public double getLastTagSeenDist()
    {
        return lastTagSeenDist;
    }

    public Pose2d getLatestPose2dBlue()
    {
        LimelightHelpers.LimelightResults limelightResults = LimelightHelpers.getLatestResults(limelightName);
        return limelightResults.targetingResults.getBotPose2d_wpiBlue();
    }

    // be careful to use this function!!! todo, not finished yet.....
    public void rotationSeekApriltag(SwerveDrivetrain swerveDrive)
    {
        Pose2d currentPose = swerveDrive.getPose();

        if(!limelight.hasValidTarget())
        {
            swerveDrive.drive(0, 0, 0.08);
            reset();
        }
        else
        {
            TagAimingRotation(swerveDrive, 0, 100);
        }
    }



    double calculatedForwardPower;
    double calculatedSidewaysPower;
    double calculatedAngledPower;

    // Use this PID for Drive to Tag
    PIDController pidTADrive = new PIDController(2.5, 0, 0); // P 1.2
    PIDController pidTXDrive = new PIDController(0.06, 0, 0.0); // P 0.08
    PIDController pidSkewDrive = new PIDController(0.02, 0, 0); // P 0.02

    Pose3d currentPose;

    // ************************ VISION ***********************
    public void calculateTagDriving(double targetTA, double targetTX, double targetskew, int tagID, int maxSamples) {
        double taOffset;
        double txOffset;
        double skewOffset;
        int foundId = -1;

        dataSampleCount++;
        if(maxSamples > 0 && dataSampleCount >= maxSamples)
        {
            calculatedForwardPower = calculatedAngledPower = calculatedSidewaysPower = 0;
        }
        else
        {

        //SmartDashboard.putNumber("TAG ID: ", getAprilTagID());
        foundId = getAprilTagID();
        if(targetId != null)
            targetId.setInteger(foundId);
        if(tagID == foundId) {
            //SmartDashboard.putBoolean("Found Right Tag ID: ", true);
            
            taOffset = targetTA - limelight.getArea_avg();
            skewOffset = targetskew - limelight.getSkew_avg();
            txOffset = targetTX - limelight.getXAngle_avg();
     
            //SmartDashboard.putNumber("TA Offset: ", taOffset);
            if(currentTaOffset != null)
                currentTaOffset.setDouble(taOffset);
            //SmartDashboard.putNumber("Skew Offset: ", skewOffset);
            if(currentAngleOffset != null)
                currentAngleOffset.setDouble(skewOffset);
            //SmartDashboard.putNumber("TX Offset: ", txOffset);
            if(currentTxOffset != null)
                currentTxOffset.setDouble(txOffset);
    
            calculatedForwardPower = pidTADrive.calculate(taOffset, 0);
            // calculatedForwardPower = calculatedForwardPower * -1;

            calculatedSidewaysPower = pidTXDrive.calculate(txOffset, 0);
            // calculatedSidewaysPower = calculatedSidewaysPower * -1;

            calculatedAngledPower = pidSkewDrive.calculate(skewOffset, 0);
            calculatedAngledPower = calculatedAngledPower * -1 * Math.sqrt(taOffset); //???or back
            //calculatedAngledPower = pidSkewDrive.calculate(txOffset, 0)  * Math.sqrt(taOffset);

    
            //SmartDashboard.putNumber("Calculated Forward Power: ", calculatedForwardPower);
            if(forwardSpeed != null)
                forwardSpeed.setDouble(calculatedForwardPower);
            //SmartDashboard.putNumber("Calculated Sideways Power: ", calculatedSidewaysPower);
            if(sidewaysSpeed != null)
                sidewaysSpeed.setDouble(calculatedSidewaysPower);
            //SmartDashboard.putNumber("Calculated Angled Power: ", calculatedAngledPower);
            if(angularSpeed != null)
                angularSpeed.setDouble(calculatedAngledPower);
        }
        else {
            calculatedForwardPower = calculatedAngledPower = calculatedSidewaysPower = 0;
            //SmartDashboard.putBoolean("Found Right Tag ID: ", false);
            
        }
    }
    }

    // Use this PID for Turn to Tag
    // PIDController pidTA = new PIDController(1.8, 0, 0);
    // PIDController pidSkew = new PIDController(0.1, 0, 0);
    PIDController pidTX = new PIDController(0.05, 0, 0); // Probably gonna use TX for this

    private void pidTuning_test() {
        VisionConstants.kPTagAngular.loadPreferences();
        VisionConstants.kITagAngular.loadPreferences();
        VisionConstants.kDTagAngular.loadPreferences();

        pidTX.setPID(VisionConstants.kPTagAngular.get(), VisionConstants.kITagAngular.get(), VisionConstants.kDTagAngular.get());
    }

    public void calculateTagTurning(double targetTX, int tagID) {
        pidTuning_test();
        double TXOffset;
        int foundId = -1;

        foundId = getAprilTagID();
        if(targetId != null)
            targetId.setInteger(foundId);
        if(tagID == foundId) {
            
            TXOffset = targetTX - limelight.getXAngle_avg();

            if(currentAngleOffset != null)
                currentAngleOffset.setDouble(TXOffset);

            calculatedAngledPower = pidTX.calculate(TXOffset, 0);
            calculatedAngledPower = calculatedAngledPower * -1;
    
            if(angularSpeed != null)
                angularSpeed.setDouble(calculatedAngledPower);
        }
        else {
            calculatedAngledPower = 0;
            
        }
    }

    //boolean initPoseByVisionDone = false;
    public void resetInitPoseByVision(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId, int minSamples)
    {
        if(limelight != null && limelight.getAprilTagID() != -1)
        {
            // 7 is blue side, 4 is red side, center of speaker; 0 don't care
            if( (limelight.getAprilTagID() == apriltagId) || apriltagId == 0)
            {
                Pose2d p = getCurrentPose3DVision().toPose2d();
                
                // please add protection here!!!!!!
                //swerveDrive.getImu().zeroHeading();
                swerveDrive.getImu().setOffset(p.getRotation().getDegrees());
                //swerveDrive.resetOdometryWithAlliance(p);
                swerveDrive.resetOdometry(p);
                dataSampleCount = minSamples;
                return;
            }
        }

        dataSampleCount++;
        if(dataSampleCount >= minSamples)
        {
            swerveDrive.getImu().setOffset(defaultPose.getRotation().getDegrees());
            swerveDrive.resetOdometry(defaultPose);
        }
    }

    public Command InitPoseByVision(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId, int minSamples) {
        return Commands.sequence(
            Commands.runOnce(() -> dataSampleCount = 0),
            Commands.run(
                () -> resetInitPoseByVision(swerveDrive, defaultPose, apriltagId, minSamples)
            ).until(() -> dataSampleCount >= minSamples )
        );
    }

    public void resetOdoPose(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId, boolean forceToFind)
    {
        if(limelight != null && limelight.getAprilTagID() != -1)
        {
            // 7 is blue side, 4 is red side, center of speaker
            if(limelight.getAprilTagID() == apriltagId || apriltagId == 0)
            {
                Pose3d p = getCurrentPose3DVision();
                swerveDrive.resetOdometry(p.toPose2d());
                TagFound = true;
                return;
            }
            // to be done: else if, don't care
        }
        if(!forceToFind)
            swerveDrive.resetOdometry(defaultPose);
    }

    boolean TagFound = false;
    public Command resetOdoPoseByVision(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId, boolean forceToFind) {
        return Commands.sequence(
            Commands.runOnce(() -> TagFound=false),
            Commands.run(
                () -> resetOdoPose(swerveDrive, defaultPose, apriltagId, forceToFind)
            ).until(() -> TagFound == true || forceToFind == false)

            // Commands.runOnce(() -> reset()),
            // Commands.run(
            //     () -> setRobotPoseByApriltag(drivetrain, tagID, resetToCurrentPose)
            // ).until(() -> dataSampleCount >= minSamples )
        );
    }

    public void resetOdoPose(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId, int maxSamples)
    {
        SmartDashboard.putBoolean("Vision reset successful", false);
        if(limelight != null && limelight.getAprilTagID() != -1)
        {
            // 7 is blue side, 4 is red side, center of speaker
            if(limelight.getAprilTagID() == apriltagId || apriltagId == 0)
            {
                Pose3d p = getCurrentPose3DVision();
                swerveDrive.resetOdometry(p.toPose2d());
                dataSampleCount = maxSamples;
                TagFound = true;
                SmartDashboard.putBoolean("Vision reset successful", true);
                return;
            }
            // to be done: else if, don't care
        }

        dataSampleCount++;
        if(dataSampleCount >= maxSamples)
            SmartDashboard.putBoolean("Vision reset successful", false);
            if(defaultPose != null)
                swerveDrive.resetOdometry(defaultPose);
            else{} // nothing to reset, use whatever drive has currently. Protection may be needed!!!
    }

    public Command resetOdoPoseByVision(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId, 
        int maxSamples) {
        return Commands.sequence(
            Commands.runOnce(() -> TagFound=false),
            Commands.runOnce(() -> reset()),
            Commands.run(
                () -> resetOdoPose(swerveDrive, defaultPose, apriltagId, maxSamples)
            ).until(() -> dataSampleCount >= maxSamples)

            // Commands.runOnce(() -> reset()),
            // Commands.run(
            //     () -> setRobotPoseByApriltag(drivetrain, tagID, resetToCurrentPose)
            // ).until(() -> dataSampleCount >= minSamples )
        );
    }

    public Command resetOdoPoseByVision(SwerveDrivetrain swerveDrive, int apriltagId, 
        int maxSamples) {
        return Commands.sequence(
            Commands.runOnce(() -> TagFound=false),
            Commands.runOnce(() -> reset()),
            Commands.run(
                () -> resetOdoPose(swerveDrive, null, apriltagId, maxSamples)
            ).until(() -> dataSampleCount >= maxSamples)
        );
    }
    

    public Command resetOdoPoseWithConversion(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId, boolean forceToFind) {
        return Commands.sequence(
            Commands.runOnce(() -> TagFound=false),
            Commands.run(
                () -> resetOdoPose(swerveDrive, defaultPose, convertTagToAlliance(apriltagId), forceToFind)
            ).until(() -> TagFound == true || forceToFind == false)
        );
    }

    public Command noteOnHoldConfirmSignal()
    {
        // this command should be called by other parallel structure commands with 2s at least
        return Commands.sequence(
            Commands.runOnce(()->limelight.turnLightOn()),
            Commands.waitSeconds(3),
            Commands.runOnce(()->limelight.turnLightOff())
        ).finallyDo(()->limelight.turnLightOff());
    }

    // Add any tag ID (align to closest tag) functionality same method different signature
    // ************************* WE TEST LATER ****************************
    // public void driveToATag(double targetTA, double targetTX, double targetskew) {
    //     double taOffset;
    //     double txOffset;
    //     double skewOffset;

    //     SmartDashboard.putNumber("TAG ID: ", getAprilTagID());
        
    //     taOffset = targetTA - getTA();
    //     txOffset = targetTX - getTX();
    //     skewOffset = targetskew - getSkew();
    
    //     SmartDashboard.putNumber("TA Offset: ", taOffset);
    //     SmartDashboard.putNumber("TX Offset: ", txOffset);
    //     SmartDashboard.putNumber("Skew Offset: ", skewOffset);

    //     calculatedForwardPower = pidTA.calculate(taOffset, 0);
    //     calculatedForwardPower = calculatedForwardPower * -1;

    //     calculatedSidewaysPower = pidTX.calculate(txOffset, 0);
    //     calculatedSidewaysPower = calculatedSidewaysPower * -1;

    //     calculatedAngledPower = pidSkew.calculate(skewOffset, 0);
    //     calculatedAngledPower = calculatedAngledPower * -1;

    //     SmartDashboard.putNumber("Calculated Forward Power: ", calculatedForwardPower);
    //     SmartDashboard.putNumber("Calculated Sideways Power: ", calculatedSidewaysPower);
    //     SmartDashboard.putNumber("Calculated Angled Power: ", calculatedAngledPower);

    // }

    public boolean hasValidTarget() {
        return limelight.hasValidTarget();
    }
    
    public double getForwardPower() {
        
        return NerdyMath.deadband(calculatedForwardPower, -0.25, 0.25);
    }

    public double getSidewaysPower() {
        return NerdyMath.deadband(calculatedSidewaysPower, -0.25, 0.25);
    }

    public double getAngledPower() {
        return NerdyMath.deadband(calculatedAngledPower, -0.25, 0.25);
    }

    public int getAprilTagID() {
        if (limelight != null) {
            if (limelight.hasValidTarget()) {
                // todo, add ta size checking!!
                return limelight.getAprilTagID();
            }
        }
        return -1;
    }

    public Pose3d getCurrentPose3DVision()
    {
        currentPose = limelight.getBotPose3D_avg();
        if(botPoseByVisionX != null)
        {
            botPoseByVisionX.setDouble(currentPose.getX());
            botPoseByVisionY.setDouble(currentPose.getY());
            botPoseByVisionR.setDouble(Units.radiansToDegrees(currentPose.getRotation().getZ()));
        }
        return currentPose;
    }

    public double getVisionFrameTimestamp()
    {
        // with LimelightHelpers.getLatency_Pipeline() and LimelightHelpers.getLatency_Capture() 
        // Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0) or 
        // Timer.getFPGATimestamp() - (botpose[6]/1000.0)
        return Timer.getFPGATimestamp() - (limelight.getLatency_Pipeline()/1000.0) - (limelight.getLatency_Capture()/1000.0);
    }

    // public Command blinkLimelight() {

    // }

    /**
     * Sets the limelight pipeline
     * @param pipeline
     */
    public void changePipeline(int pipeline) {
        this.pipeline = pipeline;
        limelight.setPipeline(pipeline);
    }

    /**
     * Toggles the limelight light on or off
     * @param lightModeOn
     */
    public void toggleLight(boolean lightModeOn) {
        lightsON = lightModeOn;
        if(lightModeOn) limelight.setLightState(LightMode.ON);
        else limelight.setLightState(LightMode.OFF);
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        
    }


    
    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(limelightName +"data");

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:
                try{
                    //tab.addCamera(limelightName + ": Stream", limelightName, VisionConstants.kLimelightBackIP);
                }catch(Exception e){}

            case MEDIUM:
                tab.addBoolean("AprilTag Found", () -> limelight.hasValidTarget());
                currentTaOffset = tab.add("Ta Avg", 0)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();
                
                currentTxOffset = tab.add("Tx Avg", 0)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

                currentAngleOffset = tab.add("Angle Avg", 0)
                .withPosition(2, 2)
                .withSize(2, 1)
                .getEntry();

                botPoseByVisionX = tab.add("PoseX", 0)
                .withPosition(4,0)
                .withSize(2,1)
                .getEntry();
                botPoseByVisionY = tab.add("PoseY", 0)
                .withPosition(4,1)
                .withSize(2,1)
                .getEntry();
                botPoseByVisionR = tab.add("PoseR", 0)
                .withPosition(4,2)
                .withSize(2,1)
                .getEntry();

            case MINIMAL: 
             
                forwardSpeed = tab.add("Forward Speed", 0)
                .withPosition(3, 0)
                .withSize(2, 1)
                .getEntry();
                
                sidewaysSpeed = tab.add("Sideways Speed", 0)
                .withPosition(3, 1)
                .withSize(2, 1)
                .getEntry();

                angularSpeed = tab.add("Angular Speed", 0)
                .withPosition(3, 2)
                .withSize(2, 1)
                .getEntry();
                
                targetId = tab.add("Target ID", -1)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();


            case OFF:
                break;
            
        }
    }

}
