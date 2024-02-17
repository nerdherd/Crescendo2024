package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.util.NerdyMath;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;

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
        limelightName = name;
        ShuffleboardTab tab = Shuffleboard.getTab(limelightName);

        try {
            limelight = new Limelight(name);
            toggleLight(false);
            changePipeline(pipeline);

            tab.add(name + " inited ", true);
        } catch (Exception e) {
            tab.add(name + " inited ", false);
        }
        
    }

    public void reset()
    {
        limelight.reinitBuffer();
        dataSampleCount = 0;
    }

    public void TagDriving(SwerveDrivetrain swerveDrive, double targetTA, double targetTX, double targetSkew, int tagID) {
        calculateTag(targetTA, targetTX, targetSkew, tagID);

        swerveDrive.drive(getForwardPower(), getSidewaysPower(), getAngledPower()); //TODO: //getSidewaysPower(), getAngledPower());
    }
    public Command driveToApriltagCommand(SwerveDrivetrain drivetrain, double targetTA, double targetTX, double targetSkew, int tagID) {
        return Commands.sequence(
            Commands.run(
                () -> TagDriving(drivetrain, targetTA, targetTX, targetSkew, tagID)
            ).until(() -> Math.abs(getForwardPower()) <= 0.1 && Math.abs(getSidewaysPower()) <= 0.1 && Math.abs(getAngledPower()) <= 0.1)
        );
    }

    public void TurnToTag(SwerveDrivetrain swerveDrive, double targetSkew, int tagID) {
        calculateTagSkew(targetSkew, tagID); // TODO: WHY IS THIS CAUSING A BUILD FAIL?

        swerveDrive.drive(0, 0, getAngledPower()); //TODO: //getSidewaysPower(), getAngledPower());
    }
    // public Command TurnToTagCommand(SwerveDrivetrain drivetrain, double targetSkew, int tagID) {
    //     return Commands.sequence(
    //         Commands.run(
    //             () -> TurnToTag(drivetrain, targetSkew, tagID)
    //         ).until(() -> Math.abs(Math.abs(getAngledPower())) <= 0.1)
    //     );
    // }

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
    public Command aimToApriltagCommand(SwerveDrivetrain drivetrain, int tagID, int minSamples, int maxSamples, Pose2d plannedPose, boolean resetToCurrentPose) {
        return Commands.sequence(
            Commands.runOnce(() -> reset()),
            Commands.run(
                () -> TagAimingRotation(drivetrain, tagID, maxSamples)
            ).until(() -> dataSampleCount >= minSamples && Math.abs(calculatedAngledPower) <= 0.1),

            Commands.runOnce(() -> reset()),
            Commands.run(
                () -> setRobotPoseByApriltag(drivetrain, tagID, resetToCurrentPose)
            ).until(() -> dataSampleCount >= minSamples )
        );
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
    public void TagAimingRotation(SwerveDrivetrain swerveDrive, int tagID, int maxSamples) {
        // make sure to reset before or after calling this function
        // make sure the cross is at the center!!! for tx
        // tagID == 0 means: don't care of tag's id, be careful for multi-tags location
        double taOffset;
        double txOffset;

        dataSampleCount++;
        if(maxSamples > 0 && dataSampleCount >= maxSamples)
        {
            calculatedAngledPower = 0;
        }
        else
        {
            int foundId = getAprilTagID();
            if(targetId != null)
                targetId.setInteger(foundId);

            if(tagID == foundId || (tagID == 0 && foundId != -1)) {
                
                taOffset = limelight.getArea_avg();
                txOffset = limelight.getXAngle_avg();
                    
                if(currentTaOffset != null)
                    currentTaOffset.setDouble(taOffset);
                if(currentTxOffset != null)
                    currentTxOffset.setDouble(txOffset);
                if(currentAngleOffset != null)
                    currentAngleOffset.setDouble(0);

                double txInRangeValue = Math.abs(TxTargetOffsetForCurrentTa(taOffset));
                if( txOffset < txInRangeValue && txOffset > -1*txInRangeValue ) // todo, tuning pls!!!
                {
                    calculatedAngledPower = 0; // in good tx ranges. faster than the pid
                } 
                // else if(){} // out of range cases. todo 
                else
                {
                    // pid based on tx, and adding ta/distance as the factor
                    calculatedAngledPower = pidTxRotation.calculate(txOffset, 0)  * Math.sqrt(taOffset);
                    calculatedAngledPower = NerdyMath.deadband(calculatedAngledPower, -0.3, 0.3); // todo, tuning pls. Have to consider the Ta for all coeffs!!! Todo
                }

                if(forwardSpeed != null)
                    forwardSpeed.setDouble(0);
                if(sidewaysSpeed != null)
                    sidewaysSpeed.setDouble(0);
                if(angularSpeed != null)
                    angularSpeed.setDouble(calculatedAngledPower);
            }
            else {
                calculatedAngledPower = 0;
            }
        }

        swerveDrive.drive(0, 0, calculatedAngledPower);
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


    public double getTA() {
        return limelight.getArea(); //filter?
    }

    public double getTX() {
        return limelight.getXAngle();//filter?
    }

    public double getSkew() {
        return limelight.getCamPoseSkew();//filter?
    }

    PIDController pidTA = new PIDController(1.8, 0, 0); // P 1.2
    PIDController pidTX = new PIDController(0.06, 0, 0.006); // P 0.08
    PIDController pidSkew = new PIDController(0.05, 0, 0); // P 0.02

    double calculatedForwardPower;
    double calculatedSidewaysPower;
    double calculatedAngledPower;

    Pose3d currentPose;

    // ************************ VISION ***********************
    public void calculateTag(double targetTA, double targetTX, double targetskew, int tagID) {
        double taOffset;
        double txOffset;
        double skewOffset;
        int foundId = -1;

        //SmartDashboard.putNumber("TAG ID: ", getAprilTagID());
        foundId = getAprilTagID();
        if(targetId != null)
            targetId.setInteger(foundId);
        if(tagID == foundId) {
            //SmartDashboard.putBoolean("Found Right Tag ID: ", true);
            
            taOffset = targetTA - getTA();
            txOffset = targetTX - getTX();
            skewOffset = targetskew - getSkew();
     
            //SmartDashboard.putNumber("TA Offset: ", taOffset);
            if(currentTaOffset != null)
                currentTaOffset.setDouble(taOffset);
            //SmartDashboard.putNumber("TX Offset: ", txOffset);
            if(currentTxOffset != null)
                currentTxOffset.setDouble(txOffset);
            //SmartDashboard.putNumber("Skew Offset: ", skewOffset);
            if(currentAngleOffset != null)
                currentAngleOffset.setDouble(skewOffset);
    
            calculatedForwardPower = pidTA.calculate(taOffset, 0);
            // calculatedForwardPower = calculatedForwardPower * -1;

            calculatedSidewaysPower = pidTX.calculate(txOffset, 0);
            // calculatedSidewaysPower = calculatedSidewaysPower * -1;

            calculatedAngledPower = pidSkew.calculate(skewOffset, 0);
            calculatedAngledPower = calculatedAngledPower * -1;
    
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

    public void calculateTagSkew(double targetskew, int tagID) {
        double skewOffset;
        int foundId = -1;

        //SmartDashboard.putNumber("TAG ID: ", getAprilTagID());
        foundId = getAprilTagID();
        if(targetId != null)
            targetId.setInteger(foundId);
        if(tagID == foundId) {
            //SmartDashboard.putBoolean("Found Right Tag ID: ", true);
            
            skewOffset = targetskew - getSkew();

            if(currentAngleOffset != null)
                currentAngleOffset.setDouble(skewOffset);

            calculatedAngledPower = pidSkew.calculate(skewOffset, 0);
            calculatedAngledPower = calculatedAngledPower * -1;
    
            //SmartDashboard.putNumber("Calculated Angled Power: ", calculatedAngledPower);
            if(angularSpeed != null)
                angularSpeed.setDouble(calculatedAngledPower);
        }
        else {
            calculatedAngledPower = 0;
            
        }
    }

    // public void calculateTag(double targetTA, double targetTX, double targetskew) {
    //     double taOffset;
    //     double txOffset;
    //     double skewOffset;

    //     // Same method, different signature: no tag ID needed
            
    //         taOffset = targetTA - getTA();
    //         txOffset = targetTX - getTX();
    //         skewOffset = targetskew - getSkew();
     
    //         //SmartDashboard.putNumber("TA Offset: ", taOffset);
    //         if(currentTaOffset != null)
    //             currentTaOffset.setDouble(taOffset);
    //         //SmartDashboard.putNumber("TX Offset: ", txOffset);
    //         if(currentTxOffset != null)
    //             currentTxOffset.setDouble(txOffset);
    //         //SmartDashboard.putNumber("Skew Offset: ", skewOffset);
    //         if(currentAngleOffset != null)
    //             currentAngleOffset.setDouble(skewOffset);
    
    //         calculatedForwardPower = pidTA.calculate(taOffset, 0);
    //         // calculatedForwardPower = calculatedForwardPower * -1;

    //         calculatedSidewaysPower = pidTX.calculate(txOffset, 0);
    //         // calculatedSidewaysPower = calculatedSidewaysPower * -1;

    //         calculatedAngledPower = pidSkew.calculate(skewOffset, 0);
    //         calculatedAngledPower = calculatedAngledPower * -1;
    
    //         //SmartDashboard.putNumber("Calculated Forward Power: ", calculatedForwardPower);
    //         if(forwardSpeed != null)
    //             forwardSpeed.setDouble(calculatedForwardPower);
    //         //SmartDashboard.putNumber("Calculated Sideways Power: ", calculatedSidewaysPower);
    //         if(sidewaysSpeed != null)
    //             sidewaysSpeed.setDouble(calculatedSidewaysPower);
    //         //SmartDashboard.putNumber("Calculated Angled Power: ", calculatedAngledPower);
    //         if(angularSpeed != null)
    //             angularSpeed.setDouble(calculatedAngledPower);
    //     }
    //     else {
    //         calculatedForwardPower = calculatedAngledPower = calculatedSidewaysPower = 0;
    //         //SmartDashboard.putBoolean("Found Right Tag ID: ", false);
            
    //     }
    // }

    boolean initPoseByVisionDone = false;
    public void resetInitPoseByVision(SwerveDrivetrain swerveDrive, Pose2d defaultPose, int apriltagId)
    {
        if(limelight != null && limelight.getAprilTagID() != -1)
        {
            // 7 is blue side, 4 is red side, center of speaker
            if(!initPoseByVisionDone && (limelight.getAprilTagID() == apriltagId))
            {
                initPoseByVisionDone = true;
                Pose3d p = getCurrentPose3DVision();
                swerveDrive.resetOdometry(p.toPose2d());
                swerveDrive.getImu().setOffset(p.getRotation().getZ());
                return;
            }
        }
        
        swerveDrive.resetOdometry(defaultPose);
        swerveDrive.getImu().setOffset(defaultPose.getRotation().getRadians());
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

    public double getForwardPower() {
        // if(calculatedForwardPower < 0.25 && calculatedForwardPower > -0.25) {
        //     calculatedForwardPower = 0;
        //     //SmartDashboard.putBoolean("Forward Condition Met? ", true);
        // }
        
        return NerdyMath.deadband(calculatedForwardPower, -0.25, 0.25);
    }

    public double getSidewaysPower() {
        // if(calculatedSidewaysPower < 0.25 && calculatedSidewaysPower > -0.25) {
        //     calculatedSidewaysPower = 0;
        //     //SmartDashboard.putBoolean("Sideways Condition Met? ", true);
        // }
        
        return NerdyMath.deadband(calculatedSidewaysPower, -0.25, 0.25);
    }

    public double getAngledPower() {
        // if(calculatedAngledPower < 0.25 && calculatedAngledPower > -0.25) {
        //     calculatedAngledPower = 0;
        //     //SmartDashboard.putBoolean("Angled Condition Met? ", true);
        // }
        
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
        currentPose = limelight.getBotPose3D();
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
        ShuffleboardTab tab = Shuffleboard.getTab(limelightName);

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:
                try{
                    tab.addCamera(limelightName + ": Stream", limelightName, VisionConstants.kLimelightBackIP);
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
