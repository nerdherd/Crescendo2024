package frc.robot.subsystems.vision.farfuture;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.imu.Gyro;

/**
 * Subsystem that uses Limelight for vision
 */
public class DriverAssist implements Reportable{
    private Limelight limelight;
    private String limelightName;

    private AprilTagFieldLayout layout;
    private int pipeline;
    private boolean lightsON;
    
    /**
     * Makes a new EMPeach to utilize vision
     * @param name name of the limelight
     */
    public DriverAssist(String name, int pipeline) {
        limelightName = name;

        try {
            limelight = new Limelight(name);
            toggleLight(false);
            changePipeline(pipeline);

            SmartDashboard.putBoolean("Limelight: " + name + " inited ", true);
        } catch (Exception e) {
            SmartDashboard.putBoolean("Limelight-" + name + " inited ", false);
        }
        
    }

    public double getTA() {
        return limelight.getArea();
    }

    public double getTX() {
        return limelight.getXAngle();
    }

    public double getSkew() {
        return limelight.periodic();
    }


    PIDController pidTA = new PIDController(0.3, 0, 0);
    PIDController pidTX = new PIDController(0.1, 0, 0);
    PIDController pidSkew = new PIDController(0.02, 0, 0);

    double calculatedForwardPower;
    double calculatedSidewaysPower;
    double calculatedAngledPower;

    // ************************ VISION ***********************
    public void driveToATag(double targetTA, double targetTX, double targetskew, int tagID) {
        double taOffset;
        double txOffset;
        double skewOffset;

        SmartDashboard.putNumber("TAG ID: ", getAprilTagID());
        if(tagID == getAprilTagID()) {
            SmartDashboard.putBoolean("Found Right Tag ID: ", true);
            
            taOffset = targetTA - getTA();
            txOffset = targetTX - getTX();
            skewOffset = targetskew - getSkew();
     
            SmartDashboard.putNumber("TA Offset: ", taOffset);
            SmartDashboard.putNumber("TX Offset: ", txOffset);
            SmartDashboard.putNumber("Skew Offset: ", skewOffset);
    
            calculatedForwardPower = pidTA.calculate(taOffset, 0);
            calculatedForwardPower = calculatedForwardPower * -1;

            calculatedSidewaysPower = pidTX.calculate(txOffset, 0);
            calculatedSidewaysPower = calculatedSidewaysPower * -1;

            calculatedAngledPower = pidSkew.calculate(skewOffset, 0);
            calculatedAngledPower = calculatedAngledPower * -1;
    
            SmartDashboard.putNumber("Calculated Forward Power: ", calculatedForwardPower);
            SmartDashboard.putNumber("Calculated Sideways Power: ", calculatedSidewaysPower);
            SmartDashboard.putNumber("Calculated Angled Power: ", calculatedAngledPower);
        }
        else {
            SmartDashboard.putBoolean("Found Right Tag ID: ", false);
            
        }
    }

    // Add any tag ID (align to closest tag) functionality same method different signature
    // ************************* WE TEST LATER ****************************
    public void driveToATag(double targetTA, double targetTX, double targetskew) {
        double taOffset;
        double txOffset;
        double skewOffset;

        SmartDashboard.putNumber("TAG ID: ", getAprilTagID());
        
        taOffset = targetTA - getTA();
        txOffset = targetTX - getTX();
        skewOffset = targetskew - getSkew();
    
        SmartDashboard.putNumber("TA Offset: ", taOffset);
        SmartDashboard.putNumber("TX Offset: ", txOffset);
        SmartDashboard.putNumber("Skew Offset: ", skewOffset);

        calculatedForwardPower = pidTA.calculate(taOffset, 0);
        calculatedForwardPower = calculatedForwardPower * -1;

        calculatedSidewaysPower = pidTX.calculate(txOffset, 0);
        calculatedSidewaysPower = calculatedSidewaysPower * -1;

        calculatedAngledPower = pidSkew.calculate(skewOffset, 0);
        calculatedAngledPower = calculatedAngledPower * -1;

        SmartDashboard.putNumber("Calculated Forward Power: ", calculatedForwardPower);
        SmartDashboard.putNumber("Calculated Sideways Power: ", calculatedSidewaysPower);
        SmartDashboard.putNumber("Calculated Angled Power: ", calculatedAngledPower);

    }

    public double getForwardPower() {
        return calculatedForwardPower;
    }

    public double getSidewaysPower() {
        return calculatedSidewaysPower;
    }

    public double getAngledPower() {
        return calculatedAngledPower;
    }

    public int getAprilTagID() {
        if (limelight != null) {
            if (limelight.hasValidTarget()) {
                return limelight.getAprilTagID();
            }
        }
        return -1;
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

            case MEDIUM:
                tab.addBoolean("AprilTag Found", () -> limelight.hasValidTarget());

            case MINIMAL:   
                tab.addCamera(limelightName + ": Stream", limelightName, VisionConstants.kLimelightFrontIP);


            case OFF:
                break;
            
        }
    }
    
}
