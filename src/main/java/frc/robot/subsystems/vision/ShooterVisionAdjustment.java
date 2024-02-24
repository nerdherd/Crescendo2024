package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelperUser;
import frc.robot.util.NerdySpline;

public class ShooterVisionAdjustment implements Reportable{
    private Limelight limelight;
    private String name;
    private LimelightHelperUser limelightHelperUser;

    private NerdySpline angleEquation;
    private NerdySpline distanceEquation;
    private AprilTagFieldLayout layout;

    private GenericEntry targetFound;
    private GenericEntry goalAngle;
    private GenericEntry distanceOffset;
    private GenericEntry poseRobot;
    private GenericEntry poseTag;
    private GenericEntry goalDistance;

    public ShooterVisionAdjustment(String name) {
        this.name = name;
        try {
            limelight = new Limelight(name);
            limelightHelperUser = new LimelightHelperUser(name);
            limelight.setLightState(LightMode.OFF);
            limelight.setPipeline(VisionConstants.kAprilTagPipeline);

            SmartDashboard.putBoolean("Limelight: " + name + " inited", true);
            SmartDashboard.putBoolean("LimelightHelper inited", true);
        } catch (Exception e) {
            SmartDashboard.putBoolean("limelight-" + name + " inited", false);
            SmartDashboard.putBoolean("LimelightHelper inited", false);
        }

        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        //TODO: Get actual input and output data
        double[] distances = {0, 1, 2, 3}; // meters
        double[] angles = {0, 0.841, 0.909, 0.141}; // degrees

        angleEquation = new NerdySpline(distances, angles);
        angleEquation.create();
        distanceEquation = new NerdySpline(angles, distances);
        distanceEquation.create();
    }

    public Pose3d getRobotPose() {
        limelight.setPipeline(VisionConstants.kAprilTagPipeline);
        if(!limelight.hasValidTarget()) return null;
        if(targetFound != null)
            targetFound.setBoolean(limelight.hasValidTarget());

        if(poseRobot != null) poseRobot.setString(limelightHelperUser.getPose3d().toString());
        return limelightHelperUser.getPose3d();
    }

    public Pose3d getTagPose(int ID) {
        if(ID < 1 || ID > 16) return null;
        if(layout == null) return null;
        Optional<Pose3d> tagPose = layout.getTagPose(ID);
        if(tagPose.isEmpty()) return null;
        
        if(poseTag != null) poseTag.setString(tagPose.toString());
        return tagPose.get();
    }

    public void getShooterAngle_test(int numberOfTests, double step) {
        double[] inputs = new double[numberOfTests];
        double[] outputs = new double[numberOfTests];
        for(int i = 0; i < outputs.length; i++) {
            inputs[i] = i * step;
            outputs[i] = angleEquation.getOutput(inputs[i]);
        }
        SmartDashboard.putNumberArray("Spline Test Inputs", inputs);
        SmartDashboard.putNumberArray("Spline Test Outputs", outputs);
    }
 
    public double getShooterAngle() {
        Pose3d currentPose = getRobotPose();
        if(currentPose == null) return -1;
        if(poseRobot != null)
            poseRobot.setString(currentPose.toString());
            
        Pose3d tagPose = getTagPose(limelight.getAprilTagID());
        if(tagPose == null) return -1;
        if(poseTag != null)
            poseTag.setString(tagPose.toString());

        double distance = Math.abs(tagPose.getX() - currentPose.getX());
        if(distanceOffset != null) distanceOffset.setDouble(distance);

        if(goalAngle != null) goalAngle.setDouble(angleEquation.getOutput(distance));
        return angleEquation.getOutput(distance);
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
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:
       
            try{
                tab.addCamera(name + ": Stream", name, VisionConstants.kLimelightFrontIP);
            }catch(Exception e){};

            case MEDIUM:

            case MINIMAL:   
                goalAngle = tab.add("Calculated Angle", false)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();

                distanceOffset = tab.add("Distance Offset", false)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

                poseRobot = tab.add("Calculated Angle", false)
                .withPosition(0, 2)
                .withSize(3, 1)
                .getEntry();

                poseTag = tab.add("Distance Offset", false)
                .withPosition(0, 3)
                .withSize(3, 1)
                .getEntry();
                
                targetFound = tab.add("Target Found", false)
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();

            case OFF:
                break;
            
        }
    }
}
