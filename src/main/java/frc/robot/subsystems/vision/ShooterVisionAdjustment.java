package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelperUser;
import frc.robot.util.NerdyMath;
import frc.robot.util.NerdySpline;

public class ShooterVisionAdjustment implements Reportable{
    private Limelight limelight;
    private String name;

    private NerdySpline angleEquation;
    private NerdySpline distanceEquation;
    private AprilTagFieldLayout layout;

    private GenericEntry targetFound;
    private GenericEntry goalAngle;
    private GenericEntry distanceOffset;
    private GenericEntry poseRobot;
    private GenericEntry poseTag;
    private GenericEntry goalDistance;

    //TODO: Test actual input and output data
    private double[] distances = {1.4257, 2.836, 3.5482}; // meters, from least to greatest
    private double[] angles = {-0.064, -0.025, -0.003}; // rotations

    public ShooterVisionAdjustment(String name, Limelight limelight) {
        this.name = name;
        this.limelight = limelight;

        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

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

        Pose3d pose = limelight.getBotPose3D();

        Pose3d newPose = new Pose3d(VisionConstants.fieldXOffset*2 - pose.getX(), VisionConstants.fieldYOffset*2 - pose.getY(), pose.getZ(), new Rotation3d(0, 0, pose.getRotation().getZ()));
        
        if(poseRobot != null)
            poseRobot.setString(pose.toString());

        return newPose;
    }

    public Pose3d getTagPose(int ID) {
        if(ID != 7 && ID != 4 && ID != 8 && ID != 3) return null;
        if(layout == null) return null;
        Optional<Pose3d> tagPose = layout.getTagPose(ID);
        if(tagPose.isEmpty()) return null;
        
        if(poseTag != null) 
            poseTag.setString(tagPose.toString());
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
        if(currentPose == null) return -0.1;
        Pose3d tagPose = getTagPose(limelight.getAprilTagID());
        if(tagPose == null) return -0.1;

        double distance = Math.sqrt(Math.pow(currentPose.getX() - tagPose.getX(), 2) + Math.pow(currentPose.getY() - tagPose.getY(), 2));
        if(distanceOffset != null) distanceOffset.setDouble(distance);
        SmartDashboard.putNumber("Distance", distance);
        if(distance < distances[0] || distance > distances[distances.length - 1]) {
            SmartDashboard.putBoolean("Visioni failed", true);
            return -0.1;
        }

        SmartDashboard.putBoolean("Visioni failed", false);

        double output = NerdyMath.clamp(angleEquation.getOutput(distance), -0.1, 0.2);
        SmartDashboard.putNumber("Vision Angle", output);
        SmartDashboard.putNumber("Vision Distance", distance);
        if(goalAngle != null) 
            goalAngle.setDouble(output);
        return output;
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
        ShuffleboardTab tab = Shuffleboard.getTab("spline:" + name);

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:
       
            try{
                tab.addCamera(name + ": Stream", name, VisionConstants.kLimelightFrontIP);
            }catch(Exception e){};

            case MEDIUM:

            case MINIMAL:   
                goalAngle = tab.add("Calculated Angle", 0)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();

                distanceOffset = tab.add("Distance Offset", 0)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

                poseRobot = tab.add("Robot Pose", "null")
                .withPosition(0, 2)
                .withSize(3, 1)
                .getEntry();

                poseTag = tab.add("Tag Pose", "null")
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
