package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Reportable;
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

    private double[] distances = {1.0, 1.356, 2.554, 2.95, 3.5, 3.828, 4.214, 5.548}; // meters, from least to greatest
    private double[] angles = {-50, -49.6, -31, -30, -23.115, -22, -21, -16.875}; // rotations // TODO: Convert to degrees

    public ShooterVisionAdjustment(String name, Limelight limelight) {
        this.name = name;
        this.limelight = limelight;

        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        for (int i = 0; i < angles.length; i++) {
            angles[i] += VisionConstants.kSplineAngleOffset; //currently 0
        }

        angleEquation = new NerdySpline(distances, angles);
        angleEquation.create();
        distanceEquation = new NerdySpline(angles, distances);
        distanceEquation.create();
    }

    public boolean hasValidTarget() {
        return limelight.hasValidTarget();
    }

    public Pose3d getRobotPose() {
        limelight.setPipeline(VisionConstants.kAprilTagPipeline);
        if(!limelight.hasValidTarget()) return null;
        if(targetFound != null)
            targetFound.setBoolean(limelight.hasValidTarget());

        Pose3d pose = limelight.getBotPose3D_avg();

        if (pose == null) {
            DriverStation.reportWarning("Robot pose is null!", true);
            return new Pose3d();
        }

        if(poseRobot != null)
            poseRobot.setString(pose.toString());

        return pose;
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
    
    private double lastAngle = 0;
    private double lastDistance = 0;

    public double getShooterAngle() {
        return getShooterAngle(false);
    }

    public double getDistanceFromTag(boolean preserveOldValue) {
        Pose3d currentPose = getRobotPose();
        if(currentPose == null) return (preserveOldValue ? lastDistance : 0.01);
        Pose3d tagPose;
        
        if(RobotContainer.IsRedSide()) {
            tagPose = getTagPose(4);
        }
        else{
            tagPose = getTagPose(7);
        }
        if(tagPose == null) return (preserveOldValue ? lastDistance : -0.1);

        lastDistance = Math.sqrt(Math.pow(currentPose.getX() - tagPose.getX(), 2) + Math.pow(currentPose.getY() - tagPose.getY(), 2));
        if(distanceOffset != null) distanceOffset.setDouble(lastDistance);
        SmartDashboard.putNumber("Distance", lastDistance);

        return lastDistance;
    }

    private double angleoffset = 0;

    public void incrementOffset(double increment) {
        angleoffset += increment;
        angleoffset = NerdyMath.clamp(angleoffset, -10, 10);
    }

    public void resetOffset() {
        angleoffset = 0;
    }
 
    public double getShooterAngle(boolean preserveOldValue) {
        double distance = getDistanceFromTag(preserveOldValue);
        if(distance < distances[0]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return (preserveOldValue ? lastAngle : -0.2);
        }
        if (distance > distances[distances.length - 1]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return (preserveOldValue ? lastAngle : -0.3);
        }

        SmartDashboard.putBoolean("Vision failed", false);

        double output = NerdyMath.clamp(angleEquation.getOutput(distance), ShooterConstants.kFullStowPosition.get(), 20);
        SmartDashboard.putNumber("Vision Angle", output);
        SmartDashboard.putNumber("Vision Distance", distance);
        if(goalAngle != null) 
            goalAngle.setDouble(output);
        output += angleoffset;
        SmartDashboard.putNumber("Angle with Offset", output);
        SmartDashboard.putNumber("Angle Offset", angleoffset);
        lastAngle = output;
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
                tab.addNumber("Angle offset", () -> angleoffset);

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
