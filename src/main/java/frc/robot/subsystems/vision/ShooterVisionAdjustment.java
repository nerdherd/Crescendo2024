package frc.robot.subsystems.vision;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.util.NerdyMath;
import frc.robot.util.NerdySpline;

public class ShooterVisionAdjustment implements Reportable{
    //private Limelight limelight;
    private String name;
    private Gyro armPositionGyro;
    //DriverAssist tagCamera;
    SuperSystem superSystem;


    private NerdySpline angleEquation;
    private NerdySpline distanceEquation;
    private AprilTagFieldLayout layout;

    private GenericEntry targetFound;
    private GenericEntry goalAngle;
    private GenericEntry distanceOffset;
    private GenericEntry poseRobot;
    private GenericEntry poseTag;
    private GenericEntry goalDistance;



    private double[] distances = {1.33, 2.82, 4.00}; // meters, from least to greatest
    private double[] angles = {ShooterConstants.kSpeakerPosition.get(), ShooterConstants.kSpeakerPosition2.get(), -23.5}; // rotations // TODO: Convert to degrees

    public ShooterVisionAdjustment( Gyro armPositionGyro,
    //DriverAssist tagCamera, 
    SuperSystem superSystem) {
        this.name = "othr";
        //this.limelight = limelight;
        this.armPositionGyro = armPositionGyro;
        this.superSystem = superSystem;
        //this.tagCamera = tagCamera;

        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        for (int i = 0; i < angles.length; i++) {
            angles[i] += VisionConstants.kSplineAngleOffset; //currently 0
        }

        angleEquation = new NerdySpline(distances, angles);
        angleEquation.create();
        distanceEquation = new NerdySpline(angles, distances);
        distanceEquation.create();
    }

    public void saveSensorDataToFile(int isGreat) {

        String s = String.format("%d, %d, %.2f, %.2f, %.2f, %.2f\n", 
            isGreat,
            //tagCamera.getLimelight().getAprilTagID(),
            RobotContainer.apriltagCamera.getLastSeenAprilTagID(),
            // armPositionGyro.getHeading(), 
            superSystem.shooterPivot.getPositionDegrees(),
            RobotContainer.apriltagCamera.getLastSeenPose3d().getX(),
            RobotContainer.apriltagCamera.getLastSeenPose3d().getY(),
            Units.radiansToDegrees(RobotContainer.apriltagCamera.getLastSeenPose3d().getRotation().getZ())
        );
        Robot.armSensorCaliLog.append(s);
        
        System.out.println("Data saved: " + s);
    }

    public Command armCalibrationTable(int buttonId) {
        return new InstantCommand(() -> saveSensorDataToFile(buttonId));
    }

    public Pose3d getRobotPose() {
        RobotContainer.apriltagCamera.getLimelight().setPipeline(VisionConstants.kAprilTagPipeline);
        if(!RobotContainer.apriltagCamera.getLimelight().hasValidTarget()) return null;
        if(targetFound != null)
            targetFound.setBoolean(RobotContainer.apriltagCamera.getLimelight().hasValidTarget());

        Pose3d pose = RobotContainer.apriltagCamera.getLimelight().getBotPose3D();

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
    
 
    public double getShooterAngle() {
        Pose3d currentPose = getRobotPose();
        if(currentPose == null) return -0.1;
        Pose3d tagPose;
        
        if(RobotContainer.IsRedSide()) {
            tagPose = getTagPose(4);
        }
        else{
            tagPose = getTagPose(7);
        }
        if(tagPose == null) return 0;

        double distance = Math.sqrt(Math.pow(currentPose.getX() - tagPose.getX(), 2) + Math.pow(currentPose.getY() - tagPose.getY(), 2));
        if(distanceOffset != null) distanceOffset.setDouble(distance);
        SmartDashboard.putNumber("Distance", distance);
        if(distance < distances[0]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return 0;
        }
        if (distance > distances[distances.length - 1]) {
            SmartDashboard.putBoolean("Vision failed", true);
            return 0;
        }

        SmartDashboard.putBoolean("Vision failed", false);

        double output = NerdyMath.clamp(angleEquation.getOutput(distance), ShooterConstants.kFullStowPosition.get(), 20);
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
                //tab.addCamera(name + ": Stream", name, VisionConstants.kLimelightFrontIP);
            }catch(Exception e){};

            case MEDIUM:

            case MINIMAL:
                // tab.add("Too Low", armCalibrationTable(-1));
                // tab.add("Too High", armCalibrationTable(1));
                // tab.add("Great!", armCalibrationTable(0));
            
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
