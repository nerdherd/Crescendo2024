package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.NerdyLine;
import frc.robot.util.NerdyMath;
import frc.robot.util.NerdySpline;

public class ShooterVisionAdjustment implements Reportable{
    //private Limelight limelight;
    private String name;
    private Gyro armPositionGyro;
    DriverAssist tagCamera;
    SuperSystem superSystem;

    private Gyro swerveGyro;


    // private NerdySpline angleEquation;
    // private NerdySpline distanceEquation;
    private NerdyLine angleLine;
    private AprilTagFieldLayout layout;

    private GenericEntry targetFound;
    private GenericEntry goalAngle;
    private GenericEntry distanceOffset;
    private GenericEntry poseRobot;
    private GenericEntry poseTag;
    private GenericEntry goalDistance;
    private Supplier<Pose2d> poseSupplier;

    private Pose3d robotPose;

    // private double[] distances = {1.0, 1.356, 2.554, 2.95, 3.10, 3.5,  3.828,     4.15,     5.548}; // meters, from least to greatest
    // private double[] angles    = {-50, -49.6,   -31,  -30,  -27, -24.5, -23.25,  -22.375, -16.875}; // rotations // TODO: Convert to degrees
    //                           //  -50, -49.6,   -31,  -30,  -27, -23,   -22.5,   -22,    -16.875
    // the values above are old

    // Warbots field
    private double[] distances = {1.2,   2.483,   3.015,    3.573,   4.267,   4.697}; // distances from 4/6
    private double[] angles = {-52.470, -32.861, -29.114, -25.663, -21.413, -20.8}; // angles from 4/6

    // at school field
    //private double[] distances = {1.2, 2.483, 3.015, 3.573, 4.267, 4.697}; // distances from 4/3
    //private double[] angles = {-52.470, -32.861, -29.114, -25.663, -22.413, -23.008}; // angles from 4/3

    public ShooterVisionAdjustment(
        DriverAssist tagCamera, 
        Gyro swerveGyro,
        SuperSystem superSystem,
        Supplier<Pose2d> poseSupplier) 
    {
        
        this.name = "othr";
        //this.limelight = limelight;
        //this.armPositionGyro = armPositionGyro;
        this.superSystem = superSystem;
        this.tagCamera = tagCamera;
        this.swerveGyro = swerveGyro;
        this.poseSupplier = poseSupplier;

        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        for (int i = 0; i < angles.length; i++) {
            angles[i] += VisionConstants.kSplineAngleOffset; //currently 0
        }

        // angleEquation = new NerdySpline(distances, angles);
        // angleEquation.create();
        // distanceEquation = new NerdySpline(angles, distances);
        // distanceEquation.create();
        angleLine = new NerdyLine(distances, angles);
    }

    public boolean hasValidTarget() {
        return tagCamera.hasValidTarget();
    }

    public void saveSensorDataToFile(int isGreat) {
        Pose3d pose = getRobotPose();
        if(pose == null)
        {
            SmartDashboard.putString("saved", "no data");
            return;
        }
        String s = String.format("%d, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
            isGreat,
            pose.getX(),
            pose.getY(),
            Math.toDegrees(pose.getRotation().getZ()),
            lastDistance, 
            superSystem.shooterPivot.getPositionDegrees(),
            superSystem.shooterPivot.getTargetPositionDegrees(),
            swerveGyro.getPitch()
            // armPositionGyro.getHeading()
            //superSystem.shooter.speed()
        );
        Robot.armSensorCaliLog.append(s);
        
        SmartDashboard.putString("saved", s);
    }

    public Command armCalibrationTable(int buttonId) {
        return new InstantCommand(() -> saveSensorDataToFile(buttonId));
    }
    public Pose3d getRobotPose() {
        //limelight.setPipeline(VisionConstants.kAprilTagPipeline);
        if(!tagCamera.getLimelight().hasValidTarget()) return null;
        if(targetFound != null)
            targetFound.setBoolean(tagCamera.getLimelight().hasValidTarget());

		Pose3d pose = tagCamera.getLimelight().getBotPose3D();
        //Pose2d pose = poseSupplier.get();

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
            outputs[i] = angleLine.getOutput(inputs[i]);
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
        robotPose = getRobotPose();
        Pose2d currentPose = (robotPose == null) ? null : robotPose.toPose2d();
        if(currentPose == null) return (preserveOldValue ? lastDistance : 0.01);
        Pose3d tagPose;
        
        if(RobotContainer.IsRedSide()) {
            tagPose = getTagPose(4);
        }
        else{
            tagPose = getTagPose(7);
        }
        if(tagPose == null) return (preserveOldValue ? lastDistance : -0.1);

        // lastDistance = currentPose.getTranslation().getDistance(tagPose.getTranslation().toTranslation2d());

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

        double output = NerdyMath.clamp(angleLine.getOutput(distance), ShooterConstants.kFullStowPosition.get(), 20);
        SmartDashboard.putNumber("Vision Angle", output);
        SmartDashboard.putNumber("Vision Distance", distance);
        if(goalAngle != null) 
            goalAngle.setDouble(output);
        output += angleoffset;
        SmartDashboard.putNumber("Angle with Offset", output);
        SmartDashboard.putNumber("Angle Offset", angleoffset);
        lastAngle = output;
        return output + 1.5;
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
				tab.add("Too Low", armCalibrationTable(-1));
                tab.add("Too High", armCalibrationTable(1));
                tab.add("Great!", armCalibrationTable(0));

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
