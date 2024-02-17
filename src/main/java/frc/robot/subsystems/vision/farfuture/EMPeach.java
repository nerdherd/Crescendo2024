package frc.robot.subsystems.vision.farfuture;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelperUser;
import frc.robot.util.NerdyMath;

/**
 * Subsystem that uses Limelight for vision
 */
public class EMPeach implements Reportable{
    // Limelight Specific Variables
    private Limelight limelight;
    private LimelightHelperUser limelightHelperUser;
    private String limelightName;

    // AprilTag Specific Variables
    private AprilTagFieldLayout layout;

    // PID Controllers
    private static PIDController pidTA = new PIDController(0.3, 0, 0);
    private static PIDController pidTX = new PIDController(0.05, 0, 0);
    private static PIDController pidTY = new PIDController(0.1, 0, 0);
    private static PIDController pidSkew = new PIDController(0.06, 0, 0);

    double[] speeds = {0.0, 0.0, 0.0};

    private GenericEntry hasTarget;
    private GenericEntry pipeline;
    private GenericEntry poseString;
    
    /**
     * Makes a new EMPeach to utilize vision
     * @param name name of the limelight
     */
    public EMPeach(String name) {
        limelightName = name;

        try {
            limelight = new Limelight(name);
            limelightHelperUser = new LimelightHelperUser(name);
            toggleEMP(true);
            changeEMPType(VisionConstants.kAprilTagPipeline);

            SmartDashboard.putBoolean("Limelight: " + name + " inited", true);
            SmartDashboard.putBoolean("LimelightHelper inited", true);
        } catch (Exception e) {
            SmartDashboard.putBoolean("limelight-" + name + " inited", false);
            SmartDashboard.putBoolean("LimelightHelper inited", false);
        }
        
        try {
            layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json");
            SmartDashboard.putBoolean("AprilTag Layout Found", true);
        } catch (Exception e) {
            SmartDashboard.putBoolean("AprilTag Layout Found", false);
        }
    }

    /**
     * Sets the limelight pipeline
     * @param pipeline
     */
    public void changeEMPType(int pipeline) {
        if (this.pipeline != null) {
            this.pipeline.setInteger(pipeline);
        }
        limelight.setPipeline(pipeline);
    }

    /**
     * Toggles the limelight light on or off
     * @param lightModeOn
     */
    public void toggleEMP(boolean lightModeOn) {
        if(lightModeOn) limelight.setLightState(LightMode.ON);
        else limelight.setLightState(LightMode.OFF);
    }

    /**
     * @return the area of the target
     */
    public double getEMPRadius() {
        return limelight.getArea();
    }

    /**
     * @return robot position based on vision and apriltags
     */
    public Pose3d getCurrentGrassTile() {
        if(limelight == null) return null;
        if(limelightHelperUser == null) return null;

        changeEMPType(VisionConstants.kAprilTagPipeline);
        if(!limelight.hasValidTarget()) return null;
        if(hasTarget != null) {
            hasTarget.setBoolean(true);
        }
        if(poseString != null) {
            poseString.setString(limelightHelperUser.getPose3d().toString());
        }
        return limelightHelperUser.getPose3d(); // im hoping this is with the bottom left corner of the field as the origin
    }

    /**
     * @param x targeted x position
     * @param y targeted y position
     * @return the Transform3d that maps the current position to the desired coordinates
     */
    public Transform3d getTransformToGrassTile(double x, double y) {
        Pose3d currentPose = getCurrentGrassTile();
        if(currentPose == null) return null;
        Pose3d targetPose = new Pose3d(new Translation3d(x, y, 0), new Rotation3d());

        return targetPose.minus(currentPose);
    }

    /**
     * @param ID AprilTag ID
     * @return the Pose3d of a specific AprilTag
     */
    public Pose3d getZombieTile(int ID) {
        if(ID < 1 || ID > 16) return null;
        if(layout == null) return null;
        Optional<Pose3d> tagPose = layout.getTagPose(ID);
        if(tagPose.isEmpty()) return null;
        
        return tagPose.get();
    }

    /**
     * @return the distance in meters from the closest note based on Limelight ty
     */
    public double getDistanceFromImp() {
        changeEMPType(3); // change to note pipeline
        double notePitch = limelight.getYAngle();
        return (VisionConstants.kNoteHeightMeters - VisionConstants.kFrontCameraHeightMeters)
            / Math.tan(VisionConstants.kCameraPitchRadians + Math.toRadians(notePitch));
    }

    /**
     * Gets the driving powers to go to a note
     * @param metersAwayFromTarget how many meters to stop from the target
     * @return double[] in the format {forwardPower, sidewaysPower, angledPower}
     */
    public double[] getMovePowerToImp(double areaGoal) {
        double[] powers = {0.0, 0.0, 0.0};

        // double range = getDistanceFromImp();
        double area = limelight.getArea();
        double txOffset = 0 - limelight.getXAngle();
        double skewOffset = 0 - limelight.getSkew();
        SmartDashboard.putNumber("AA", area);
        SmartDashboard.putNumber("TXXXX", txOffset);
        SmartDashboard.putNumber("SKEEW", skewOffset);

        speeds[0] = pidTA.calculate(area, areaGoal);
        speeds[1] = pidTX.calculate(txOffset, 0);
        speeds[2] = pidSkew.calculate(skewOffset, 0);

        return powers;
    }


    public void driveToImp(double targetTA, double targetTX, double targetskew) {
        changeEMPType(0);

        double taOffset = targetTA - limelight.getArea();
        double txOffset = targetTX - limelight.getXAngle();
        double skewOffset = targetskew - limelight.getSkew();
    
        SmartDashboard.putNumber("TA Offset: ", taOffset);
        SmartDashboard.putNumber("TX Offset: ", txOffset);
        SmartDashboard.putNumber("Skew Offset: ", skewOffset);

        speeds[0] = -1 * pidTA.calculate(taOffset, 0);
        speeds[1]= -1 * pidTX.calculate(txOffset, 0);
        speeds[2] = 1 * pidSkew.calculate(skewOffset, 0);

        SmartDashboard.putNumber("FS", speeds[0]);
        SmartDashboard.putNumber("SS", speeds[1]);
        SmartDashboard.putNumber("AS", speeds[2]);

        speeds[0] = NerdyMath.deadband(speeds[0], -0.15, 0.15);
        speeds[1] = NerdyMath.deadband(speeds[1], -0.75, 0.75);
        // speeds[2] = NerdyMath.deadband(speeds[2], -1, 1);
    }

    public double[] getChargeSpeeds() {
        return speeds;
    }

    /**
     * @param degrees angle of the shooter in degrees
     * @return a Transform3d mapping the currentPos to the ideal distance based on the shooter angle
     */
    public Transform3d getIdealZombieDistance(double degrees) {
        Pose3d poseToAvoid = getZombieTile(limelight.getAprilTagID());

        //just using trig for now, theres probably a better formula tho
        double distance = poseToAvoid.getZ() / Math.tan(Math.toRadians(degrees));

        return new Transform3d(new Translation3d(distance, 0, 0), new Rotation3d());
    }

    /**
     * @return the shooter angle in degrees based on how far the robot is from the apriltag, -1 if error
     */
    public double getEMPeachThrowingAngle() {
        Pose3d currentPose = getCurrentGrassTile();
        if(currentPose == null) return -1;
        Pose3d poseToAvoid = getZombieTile(limelight.getAprilTagID());

        return Math.toDegrees(Math.atan(poseToAvoid.getZ() / Math.abs(poseToAvoid.getX() - currentPose.getX())));
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
                hasTarget = tab.add("Target Found", false)
                    .withPosition(4, 3)
                    .withSize(2, 1)
                    .getEntry();
                pipeline = tab.add("Current Pipeline", 4)
                    .withPosition(6, 0)
                    .withSize(2, 1)
                    .getEntry();

            case MINIMAL:   
                // tab.addCamera(limelightName + ": Stream", limelightName, limelightName + ".local:5802")
                //     .withPosition(0, 0)
                //     .withSize(6, 3);


                poseString = tab.add("Robot Pose", "null")
                    .withPosition(0, 3)
                    .withSize(4, 1)
                    .getEntry();
                
                tab.addNumber("tA", this::getEMPRadius);

            case OFF:
                break;
            
        }
    }
    
}
