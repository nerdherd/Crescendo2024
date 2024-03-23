package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Arrays;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Reportable;
import frc.robot.util.NerdyMath;
import frc.robot.util.filters.ExponentialSmoothingFilter;

public class Limelight implements Reportable{
    private static Limelight m_Instance;

    private NetworkTable table; // Network table to access Lime Light Values

    // Frequently used entries to store
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private String name = "";

    private double tXList[] = new double[10];
    private double tAList[] = new double[10];
    private double tYList[] = new double[10];
    private double tSKList[] = new double[10];

    public enum LightMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private final int ledMode;

        private LightMode(int ledMode) {
            this.ledMode = ledMode;
        }

        /**
         * Get the current LED Mode
         * 
         * @return the LED mode as int
         */
        public int getLedMode() {
            return ledMode;
        }

        public enum StreamMode {
            // Side by side, Secondary camera is placed in lower right, Primary camera is
            // placed in lower right
            STANDARD(0), MAIN(1), SECONDARY(2);

            private final int mode;

            private StreamMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Stream mode
             * 
             * @return The Stream Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

        public enum CamMode {
            // Camera feed uses vision processing, camera feed increases exposure and
            // disables vision processing
            VISION(0), DRIVER(1);

            private int mode;

            private CamMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Camera mode
             * 
             * @return The Cam Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

        public enum SnapMode {
            // Enables snapshots, Disables snapshots
            DISABLED(0), ENABLED(1);

            private int mode;

            private SnapMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Camera mode
             * 
             * @return The Cam Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

    }

    // Light mode tracking values
    private LightMode currentState = LightMode.OFF;
    private final LightMode LIGHT_ON = LightMode.ON;
    private final LightMode LIGHT_OFF = LightMode.OFF;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the center of the limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = 60.0;

    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    private final NetworkTableEntry m_botPosRed;
    private final NetworkTableEntry m_botPosBlue;
    private final NetworkTableEntry m_camPos;

    public Limelight(String keyN)
    {
        reinitBuffer(); // need to reset everytime change pipeline

        table = NetworkTableInstance.getDefault().getTable(keyN);
        this.name = keyN;

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        // same as Pathfinder's Coordinate System
        //if(RobotContainer.IsRedSide())
        {
            m_botPosRed = table.getEntry("botpose_wpired");
        }
        //else
        {
            m_botPosBlue = table.getEntry("botpose_wpiblue");
        }

        m_camPos = table.getEntry("targetpose_cameraspace");

        setLightState(LIGHT_OFF);
    }

    public void resetLists() {
        //tAList = new double[10]; // do not need it?
        initDoneTA = false;
        //tXList = new double[10]; // do not need it?
        initDoneTX = false;
        //tYList = new double[10]; // do not need it?
        initDoneTY = false;
        initDoneSK = false;
        
        indexTX = 0;
        indexTY = 0;
        indexTA = 0;
        indexSK = 0;
    }

    public double getCamPoseSkew() {
        //double[] botPose = m_botPos.getDoubleArray(new double[6]);
        double[] camPose = m_camPos.getDoubleArray(new double[6]);

        // if(botPose.length != 0) {
        //     SmartDashboard.putNumber("x bot pose: ", botPose[0]);
        //     SmartDashboard.putNumber("y bot pose: ", botPose[1]);
        //     SmartDashboard.putNumber("z bot pose: ", botPose[2]);

        //     SmartDashboard.putNumber("BOT POSE 4: ", botPose[3]);
        //     SmartDashboard.putNumber("BOT POSE 5: ", botPose[4]);
        //     SmartDashboard.putNumber("BOT POSE 6: ", botPose[5]);
        // }

        if(camPose.length >= 6) {
            SmartDashboard.putNumber("x tag pose: ", camPose[0]);
            SmartDashboard.putNumber("y tag pose: ", camPose[1]);
            SmartDashboard.putNumber("z tag pose: ", camPose[2]);

            SmartDashboard.putNumber("TAG POSE 4: ", camPose[3]);
            SmartDashboard.putNumber("TAG POSE 5: ", camPose[4]);
            SmartDashboard.putNumber("TAG POSE 6: ", camPose[5]);

            return camPose[4];

        }
        else {
            return 0;
        }

    }

    int indexSK = 0;
    boolean initDoneSK = false;
    public double getSkew_avg() {
        tSKList[indexSK] = getCamPoseSkew();
        indexSK ++;
        if(indexSK >= tSKList.length) {
            indexSK = 0;
            initDoneSK = true;
        }

        //SmartDashboard.putNumberArray("txFiltered", tXList);

        double SKSum = 0;
        if(initDoneSK) {
            for(int i = 0; i < tSKList.length; i++) {
                SKSum += tSKList[i];
            }
            
            //SmartDashboard.putNumber("TXAverage", TXSum / tXList.length);

            return SKSum / tSKList.length;
        }
        else {
            for(int i = 0; i < indexSK; i++) {
                SKSum += tSKList[i];
            }

            return SKSum / indexSK;
        }
    }


    // public Pose2d getBotPose2D()
    // {
    //     double[] botPose = m_botPos.getDoubleArray(new double[6]);

    //     Translation2d tran2d = new Translation2d(botPose[0], botPose[1]);
    //     Rotation2d r2d = new Rotation2d(Units.degreesToRadians(botPose[5]));
    //     return new Pose2d(tran2d, r2d);
    // }

    public Pose3d getBotPose3D()
    {
        double[] botPose;
        // if(RobotContainer.IsRedSide())
        // botPose = m_botPosRed.getDoubleArray(new double[6]);
        // else 
        botPose = m_botPosBlue.getDoubleArray(new double[6]);

        return new Pose3d(
            new Translation3d(botPose[0], botPose[1], botPose[2]),
            new Rotation3d(Units.degreesToRadians(botPose[3]), Units.degreesToRadians(botPose[4]),
                    Units.degreesToRadians(botPose[5])));
    }

    public ExponentialSmoothingFilter filterPoseX = new ExponentialSmoothingFilter(1.0 / 40);
    public ExponentialSmoothingFilter filterPoseY = new ExponentialSmoothingFilter(1.0 / 40);
    public ExponentialSmoothingFilter filterPoseZ = new ExponentialSmoothingFilter(1.0 / 40);
    private Pose3d currentPoseAvg = null;
    public void updateAvgPose() {
        Pose3d pose = getBotPose3D();
        if(!hasValidTarget()) return;
        if(currentPoseAvg == null) currentPoseAvg = pose;
        else currentPoseAvg = new Pose3d(filterPoseX.calculate(pose.getX()), filterPoseY.calculate(pose.getY()), filterPoseZ.calculate(pose.getZ()), pose.getRotation());
    }
    public Pose3d getBotPose3D_avg() {
        // updateAvgPose();
        return currentPoseAvg;
    }

    public String getName() {
        return this.name;
    }

    public void reinitBuffer() // todo: duplct
    {
        indexTX = 0;
        initDoneTX = false;
        indexTA = 0;
        initDoneTA = false;
        indexTY = 0;
        initDoneTY = false;
    }
    
    /**
     * Are we currently tracking any potential targets
     * 
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasValidTarget() {
        boolean has = NerdyMath.inRange(table.getEntry("tv").getDouble(0), -0.01, 0.01);
        return !has;
    }

    /**
     * Get ID of April Tag
     * 
     * @return Integer ID of April Tag or -1 if no ID exists
     */
    public int getAprilTagID() {
        return (int)table.getEntry("tid").getDouble(-1.0);
    }

    
    /**
     * Horizontal offset from crosshair to target
     * 
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
     *         degrees | LL2: -29.8 to 29.8 degrees)
     */
    int indexTX = 0;
    boolean initDoneTX = false;
    public double getXAngle_avg() {
        tXList[indexTX] = tx.getDouble(0);
        indexTX ++;
        if(indexTX >= tXList.length) {
            indexTX = 0;
            initDoneTX = true;
        }

        //SmartDashboard.putNumberArray("txFiltered", tXList);

        double TXSum = 0;
        if(initDoneTX) {
            for(int i = 0; i < tXList.length; i++) {
                TXSum += tXList[i];
            }
            
            //SmartDashboard.putNumber("TXAverage", TXSum / tXList.length);

            return TXSum / tXList.length;
        }
        else {
            for(int i = 0; i < indexTX; i++) {
                TXSum += tXList[i];
            }

            return TXSum / indexTX;
        }
    }

    public double getXAngleFiltered(int standardDeviationsAway) {
        double newTX = tx.getDouble(0);
        if(initDoneTX) {
            if(NerdyMath.withinStandardDeviation(tAList, standardDeviationsAway, newTX)) {
                tXList[indexTX] = newTX;
                indexTX ++;
                if(indexTX >= tXList.length) {
                    indexTX = 0;
                }
            }
        }
        else {
            tXList[indexTX] = newTX;
            indexTX ++;
            if(indexTX >= tXList.length) {
                indexTX = 0;
                initDoneTX = true;
            }
        }

        double TXSum = 0.0;
        int length = initDoneTX ? tXList.length : indexTX;
        for (int i = 0; i < length; i++) {
            TXSum += tXList[i];
        }
        return TXSum / length;
    }

    public double getXAngle()
    {
        return tx.getDouble(0);
    }

    /**
     * Vertical offset from crosshair to target
     * 
     * @return Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
     *         degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getYAngle() {
        return ty.getDouble(0);
    }

    int indexTY = 0;
    boolean initDoneTY = false;
    public double getYAngle_avg() {
        tYList[indexTY] = ty.getDouble(0);
        indexTY ++;
        if(indexTY >= tYList.length) {
            indexTY = 0;
            initDoneTY = true;
        }

        //SmartDashboard.putNumberArray("txFiltered", tXList);

        double TYSum = 0;
        if(initDoneTY) {
            for(int i = 0; i < tYList.length; i++) {
                TYSum += tYList[i];
            }
            
            //SmartDashboard.putNumber("TXAverage", TXSum / tXList.length);

            return TYSum / tYList.length;
        }
        else {
            for(int i = 0; i < indexTY; i++) {
                TYSum += tYList[i];
            }

            return TYSum / indexTY;
        }
    }

    /**
     * Get the area of the vision tracking box
     * 
     * @return Target Area (0% of image to 100% of image)
     */
    int indexTA = 0;
    boolean initDoneTA = false;
    public double getArea_avg() {
        // double previousValue = -1;
        // if(indexTA > 0 ) previousValue  = tAList[indexTA - 1];
        // if(NerdyMath.deadband(tAList[indexTA], previousValue + 5, previousValue)) tAList[indexTA] = ta.getDouble(0);
        // tAList[indexTA] = NerdyMath.deadband(ta.getDouble(0), previousValue, previousValue);
        tAList[indexTA] = ta.getDouble(0);
        indexTA ++;
        if(indexTA >= tAList.length) {
            indexTA = 0;
            initDoneTA = true;
        }

        //SmartDashboard.putNumberArray("taFiltered", tAList);
        
        double TASum = 0;
        if(initDoneTA) {
            for(int i = 0; i < tAList.length; i++) {
                TASum += tAList[i];
            }
            //SmartDashboard.putNumber("TAAverage", TASum / tAList.length);

            return TASum / tAList.length;
        }
        else {
            for(int i = 0; i < indexTA; i++) {
                TASum += tAList[i];
            }

            return TASum / indexTA;
        }
    }

    public double getAreaFiltered(int standardDeviationsAway) {
        double newArea = ta.getDouble(0);
        if(initDoneTA) {
            if(NerdyMath.withinStandardDeviation(tAList, standardDeviationsAway, newArea)) {
                tAList[indexTA] = newArea;
                indexTA ++;
                if(indexTA >= tAList.length) {
                    indexTA = 0;
                }
            }
        }
        else {
            tAList[indexTA] = newArea;
            indexTA ++;
            if(indexTA >= tAList.length) {
                indexTA = 0;
                initDoneTA = true;
            }
        }

        double TASum = 0.0;
        int length = initDoneTA ? tAList.length : indexTA;
        for (int i = 0; i < length; i++) {
            TASum += tAList[i];
        }
        return TASum / length;
    }

    public double getArea(){
        return ta.getDouble(0);
    }

    /**
     * Rotation of the object
     * 
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew() {
        return table.getEntry("ts").getDouble(0);
    }

    /**
     * Latency in ms of the pipeline
     * 
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image
     *         capture latency.
     */
    public double getLatency_Pipeline() {
        return table.getEntry("tl").getDouble(0);
    }

    public double getLatency_Capture() {
        return table.getEntry("cl").getDouble(0);
    }

    /**
     * The length of the shortest side of the bounding box in pixels
     * 
     * @return Side length of shortest side of the fitted bounding box (pixels)
     */
    public double getShortLength() {
        return table.getEntry("tshort").getDouble(0);
    }

    /**
     * The length of the longest side of the bounding box in pixels
     * 
     * @return Side length of longest side of the fitted bounding box (pixels)
     */
    public double getLongLength() {
        return table.getEntry("tlong").getDouble(0);
    }

    /**
     * The length of the horizontal side of the box (0-320 pixels)
     * 
     * @return Horizontal side length of the rough bounding box (0 - 320 pixels)
     */
    public double getHorizontalLength() {
        return table.getEntry("thor").getDouble(0);
    }

    /**
     * The length of the vertical side of the box (0-320 pixels)
     * 
     * @return Vertical side length of the rough bounding box (0 - 320 pixels)
     */
    public double getVerticalLength() {
        return table.getEntry("tvert").getDouble(0);
    }

    /**
     * Returns the index of the current vision pipeline (0... 9)
     * 
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public int getPipeIndex() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    /**
     * The X-Coordinates of the tracked box
     * 
     * @return Number array of corner x-coordinates
     */
    public double[] getXCorners() {
        return table.getEntry("tcornx").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * The Y-Coordinates of the tracked box
     * 
     * @return Number array of corner y-coordinates
     */
    public double[] getYCorners() {
        return table.getEntry("tcorny").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * Sets the Lime Light LED's
     * 
     * @param mode - LightMode (On, Off, Blinking, or determined by the pipeline)
     */
    public void setLightState(LightMode mode) {
        currentState = mode;
        table.getEntry("ledMode").setNumber(currentState.getLedMode());
    }

    /**
     * Set the Lime Light Camera Mode
     * 
     * @param mode - VISION enables vision processing and decreases exposure, DRIVER
     *             disables vision processing and increases exposure
     */
    public void setCamMode(LightMode.CamMode mode) {
        table.getEntry("camMode").setNumber(mode.getMode());

    }

    /**
     * Sets the limelights current pipeline
     * 
     * @param pipeline The pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Sets the layout of the cameras viewed at 10.56.67.11:5800
     * 
     * @param streamMode - STANDARD is side by side, MAIN is Limelight big with
     *                   secondary camera in bottom right, SECONDARY is vice versa
     */
    public void setStreamMode(LightMode.StreamMode streamMode) {
        table.getEntry("stream").setNumber(streamMode.getMode());
    }

    /**
     * Allow the Lime Light to take snapshots so that it can be tuned off the field
     * 
     * @param mode DISABLED turns Snap Mode off, Enabled turns Snap Mode on
     */
    public void takeSnapshots(LightMode.SnapMode mode) {
        table.getEntry("snapshot").setNumber(mode.getMode());

    }

    /**
     * Toggle the current Lime Light LED's between on and off states
     */
    public void toggleLight() {
        currentState = (currentState == LIGHT_ON) ? LIGHT_OFF : LIGHT_ON;
        setLightState(currentState);
    }

    /**
     * Turn the Lime Light LED's off
     */
    public void turnLightOff() {
        setLightState(LIGHT_OFF);
    }

    /**
     * Turn the Lime Light LED's on
     */
    public void turnLightOn() {
        setLightState(LIGHT_ON);
    }

    /**
     * Get the Lime Light Subsystem instance
     * 
     * @return The Lime Light instance
     */
    public static Limelight getInstance(String keyN) {
        if (m_Instance == null) {
            m_Instance = new Limelight(keyN);
        }

        return m_Instance;
    }

    /**
     * Get the distance of the limelight target
     * 
     * @param h1 - The height of the limelight with respect to the floor
     * @param h2 - The height of the target
     * @param a1 - The mounting angle for the limelight
     * @param a2 - The angle between the limelight angle and the target
     * 
     * @return The distance between the robot and the limelight
     */
    public double getDistance(double h1, double h2, double a1, double a2) {
        return (h2 - h1) / Math.abs(Math.tan(Math.toRadians(a1) + Math.toRadians(a2)));
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }

        ShuffleboardTab tab = Shuffleboard.getTab(this.getName());

        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                tab.addNumber("Horizontal Offset", this::getXAngle);
                tab.addNumber("Vertical Offset", this::getYAngle);
                tab.addNumber("Area", this::getArea);
                tab.addNumber("Skew", this::getSkew);
                tab.addString("XCorners", () -> Arrays.toString(getXCorners()));
                tab.addString("YCorners", () -> Arrays.toString(getYCorners()));
                tab.add("LED ON", Commands.runOnce(() -> this.turnLightOn()));
                tab.add("LED OFF", Commands.runOnce(() -> this.turnLightOff()));
            case MINIMAL:
                tab.addBoolean("HasTarget", this::hasValidTarget);
                break;
        }

    }

    /**
     * Output diagnostics
     */
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                SmartDashboard.putNumber("Horizontal Offset", getXAngle());
                SmartDashboard.putNumber("Vertical Offset", getYAngle());
                SmartDashboard.putNumber("Area", getArea());
                SmartDashboard.putNumber("Skew", getSkew());
                SmartDashboard.putString("XCorners", Arrays.toString(getXCorners()));
                SmartDashboard.putString("YCorners", Arrays.toString(getYCorners()));
            case MINIMAL:
                SmartDashboard.putBoolean("HasTarget", hasValidTarget());
                break;
        }
    }

    /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }

  }
