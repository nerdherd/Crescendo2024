package frc.robot.subsystems;

import frc.robot.Constants.BannerSensorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class BannerSensor implements Reportable {
    private final int blackPort;
    private final int whitePort;
    private final DigitalInput bannerSensorBlack;
    private final DigitalInput bannerSensorWhite;

    private boolean noteDetected;

    public BannerSensor() {
        blackPort = BannerSensorConstants.blackPort;
        whitePort = BannerSensorConstants.whitePort;
        bannerSensorBlack = new DigitalInput(blackPort);
        bannerSensorWhite = new DigitalInput(whitePort);

    }

    public boolean noteIntook() {
        // TODO: check if this needs to be inverted
        if(bannerSensorBlack.get()&&!bannerSensorWhite.get()){
            noteDetected = true;
        }
        else if(!bannerSensorBlack.get()&&bannerSensorWhite.get()){
            noteDetected = false;
        }
        else{
            DriverStation.reportError("Fault in banner sensor, error code: ", true);
            noteDetected = false;
        }
        return noteDetected;
    }

    public boolean noteIntookWithoutPolling() {
        return noteDetected;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addBoolean("Note Detected", this::noteIntook);
    }
    
}