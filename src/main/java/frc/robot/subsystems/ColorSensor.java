// package frc.robot.subsystems;

// import frc.robot.Constants.ColorSensorConstants;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// import com.revrobotics.ColorSensorV3;


// public class ColorSensor implements Reportable {
//     private final I2C.Port i2cPort;
//     private final ColorSensorV3 colorSensor;

//     private int proximity;
//     private boolean noteDetected = false;

//     public ColorSensor() {
//         i2cPort = I2C.Port.kOnboard;
//         colorSensor = new ColorSensorV3(i2cPort);
//     }

//     public boolean noteIntook() {
//         proximity = colorSensor.getProximity();
//         noteDetected = proximity > ColorSensorConstants.inProximity;
//         return noteDetected;
//     }

//     public boolean noteIntookWithoutPolling() {
//         return noteDetected;
//     }

//     @Override
//     public void reportToSmartDashboard(LOG_LEVEL priority) {}

//     @Override
//     public void initShuffleboard(LOG_LEVEL priority) {
//         ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
//         tab.addBoolean("Note Detected", this::noteIntookWithoutPolling);
//         tab.addNumber("Distance Detected (When last polled)", () -> proximity);
//     }
    
// }