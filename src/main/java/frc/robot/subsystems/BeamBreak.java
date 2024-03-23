// package frc.robot.subsystems;

// import frc.robot.Constants.BeamBreakConstants;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


// public class BeamBreak implements Reportable {
//     private final int port;
//     private final DigitalInput beamBreak;

//     public BeamBreak() {
//         port = BeamBreakConstants.beamBreakPort;
//         beamBreak = new DigitalInput(port);
//     }

//     public boolean noteIntook() {
//         // TODO: check if this needs to be inverted
//         return beamBreak.get();
//     }

//     @Override
//     public void reportToSmartDashboard(LOG_LEVEL priority) {}

//     @Override
//     public void initShuffleboard(LOG_LEVEL priority) {
//         ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
//         tab.addBoolean("Note Detected", this::noteIntook);
//     }
    
// }