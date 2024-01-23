// import frc.robot.subsystems.*;
// import com.revrobotics.ColorMatch;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorSensorV3;

// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;

// public class ColorSensor{
//     private final I2C.Port i2cPort;
//     private final ColorSensorV3 colorSensor;

//     private boolean noteIntook;

//     private int proximity;

//     public ColorSensor(){
//         i2cPort = I2C.Port.kOnboard;
//         colorSensor = new ColorSensorV3(i2cPort);
//     }

//     public void noteIntook() {
      
//         Color detectedColor = colorSensor.getColor();
//         proximity = colorSensor.getProximity();
        
//   }
// }