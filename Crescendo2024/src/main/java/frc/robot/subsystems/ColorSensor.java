package frc.robot.subsystems;

import frc.robot.Constants.ColorSensorConstants;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;


public class ColorSensor{
    private final I2C.Port i2cPort;
    private final ColorSensorV3 colorSensor;

    private int proximity;

    public ColorSensor(){
        i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
    }

    public boolean noteIntook() {
        
        proximity = colorSensor.getProximity();
        if (proximity < ColorSensorConstants.inProximity){
            return true;
        } 
        else {
            return false;
        }
    
  }
}