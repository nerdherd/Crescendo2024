package frc.robot.subsystems.vision.farfuture;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.NerdyMath;

public class DriverAssistTeleop implements Reportable {
    private Limelight limelight;
    private String limelightName;

    private PIDController pidAngleDistance = new PIDController(0.3, 0, 0);

    private GenericEntry currentAngleDistanceOffset;
    private GenericEntry angleDistanceVelocity;
    private GenericEntry currentTagID;

    ShuffleboardTab tab;
    
    public DriverAssistTeleop(String name) {
        limelightName = name;
        limelight = Limelight.getInstance(name);

        try {
            limelight = new Limelight(name);
            limelight.turnLightOff();
            limelight.setPipeline(VisionConstants.kAprilTagPipeline);

            tab.add(name + " inited ", true);
        } catch (Exception e) {
            tab.add(name + " inited ", false);
        }
    }

    public void setTagAlignment(int tagID) {
        if(limelight == null) return;

        int foundId = -1;

        foundId = limelight.getAprilTagID();

        if(currentTagID != null)
            currentTagID.setInteger(foundId);

        if(tagID != foundId) return;

        double angleDistanceOffset = limelight.getArea_avg() * limelight.getXAngle_avg();

        if(currentAngleDistanceOffset != null) currentAngleDistanceOffset.setDouble(angleDistanceOffset);

        double calculatedAngleDistancePower = pidAngleDistance.calculate(angleDistanceOffset, 0);

        if(angleDistanceVelocity != null) angleDistanceVelocity.setDouble(calculatedAngleDistancePower);
        
        calculatedAngleDistancePower = NerdyMath.deadband(calculatedAngleDistancePower, -0.2, 0.2);
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        tab = Shuffleboard.getTab(limelightName);

        switch (priority) {
            case ALL:
                
                break;

            case MEDIUM:
                break;

            case MINIMAL:
                currentAngleDistanceOffset = tab.add("Angle", 0).getEntry();

                angleDistanceVelocity = tab.add("AngleDistance Velocity", 0 ).getEntry();

                currentTagID = tab.add("Tag ID", 0).getEntry();

                break;
        
            default:
                break;
        }
        
    }
}
