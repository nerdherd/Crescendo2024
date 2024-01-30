package frc.robot.subsystems.vision.farfuture;

import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.vision.Limelight;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.util.NerdyMath;

public class DriverAssistTeleop implements Reportable {
    private Limelight limelight;
    private String limelightName;

    private PIDController pidAngleDistance = new PIDController(0.3, 0, 0);
    private double calculatedAngledDistancePower;

    private GenericEntry currentAngleDistanceOffset;
    private GenericEntry angleDistanceVelocity;
    
    public DriverAssistTeleop(String name) {
        limelightName = name;
        limelight = Limelight.getInstance(name);

        try {
            limelight = new Limelight(name);
            toggleLight(false);
            changePipeline(pipeline);

            tab.add(name + " inited ", true);
        } catch (Exception e) {
            tab.add(name + " inited ", false);
        }
    }

    public void setTagAlignment(int tagID) {
        if(limelight == null) return;

        int foundId = -1;

        foundId = getAprilTagID();

        if(targetId != null)
            targetId.setInteger(foundId);

        if(tagID != foundId) return;

        double angleDistanceOffset = limelight.getArea_avg() * limelight.getXAngle_avg();

        if(currentAngleDistanceOffset != null) currentAngleDistanceOffset.setDouble(angleDistanceOffset);

        calculatedAngleDistancePower = pidAngleDistance.calculate(angleOffset, 0);

        if(angleDistanceVelocity != null) angleDistanceVelocity.setDouble(calculatedAngledDistancePower);
        
        calculatedAngledDistancePower = NerdyMath.deadband(calculatedAngledDistancePower, -0.2, 0.2);
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        switch (priority) {
            case ALL:
                
                break;

            case MEDIUM:
                break;

            case MINIMAL:
                currentAngleDistanceOffset = tab.add("Angle", 0).getEntry();

                angleDistanceVelocity = tab.add("AngleDistance Velocity", 0 ).getEntry();

                break;
        
            default:
                break;
        }
        
    }
}
