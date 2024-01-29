package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class NoteAssistance implements Reportable{
    private Limelight limelight;
    private String name;

    private PIDController areaController;
    private PIDController txController;
    private PIDController skewController;

    private GenericEntry targetFound;
    private GenericEntry currentArea;
    private GenericEntry currentTX;
    private GenericEntry currentSkew;
    private GenericEntry forwardSpeed;
    private GenericEntry sidewaysSpeed;
    private GenericEntry angularSpeed;

    double[] speeds = {0.0, 0.0, 0.0};

    //ComplexWidget v;

    public NoteAssistance(String name) {
        this.name = name;
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        //TODO set pid constants to preferences for easy tuning
        areaController = new PIDController(0.34, 0, 0);
        txController = new PIDController(0.08, 0, 0.006);
        skewController = new PIDController(0.05, 0, 0);

        try {
            limelight = new Limelight(name);
            tab.add(name + " inited", true);
            limelight.setPipeline(VisionConstants.kNotePipeline);

        } catch (Exception e) {
            limelight = null;
            tab.add(name + " inited", false);
        }
        //initShuffleboard(LOG_LEVEL.ALL);
    }

    public void pidTuning_test() {
        VisionConstants.kPNoteForward.loadPreferences();
        VisionConstants.kINoteForward.loadPreferences();
        VisionConstants.kDNoteForward.loadPreferences();
        VisionConstants.kPNoteSide.loadPreferences();
        VisionConstants.kINoteSide.loadPreferences();
        VisionConstants.kDNoteSide.loadPreferences();
        VisionConstants.kPNoteAngle.loadPreferences();
        VisionConstants.kINoteAngle.loadPreferences();
        VisionConstants.kDNoteAngle.loadPreferences();
        
        areaController = new PIDController(VisionConstants.kPNoteForward.get(), VisionConstants.kINoteForward.get(), VisionConstants.kDNoteForward.get());
        txController = new PIDController(VisionConstants.kPNoteSide.get(), VisionConstants.kINoteSide.get(), VisionConstants.kDNoteSide.get());
        skewController = new PIDController(VisionConstants.kPNoteAngle.get(), VisionConstants.kINoteAngle.get(), VisionConstants.kDNoteAngle.get());
    }

    public void reset() {
        limelight.resetLists();
    }

    public void speedToNote(double targetArea, double targetTX, double targetSkew) {
        if(limelight == null) return;

        // Need to add a max TA for over-image ....

        
        boolean hasTarget = limelight.hasValidTarget();
        if(targetFound != null)
            targetFound.setBoolean(hasTarget);
        if(!hasTarget) return;

        // pidTuning_test();

        // double area = limelight.getAreaFiltered(10);
        double area = limelight.getArea_avg();
        if(currentArea != null)
            currentArea.setDouble(area);
        // SmartDashboard.putNumber("area", area);
        // double tx = limelight.getXAngleFiltered(3);
        double tx = limelight.getXAngle_avg();
        if(currentTX != null)
            currentTX.setDouble(tx);
        // SmartDashboard.putNumber("tx", tx);
        double skew = limelight.getSkew();
        if(currentSkew != null)
            currentSkew.setDouble(skew);
        // SmartDashboard.putNumber("skew", skew);

        speeds[0] = 1 * areaController.calculate(area, targetArea);
        speeds[1]= 1 * txController.calculate(tx, targetTX);
        speeds[2] = 1 * skewController.calculate(skew, targetSkew);

        if(forwardSpeed != null)
            forwardSpeed.setDouble(speeds[0]);
        if(sidewaysSpeed != null)
            sidewaysSpeed.setDouble(speeds[1]);
        if(angularSpeed != null)
            angularSpeed.setDouble(speeds[2]);
        // SmartDashboard.putNumber("fs", speeds[0]);
        // SmartDashboard.putNumber("ss", speeds[1]);
        // SmartDashboard.putNumber("as", speeds[2]);

        speeds[0] = NerdyMath.deadband(speeds[0], -0.2, 0.2);
        speeds[1] = NerdyMath.deadband(speeds[1], -0.35, 0.35);
        speeds[2] = NerdyMath.deadband(speeds[2], -0.5, 0.5);
    }

    public void driveToNote(SwerveDrivetrain drivetrain, double targetArea, double targetTX, double targetSkew) {
        speedToNote(targetArea, targetTX, targetSkew);
        drivetrain.drive(getForwardSpeed(), getSidewaysSpeed(), 0);
    }

    public Command driveToNoteCommand(SwerveDrivetrain drivetrain, double targetArea) {
        return Commands.sequence(
            Commands.runOnce(() -> limelight.resetLists()),
            Commands.run(
                () -> driveToNote(drivetrain, targetArea, 0, 0)
            ).until(() -> Math.abs(getForwardSpeed()) <= 0.1 && Math.abs(getSidewaysSpeed()) <= 0.1)
        );
    }

    public double getForwardSpeed() { return speeds[0]; }
    public double getSidewaysSpeed() { return speeds[1]; }
    public double getAngularSpeed() { return speeds[2]; }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
    }

    public void initShuffleboard(LOG_LEVEL priority) {
        if (priority == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        //the lack of "break;"'s is intentional
        switch (priority) {
            case ALL:
       
            try{
                tab.addCamera(name + ": Stream", name, VisionConstants.kLimelightFrontIP);
            }catch(Exception e){};

            case MEDIUM:
                currentArea = tab.add("Area", 0)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();
                
                currentTX = tab.add("TX", 0)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

                currentSkew = tab.add("Angle", 0)
                .withPosition(2, 2)
                .withSize(2, 1)
                .getEntry();

            case MINIMAL:   
                forwardSpeed = tab.add("Forward Speed", 0)
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();
                
                sidewaysSpeed = tab.add("Sideways Speed", 0)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();

                angularSpeed = tab.add("Angular Speed", 0)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();
                
                targetFound = tab.add("Target Found", false)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();

            case OFF:
                break;
            
        }
    }
    
}
