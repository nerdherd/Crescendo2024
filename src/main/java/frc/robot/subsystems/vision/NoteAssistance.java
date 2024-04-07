package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.util.NerdyMath;

public class NoteAssistance implements Reportable{
    private Limelight limelight;
    private String name;

    private PIDController areaController;
    private PIDController txController;
    private PIDController rotationController;

    private GenericEntry targetFound;
    private GenericEntry currentArea;
    private GenericEntry currentTX;
    private GenericEntry currentTY;

    private GenericEntry forwardSpeed;
    private GenericEntry sidewaysSpeed;
    private GenericEntry rotationSpeed;

    double[] speeds = {0.0, 0.0, 0.0};

    public NoteAssistance(String name) {
        this.name = name;
        ShuffleboardTab tab = Shuffleboard.getTab(name);

        areaController = new PIDController(0.12, 0, 0.004);// todo, tuning pls!!!// 0.12, 0, 0.001 Pre 4/6 Original Values
        txController = new PIDController(0.035, 0, 0.001);// todo, tuning pls!!!
        // 0.025, 0, 0.001 Pre 4/6 Original Values
        rotationController = new PIDController(0.08, 0, 0.006); // d = 0.008

        try { // TODO , we don't need to try-catch
            limelight = new Limelight(name);
            tab.add(name + " inited", true);
            limelight.setPipeline(VisionConstants.kNotePipeline);

        } catch (Exception e) {
            limelight = null;
            tab.add(name + " inited", false);
        }
    }

    private void pidTuning_test() {
        VisionConstants.kPNoteForward.loadPreferences();
        VisionConstants.kINoteForward.loadPreferences();
        VisionConstants.kDNoteForward.loadPreferences();
        VisionConstants.kPNoteSide.loadPreferences();
        VisionConstants.kINoteSide.loadPreferences();
        VisionConstants.kDNoteSide.loadPreferences();
        
        areaController = new PIDController(VisionConstants.kPNoteForward.get(), VisionConstants.kINoteForward.get(), VisionConstants.kDNoteForward.get());
        txController = new PIDController(VisionConstants.kPNoteSide.get(), VisionConstants.kINoteSide.get(), VisionConstants.kDNoteSide.get());
    }

    int dataSampleCount = 0;
    public void reset() {
        limelight.resetLists();
        dataSampleCount = 0;
    }

    private void speedToNote(double targetArea, double targetTX, double targetTY) {
        if(limelight == null) return; // todo, never happens
        
        boolean hasTarget = limelight.hasValidTarget();
        if(targetFound != null)
            targetFound.setBoolean(hasTarget);
        if(hasTarget) 
        {
            double area = limelight.getArea(); // try no avg
            if(currentArea != null)
                currentArea.setDouble(area);
                
            double tx = limelight.getXAngle();
            if(currentTX != null)
                currentTX.setDouble(tx);
                
            double ty = limelight.getYAngle();
            if(currentTY != null)
                currentTY.setDouble(ty);

            // if( area < 0.5 || area > targetArea*1.1 ) // todo, tuning pls!!!
            // {
            //     speeds[0] = speeds[1] = 0; // something is wrong! or filter it out by camera dashboard
            // } 
            // else if( tx < targetTX && tx > -1*targetTX && area > targetArea*0.8 ) // todo, tuning pls!!!
            // {
            //     speeds[0] = speeds[1] = 0; // arrived! good ranges to get the note. cut off here is faster than the pid
            // } 
            // else if( ty < targetTY ) // need a lot of testing here
            // {
            //     speeds[0] = speeds[1] = 0; // stop it otherwise too close to the note
            // } 
            // else
            {
                speeds[0] = 1 * areaController.calculate(area, targetArea);
                speeds[1]= 1 * txController.calculate(tx, 0);

                speeds[0] = NerdyMath.deadband(speeds[0], -0.25, 0.25); // todo, tuning pls!!!
                speeds[1] = NerdyMath.deadband(speeds[1], -0.1, 0.1);// todo, tuning pls!!!
            }
        }
        else
        {
            // todo, move the robot a bit and retry?
            speeds[0] = speeds[1] = 0;
        }
    }

    // no time limit if MaxSamples is less than 0
    // robot can move to any location 
    // for Teleop, PID tuning...
    // double targetArea: the ta to pickup
    // double targetTX: max left tx value to pickup, min right tx value(-1*)
    // double targetTY: the bottom cutoff value
    public void driveToNote(SwerveDrivetrain drivetrain, double targetArea, double targetTX, double targetTY, int maxSamples) {
        // must reset counts before or after call this function!!!
        dataSampleCount++;
        
        if(maxSamples > 0 && dataSampleCount > maxSamples)
            speeds[0] = speeds[1] = 0;
        else
            speedToNote(targetArea, targetTX, targetTY);

        if(forwardSpeed != null)
            forwardSpeed.setDouble(speeds[0]);
        if(sidewaysSpeed != null)
            sidewaysSpeed.setDouble(speeds[1]);
        drivetrain.drive(getForwardSpeed(), getSidewaysSpeed(), 0);
    }

    // no time limit if MaxSamples is less than 0
    // robot runs in limited zone
    // for Auto
    public void driveToNote(SwerveDrivetrain drivetrain, double targetArea, double targetTX, double targetTY, int maxSamples, Pose2d defaultPose) {
        // must reset counts before or after call this function!!!
        dataSampleCount++;

        Pose2d currentPose = drivetrain.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentR = currentPose.getRotation().getDegrees();

        if(defaultPose != null) {
            double defaultX = defaultPose.getX();
            double defaultY = defaultPose.getY();
            double defaultR = defaultPose.getRotation().getDegrees();
        
        

        // todo, need to convert angle to continues value!!! bug
        if(currentX < defaultX-1 || currentX > defaultX+1 ||  // todo, tuning pls
           currentY < defaultY-1 || currentY > defaultY+1 ||
           currentR < NerdyMath.continueAngle(defaultR, defaultR - 30) || currentR > NerdyMath.continueAngle(defaultR, defaultR + 30))
        {
            speeds[0] = speeds[1] = 0; // stop because moved too far
        }
        }
        if(maxSamples > 0 && dataSampleCount > maxSamples)
        {
            speeds[0] = speeds[1] = 0; // stop because running time too long
        }
        else 
        {
            speedToNote(targetArea, targetTX, targetTY);
        }

        if(forwardSpeed != null)
            forwardSpeed.setDouble(speeds[0]);
        if(sidewaysSpeed != null)
            sidewaysSpeed.setDouble(speeds[1]);
        drivetrain.drive(getForwardSpeed(), getSidewaysSpeed(), 0);
    }

    // for the auto 
    // a min running time is required by minSamples 
    // double targetArea: the ta to pickup
    // double targetTX: max left tx value to pickup, min right tx value(-1*)
    // double targetTY: the bottom cutoff value
    public Command driveToNoteCommand(SwerveDrivetrain drivetrain, double targetArea, double targetTX, double targetTY, int minSamples, int maxSamples, Pose2d defaultPose) {
        return Commands.sequence(
            Commands.runOnce(() -> reset()),
            Commands.run( () -> driveToNote(drivetrain, targetArea, targetTX, targetTY, maxSamples, defaultPose))// todo, tuning pls!!!
                .until(() -> (dataSampleCount >= minSamples && 
                    Math.abs(getForwardSpeed()) <= 0.1 && 
                    Math.abs(getSidewaysSpeed()) <= 0.1) )// todo, tuning pls!!!
        );
    }

    boolean foundNote = false;
    public void seekForFarNote(SwerveDrivetrain drivetrain, double targetArea, double targetTX, double targetTY, int maxSamples, Pose2d defaultPose) {
        // must reset counts before or after call this function!!!
        dataSampleCount++;

        Pose2d currentPose = drivetrain.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentR = currentPose.getRotation().getDegrees();

        double defaultX = defaultPose.getX();
        double defaultY = defaultPose.getY();
        double defaultR = defaultPose.getRotation().getDegrees();

        // todo, need to convert angle to continues value!!! bug
        if(currentX < defaultX-1 || currentX > defaultX+1 ||  // todo, tuning pls
           currentY < defaultY-1 || currentY > defaultY+1 ||
           currentR < defaultR-30 || currentR > defaultR+30 )
        {
            speeds[0] = speeds[1] = 0; // stop because moved too far
            foundNote = false;
        }
        else if(maxSamples > 0 && dataSampleCount > maxSamples)
        {
            speeds[0] = speeds[1] = 0; // stop because running time too long
            foundNote = false;
        }
        else 
        {
            speedToNote(targetArea, targetTX, targetTY);
            foundNote = true;
        }

        if(forwardSpeed != null)
            forwardSpeed.setDouble(speeds[0]);
        if(sidewaysSpeed != null)
            sidewaysSpeed.setDouble(speeds[1]);
        drivetrain.drive(getForwardSpeed(), getSidewaysSpeed(), 0);
    }
    
    // for the auto 
    // a min running time is required by minSamples 
    public Command seekForFarNoteCommand(SwerveDrivetrain drivetrain, double targetArea, int minSamples, int maxSamples, Pose2d defaultPose) {
        return Commands.sequence(
            Commands.runOnce(() -> reset()),
            Commands.run( () -> seekForFarNote(drivetrain, targetArea, 0, 0.1, maxSamples, defaultPose))// todo, tuning pls!!!
                .until(() -> ((foundNote == true || dataSampleCount >= minSamples) && 
                    Math.abs(getForwardSpeed()) <= 0.1 && 
                    Math.abs(getSidewaysSpeed()) <= 0.1 
                    ) )// todo, tuning pls!!!
        );
    }

    public void calculateTranslationSpeeds(double targetTA, double targetTX, double targetTY, int maxSamples) {
        dataSampleCount++;
        if(dataSampleCount > maxSamples && maxSamples > 0) {
            speeds[0] = 0;
            speeds[1] = 0;
            return;
        }

        boolean hasTarget = limelight.hasValidTarget();
        if(targetFound != null)
            targetFound.setBoolean(hasTarget);

        if(!hasTarget) {
            speeds[0] = 0;
            speeds[1] = 0;
            return;
        }

        double currentTA = limelight.getArea_avg();
        double currentTX = limelight.getXAngle_avg();
        double currentTY = limelight.getYAngle_avg();

        if(NerdyMath.inRange(currentTA, targetTA - 0.2, targetTA * 1.1)) 
            speeds[0] = 0;
        // else if(NerdyMath.inRange(currentTY, targetTY - 0.1, targetTY + 0.1))
        //     speeds[0] = 0;
        if(NerdyMath.inRange(currentTX, targetTX - 0.1, targetTX + 0.1))
            speeds[1] = 0;

        speeds[0] = 1 * areaController.calculate(currentTA, targetTA);
        speeds[1] = 1 * txController.calculate(currentTX, targetTX);

        if(forwardSpeed != null)
            forwardSpeed.setDouble(speeds[0]);
        if(sidewaysSpeed != null)
            sidewaysSpeed.setDouble(speeds[1]);
    }

    private void pidTurnTuning_test() {
        VisionConstants.kPNoteAngle.loadPreferences();
        VisionConstants.kINoteAngle.loadPreferences();
        VisionConstants.kDNoteAngle.loadPreferences();

        rotationController.setPID(VisionConstants.kPNoteAngle.get(), VisionConstants.kINoteAngle.get(), VisionConstants.kDNoteAngle.get());
    }

    public void calculateRotationSpeed(double targetTX, int maxSamples) {
        pidTurnTuning_test();
        dataSampleCount++;
        if(dataSampleCount > maxSamples && maxSamples > 0) {
            speeds[2] = 0;
            return;
        }
        
        boolean hasTarget = limelight.hasValidTarget();
        if(targetFound != null)
            targetFound.setBoolean(hasTarget);

        if(!hasTarget) {
            speeds[2] = 0;
            return;
        }

        double currentTX = limelight.getXAngle_avg();
        
        if(NerdyMath.inRange(currentTX, targetTX - 0.1, targetTX + 0.1)) {
            speeds[2] = 0;
            return;
        }

        speeds[2] = 1 * rotationController.calculate(currentTX, targetTX); // change to negative 1 if opposite

        if(rotationSpeed != null)
            rotationSpeed.setDouble(speeds[2]);
    }

    /**
     * runs until reaches targetTX or reaches maximumSamples
     * @param drivetrain
     * @param targetTX
     * @param minSamples minimum samples to have before command stops
     * @param maxSamples maximum samples before command stops, 0 if just want to run reach targetTX
     * @return
     */
    public Command turnToNoteCommand(SwerveDrivetrain drivetrain, double targetTX, int minSamples, int maxSamples) {
        return Commands.sequence(
            Commands.runOnce(() -> reset()),
            Commands.run(
                () -> {
                    calculateRotationSpeed(targetTX, maxSamples);
                    drivetrain.drive(0, 0, getRotationSpeed());
                }  
            ).until(() ->
                    Math.abs(speeds[2]) == 0 &&
                    dataSampleCount >= minSamples
                )
        );
    }

    private double getForwardSpeed() { return speeds[0]; }
    private double getSidewaysSpeed() { return speeds[1]; }
    public double getRotationSpeed() { return speeds[2]; }

    public boolean hasTarget() { return limelight.hasValidTarget(); }

    public void setLight(boolean lightModeOn) {
        if(lightModeOn) limelight.setLightState(LightMode.ON);
        else limelight.setLightState(LightMode.OFF);
    }

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
       
            // try{
            //     tab.addCamera(name + ": Stream", name, VisionConstants.kLimelightFrontIP);
            // }catch(Exception e){};

            case MEDIUM:
                currentArea = tab.add("Area", 0)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();
                
                currentTX = tab.add("Tx", 0)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();

                currentTY = tab.add("Ty", 0)
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

                rotationSpeed = tab.add("Rotation Speed", 0)
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
