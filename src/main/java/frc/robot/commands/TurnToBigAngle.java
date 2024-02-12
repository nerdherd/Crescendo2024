package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class TurnToBigAngle extends Command {
    private double targetAngle;
    private SwerveDrivetrain swerveDrive;
    private PIDController pidController;
    // private SlewRateLimiter limiter;

    /**
     * Construct a new TurnToAngle Command
     * @param targetAngle   Target angle (degrees)
     * @param swerveDrive   Swerve drivetrain to rotate
     * @param period        Time between each calculation (default 20ms)
     */
    public TurnToBigAngle(double targetAngle, SwerveDrivetrain swerveDrive, double period) {
        this.targetAngle = targetAngle;
        this.swerveDrive = swerveDrive;

        this.pidController = new PIDController(
            SwerveAutoConstants.kPTurnToAngle.get(), 
            SwerveAutoConstants.kITurnToAngle.get(), 
            SwerveAutoConstants.kDTurnToAngle.get(), 
            period);
        
        this.pidController.setTolerance(
            SwerveAutoConstants.kTurnToAnglePositionToleranceAngle, 
            SwerveAutoConstants.kTurnToAngleVelocityToleranceAnglesPerSec * period);
        
        this.pidController.enableContinuousInput(0, 360);
        
        // this.limiter = new SlewRateLimiter(Math.PI / 4);

        addRequirements(swerveDrive);
    }

    public TurnToBigAngle(double targetAngle, SwerveDrivetrain swerveDrive) {
        // Default period is 20 ms
        this(targetAngle, swerveDrive, 0.02);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Calculate turning speed with PID
        double turningSpeed = pidController.calculate(swerveDrive.getImu().getHeading(), targetAngle);
        turningSpeed = Math.toRadians(turningSpeed);

        turningSpeed = NerdyMath.clamp(
            turningSpeed, 
            -SwerveDriveConstants.kTurnToBigAngleMaxAngularSpeedRadiansPerSecond, 
            SwerveDriveConstants.kTurnToBigAngleMaxAngularSpeedRadiansPerSecond);
        // SmartDashboard.putNumber("Turning speed limited", turningSpeed);
        
        // Convert speed into swerve states
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, turningSpeed, swerveDrive.getImu().getRotation2d());
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Set swerve states
        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
