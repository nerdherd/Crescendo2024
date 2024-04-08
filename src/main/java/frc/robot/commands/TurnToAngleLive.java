package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class TurnToAngleLive extends Command {
    private Supplier<Double> targetAngle;
    private SwerveDrivetrain swerveDrive;
    private PIDController pidController;
    private double target;
    private double tolerance;
    // private SlewRateLimiter limiter;

    /**
     * Construct a new TurnToAngle Command
     * @param targetAngle   Target angle (degrees)
     * @param swerveDrive   Swerve drivetrain to rotate
     * @param period        Time between each calculation (default 20ms)
     */
    public TurnToAngleLive(Supplier<Double> targetAngle, SwerveDrivetrain swerveDrive, double period, double tolerance) {
        this.targetAngle = targetAngle;
        this.swerveDrive = swerveDrive;

        this.pidController = new PIDController(
            SwerveDriveConstants.kPThetaTeleop.get(), 
            SwerveDriveConstants.kIThetaTeleop.get(), 
            SwerveDriveConstants.kDThetaTeleop.get(), 
            period);
        
        this.pidController.setTolerance(
            SwerveAutoConstants.kTurnToAnglePositionToleranceAngle, 
            SwerveAutoConstants.kTurnToAngleVelocityToleranceAnglesPerSec * period);
        
        this.pidController.enableContinuousInput(0, 360);
        
        // this.limiter = new SlewRateLimiter(Math.PI / 4);

        addRequirements(swerveDrive);
    }

    public TurnToAngleLive(Supplier<Double> targetAngle, SwerveDrivetrain swerveDrive, double angleTolerance) {
        // Default period is 20 ms
        this(targetAngle, swerveDrive, 0.02, angleTolerance);
    }

    @Override
    public void initialize() {
        target = targetAngle.get();
    }

    @Override
    public void execute() {
        // Calculate turning speed with PID
        double turningSpeed = pidController.calculate(swerveDrive.getImu().getHeading(), target);
        turningSpeed += Math.signum(turningSpeed) * SwerveAutoConstants.kTurnToAngleFeedForwardDegreesPerSecond;
        turningSpeed = Math.toRadians(turningSpeed);

        turningSpeed = NerdyMath.clamp(
            turningSpeed, 
            -SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond, 
            SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond);
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
        return Math.abs(swerveDrive.getImu().getHeading() - target) < tolerance;
    }
}
