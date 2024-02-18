package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.util.filters.DeadbandFilter;
import frc.robot.util.filters.Filter;
import frc.robot.util.filters.FilterSeries;
import frc.robot.util.filters.ScaleFilter;
import frc.robot.filters.OldDriverFilter2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;

import static frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveJoystickCommand extends Command {
    private final SwerveDrivetrain swerveDrive;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final Supplier<Boolean> towSupplier, precisionSupplier;
    private final Supplier<Double> desiredAngle;
    private final Supplier<Boolean> turnToAngleSupplier;
    private final PIDController turnToAngleController;
    private Filter xFilter, yFilter, turningFilter;

    public static double targetAngle = 0;

    public enum DodgeDirection {
        LEFT,
        RIGHT,
        NONE
    }

    /**
     * Construct a new SwerveJoystickCommand
     * 
     * @param swerveDrive           The Swerve Drive subsystem
     * @param xSpdFunction          A supplier returning the desired x speed
     * @param ySpdFunction          A supplier returning the desired y speed
     * @param turningSpdFunction    A supplier returning the desired turning speed
     * @param fieldOrientedFunction A boolean supplier that toggles field oriented/robot oriented mode.
     * @param towSupplier           A boolean supplier that toggles the tow mode.
     * @param precisionSupplier     A boolean supplier that toggles the precision mode.
     */
    public SwerveJoystickCommand(SwerveDrivetrain swerveDrive,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> towSupplier, 
            Supplier<Boolean> precisionSupplier,
            Supplier<Boolean> turnToAngleSupplier,
            Supplier<Double> desiredAngleSupplier
        ) {
        this.swerveDrive = swerveDrive;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.towSupplier = towSupplier;
        this.precisionSupplier = precisionSupplier;
        
        this.turnToAngleSupplier = turnToAngleSupplier;
        this.desiredAngle = desiredAngleSupplier;

        this.xFilter = new OldDriverFilter2(
            ControllerConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.yFilter = new OldDriverFilter2(
            ControllerConstants.kDeadband, 
            kMinimumMotorOutput,
            kTeleDriveMaxSpeedMetersPerSecond, 
            kDriveAlpha, 
            kTeleMaxAcceleration, 
            kTeleMaxDeceleration);
        this.turningFilter = new FilterSeries(
            new DeadbandFilter(ControllerConstants.kRotationDeadband),
            new ScaleFilter(kTeleDriveMaxAngularSpeedRadiansPerSecond)
            );
        

        this.turnToAngleController = new PIDController(
            SwerveAutoConstants.kPTurnToAngle.get(),
            SwerveAutoConstants.kITurnToAngle.get(),
            SwerveAutoConstants.kDTurnToAngle.get()
            );

        // this.turnToAngleController = new PIDController(
        //     SwerveAutoConstants.kPTurnToAngle, 
        //     SwerveAutoConstants.kITurnToAngle, 
        //     SwerveAutoConstants.kDTurnToAngle, 
        //     0.02);
        
        this.turnToAngleController.setTolerance(
            SwerveAutoConstants.kTurnToAnglePositionToleranceAngle, 
            SwerveAutoConstants.kTurnToAngleVelocityToleranceAnglesPerSec * 0.02);
    

        this.turnToAngleController.enableContinuousInput(0, 360);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (towSupplier.get()) {
            swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
            return;
        }

        // get speeds
        double turningSpeed;
        double xSpeed = xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();

        double filteredTurningSpeed;
        double filteredXSpeed = xFilter.calculate(xSpeed);
        double filteredYSpeed = yFilter.calculate(ySpeed);

        if (turnToAngleSupplier.get()) {
            double tempAngle = desiredAngle.get();
            if (tempAngle != 1000.0) {
                targetAngle = tempAngle;
            } else {
                targetAngle = ((targetAngle + turningSpdFunction.get() % 360) + 360) % 360;
                SwerveDriveConstants.kPThetaTeleop.loadPreferences();
                SwerveDriveConstants.kIThetaTeleop.loadPreferences();
                SwerveDriveConstants.kDThetaTeleop.loadPreferences();
                turnToAngleController.setP(SwerveDriveConstants.kPThetaTeleop.get());
                turnToAngleController.setI(SwerveDriveConstants.kIThetaTeleop.get());
                turnToAngleController.setD(SwerveDriveConstants.kDThetaTeleop.get());
            }
            turningSpeed = turnToAngleController.calculate(swerveDrive.getImu().getHeading(), targetAngle);
            turningSpeed = Math.toRadians(turningSpeed);
            turningSpeed = MathUtil.clamp(
                turningSpeed, 
                -SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond, 
                SwerveDriveConstants.kTurnToAngleMaxAngularSpeedRadiansPerSecond);
            SmartDashboard.putNumber("Turning Speed", turningSpeed);
            
            filteredTurningSpeed = turningSpeed;
        }
        else {
            // Manual turning
            turningSpeed = turningSpdFunction.get();
            turningSpeed *= -1;
            filteredTurningSpeed = turningFilter.calculate(turningSpeed);
        }


        if (precisionSupplier.get()) {
            filteredXSpeed /= 4;
            filteredYSpeed /= 4;
            // filteredTurningSpeed /= 4; // Also slows down the turn to angle speed
        }
        
        ChassisSpeeds chassisSpeeds;
        // Check if in field oriented mode
        if (!fieldOrientedFunction.get()) {
            swerveDrive.setDriveMode(DRIVE_MODE.FIELD_ORIENTED);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed, 
                swerveDrive.getImu().getRotation2d());
        } else {
            swerveDrive.setDriveMode(DRIVE_MODE.ROBOT_ORIENTED);
            chassisSpeeds = new ChassisSpeeds(
                filteredXSpeed, filteredYSpeed, filteredTurningSpeed);
        }

        SwerveModuleState[] moduleStates;

        moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Calculate swerve module states
        swerveDrive.setModuleStates(moduleStates);

        
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}