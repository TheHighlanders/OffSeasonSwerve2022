package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCMD extends CommandBase {
    XboxController xbox = new XboxController(1);
    private final SwerveSubsystem swerveSubsystem; //MAYBE THESE NEED FINAL
    private final Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public SwerveJoystickCMD(SwerveSubsystem swerveSubsystem, 
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction, 
        Supplier<Boolean> fieldOrientedFunction) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turnSpdFunction = turnSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turnLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //Get Joystick Readings
        // double xSpeed = xSpdFunction.get();
        // double ySpeed = ySpdFunction.get();
        // double turnSpeed = turnSpdFunction.get();
        double xSpeed = xbox.getRawAxis(OIConstants.kDriverXAxis);
        double ySpeed = xbox.getRawAxis(OIConstants.kDriverYAxis);
        double turnSpeed = xbox.getRawAxis(OIConstants.kDriverTurnAxis);
        DriverStation.reportWarning("Spiid: " + turnSpeed, false);
        //Deadbanding
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turnSpeed =Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;
        DriverStation.reportWarning("Spood: " + turnSpeed, false);
        //Ratelimiting
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turnSpeed = turnLimiter.calculate(turnSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond; 
        DriverStation.reportWarning("Speed: " + turnSpeed, false);
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        // if(fieldOrientedFunction.get()){
        //     //Field Relative
        //     ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2D());
        // }else{
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        // }
        //Making Module States
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Output to Wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}