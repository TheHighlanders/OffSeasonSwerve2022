package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivePort, 
        DriveConstants.kFrontLeftAnglePort, 
        DriveConstants.kFrontLeftDriveReversed, 
        DriveConstants.kFrontLeftAngleReversed, 
        DriveConstants.kFrontLeftAbsoluteEncoderPort, 
        DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivePort, 
        DriveConstants.kFrontRightAnglePort, 
        DriveConstants.kFrontRightDriveReversed, 
        DriveConstants.kFrontRightAngleReversed, 
        DriveConstants.kFrontRightAbsoluteEncoderPort, 
        DriveConstants.kFrontRightAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontRightAbsoluteEncoderReversed);
    
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDrivePort, 
        DriveConstants.kBackLeftAnglePort, 
        DriveConstants.kBackLeftDriveReversed, 
        DriveConstants.kBackLeftAngleReversed, 
        DriveConstants.kBackLeftAbsoluteEncoderPort, 
        DriveConstants.kBackLeftAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDrivePort, 
        DriveConstants.kBackRightAnglePort, 
        DriveConstants.kBackRightDriveReversed, 
        DriveConstants.kBackRightAngleReversed, 
        DriveConstants.kBackRightAbsoluteEncoderPort, 
        DriveConstants.kBackRightAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackRightAbsoluteEncoderReversed);


    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
        new Rotation2d(0), new SwerveModulePosition[] {
            frontLeft.getState(), frontRight.getState(),
            backLeft.getState(),  backRight.getState()
        } ); // Could Add OPTIONAL ROBOT Starting pose for field posing

    public SwerveSubsystem (){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2D(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d(){
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2D(), new SwerveModulePosition[] {
            frontLeft.getState(), frontRight.getState(),
            backLeft.getState(),  backRight.getState()
        }, pose);
    }

    @Override
    public void periodic(){
        odometer.update(getRotation2D(), 
            new SwerveModulePosition[] {
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(),  backRight.getState()
            });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    public void homeAllModules(){
        // frontLeft.homingRoutine();
        // frontRight.homingRoutine();
        // backLeft.homingRoutine();
        // backRight.homingRoutine();
    }

    public void zeroAllModules(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}