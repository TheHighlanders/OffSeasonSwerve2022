package frc.robot.subsystems;
// BASED ON https://www.youtube.com/watch?v=0Xi9yb1IMyA
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//Encoder imports
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;


public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder angleEncoder;

  private final PIDController anglePIDController;

  private final SparkMaxLimitSwitch fakeLimit;

  //Alt Switch Code
  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 2048;
    /**
   * An alternate encoder object is constructed using the GetAlternateEncoder() 
   * method on an existing CANSparkMax object.
   */
  private RelativeEncoder absoluteEncoder;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed, 
  int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
  
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
 
    driveMotor.setInverted(driveMotorReversed);
    angleMotor.setInverted(angleMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

    fakeLimit = angleMotor.getForwardLimitSwitch(ModuleConstants.kEncoderFakeLimitType);
    //
    absoluteEncoder = angleMotor.getAlternateEncoder(kAltEncType, kCPR);


    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMotorEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMotorEncoderRPM2MeterPerSec);
    angleEncoder.setPositionConversionFactor(ModuleConstants.kAngleMotorEncoderRot2Rad);
    angleEncoder.setVelocityConversionFactor(ModuleConstants.kAngleMotorEncoderRPM2RadPerSec);

    anglePIDController = new PIDController(ModuleConstants.kPAngle, 0, 0);
    anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

    homingRoutine(); //CHANGED FROM resetEncoders() UNTESTED
    }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getAnglePosition(){
    return angleEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getAngleVelocity(){
    return angleEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getPosition();
    angle *= 2 *Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    angleEncoder.setPosition(getAbsoluteEncoderRad());
    absoluteEncoder.setPosition(0);

  }

  public void homingRoutine(){
    while(-0.3 < absoluteEncoder.getPosition()  && absoluteEncoder.getPosition() < 0.3){
      if(absoluteEncoder.getPosition() < 0){
        angleMotor.set(-ModuleConstants.kAngleHomingSpeed); 
      }
      if(absoluteEncoder.getPosition() < 0){
        angleMotor.set(ModuleConstants.kAngleHomingSpeed);
      }
    }

    angleMotor.stopMotor();

    absoluteEncoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    
    
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    angleMotor.set(anglePIDController.calculate(getAnglePosition(),state.angle.getRadians()));
    //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    SmartDashboard.putString("Swerve[" + angleMotor.getDeviceId() + "] state", state.toString());
  }

  public void stop(){
    driveMotor.set(0);
    angleMotor.set(0);
  }
}