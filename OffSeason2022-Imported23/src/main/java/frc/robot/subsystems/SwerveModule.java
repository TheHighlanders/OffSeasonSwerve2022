package frc.robot.subsystems;

// BASED ON https://www.youtube.com/watch?v=0Xi9yb1IMyA
//import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

//import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax angleMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder angleEncoder;

  private final PIDController anglePIDController;

  private final edu.wpi.first.wpilibj.AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  //TODO: Instead of hardcoding driveMotorReversed and angleMotorReversed, create a enumeration type to represent the different types of limit switches, such as Forward, Reverse
  public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed,
      int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    absoluteEncoder = new edu.wpi.first.wpilibj.AnalogInput(absoluteEncoderID);

    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    angleMotor.setInverted(angleMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMotorEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMotorEncoderRPM2MeterPerSec);
    angleEncoder.setPositionConversionFactor(ModuleConstants.kAngleMotorEncoderRot2Rad);
    angleEncoder.setVelocityConversionFactor(ModuleConstants.kAngleMotorEncoderRPM2RadPerSec);

    anglePIDController = new PIDController(ModuleConstants.kPAngle, 0, 0);
    anglePIDController.enableContinuousInput(0, 2 * Math.PI);
    //TODO: Instead of hardcoding Type.kNormallyClosed for the limit switches, create a enumeration type to represent the different types of limit switches, such as NormallyClosed, NormallyOpen
    angleMotor.getForwardLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false); //disables the limit switches for the drive and angle motors
    angleMotor.getReverseLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false);

    driveMotor.getForwardLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false);
    driveMotor.getReverseLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(false);

  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getAnglePosition() {
    return angleEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getAngleVelocity() {
    return angleEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public double getAbsoluteEncoderRadNoOffset() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    angleEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModulePosition getState() {
    return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getAnglePosition()));
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);

    if (Math.abs(delta.getDegrees()) > 90.0) {
      //DriverStation.reportWarning("Optizme Trigged", false);
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  public void setDesiredState(SwerveModuleState state) {
    // boolean useDashboard = SmartDashboard.getBoolean("Setpoint Control Enabled", false);
    //angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));
    // SmartDashboard.putNumber("State Get DEG" + absoluteEncoder.getChannel() + ":", state.angle.getDegrees());
    // if(Math.abs(state.speedMetersPerSecond) < 0.001){
    //   stop();
    //   return;
    // }

    state = optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));
    // SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop() {
    driveMotor.set(0);
  }
}