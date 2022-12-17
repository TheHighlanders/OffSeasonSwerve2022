// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ModuleConstants{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/5.8462;
        public static final double kAngleMotorGearRatio = 1/18;
        public static final double kDriveMotorEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kAngleMotorEncoderRot2Rad = kAngleMotorGearRatio * 2 * Math.PI;
        public static final double kDriveMotorEncoderRPM2MeterPerSec = kDriveMotorEncoderRot2Meter /60;
        public static final double kAngleMotorEncoderRPM2RadPerSec = kAngleMotorEncoderRot2Rad / 60;
        public static final double kPAngle = 0.1;

        public static final SparkMaxLimitSwitch.Type kEncoderFakeLimitType = Type.kNormallyOpen;
        public static final double kAngleHomingSpeed = 0.25;
    }

    public static final class DriveConstants{
    //Robot Physical Parameters        
        //Speed Stats
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond/4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond/4;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        
        //Size Info
        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
           new Translation2d(kWheelBase/2, -kTrackWidth/2),
           new Translation2d(kWheelBase/2, kTrackWidth/2),
           new Translation2d(-kWheelBase/2, -kTrackWidth/2),
           new Translation2d(-kWheelBase/2, kTrackWidth/2)
           );
        
        
    //Module Info
        //Front Left
        public static final int kFrontLeftDrivePort = 2;
        public static final int kFrontLeftAnglePort = 4;
        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kFrontLeftAngleReversed = false;
        public static final int kFrontLeftAbsoluteEncoderPort = 2;
        public static final double kFrontLeftAbsoluteEncoderOffsetRad = 0;
        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
           
        //Front Right
        public static final int kFrontRightDrivePort = 7;
        public static final int kFrontRightAnglePort = 8;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kFrontRightAngleReversed = false;
        public static final int kFrontRightAbsoluteEncoderPort = 5;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = 0;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        
        //Back Left
        public static final int kBackLeftDrivePort = 3;
        public static final int kBackLeftAnglePort = 1;
        public static final boolean kBackLeftDriveReversed = false;
        public static final boolean kBackLeftAngleReversed = false;
        public static final int kBackLeftAbsoluteEncoderPort = 8;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = 0;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;

        //Back Right
        public static final int kBackRightDrivePort = 6;
        public static final int kBackRightAnglePort = 5;
        public static final boolean kBackRightDriveReversed = false;
        public static final boolean kBackRightAngleReversed = false;
        public static final int kBackRightAbsoluteEncoderPort = 11;
        public static final double kBackRightAbsoluteEncoderOffsetRad = 0;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;
    }

    public static final class OIConstants{
        public static final double kDeadband = 0.05;
        public static final int kdriverJoystick = 1;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverTurnAxis = 4;
        public static final int kDriverFieldOrientButton = 3;
    }

    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;


        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final TrapezoidProfile.Constraints kPThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, 
                kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}
