// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
        public static final double kPAngle = 0.5;

    }

    public static final class DriveConstants{
    //Robot Physical Parameters        
        //Speed Stats
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 3;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        
        //Size Info
        public static final double kTrackWidth = Units.inchesToMeters(30);
        public static final double kWheelBase = Units.inchesToMeters(30);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
           new Translation2d(kWheelBase/2, -kTrackWidth/2),
           new Translation2d(kWheelBase/2, kTrackWidth/2),
           new Translation2d(-kWheelBase/2, -kTrackWidth/2),
           new Translation2d(-kWheelBase/2, kTrackWidth/2)
           );
        
        
    //Module Info
        //Front Left
        public static final int kFrontLeftDrivePort = 0;
        public static final int kFrontLeftAnglePort = 1;
        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kFrontLeftAngleReversed = false;
        public static final int kFrontLeftAbsoluteEncoderPort = 2;
        public static final double kFrontLeftAbsoluteEncoderOffsetRad = 0;
        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
           
        //Front Right
        public static final int kFrontRightDrivePort = 3;
        public static final int kFrontRightAnglePort = 4;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kFrontRightAngleReversed = false;
        public static final int kFrontRightAbsoluteEncoderPort = 5;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = 0;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        
        //Back Left
        public static final int kBackLeftDrivePort = 6;
        public static final int kBackLeftAnglePort = 7;
        public static final boolean kBackLeftDriveReversed = false;
        public static final boolean kBackLeftAngleReversed = false;
        public static final int kBackLeftAbsoluteEncoderPort = 8;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = 0;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;

        //Back Right
        public static final int kBackRightDrivePort = 9;
        public static final int kBackRightAnglePort = 10;
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
        public static final int kDriverTurnAxis = 2;
        public static final int kDriverFieldOrientButton = 3;
    }
}
