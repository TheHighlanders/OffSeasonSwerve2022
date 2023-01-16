// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AUTOsubsystem extends SubsystemBase {
  /** Creates a new AUTOsubsystem. */
  //Config Setup
  TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

  //create trajectroy from points
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(1, 0),
          new Translation2d(1, -1)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

  //PID Controllers for trajectory
  PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
      AutoConstants.kPThetaControllerConstraints);

  public AUTOsubsystem() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

  }

  public PIDController getXController() {
    return xController;
  }

  public PIDController getYController() {
    return yController;
  }

  public ProfiledPIDController getThetaController() {
    return thetaController;
  }

  public Trajectory getTrajectory() {
    return trajectory;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
