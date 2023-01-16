// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.time.Instant;
//import java.util.List;
//import java.util.function.Supplier;

import frc.robot.commands.AUTOhomeModulesCMD;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.ZeroHeadingCMD;
import frc.robot.commands.ToggleFieldOrientedCMD;
//import frc.robot.commands.encoderPrintout;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AUTOsubsystem;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final AUTOsubsystem auto = new AUTOsubsystem();

  private final XboxController driverJoystick = new XboxController(OIConstants.kdriverJoystick);

  //private final encoderPrintout encoderPrintoutCMD = new encoderPrintout(swerveSubsystem);

  private final ZeroHeadingCMD zeroHeadingCMD = new ZeroHeadingCMD(swerveSubsystem);

  private final ToggleFieldOrientedCMD toggleFieldOrientedCMD = new ToggleFieldOrientedCMD(swerveSubsystem);
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(swerveSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 2).onTrue(zeroHeadingCMD);

    new JoystickButton(driverJoystick, 1).onTrue(toggleFieldOrientedCMD);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //Construct Auto Swerve Command Using Points and Objects from AUTOsubsystem (auto)
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        auto.getTrajectory(),
        swerveSubsystem::getPose2d,
        DriveConstants.kDriveKinematics,
        auto.getXController(),
        auto.getYController(),
        auto.getThetaController(),
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    return new SequentialCommandGroup(
        new AUTOhomeModulesCMD(swerveSubsystem),
        //new InstantCommand(()-> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), // SOME TRAJECTORY STUFF: TODO: MIGRATE TO ACTUAL COMMAND
        //swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
