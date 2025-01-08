// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

  PS5Controller driver = new PS5Controller(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubsystems();
    setDefaultCommands();
  }

  private void initializeSubsystems() {
    DrivetrainSubsystem.getInstance();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
                    new Drive(() -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                    DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                    ? -driver.getLeftY()
                                                    : driver.getLeftY(),
                                    Constants.OIConstants.kJoystickDeadband),
                                    () -> MathUtil.applyDeadband(DriverStation.getAlliance().orElse(
                                                    DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                                                                    ? -driver.getLeftX()
                                                                    : driver.getLeftX(),
                                                    Constants.OIConstants.kJoystickDeadband),
                                    () -> MathUtil.applyDeadband(-driver.getRightX(),
                                                    Constants.OIConstants.kJoystickDeadband),
                                    () -> 1));
}

  
}
