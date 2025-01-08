// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
  public final class Constants {

  public static final class DriveConstants {

      // NEEDS TO BE CHANGED

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 6.01; // NEEDS TO BE CHANGED
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(25.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // CAN IDs - Need to be set
        public static final int kFrontLeftDrivingCanId = 10;
        public static final int kRearLeftDrivingCanId = 12;
        public static final int kFrontRightDrivingCanId = 11;
        public static final int kRearRightDrivingCanId = 13;

        public static final int kFrontLeftTurningCanId = 20;
        public static final int kRearLeftTurningCanId = 22;
        public static final int kFrontRightTurningCanId = 21;
        public static final int kRearRightTurningCanId = 23;

        public static final double kAutoAimTeleopErrorMargin = 4;
        public static final double kAutoAimAutoErrorMargin = 4;
        public static final double kSlowMode = 0.1;

        public static final double kAutoAimP = 3;
        public static final double kAutoAimI = 0.0;
        public static final double kAutoAimD = 0;
        public static final int kPigeonCanID = 60;
        public static final boolean kUsePigeon = false;
    }

    // Need to be adjusted
    public static final class ModuleConstants {
      // Diameter of the swerve module wheels in meters (4 inches converted to meters)
      public static final double kWheelDiameterMeters = 0.1016;
  
      // Circumference of the wheel, used to calculate distance traveled per wheel rotation
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  
      // Gear ratio for the driving motor (MK4i L1 configuration: 6.12:1 reduction)
      public static final double kDriveGearRatio = 6.12;
  
      // Gear ratio for the turning motor (MK4i uses a standard 12.8:1 reduction)
      public static final double kTurningGearRatio = 12.8;
  
      // Conversion factor for driving encoder: meters traveled per motor rotation
      public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters / kDriveGearRatio;
  
      // Conversion factor for driving encoder: meters per second from RPM
      public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0;
  
      // Conversion factor for turning encoder: radians per motor rotation
      public static final double kTurningEncoderPositionFactor = 2 * Math.PI;
  
      // Conversion factor for turning encoder: radians per second from RPM
      public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0;
  
      // Indicates whether the turning encoder is inverted (depends on wiring/setup)
      public static final boolean kTurningEncoderInverted = false;
  
      // PID coefficients for the driving motor (adjust through tuning)
      public static final double kDrivingP = 0.1; // Proportional gain for driving
      public static final double kDrivingI = 0.0; // Integral gain for driving
      public static final double kDrivingD = 0.0; // Derivative gain for driving
      public static final double kDrivingFF = 0.2; // Feedforward term for driving
  
      // PID coefficients for the turning motor (adjust through tuning)
      public static final double kTurningP = 0.5; // Proportional gain for turning
      public static final double kTurningI = 0.0; // Integral gain for turning
      public static final double kTurningD = 0.1; // Derivative gain for turning
      public static final double kTurningFF = 0.0; // Feedforward term for turning
  
      // Minimum and maximum output for the driving motor (-1 to 1 represents full reverse to full forward)
      public static final double kDrivingMinOutput = -1;
      public static final double kDrivingMaxOutput = 1;
  
      // Minimum and maximum output for the turning motor (-1 to 1 represents full reverse to full forward)
      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;
  
      // Motor idle modes: kBrake mode applies resistance when motors are idle, helping the robot hold position
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
  
      // Current limits for the motors (in amps) to protect the motors and controllers
      public static final int kDrivingMotorCurrentLimit = 40; // Maximum current for driving motor
      public static final int kTurningMotorCurrentLimit = 30; // Maximum current for turning motor
  }  
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kJoystickDeadband = 0.075;
  }

}
