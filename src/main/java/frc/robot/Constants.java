// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
  public final class Constants {

    public static final class AutoConstants {
      public static int kMaxSpeedMetersPerSecond = 4;
    }

  public static final class DriveConstants {

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 6.01; 
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(25.5);
        public static final double kWheelBase = Units.inchesToMeters(20.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // CAN IDs
        public static final int kFrontLeftDrivingCanId = 10;
        public static final int kRearLeftDrivingCanId = 12;
        public static final int kFrontRightDrivingCanId = 11;
        public static final int kRearRightDrivingCanId = 13;

        public static final int kFrontLeftTurningCanId = 20;
        public static final int kRearLeftTurningCanId = 22;
        public static final int kFrontRightTurningCanId = 21;
        public static final int kRearRightTurningCanId = 23;

        public static final int kFrontLeftTurningEncoderCANId = 0;
        public static final int kRearLeftTurningEncoderCANId = 0;
        public static final int kFrontRightTurningEncoderCANId = 0;
        public static final int kRearRightTurningEncoderCANId = 0;

        public static final double kAutoAimTeleopErrorMargin = 4;
        public static final double kAutoAimAutoErrorMargin = 4;
        public static final double kSlowMode = 0.1;

        public static final int kPigeonCanID = 60;
        public static final boolean kUsePigeon = false;
    }

    public static final class ModuleConstants {
      public static final double kWheelDiameterMeters = 0.1016;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      public static final double kDriveGearRatio = 6.75; // Example ratio for TalonFX swerve
      public static final double kTurningGearRatio = 12.8;

      public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters / kDriveGearRatio;
      public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0;
      public static final double kTurningEncoderPositionFactor = 2 * Math.PI;
      public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0;

      public static final boolean kTurningEncoderInverted = false;

      public static final double kDrivingP = 0.1;
      public static final double kDrivingI = 0.0;
      public static final double kDrivingD = 0.0;
      public static final double kDrivingFF = 0.2;

      public static final double kTurningP = 0.5;
      public static final double kTurningI = 0.0;
      public static final double kTurningD = 0.1;
      public static final double kTurningFF = 0.0;

      public static final double kDrivingMinOutput = -1;
      public static final double kDrivingMaxOutput = 1;

      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;

      public static final double kDriveEncoderVelocityConversionFactor = 1;

      // Updated for TalonFX Neutral Mode
      public static final NeutralModeValue kDrivingMotorNeutralMode = NeutralModeValue.Brake;
      public static final NeutralModeValue kTurningMotorNeutralMode = NeutralModeValue.Brake;

      public static final int kDrivingMotorCurrentLimit = 40;
      public static final int kTurningMotorCurrentLimit = 30;
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