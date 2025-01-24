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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kDriveMotorGearRatio = 8.14;
    public static final double kTurningMotorGearRatio = 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kModuleDeadband = 0.005;
    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.01;
    public static final double kTurningD = 0.0;
    public static final double kTurningPeriod = .005;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
    public static final double kDriveDeadband = 0.02;
    public static final double kTriggerDeadband = 0.75;
  }

  public static final class DriveConstants {

    // TODO turn field oriented on or off
    public static final boolean kFieldOriented = true;

    //Preset drive angles
    public static final double kSourceTheta = 60;
    public static final double kSpeakerTheta = 180;
    
    //Turn theta 
    public static final double kTurnThetaMaxSpeed = 0.9;
    public static final double kTurnThetaShutoffSensitivity = 0.005;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(24.75);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.75);
    // Positions of modules relative to the center of mass
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // Front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left
        new Translation2d(kWheelBase / 2, kTrackWidth / 2) // Back right
    );

    // TODO Using tall bot?
    public static final boolean tallBot = true;

    // Short bot offsets
    public static final double kFROffset = Math.PI / 2 - 2;
    public static final double kBROffset = -Math.PI;
    public static final double kBLOffset = -Math.PI / 2;
    public static final double kFLOffset = 0;

    // Tall bot offsets
    public static final double kTallFROffset = -.097 * Math.PI * 2;
    public static final double kTallBROffset = .179 * Math.PI * 2;
    public static final double kTallBLOffset = .314 * Math.PI * 2 + (Math.PI / 2);
    public static final double kTallFLOffset = .106 * Math.PI * 2;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontRightChassisAngularOffset = (tallBot ? kTallFROffset : kFROffset);
    public static final double kBackRightChassisAngularOffset = (tallBot ? kTallBROffset : kBROffset);
    public static final double kBackLeftChassisAngularOffset = (tallBot ? kTallBLOffset : kBLOffset);
    public static final double kFrontLeftChassisAngularOffset = (tallBot ? kTallFLOffset : kFLOffset);

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 21;
    public static final int kRearLeftDrivingCanId = 31;
    public static final int kFrontLeftDrivingCanId = 41;

    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 22;
    public static final int kRearLeftTurningCanId = 32;
    public static final int kFrontLeftTurningCanId = 42;

    public static final int kFrontRightEncoderCanId = 13;
    public static final int kRearRightEncoderCanId = 23;
    public static final int kRearLeftEncoderCanId = 33;
    public static final int kFrontLeftEncoderCanId = 43;

    public static final boolean kGyroReversed = false;

    // Reverse encoders if needed; note that this will break everything if you don't
    // go through and fix everything afterward
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontLeftAbsoluteEncoderReversed = true;

    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kFrontRightAbsoluteEncoderReversed = true;

    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftAbsoluteEncoderReversed = true;

    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final boolean kBackRightAbsoluteEncoderReversed = true;

    public static final long kBootupDelay = 1000; // milliseconds of delay to allow the navx to start up

    public static final double kXSlewRateLimit = 8; // TODO: adjust slew limits
    public static final double kYSlewRateLimit = 8;
    public static final double kTurnSlewRateLimit = 10;

    public static final double kTeleMaxRadiansPerSec = Math.PI / 2; // TODO adjust max teleop speeds
    public static final double kFastTeleMaxRadiansPerSec = Math.PI;
    public static final double kFasterTeleMaxRadiansPerSec = Math.PI;

    public static final double kTeleMaxMetersPerSec = 0.3;
    public static final double kFastTeleMaxMetersPerSec = 1.0;
    public static final double kFasterTeleMaxMetersPerSec = 1.8;
    public static final double kNudgeSpeed = 0.8;

    public static final double kAutoSpeedLimit = 0;

    //public static final Button kTestMotorButton = Button.kLeftBumper;
  }
}
