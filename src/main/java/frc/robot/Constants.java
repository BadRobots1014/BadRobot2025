// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class BlinkinConstants {
    public final static int kBlinkinPort = 0;
    public final static double kBlinkinDefaultColorCode = .57;
    public static final HashMap<String, Double> colorCode = new HashMap<>(
      Map.ofEntries(
        Map.entry("rainbow-rainbow", -0.99d),
        Map.entry("rainbow-party", -0.97d),
        Map.entry("rainbow-ocean", -0.95d),
        Map.entry("rainbow-lava", -0.93d),
        Map.entry("rainbow-forest", -0.91d),
        Map.entry("rainbow-glitter", -0.89d),
        Map.entry("confetti", -0.87d),
        Map.entry("shot-red", -0.85d),
        Map.entry("shot-blue", -0.83d),
        Map.entry("shot-white", -0.81d),
        Map.entry("sinelon-rainbow", -0.79d),
        Map.entry("sinelon-party", -0.77d),
        Map.entry("sinelon-ocean", -0.75d),
        Map.entry("sinelon-lava", -0.73d),
        Map.entry("sinelon-forest", -0.71d),
        Map.entry("bpm-rainbow", -0.69d),
        Map.entry("bpm-party", -0.67d),
        Map.entry("bpm-ocean", -0.65d),
        Map.entry("bpm-lava", -0.63d),
        Map.entry("bpm-forest", -0.61d),
        Map.entry("fire-medium", -0.59d),
        Map.entry("fire-large", -0.57d),
        Map.entry("twinkles-rainbow", -0.55d),
        Map.entry("twinkles-party", -0.53d),
        Map.entry("twinkles-ocean", -0.51d),
        Map.entry("twinkles-lava", -0.49d),
        Map.entry("twinkles-forest", -0.47d),
        Map.entry("waves-rainbow", -0.45d),
        Map.entry("waves-party", -0.43d),
        Map.entry("waves-ocean", -0.41d),
        Map.entry("waves-lava", -0.39d),
        Map.entry("waves-forest", -0.37d),
        Map.entry("scanner-red", -0.35d),
        Map.entry("scanner-gray", -0.33d),
        Map.entry("chase-red", -0.31d),
        Map.entry("chase-blue", -0.29d),
        Map.entry("chase-gray", -0.27d),
        Map.entry("heartbeat-red", -0.25d),
        Map.entry("heartbeat-blue", -0.23d),
        Map.entry("heartbeat-white", -0.21d),
        Map.entry("heartbeat-gray", -0.19d),
        Map.entry("breath-red", -0.17d),
        Map.entry("breath-blue", -0.15d),
        Map.entry("breath-gray", -0.13d),
        Map.entry("strobe-red", -0.11d),
        Map.entry("strobe-blue", -0.09d),
        Map.entry("strobe-gold", -0.07d),
        Map.entry("strobe-white", -0.05d),
        Map.entry("blend-color1", -0.03d),
        Map.entry("scanner-color1", -0.01d),
        Map.entry("chase-color1", 0.01d),
        Map.entry("heartbeat-color1-slow", 0.03d),
        Map.entry("heartbeat-color1-medium", 0.05d),
        Map.entry("heartbeat-color1-fast", 0.07d),
        Map.entry("breath-color1-slow", 0.09d),
        Map.entry("breath-color1-fast", 0.11d),
        Map.entry("shot-color1", 0.13d),
        Map.entry("strobe-color1", 0.15d),
        Map.entry("blend-color2", 0.17d),
        Map.entry("scanner-color2", 0.19d),
        Map.entry("chase-color2", 0.21d),
        Map.entry("heartbeat-color2-slow", 0.23d),
        Map.entry("heartbeat-color2-medium", 0.25d),
        Map.entry("heartbeat-color2-fast", 0.27d),
        Map.entry("breath-color2-slow", 0.29d),
        Map.entry("breath-color2-fast", 0.31d),
        Map.entry("shot-color2", 0.33d),
        Map.entry("strobe-color2", 0.35d),
        Map.entry("sparkle-1on2", 0.37d),
        Map.entry("sparkle-2on1", 0.39d),
        Map.entry("gradient-1on2", 0.41d),
        Map.entry("bpm-1and2", 0.43d),
        Map.entry("blend-1to2", 0.45d),
        Map.entry("blend-2to1", 0.47d),
        Map.entry("setup-1and2", 0.49d),
        Map.entry("twinkles-1and2", 0.51d),
        Map.entry("waves-1and2", 0.53d),
        Map.entry("sinelon-1and2", 0.55d),
        Map.entry("solid-hotpink", 0.57d),
        Map.entry("solid-darkred", 0.59d),
        Map.entry("solid-red", 0.61d),
        Map.entry("solid-redorange", 0.63d),
        Map.entry("solid-orange", 0.65d),
        Map.entry("solid-gold", 0.67d),
        Map.entry("solid-yellow", 0.69d),
        Map.entry("solid-lawngreen", 0.71d),
        Map.entry("solid-lime", 0.73d),
        Map.entry("solid-darkgreen", 0.75d),
        Map.entry("solid-green", 0.77d),
        Map.entry("solid-bluegreen", 0.79d),
        Map.entry("solid-aqua", 0.81d),
        Map.entry("solid-skyblue", 0.83d),
        Map.entry("solid-darkblue", 0.85d),
        Map.entry("solid-blue", 0.87d),
        Map.entry("solid-blueviolet", 0.89d),
        Map.entry("solid-violet", 0.91d),
        Map.entry("solid-white", 0.93d),
        Map.entry("solid-gray", 0.95d),
        Map.entry("solid-darkgray", 0.97d),
        Map.entry("solid-black", 0.99d)
      )
    );
  } 

  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = 0.102;
    public static final double kDriveMotorGearRatio = 1/9.1d;
    public static final double kTurningMotorGearRatio = 1/12.8;
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

    // Preset drive angles
    public static final double kSourceTheta = 60;
    public static final double kSpeakerTheta = 180;

    // Turn theta
    public static final double kTurnThetaMaxSpeed = 0.9;
    public static final HashMap<Integer, Double> aprilTagAngles = new HashMap<>(
      Map.ofEntries(
        Map.entry(1, 126d),
        Map.entry(2, 234d),
        Map.entry(3, 270d),
        Map.entry(4, 0d),
        Map.entry(5, 0d),
        Map.entry(6, 300d),
        Map.entry(7, 0d),
        Map.entry(8, 60d),
        Map.entry(9, 120d),
        Map.entry(10, 180d),
        Map.entry(11, 240d),
        Map.entry(12, 54d),
        Map.entry(13, 306d),
        Map.entry(14, 180d),
        Map.entry(15, 180d),
        Map.entry(16, 90d),
        Map.entry(17, 240d),
        Map.entry(18, 180d),
        Map.entry(19, 120d),
        Map.entry(20, 60d),
        Map.entry(21, 0d),
        Map.entry(22, 300d)
      )
    );
    public static final double kTurnThetaShutoffSensitivity = 0.005;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(24.75);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.75);
    // Positions of modules relative to the center of mass
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front left
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front right
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // Back left
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2) // Back right
    );

    // TODO Using tall bot?
    public static final boolean tallBot = false;
    public static final boolean steve = true;

    // Short bot offsets
    public static final double kFROffset = Math.PI / 2 - 2;
    public static final double kBROffset = -Math.PI;
    public static final double kBLOffset = -Math.PI / 2;
    public static final double kFLOffset = 0;

    // Tall bot offsets
    public static final double kTallBLOffset = -.097 * Math.PI * 2 + Math.PI/2;
    public static final double kTallFLOffset = .179 * Math.PI * 2 + Math.PI/2;
    public static final double kTallFROffset = .314 * Math.PI * 2 + (Math.PI / 2) + Math.PI/2;
    public static final double kTallBROffset = .106 * Math.PI * 2 + Math.PI/2;

    public static final double kSteveBLOffset = -.007568 * Math.PI * 2;
    public static final double kSteveFLOffset = -.083252 * Math.PI * 2;
    public static final double kSteveFROffset = .480713 * Math.PI * 2;
    public static final double kSteveBROffset = -.250997 * Math.PI * 2;

    // Angular offsets of the modules relative to the chassis in radians

    public static final double kFrontRightChassisAngularOffset = kSteveFROffset; //(steve ? kSteveFROffset : tallBot ? kTallBROffset : kBROffset);
    public static final double kBackRightChassisAngularOffset = kSteveBROffset; //(steve ? kSteveBROffset : tallBot ? kTallBROffset : kBROffset);
    public static final double kBackLeftChassisAngularOffset = kSteveBLOffset; // (steve ? kSteveBLOffset : tallBot ? kTallBLOffset : kBLOffset);
    public static final double kFrontLeftChassisAngularOffset = kSteveFLOffset; // (steve ? kSteveFLOffset : tallBot ? kTallFLOffset : kFLOffset);

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kFrontLeftDrivingCanId = 21;
    public static final int kRearLeftDrivingCanId = 31;
    public static final int kRearRightDrivingCanId = 41;

    public static final int kFrontRightTurningCanId = 12;
    public static final int kFrontLeftTurningCanId = 22;
    public static final int kRearLeftTurningCanId = 32;
    public static final int kRearRightTurningCanId = 42;

    public static final int kFrontRightEncoderCanId = 13;
    public static final int kFrontLeftEncoderCanId = 23;
    public static final int kRearLeftEncoderCanId = 33;
    public static final int kRearRightEncoderCanId = 43;

    public static final boolean kGyroReversed = false;

    // Reverse encoders if needed; note that this will break everything if you don't
    // go through and fix everything afterward
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontLeftAbsoluteEncoderReversed = true;

    public static final boolean kFrontRightDriveEncoderReversed = true;
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

    // public static final Button kTestMotorButton = Button.kLeftBumper;
  }

  public static final class ElevatorConstants {
    public static final int kLeftElevatorCanId = 58;
    public static final int kRightElevatorCanId = 59;
    public static final int kEncoderCanId = 60;

    public static final double kElevatorUpPower = .2;
    public static final double kElevatorDownPower = -.2;
    public static final int kElevatorMaxAmps = 140;

    public static final double kLvlOnePos = 90.0;
    public static final double kLvlTwoPos = 180.0;
    public static final double kLvlThreePos = 270.0;
    public static final double kLvlFourPos = 359.0;

    public static final double kElevatorP = 1.0;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;
    // Limelight Automation
    public static final double kAutoSpeedLimit = 2;
    public static final double kAutoTargetDistance = 0.1524; // How far away it aims to be from april tag

    // public static final Button kTestMotorButton = Button.kLeftBumper;
  }

  public static final class CoralConstants {
    public static final double kCoralInSpeed = .5;
    public static final double kCoralOutSpeed = -.2;
    public static final int kCoralCanID = -1;
    // public static final Button kTestMotorButton = Button.kLeftBumper;
  }

  public static class AlgaeConstants {
    public static final int kLeftCanId = 51;
    public static final int kRightCanId = 52;

    public static final int kMaxAmps = 140;
    public static final double kPower = .5;

    public static final double kTopMaxSpeed = 0.1;
    public static final double kBottomMaxSpeed = 0.1;
  }
}
