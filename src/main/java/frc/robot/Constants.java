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
            Map.entry("solid-black", 0.99d)));
    public final static double kBlinkinDefaultColorCode = colorCode.get("scanner-color1");
  }

  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = 0.102;
    public static final double kDriveMotorGearRatio = 1 / 9.1d;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kModuleDeadband = 0.005;
    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.01;
    public static final double kTurningD = 0.0;
    public static final double kTurningPeriod = .005;
    public static final double kRampRate = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
    public static final double kDriveDeadband = 0.02;
    public static final double kTriggerDeadband = 0.75;
  }

  public final class ControllerConstants {
    public final static int kSecondControllerPortOne = 1;
    public final static int kSecondControllerPortTwo = 2;
  }

  public static final class CoralControllerConstants {
    // Coral Levels
    public final static int kLeftLevel1 = 1;
    public final static int kRightLevel1 = 2;
    public final static int kLeftLevel2 = 3;
    public final static int kRightLevel2 = 4;
    public final static int kLeftLevel3 = 5;
    public final static int kRightLevel3 = 6;
    public final static int kLeftLevel4 = 7;
    public final static int kRightLevel4 = 8;

    // Coral Side
    public static final int directionLeft = -1;
    public static final int directionRight = 1;
  }

  public static final class AuxControllerConstants {
    // Uses Controller secondaryControllerOne
    public final static int kLeftBottom = 1;
    public final static int kRightBottom = 2;
    public final static int kLeftLowerMid = 3;
    public final static int kRightLowerMid = 4;
    public final static int kLeftUpperMid = 5;
    public final static int kRightUpperMid = 6;
    public final static int kLeftTop = 7;
    public final static int kRightTop = 8;
  }

  public static final class HexControllerConstants {
    // Uses Controller 1
    public final static int kTopLeft = 9;
    public final static int kTop = 10;
    public final static int kTopRight = 11;
    // Uses controller 2
    public final static int kBottomLeft = 9;
    public final static int kBottom = 10;
    public final static int kBottomRight = 11;
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
            Map.entry(22, 300d)));
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

    // TODO Which bot?
    public static final boolean tallBot = false;
    public static final boolean steve = true;

    // Short bot offsets
    public static final double kFROffset = Math.PI / 2 - 2;
    public static final double kBROffset = -Math.PI;
    public static final double kBLOffset = -Math.PI / 2;
    public static final double kFLOffset = 0;

    // Tall bot offsets
    public static final double kTallBLOffset = -.097 * Math.PI * 2 + Math.PI / 2;
    public static final double kTallFLOffset = .179 * Math.PI * 2 + Math.PI / 2;
    public static final double kTallFROffset = .314 * Math.PI * 2 + (Math.PI / 2) + Math.PI / 2;
    public static final double kTallBROffset = .106 * Math.PI * 2 + Math.PI / 2;

    public static final double kSteveBLOffset = -.007568 * Math.PI * 2;
    public static final double kSteveFLOffset = -.083252 * Math.PI * 2;
    public static final double kSteveFROffset = .480713 * Math.PI * 2;
    public static final double kSteveBROffset = -.250997 * Math.PI * 2;

    // public static final double kSteveBLOffset = .017822 * Math.PI * 2;
    // public static final double kSteveFLOffset = .072998 * Math.PI * 2;
    // public static final double kSteveFROffset = -.004639 * Math.PI * 2;
    // public static final double kSteveBROffset = .242188 * Math.PI * 2;

    // Angular offsets of the modules relative to the chassis in radians

    public static final double kFrontRightChassisAngularOffset = kSteveFROffset; // (steve ? kSteveFROffset : tallBot ?
                                                                                 // kTallBROffset : kBROffset);
    public static final double kBackRightChassisAngularOffset = kSteveBROffset; // (steve ? kSteveBROffset : tallBot ?
                                                                                // kTallBROffset : kBROffset);
    public static final double kBackLeftChassisAngularOffset = kSteveBLOffset; // (steve ? kSteveBLOffset : tallBot ?
                                                                               // kTallBLOffset : kBLOffset);
    public static final double kFrontLeftChassisAngularOffset = kSteveFLOffset; // (steve ? kSteveFLOffset : tallBot ?
                                                                                // kTallFLOffset : kFLOffset);

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = steve ? 11 : 31;
    public static final int kFrontRightTurningCanId = steve ? 12 : 32;
    public static final int kFrontRightEncoderCanId = steve ? 13 : 33;

    public static final int kFrontLeftDrivingCanId = 21;
    public static final int kFrontLeftTurningCanId = 22;
    public static final int kFrontLeftEncoderCanId = 23;

    public static final int kRearLeftDrivingCanId = steve ? 31 : 11;
    public static final int kRearLeftTurningCanId = steve ? 32 : 12;
    public static final int kRearLeftEncoderCanId = steve ? 33 : 13;

    public static final int kRearRightDrivingCanId = 41;
    public static final int kRearRightTurningCanId = 42;
    public static final int kRearRightEncoderCanId = 43;

    public static final boolean kGruReversed = false;

    // Reverse encoders if needed; note that this will break everything if you don't
    // go through and fix everything afterward
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontLeftAbsoluteEncoderReversed = true;
    public static final boolean kFrontLeftDrivingReversed = false;

    public static final boolean kFrontRightDriveEncoderReversed = !steve; // This encoder is reversed only on the steve bot.
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kFrontRightAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDrivingReversed = true;

    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDrivingReversed = false;

    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final boolean kBackRightAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDrivingReversed = false;

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

    // Limelight Automation
    public static final double kAutoSpeedLimit = 2;
    public static final double kAutoTargetDistance = 0.1524; // How far away it aims to be from april tag
    public static final double kAutoTranslationalP = 3;
    public static final double kAutoTranslationalI = 0;
    public static final double kAutoTranslationalD = 0;
    public static final double kAutoRotationalP = 1;
    public static final double kAutoRotationalI = 0;
    public static final double kAutoRotationalD = 0;

    // public static final Button kTestMotorButton = Button.kLeftBumper;
  }

  public static final class ElevatorConstants {
    public static final int kLeftElevatorCanId = 58;
    public static final int kRightElevatorCanId = 59;

    public static final double kElevatorUpPower = .3;
    public static final double kElevatorDownPower = -.2;
    public static final int kElevatorMaxAmps = 140;

    public static final double kLvlOnePos = 0;
    public static final double kLvlTwoPos = 1;
    public static final double kLvlThreePos = 2;
    public static final double kLvlFourPos = 3;

    public static final double kLvlAlgaeOnePos = 359.0;
    public static final double kLvlAlgaeTwoPos = 359.0;

    public static final double kElevatorP = 0.005;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;

    // public static final Button kTestMotorButton = Button.kLeftBumper;
  }

  public static final class CoralConstants {
    public static final double kCoralUpSpeed = .3;
    public static final double kCoralDownSpeed = -.3;

    public static final double kCoralDurationSeconds = 1;
    public static final double kCoralDurationNano = kCoralDurationSeconds * 1_000_000_000L; // For timeout

    public static final int kCoralCanID = 54;

    public enum CoralMode {
      UP,
      DOWN,
      UP_OVERRIDE
    }
  }

  public static class AlgaeConstants {
    public static final int kLeftCanId = 51;
    public static final int kRightCanId = 52;

    public static final int kMaxAmps = 140;
    public static final double kPower = 1;
  }

  public static final class ClimberConstants {
    //public static final int kLeftClimberCanId = 0;
    //public static final int kRightClimberCanId = 0;
    public static final int kClimberCanId = 53;
    public static final int kClimberUpPower = -1;
    public static final int kClimberDownPower = 1;
    public static final int kClimberMaxAmps = 200000;
  }

  public static final class WinchConstants {
    public static final int kWinchCanId = 55;
    public static final double kWinchUpPower = 1;
    public static final double kWinchDownPower = -1;
  }
}
