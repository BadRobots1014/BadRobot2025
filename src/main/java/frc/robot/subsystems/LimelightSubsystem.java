// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Subsystem dedicated to specific processes to control camera
 * Commands trigger processes contained in this class
 */

 package frc.robot.subsystems;

 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.util.LimelightHelpers;
 
 public class LimelightSubsystem extends SubsystemBase {
 
     /** Creates a new LimelightSubsystem. */
     public LimelightSubsystem(double forward, double side, double height) {
         // Change the camera pose relative to robot center (x forward, y left, z up)
         LimelightHelpers.setCameraPose_RobotSpace("",
                 forward, // Forward offset (meters)
                 side, // Side offset (meters)
                 height, // Height offset (meters)
                 90.0, // Roll (degrees)
                 0.0, // Pitch (degrees)
                 0.0 // Yaw (degrees)
         );
 
         // Crop window for performance
         //LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
     }
 
     /*
      * Creates a new LimelightSubsystem.
      * TODO: Add constants to offsets in Constants file
      */
     public LimelightSubsystem() {
         // Change the camera pose relative to robot center (x forward, y left, z up)
         LimelightHelpers.setCameraPose_RobotSpace("", //blank because idk
                 0, // Forward offset (meters)
                 0, // Side offset (meters)
                 0, // Height offset (meters)
                 90.0, // Roll (degrees)
                 0.0, // Pitch (degrees)
                 0.0 // Yaw (degrees)
         );
 
         // Crop window for performance
         //LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
     }
 }
 