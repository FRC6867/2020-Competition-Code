/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.VideoCamera;

import frc.robot.Constants.VisionConstants;

/**
 * yay i got a camera...
 * 
 * Just don't ask me how it works ok
 */
public class Vision extends SubsystemBase {
  private VideoCamera m_mainCamera;
  //private VideoCamera m_climbCamera; // Climb camera is disabled for now

  /**
   * Creates a new Camera vision thing.
   */
  public Vision() {
    // Turn on main cam
    m_mainCamera = CameraServer.getInstance().startAutomaticCapture(
      VisionConstants.MAIN_CAMERA_NAME,
      VisionConstants.MAIN_CAMERA_PORT_NUM
    );

    m_mainCamera.setResolution(VisionConstants.MAIN_CAMERA_WIDTH, VisionConstants.MAIN_CAMERA_HEIGTH);
    m_mainCamera.setFPS(VisionConstants.MAIN_CAMERA_FPS);
  }

  // Not nessesary for now
  // // Turns on climb cam
  // private void turnOnClimbCam() {
  //   m_climbCamera = CameraServer.getInstance().startAutomaticCapture(
  //     VisionConstants.CLIMB_CAMERA_NAME,
  //     VisionConstants.CLIMB_CAMERA_PORT_NUM
  //   );
  // }

  // public boolean isClimbTime() {
  //   // We want this to return true when we are in TeleOp and there's only x seconds remaining.
  //   final DriverStation m_ds = DriverStation.getInstance();
  //   return m_ds.isOperatorControl() && m_ds.getMatchTime() < VisionConstants.CAMERA_TURN_ON_TIME;
  // }

  // @Override
  // public void periodic() {
  //   if (m_climbCamera == null && isClimbTime()) { // If should turn on and not turned on already
  //     turnOnClimbCam();
  //   }
  // }
}